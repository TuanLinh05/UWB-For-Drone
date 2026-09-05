import type { AnchorSample, FirmwareInfo, RangeSample } from './types';

export const TAG_ST_CALIBRATION_MISSING = 0x20;
export const TAG_ST_RANGE_REJECT = 0x40;
export const TELEMETRY_INFO_SCHEMA = 2;
const TELEM_INFO_FLAG_LEGACY_ADAPTIVE_SHIFT = 3;
const TELEM_INFO_FLAG_LEGACY_ADAPTIVE_MASK = 0x18;

export interface RangeSampleInput {
  seq: number;
  transportSeq?: number;
  timeMs: number;
  clientTime?: number;
  anchors: AnchorSample[];
}

export interface FirmwareInfoInput {
  schemaVersion: number;
  flags: number;
  rangingMode: number;
  dsCalibratedMask: number;
  rangeFilterMode: number;
  phyProfileId?: number;
  spiClockMhz?: number;
  c9_2MotionMode?: number;
  c9_2GlobalMotionState?: number;
  offsets: Array<{ id: number; activeOffsetUm: number }>;
  receivedAt?: number;
  firmwareBuildId?: string;
}

function isUint32(value: number): boolean {
  return Number.isInteger(value) && value >= 0 && value <= 0xffff_ffff;
}

function finiteNumber(value: number | null | undefined): value is number {
  return typeof value === 'number' && Number.isFinite(value);
}

function normalizeAnchor(input: AnchorSample): AnchorSample {
  const status = Number.isInteger(input.status) ? (input.status as number) & 0xff : undefined;
  const ageIsValid = Number.isInteger(input.ageMs) && input.ageMs >= 0 && input.ageMs <= 0xffff;
  const rawValue = input.rawMm;
  const filteredValue = input.filtMm;
  const fppValue = input.fppDbm;
  const rawIsFinite = finiteNumber(rawValue) && rawValue >= 0;
  const filteredIsFinite = finiteNumber(filteredValue) && filteredValue >= 0;
  const fppIsFinite = finiteNumber(fppValue);
  const valid = input.valid === true && ageIsValid && rawIsFinite && filteredIsFinite;

  const anchor: AnchorSample = {
    id: input.id,
    valid,
    ageMs: ageIsValid ? input.ageMs : 0xffff,
    rawMm: valid && rawIsFinite ? rawValue : null,
    filtMm: valid && filteredIsFinite ? filteredValue : null,
    fppDbm: valid && fppIsFinite ? fppValue : null,
    ...(status === undefined ? {} : { status }),
  };

  // Firmware intentionally preserves raw evidence for calibration-missing and
  // C9 conditioner-rejected measurements while valid=0. Keep it in a separately
  // named diagnostic channel so production consumers can never consume it.
  const carriesRejectedDiagnostic = status !== undefined
    && (status & (TAG_ST_CALIBRATION_MISSING | TAG_ST_RANGE_REJECT)) !== 0;
  if (!valid && carriesRejectedDiagnostic) {
    if (rawIsFinite) anchor.diagnosticRawMm = rawValue;
    if (fppIsFinite) anchor.diagnosticFppDbm = fppValue;
  }

  return anchor;
}

/**
 * Validate a wire sample and establish the one canonical identity boundary.
 * Duplicate or malformed IDs reject the complete sample; silently choosing one
 * record would pair a range with the wrong physical anchor.
 */
export function canonicalizeRangeSample(input: RangeSampleInput): RangeSample | null {
  const transportSeq = input.transportSeq ?? input.seq;
  if (!isUint32(input.seq) || !isUint32(transportSeq) || !isUint32(input.timeMs)) return null;
  if (!Array.isArray(input.anchors)) return null;

  const anchorsById: Record<number, AnchorSample> = {};
  for (const wireAnchor of input.anchors) {
    if (!Number.isInteger(wireAnchor.id) || wireAnchor.id <= 0 || wireAnchor.id > 0xffff) return null;
    if (Object.prototype.hasOwnProperty.call(anchorsById, wireAnchor.id)) return null;
    anchorsById[wireAnchor.id] = normalizeAnchor(wireAnchor);
  }

  const anchors = Object.values(anchorsById).sort((left, right) => left.id - right.id);
  return {
    seq: input.seq >>> 0,
    transportSeq: transportSeq >>> 0,
    timeMs: input.timeMs >>> 0,
    clientTime: input.clientTime ?? Date.now(),
    anchors,
    anchorsById,
  };
}

/** Number of missing transport packets, wrap-safe for uint32 sequences. */
export function missingTransportPackets(lastSeq: number | null, currentSeq: number): number {
  if (lastSeq === null || !isUint32(lastSeq) || !isUint32(currentSeq)) return 0;
  const delta = (currentSeq - lastSeq) >>> 0;
  if (delta === 1) return 0;
  if (delta > 1 && delta < 0x8000_0000) return delta - 1;
  // Duplicate/out-of-order/reset is still one continuity fault, but must not
  // become a multi-billion packet gap through unsigned arithmetic.
  return 1;
}

export function decodeFirmwareInfo(input: FirmwareInfoInput): FirmwareInfo | null {
  /* Schema 1 has no C9.2 fields. Keep it readable for already-flashed tags;
   * schema 2 is emitted by the current firmware. */
  if (input.schemaVersion !== 1 && input.schemaVersion !== TELEMETRY_INFO_SCHEMA) return null;
  if (!Number.isInteger(input.flags) || !Number.isInteger(input.rangingMode)) return null;
  if (!Number.isInteger(input.dsCalibratedMask) || !Number.isInteger(input.rangeFilterMode)) return null;
  if (input.phyProfileId !== undefined && !Number.isInteger(input.phyProfileId)) return null;
  if (input.spiClockMhz !== undefined && !Number.isInteger(input.spiClockMhz)) return null;
  if (input.c9_2MotionMode !== undefined && !Number.isInteger(input.c9_2MotionMode)) return null;
  if (input.c9_2GlobalMotionState !== undefined && !Number.isInteger(input.c9_2GlobalMotionState)) return null;

  const activeOffsetsM: Record<number, number> = {};
  for (const record of input.offsets) {
    if (!Number.isInteger(record.id) || record.id <= 0 || record.id > 0xffff) return null;
    if (!Number.isInteger(record.activeOffsetUm) || !Number.isFinite(record.activeOffsetUm)) return null;
    if (Object.prototype.hasOwnProperty.call(activeOffsetsM, record.id)) return null;
    activeOffsetsM[record.id] = record.activeOffsetUm / 1_000_000;
  }

  const hardwareAntennaDelay = (input.flags & 0x01) !== 0;
  const legacyOffsetEnabled = (input.flags & 0x02) !== 0;
  const dsBuild = (input.flags & 0x04) !== 0;
  const legacyAdaptiveCode = (input.flags & TELEM_INFO_FLAG_LEGACY_ADAPTIVE_MASK)
    >>> TELEM_INFO_FLAG_LEGACY_ADAPTIVE_SHIFT;
  const legacyAdaptiveMode: FirmwareInfo['legacyAdaptiveMode'] = legacyAdaptiveCode === 0
    ? 'off'
    : legacyAdaptiveCode === 1
      ? 'shadow'
      : legacyAdaptiveCode === 2
        ? 'active'
        : 'unknown';
  const c9_2MotionMode: FirmwareInfo['c9_2MotionMode'] = input.schemaVersion < 2
    ? 'unknown'
    : input.c9_2MotionMode === 0
      ? 'off'
      : input.c9_2MotionMode === 1
        ? 'shadow'
        : input.c9_2MotionMode === 2
          ? 'active'
          : 'unknown';
  const c9_2GlobalMotionState: FirmwareInfo['c9_2GlobalMotionState'] = input.schemaVersion < 2
    ? 'unknown'
    : input.c9_2GlobalMotionState === 0
      ? 'reacquire'
      : input.c9_2GlobalMotionState === 1
        ? 'static'
        : input.c9_2GlobalMotionState === 2
          ? 'slow'
          : input.c9_2GlobalMotionState === 3
            ? 'fast'
            : input.c9_2GlobalMotionState === 4
              ? 'settling'
              : input.c9_2GlobalMotionState === 5
                ? 'degraded'
                : 'unknown';
  const rangingMode = input.rangingMode === 0 ? 'ss' : input.rangingMode === 1 ? 'ds' : 'unknown';
  const phyProfileId = (input.phyProfileId ?? 0) & 0xff;
  const phyProfile: FirmwareInfo['phyProfile'] = phyProfileId === 1
    ? 'legacy-1024'
    : phyProfileId === 2
      ? 'fast-256'
      : 'unknown';

  let calibrationProfile: FirmwareInfo['calibrationProfile'] = 'unknown';
  if (rangingMode === 'ds' && dsBuild) calibrationProfile = 'ds';
  else if (rangingMode === 'ss' && !dsBuild && legacyOffsetEnabled && !hardwareAntennaDelay) {
    calibrationProfile = 'legacy';
  }
  else if (rangingMode === 'ss' && !dsBuild && !legacyOffsetEnabled && hardwareAntennaDelay) {
    calibrationProfile = 'residual-hw';
  }

  return {
    schemaVersion: input.schemaVersion,
    calibrationProfile,
    rangingMode,
    activeOffsetsM,
    hardwareAntennaDelay,
    legacyOffsetEnabled,
    dsCalibratedMask: input.dsCalibratedMask & 0xff,
    rangeFilterMode: input.rangeFilterMode & 0xff,
    legacyAdaptiveMode,
    c9_2MotionMode,
    c9_2GlobalMotionState,
    phyProfileId,
    phyProfile,
    spiClockMhz: (input.spiClockMhz ?? 0) & 0xff,
    receivedAt: input.receivedAt ?? Date.now(),
    ...(input.firmwareBuildId ? { firmwareBuildId: input.firmwareBuildId } : {}),
  };
}
