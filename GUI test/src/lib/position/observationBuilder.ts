import type { AnchorPosition, FirmwareInfo } from '../types';
import type {
  CalibrationMode,
  CalibrationRegistry,
  ObservationBuildInput,
  ObservationBuildResult,
  PositionRangingMode,
  RangeCalibrationProfile,
} from './types';

const TAG_ST_RADIO_OR_COMPUTE_FAILURE = 0x0f;
const TAG_ST_DS_FALLBACK = 0x10;
const TAG_ST_CALIBRATION_MISSING = 0x20;
const TAG_ST_RANGE_REJECT = 0x40;

const clamp = (value: number, low: number, high: number) => Math.max(low, Math.min(high, value));

function calibrationModeFor(mode: PositionRangingMode): CalibrationMode {
  return mode === 'DS' ? 'DS' : 'SS';
}

function decodeRangingMode(status: number, defaultMode: 'SS' | 'DS'): PositionRangingMode {
  if ((status & TAG_ST_DS_FALLBACK) !== 0) return 'SS_FALLBACK';
  return defaultMode;
}

function profileSigma(profile: RangeCalibrationProfile, source: 'raw' | 'filtered'): number {
  return source === 'raw' ? profile.rawSigmaM : profile.filteredSigmaM;
}

function statusRejectsSolver(status: number): boolean {
  return (status & (TAG_ST_RADIO_OR_COMPUTE_FAILURE | TAG_ST_CALIBRATION_MISSING | TAG_ST_RANGE_REJECT)) !== 0;
}

function effectiveSigmaM(
  baseSigmaM: number,
  ageMs: number,
  fppDbm: number | null,
  mode: PositionRangingMode,
  config: ObservationBuildInput['config'],
): number {
  const fppPenalty = fppDbm === null || !Number.isFinite(fppDbm)
    ? config.maxFppVarianceMultiplier
    : clamp(1 + Math.max(0, -75 - fppDbm) / 20
      * (config.maxFppVarianceMultiplier - 1), 1, config.maxFppVarianceMultiplier);
  const agePenalty = clamp(1 + ageMs / Math.max(1, config.maxAgeMs)
    * (config.maxAgeVarianceMultiplier - 1), 1, config.maxAgeVarianceMultiplier);
  const statusPenalty = mode === 'SS_FALLBACK' ? config.fallbackVarianceMultiplier : 1;
  return clamp(
    baseSigmaM * Math.sqrt(fppPenalty * agePenalty * statusPenalty),
    config.sigmaFloorM,
    config.sigmaCeilingM,
  );
}

/**
 * Provisional registry for C6 shadow execution. Sigma seeds are deliberately
 * conservative and must be replaced by C4 per-anchor/range profiles before C7.
 */
export function provisionalCalibrationRegistry(info: FirmwareInfo | null): CalibrationRegistry {
  if (!info) return {};
  const registry: Record<number, Partial<Record<CalibrationMode, RangeCalibrationProfile>>> = {};
  for (const id of Object.keys(info.activeOffsetsM).map(Number)) {
    if (!Number.isInteger(id) || id <= 0) continue;
    const entry: Partial<Record<CalibrationMode, RangeCalibrationProfile>> = {};
    const dsCalibrated = id <= 8 && (info.dsCalibratedMask & (1 << (id - 1))) !== 0;
    entry.DS = { calibrated: dsCalibrated, rawSigmaM: 0.15, filteredSigmaM: 0.10 };
    const ssOffset = info.activeOffsetsM[id];
    entry.SS = {
      calibrated: info.calibrationProfile !== 'ds' && Number.isFinite(ssOffset) && Math.abs(ssOffset) > 1e-9,
      rawSigmaM: 0.15,
      filteredSigmaM: 0.10,
    };
    registry[id] = entry;
  }
  return registry;
}

export function buildObservations(input: ObservationBuildInput): ObservationBuildResult {
  const { sample, layout, calibration, config } = input;
  const excluded: ObservationBuildResult['excluded'] = [];
  const usable: ObservationBuildResult['usable'] = [];
  const layoutById = new Map<number, AnchorPosition>();
  let fatal = false;

  for (const anchor of layout) {
    if (!Number.isInteger(anchor.id) || !Number.isFinite(anchor.x) || !Number.isFinite(anchor.y)) {
      excluded.push({ id: Number.isInteger(anchor.id) ? anchor.id : null, reason: 'non-finite' });
      fatal = true;
      continue;
    }
    if (layoutById.has(anchor.id)) {
      excluded.push({ id: anchor.id, reason: 'duplicate-id' });
      fatal = true;
      continue;
    }
    layoutById.set(anchor.id, anchor);
  }

  const seen = new Set<number>();
  for (const record of sample.anchors) {
    if (!Number.isInteger(record.id)) {
      excluded.push({ id: null, reason: 'unknown-anchor-id' });
      fatal = true;
      continue;
    }
    if (seen.has(record.id)) {
      excluded.push({ id: record.id, reason: 'duplicate-id' });
      fatal = true;
      continue;
    }
    seen.add(record.id);

    const anchor = layoutById.get(record.id);
    if (!anchor) {
      excluded.push({ id: record.id, reason: 'missing-layout' });
      continue;
    }
    const profiles = calibration[record.id];
    if (!profiles) {
      excluded.push({ id: record.id, reason: 'unknown-anchor-id' });
      continue;
    }
    const status = record.status ?? 0;
    const mode = decodeRangingMode(status, config.defaultRangingMode);
    const calibrationMode = calibrationModeFor(mode);
    const profile = profiles[calibrationMode];
    if (!profile) {
      const hasOtherMode = Object.values(profiles).some(candidate => candidate?.calibrated);
      excluded.push({ id: record.id, reason: hasOtherMode ? 'wrong-calibration-mode' : 'missing-calibration' });
      continue;
    }
    if (!profile.calibrated) {
      excluded.push({ id: record.id, reason: 'missing-calibration' });
      continue;
    }
    if (!record.valid) {
      excluded.push({ id: record.id, reason: 'invalid-flag' });
      continue;
    }
    if (statusRejectsSolver(status)) {
      excluded.push({ id: record.id, reason: 'filter-rejected' });
      continue;
    }
    if (!Number.isInteger(record.ageMs) || record.ageMs < 0 || record.ageMs > config.maxAgeMs) {
      excluded.push({ id: record.id, reason: 'stale' });
      continue;
    }

    const selectedMm = config.rangeSource === 'raw' ? record.rawMm : record.filtMm;
    if (typeof selectedMm !== 'number' || !Number.isFinite(selectedMm)
      || typeof record.rawMm !== 'number' || !Number.isFinite(record.rawMm)) {
      excluded.push({ id: record.id, reason: 'non-finite' });
      continue;
    }
    if (selectedMm <= 0 || record.rawMm <= 0) {
      excluded.push({ id: record.id, reason: 'non-positive' });
      continue;
    }
    const rangeM = selectedMm / 1000;
    if (rangeM > config.maxRangeM) {
      excluded.push({ id: record.id, reason: 'over-physical-limit' });
      continue;
    }
    const sigmaM = effectiveSigmaM(
      profileSigma(profile, config.rangeSource),
      record.ageMs,
      record.fppDbm,
      mode,
      config,
    );
    if (!Number.isFinite(sigmaM) || sigmaM < config.sigmaFloorM || sigmaM > config.sigmaCeilingM) {
      excluded.push({ id: record.id, reason: 'non-finite' });
      continue;
    }
    usable.push({
      id: record.id,
      x: anchor.x,
      y: anchor.y,
      rangeM,
      rawRangeM: record.rawMm / 1000,
      sigmaM,
      ageMs: record.ageMs,
      fppDbm: record.fppDbm,
      status,
      rangingMode: mode,
    });
  }

  usable.sort((left, right) => left.id - right.id);
  const ages = usable.map(observation => observation.ageMs);
  return {
    usable,
    excluded,
    ageSpreadMs: ages.length > 0 ? Math.max(...ages) - Math.min(...ages) : 0,
    fatal,
  };
}
