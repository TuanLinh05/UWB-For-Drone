import type {
  AnchorPosition,
  ConnectionMode,
  FirmwareInfo,
  RangeSample,
} from './types';

/**
 * Increment whenever replay/log semantics change. This is deliberately
 * independent from the telemetry wire schema and firmware range-filter mode.
 */
export const PIPELINE_CONFIG_VERSION = 'c5-observability-v2';
export const REPLAY_CSV_FORMAT_VERSION = 'uwb-replay-csv-v2';

const UINT32_MODULUS = 0x1_0000_0000;
const TAG_ST_DS_FALLBACK = 0x10;
const TAG_ST_CALIBRATION_MISSING = 0x20;

export interface ReplayLogContext {
  startedAtMs: number;
  connectionMode: ConnectionMode;
  firmwareInfo: FirmwareInfo | null;
  pipelineConfigVersion: string;
  profileFingerprint: string;
  anchorLayout: AnchorPosition[];
  anchorLayoutFingerprint: string;
  positionRangeSource: 'raw' | 'filtered';
}

function cloneFirmwareInfo(info: FirmwareInfo | null): FirmwareInfo | null {
  return info
    ? { ...info, activeOffsetsM: { ...info.activeOffsetsM } }
    : null;
}

function finiteOrUnknown(value: number): string {
  return Number.isFinite(value) ? String(value) : 'unknown';
}

export function anchorLayoutFingerprint(layout: readonly AnchorPosition[]): string {
  return [...layout]
    .filter(anchor => Number.isInteger(anchor.id) && Number.isFinite(anchor.x) && Number.isFinite(anchor.y))
    .sort((left, right) => left.id - right.id)
    .map(anchor => `A${anchor.id}:${anchor.x.toFixed(6)},${anchor.y.toFixed(6)}`)
    .join(';') || 'layout-unavailable';
}

/**
 * Deterministic identity of the active radio/calibration/filter configuration.
 * This is not a source-control build hash; an absent firmware build ID remains
 * explicit instead of being guessed from the browser session or filename.
 */
export function firmwareProfileFingerprint(info: FirmwareInfo | null): string {
  if (!info) return 'firmware-info-unavailable';

  const offsets = Object.entries(info.activeOffsetsM)
    .map(([id, offsetM]) => [Number(id), offsetM] as const)
    .filter(([id, offsetM]) => Number.isInteger(id) && Number.isFinite(offsetM))
    .sort(([left], [right]) => left - right)
    .map(([id, offsetM]) => `A${id}:${offsetM.toFixed(6)}`)
    .join(';');

  return [
    `schema:${finiteOrUnknown(info.schemaVersion)}`,
    `build:${info.firmwareBuildId ?? 'unknown'}`,
    `mode:${info.rangingMode}`,
    `phy:${info.phyProfile}`,
    `spi:${finiteOrUnknown(info.spiClockMhz)}`,
    `cal:${info.calibrationProfile}`,
    `ds-mask:${finiteOrUnknown(info.dsCalibratedMask)}`,
    `filter:${finiteOrUnknown(info.rangeFilterMode)}`,
    `legacy-adaptive:${info.legacyAdaptiveMode}`,
    `c9.2-motion:${info.c9_2MotionMode}`,
    `hw-ant:${info.hardwareAntennaDelay ? 1 : 0}`,
    `legacy-offset:${info.legacyOffsetEnabled ? 1 : 0}`,
    `offsets:${offsets || 'none'}`,
  ].join('|');
}

export function createReplayLogContext(input: {
  startedAtMs?: number;
  connectionMode: ConnectionMode;
  firmwareInfo: FirmwareInfo | null;
  anchorLayout?: readonly AnchorPosition[];
  positionRangeSource?: 'raw' | 'filtered';
}): ReplayLogContext {
  const firmwareInfo = cloneFirmwareInfo(input.firmwareInfo);
  const anchorLayout = (input.anchorLayout ?? []).map(anchor => ({ ...anchor }));
  return {
    startedAtMs: input.startedAtMs ?? Date.now(),
    connectionMode: input.connectionMode,
    firmwareInfo,
    pipelineConfigVersion: PIPELINE_CONFIG_VERSION,
    profileFingerprint: firmwareProfileFingerprint(firmwareInfo),
    anchorLayout,
    anchorLayoutFingerprint: anchorLayoutFingerprint(anchorLayout),
    positionRangeSource: input.positionRangeSource ?? 'filtered',
  };
}

export function anchorMeasurementTimeMs(sampleTimeMs: number, ageMs: number): number | null {
  if (!Number.isFinite(sampleTimeMs) || !Number.isFinite(ageMs)
    || ageMs < 0 || ageMs >= 0xffff) return null;
  const time = Math.trunc(sampleTimeMs);
  const age = Math.trunc(ageMs);
  return ((time - age) % UINT32_MODULUS + UINT32_MODULUS) % UINT32_MODULUS;
}

function rangeModeForRecord(info: FirmwareInfo | null, status: number | undefined): string {
  if (status !== undefined && (status & TAG_ST_DS_FALLBACK) !== 0) return 'SS_FALLBACK';
  if (info?.rangingMode === 'ds') return 'DS';
  if (info?.rangingMode === 'ss') return 'SS';
  return 'UNKNOWN';
}

function csvCell(value: unknown): string {
  if (value === null || value === undefined) return '';
  const text = String(value);
  return /[",\r\n]/.test(text) ? `"${text.replace(/"/g, '""')}"` : text;
}

/**
 * Versioned replay CSV. Existing A*_raw_mm/filter_mm column names are retained
 * for Filter Lab compatibility while the C5 metadata needed for deterministic
 * replay is added to every row.
 */
export function buildReplayCsv(
  log: readonly RangeSample[],
  context: ReplayLogContext,
): string {
  const anchorIds = Array.from(new Set(log.flatMap(sample =>
    sample.anchors.map(anchor => anchor.id))))
    .filter(Number.isFinite)
    .sort((left, right) => left - right);
  const info = context.firmwareInfo;

  const header = [
    'Replay_Format_Version',
    'Timestamp',
    'Time_ISO',
    'Log_Start_ms',
    'Pipeline_Config_Version',
    'Profile_Fingerprint',
    'Anchor_Layout_Fingerprint',
    'Anchor_Layout_JSON',
    'Position_Range_Source',
    'Telemetry_Schema_Version',
    'Firmware_Build_ID',
    'Connection_Mode',
    'Ranging_Mode',
    'Calibration_Profile',
    'Radio_Profile_ID',
    'PHY_Profile',
    'SPI_Clock_MHz',
    'DS_Calibrated_Mask',
    'Range_Filter_Mode',
    'Legacy_Adaptive_Mode',
    'HW_Antenna_Delay',
    'Legacy_Offset_Enabled',
    'SourceSeq',
    'TransportSeq',
    'MCU_Time_ms',
    'Cycle_Duration_us',
    ...anchorIds.flatMap(id => [
      `A${id}_valid`,
      `A${id}_age_ms`,
      `A${id}_measurement_time_ms`,
      `A${id}_range_mode`,
      `A${id}_raw_mm`,
      `A${id}_filter_mm`,
      `A${id}_fpp_dbm`,
      `A${id}_status`,
      `A${id}_calibration_missing`,
      `A${id}_diagnostic_raw_mm`,
      `A${id}_diagnostic_fpp_dbm`,
    ]),
  ];

  const rows = log.map(sample => [
    REPLAY_CSV_FORMAT_VERSION,
    sample.clientTime,
    new Date(sample.clientTime).toISOString(),
    context.startedAtMs,
    context.pipelineConfigVersion,
    context.profileFingerprint,
    context.anchorLayoutFingerprint,
    JSON.stringify(context.anchorLayout),
    context.positionRangeSource,
    info?.schemaVersion ?? '',
    info?.firmwareBuildId ?? '',
    context.connectionMode,
    info?.rangingMode ?? 'unknown',
    info?.calibrationProfile ?? 'unknown',
    info?.phyProfile ?? 'unknown',
    info?.phyProfile ?? 'unknown',
    info?.spiClockMhz ?? '',
    info?.dsCalibratedMask ?? '',
    info?.rangeFilterMode ?? '',
    info?.legacyAdaptiveMode ?? '',
    info ? (info.hardwareAntennaDelay ? 1 : 0) : '',
    info ? (info.legacyOffsetEnabled ? 1 : 0) : '',
    sample.seq,
    sample.transportSeq,
    sample.timeMs,
    '',
    ...anchorIds.flatMap(id => {
      const anchor = sample.anchorsById[id];
      if (!anchor) {
        return [0, 0xffff, '', 'UNKNOWN', '', '', '', '', 0, '', ''];
      }
      const status = anchor.status;
      return [
        anchor.valid ? 1 : 0,
        anchor.ageMs,
        anchorMeasurementTimeMs(sample.timeMs, anchor.ageMs) ?? '',
        rangeModeForRecord(info, status),
        anchor.rawMm ?? '',
        anchor.filtMm ?? '',
        anchor.fppDbm ?? '',
        status ?? '',
        status !== undefined && (status & TAG_ST_CALIBRATION_MISSING) !== 0 ? 1 : 0,
        anchor.diagnosticRawMm ?? '',
        anchor.diagnosticFppDbm ?? '',
      ];
    }),
  ]);

  return [header, ...rows]
    .map(row => row.map(csvCell).join(','))
    .join('\n');
}
