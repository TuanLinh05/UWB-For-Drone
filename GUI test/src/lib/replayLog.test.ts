import {
  PIPELINE_CONFIG_VERSION,
  anchorMeasurementTimeMs,
  buildReplayCsv,
  createReplayLogContext,
  firmwareProfileFingerprint,
} from './replayLog';
import type { FirmwareInfo, RangeSample } from './types';

function assert(condition: boolean, message: string): asserts condition {
  if (!condition) throw new Error(`Replay log self-test failed: ${message}`);
}

export function runReplayLogSelfTests() {
  const firmwareInfo: FirmwareInfo = {
    schemaVersion: 2,
    calibrationProfile: 'legacy',
    rangingMode: 'ss',
    activeOffsetsM: { 4: 0, 2: 156.1, 1: 156.7, 3: 155.9 },
    hardwareAntennaDelay: false,
    legacyOffsetEnabled: true,
    dsCalibratedMask: 0,
    rangeFilterMode: 0,
    legacyAdaptiveMode: 'off',
    c9_2MotionMode: 'off',
    c9_2GlobalMotionState: 'static',
    phyProfileId: 1,
    phyProfile: 'legacy-1024',
    spiClockMhz: 2,
    receivedAt: 123,
    firmwareBuildId: 'build,one',
  };
  const reorderedInfo: FirmwareInfo = {
    ...firmwareInfo,
    activeOffsetsM: { 3: 155.9, 1: 156.7, 4: 0, 2: 156.1 },
  };
  assert(
    firmwareProfileFingerprint(firmwareInfo) === firmwareProfileFingerprint(reorderedInfo),
    'fingerprint must be independent of object insertion order',
  );
  assert(
    firmwareProfileFingerprint(firmwareInfo)
      !== firmwareProfileFingerprint({ ...firmwareInfo, legacyAdaptiveMode: 'shadow' }),
    'Adaptive Legacy mode must split replay sessions rather than mix A/B builds',
  );
  assert(anchorMeasurementTimeMs(0x10, 0x20) === 0xfffffff0,
    'per-anchor measurement time must handle uint32 wrap');
  assert(anchorMeasurementTimeMs(100, 0xffff) === null,
    'never-measured age must not fabricate a timestamp');

  const valid = {
    id: 1, valid: true, ageMs: 5, rawMm: 1000, filtMm: 990, fppDbm: -72, status: 0,
  };
  const calibrationMissing = {
    id: 4,
    valid: false,
    ageMs: 8,
    rawMm: null,
    filtMm: null,
    fppDbm: null,
    status: 0x20,
    diagnosticRawMm: 157_000,
    diagnosticFppDbm: -78,
  };
  const sample: RangeSample = {
    seq: 7,
    transportSeq: 9,
    timeMs: 0x10,
    clientTime: Date.UTC(2026, 6, 26),
    anchors: [calibrationMissing, valid],
    anchorsById: { 1: valid, 4: calibrationMissing },
  };
  const context = createReplayLogContext({
    startedAtMs: 456,
    connectionMode: 'usb',
    firmwareInfo,
  });
  const csv = buildReplayCsv([sample], context);

  assert(csv.includes('Pipeline_Config_Version'), 'config version column must exist');
  assert(csv.includes('A4_measurement_time_ms'), 'per-anchor measurement timestamp must exist');
  assert(csv.includes('A4_calibration_missing'), 'calibration diagnostic flag must exist');
  assert(csv.includes('Legacy_Adaptive_Mode'), 'Adaptive Legacy build mode must be explicit in the log');
  assert(csv.includes(PIPELINE_CONFIG_VERSION), 'row must identify pipeline config');
  assert(csv.includes('"build,one"'), 'CSV values containing commas must be escaped');
  assert(csv.includes('157000'), 'uncalibrated diagnostic raw value must remain replayable');
}
