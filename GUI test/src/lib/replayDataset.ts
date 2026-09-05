export const LEGACY_REPLAY_CSV_FORMAT_VERSION = 'uwb-replay-csv-v1';

const UINT32_MAX = 0xffff_ffff;
const TAG_ST_CALIBRATION_MISSING = 0x20;
const MAX_PARSE_ERRORS = 100;

export type ReplayRangingMode = 'SS' | 'DS' | 'SS_FALLBACK' | 'UNKNOWN';

export interface ReplayAnchorRecord {
  id: number;
  valid: boolean;
  status: number;
  ageMs: number;
  measurementTimeMs: number | null;
  correctedRawMm: number | null;
  filteredMm: number | null;
  fppDbm: number | null;
  rangingMode: ReplayRangingMode;
  calibrationMissing: boolean;
  diagnosticRawMm: number | null;
  diagnosticFppDbm: number | null;
}

export interface ReplayRangeFrame {
  replayFormatVersion: string;
  telemetrySchemaVersion: number | null;
  pipelineConfigVersion: string;
  profileFingerprint: string;
  calibrationProfileId: string;
  radioProfileId: string;
  rangeFilterMode: string;
  connectionMode: string;
  anchorLayoutJson: string;
  anchorLayoutFingerprint: string;
  positionRangeSource: 'raw' | 'filtered' | 'unknown';
  seq: number;
  transportSeq: number;
  timeMs: number;
  clientTimeMs: number;
  cycleDurationUs?: number;
  records: ReplayAnchorRecord[];
}

export interface ReplayDatasetMetadata {
  replayFormatVersion: string;
  telemetrySchemaVersion: number | null;
  pipelineConfigVersion: string;
  profileFingerprint: string;
  calibrationProfileId: string;
  radioProfileId: string;
  rangeFilterMode: string;
  anchorLayout: { id: number; x: number; y: number }[];
  anchorLayoutFingerprint: string;
  positionRangeSource: 'raw' | 'filtered' | 'unknown';
}

export interface ReplayDataset {
  metadata: ReplayDatasetMetadata;
  anchorIds: number[];
  frames: ReplayRangeFrame[];
  warnings: string[];
}

export type ReplayCsvParseResult =
  | { ok: true; dataset: ReplayDataset }
  | { ok: false; errors: string[] };

const ANCHOR_REQUIRED_SUFFIXES = [
  'valid',
  'age_ms',
  'measurement_time_ms',
  'range_mode',
  'raw_mm',
  'filter_mm',
  'fpp_dbm',
  'status',
  'calibration_missing',
  'diagnostic_raw_mm',
  'diagnostic_fpp_dbm',
] as const;

function parseCsvRows(text: string): { rows: string[][]; error?: string } {
  const rows: string[][] = [];
  let row: string[] = [];
  let field = '';
  let quoted = false;

  for (let index = 0; index < text.length; index++) {
    const char = text[index];
    if (quoted) {
      if (char === '"') {
        if (text[index + 1] === '"') {
          field += '"';
          index++;
        } else {
          quoted = false;
        }
      } else {
        field += char;
      }
      continue;
    }

    if (char === '"' && field.length === 0) {
      quoted = true;
    } else if (char === ',') {
      row.push(field);
      field = '';
    } else if (char === '\n') {
      row.push(field);
      rows.push(row);
      row = [];
      field = '';
    } else if (char !== '\r') {
      field += char;
    }
  }

  if (quoted) return { rows, error: 'CSV contains an unterminated quoted field.' };
  if (field.length > 0 || row.length > 0) {
    row.push(field);
    rows.push(row);
  }
  return { rows };
}

function isBlankRow(row: readonly string[]): boolean {
  return row.every(cell => cell.trim() === '');
}

function parseFinite(
  value: string,
  label: string,
  errors: string[],
  options: { optional?: boolean; integer?: boolean; min?: number; max?: number } = {},
): number | null {
  const trimmed = value.trim();
  if (trimmed === '') {
    if (!options.optional) errors.push(`${label} is required.`);
    return null;
  }
  const parsed = Number(trimmed);
  if (!Number.isFinite(parsed)) {
    errors.push(`${label} must be finite.`);
    return null;
  }
  if (options.integer && !Number.isInteger(parsed)) errors.push(`${label} must be an integer.`);
  if (options.min !== undefined && parsed < options.min) errors.push(`${label} is below ${options.min}.`);
  if (options.max !== undefined && parsed > options.max) errors.push(`${label} is above ${options.max}.`);
  return parsed;
}

function parseBoolean(value: string, label: string, errors: string[], optional = false): boolean | null {
  const normalized = value.trim().toLowerCase();
  if (normalized === '') {
    if (!optional) errors.push(`${label} is required.`);
    return null;
  }
  if (normalized === '1' || normalized === 'true') return true;
  if (normalized === '0' || normalized === 'false') return false;
  errors.push(`${label} must be 0/1 or true/false.`);
  return null;
}

function uint32Subtract(timeMs: number, ageMs: number): number {
  return ((timeMs - ageMs) % 0x1_0000_0000 + 0x1_0000_0000) % 0x1_0000_0000;
}

function consistentMetadata(frames: readonly ReplayRangeFrame[], errors: string[]): ReplayDatasetMetadata | null {
  const first = frames[0];
  if (!first) return null;
  const metadata: ReplayDatasetMetadata = {
    replayFormatVersion: first.replayFormatVersion,
    telemetrySchemaVersion: first.telemetrySchemaVersion,
    pipelineConfigVersion: first.pipelineConfigVersion,
    profileFingerprint: first.profileFingerprint,
    calibrationProfileId: first.calibrationProfileId,
    radioProfileId: first.radioProfileId,
    rangeFilterMode: first.rangeFilterMode,
    anchorLayout: [],
    anchorLayoutFingerprint: '',
    positionRangeSource: 'unknown',
  };
  const keys: Array<
    'replayFormatVersion' | 'telemetrySchemaVersion' | 'pipelineConfigVersion'
    | 'profileFingerprint' | 'calibrationProfileId' | 'radioProfileId' | 'rangeFilterMode'
  > = [
    'replayFormatVersion', 'telemetrySchemaVersion', 'pipelineConfigVersion',
    'profileFingerprint', 'calibrationProfileId', 'radioProfileId', 'rangeFilterMode',
  ];
  frames.forEach((frame, index) => {
    for (const key of keys) {
      if (frame[key] !== metadata[key]) {
        errors.push(`Row ${index + 2} changes session metadata ${key}; split the log at the config change.`);
      }
    }
    if (frame.anchorLayoutFingerprint !== first.anchorLayoutFingerprint
      || frame.anchorLayoutJson !== first.anchorLayoutJson
      || frame.positionRangeSource !== first.positionRangeSource) {
      errors.push(`Row ${index + 2} changes anchor layout/range-source metadata; split the log at the config change.`);
    }
  });
  const layoutJson = frames[0] ?? null;
  if (layoutJson) {
    metadata.anchorLayoutFingerprint = layoutJson.anchorLayoutFingerprint ?? '';
    metadata.positionRangeSource = layoutJson.positionRangeSource ?? 'unknown';
    if (layoutJson.anchorLayoutJson) {
      try {
        const candidate: unknown = JSON.parse(layoutJson.anchorLayoutJson);
        if (!Array.isArray(candidate)) throw new Error('layout is not an array');
        const seen = new Set<number>();
        metadata.anchorLayout = candidate.map((entry, index) => {
          if (typeof entry !== 'object' || entry === null) throw new Error(`layout[${index}] is not an object`);
          const record = entry as Record<string, unknown>;
          if (!Number.isInteger(record.id)
            || typeof record.x !== 'number' || !Number.isFinite(record.x)
            || typeof record.y !== 'number' || !Number.isFinite(record.y)) {
            throw new Error(`layout[${index}] has invalid id/x/y`);
          }
          const id = record.id as number;
          if (seen.has(id)) throw new Error(`layout contains duplicate anchor ID ${id}`);
          seen.add(id);
          return { id, x: record.x as number, y: record.y as number };
        });
      } catch (error) {
        errors.push(`Anchor_Layout_JSON is invalid: ${error instanceof Error ? error.message : String(error)}.`);
      }
    }
  }
  return metadata;
}

/**
 * Parse the versioned wide CSV emitted by buildReplayCsv. Parsing is fail-closed:
 * malformed numeric fields, duplicate headers, incomplete anchor records or a
 * mid-session profile change return explicit errors instead of partial replay.
 */
export function parseReplayCsv(text: string): ReplayCsvParseResult {
  const parsedCsv = parseCsvRows(text.replace(/^\uFEFF/, ''));
  if (parsedCsv.error) return { ok: false, errors: [parsedCsv.error] };
  const rows = parsedCsv.rows.filter(row => !isBlankRow(row));
  if (rows.length < 2) return { ok: false, errors: ['Replay CSV must contain a header and at least one data row.'] };

  const header = rows[0].map(cell => cell.trim());
  const headerIndex = new Map<string, number>();
  const errors: string[] = [];
  header.forEach((name, index) => {
    if (!name) errors.push(`Header column ${index + 1} is empty.`);
    else if (headerIndex.has(name)) errors.push(`Duplicate CSV header: ${name}.`);
    else headerIndex.set(name, index);
  });

  const requiredGlobal = [
    'Timestamp', 'Pipeline_Config_Version', 'Profile_Fingerprint',
    'SourceSeq', 'TransportSeq', 'MCU_Time_ms',
  ];
  requiredGlobal.forEach(name => {
    if (!headerIndex.has(name)) errors.push(`Missing required CSV header: ${name}.`);
  });

  const anchorIds = Array.from(new Set(header.flatMap(name => {
    const match = /^A(\d+)_valid$/.exec(name);
    return match ? [Number(match[1])] : [];
  }))).sort((left, right) => left - right);
  if (anchorIds.length === 0) errors.push('Replay CSV contains no A*_valid anchor columns.');
  for (const id of anchorIds) {
    for (const suffix of ANCHOR_REQUIRED_SUFFIXES) {
      const name = `A${id}_${suffix}`;
      if (!headerIndex.has(name)) errors.push(`Anchor A${id} is missing CSV header ${name}.`);
    }
  }
  if (errors.length > 0) return { ok: false, errors: errors.slice(0, MAX_PARSE_ERRORS) };

  const frames: ReplayRangeFrame[] = [];
  const warnings = new Set<string>();
  const cell = (row: readonly string[], name: string) => row[headerIndex.get(name) as number] ?? '';

  for (let rowIndex = 1; rowIndex < rows.length; rowIndex++) {
    const row = rows[rowIndex];
    const rowErrors: string[] = [];
    if (row.length !== header.length) {
      rowErrors.push(`Row ${rowIndex + 1} has ${row.length} columns; expected ${header.length}.`);
    }
    const label = (name: string) => `Row ${rowIndex + 1} ${name}`;
    const clientTimeMs = parseFinite(cell(row, 'Timestamp'), label('Timestamp'), rowErrors, { min: 0 });
    const seq = parseFinite(cell(row, 'SourceSeq'), label('SourceSeq'), rowErrors,
      { integer: true, min: 0, max: UINT32_MAX });
    const transportSeq = parseFinite(cell(row, 'TransportSeq'), label('TransportSeq'), rowErrors,
      { integer: true, min: 0, max: UINT32_MAX });
    const timeMs = parseFinite(cell(row, 'MCU_Time_ms'), label('MCU_Time_ms'), rowErrors,
      { integer: true, min: 0, max: UINT32_MAX });
    const schemaVersion = parseFinite(cell(row, 'Telemetry_Schema_Version'), label('Telemetry_Schema_Version'), rowErrors,
      { optional: true, integer: true, min: 0 });
    const cycleDurationUs = parseFinite(cell(row, 'Cycle_Duration_us'), label('Cycle_Duration_us'), rowErrors,
      { optional: true, integer: true, min: 0 });

    const replayFormatVersion = cell(row, 'Replay_Format_Version').trim() || LEGACY_REPLAY_CSV_FORMAT_VERSION;
    if (!headerIndex.has('Replay_Format_Version')) warnings.add('Legacy replay CSV has no Replay_Format_Version column.');
    const pipelineConfigVersion = cell(row, 'Pipeline_Config_Version').trim();
    const profileFingerprint = cell(row, 'Profile_Fingerprint').trim();
    const calibrationProfileId = cell(row, 'Calibration_Profile').trim() || 'unknown';
    const radioProfileId = cell(row, 'Radio_Profile_ID').trim()
      || cell(row, 'PHY_Profile').trim()
      || 'unknown';
    const rangeFilterMode = cell(row, 'Range_Filter_Mode').trim() || 'unknown';
    const connectionMode = cell(row, 'Connection_Mode').trim() || 'unknown';
    const anchorLayoutJson = cell(row, 'Anchor_Layout_JSON').trim();
    const anchorLayoutFingerprint = cell(row, 'Anchor_Layout_Fingerprint').trim();
    const rangeSourceCell = cell(row, 'Position_Range_Source').trim().toLowerCase();
    const positionRangeSource = rangeSourceCell === 'raw' || rangeSourceCell === 'filtered'
      ? rangeSourceCell
      : 'unknown';
    if (!anchorLayoutJson) warnings.add('Anchor layout is unavailable; solver variants cannot be replayed deterministically.');
    if (!anchorLayoutFingerprint) warnings.add('Anchor layout fingerprint is unavailable.');
    if (!pipelineConfigVersion) rowErrors.push(`${label('Pipeline_Config_Version')} is required.`);
    if (!profileFingerprint) rowErrors.push(`${label('Profile_Fingerprint')} is required.`);
    if (radioProfileId === 'unknown') warnings.add('Radio profile is unknown; do not use this dataset for acceptance.');

    const records: ReplayAnchorRecord[] = [];
    for (const id of anchorIds) {
      const prefix = `A${id}_`;
      const valid = parseBoolean(cell(row, `${prefix}valid`), label(`${prefix}valid`), rowErrors);
      const ageMs = parseFinite(cell(row, `${prefix}age_ms`), label(`${prefix}age_ms`), rowErrors,
        { integer: true, min: 0, max: 0xffff });
      const explicitMeasurementTime = parseFinite(
        cell(row, `${prefix}measurement_time_ms`), label(`${prefix}measurement_time_ms`), rowErrors,
        { optional: true, integer: true, min: 0, max: UINT32_MAX },
      );
      const correctedRawMm = parseFinite(cell(row, `${prefix}raw_mm`), label(`${prefix}raw_mm`), rowErrors,
        { optional: true });
      const filteredMm = parseFinite(cell(row, `${prefix}filter_mm`), label(`${prefix}filter_mm`), rowErrors,
        { optional: true });
      const fppDbm = parseFinite(cell(row, `${prefix}fpp_dbm`), label(`${prefix}fpp_dbm`), rowErrors,
        { optional: true });
      const status = parseFinite(cell(row, `${prefix}status`), label(`${prefix}status`), rowErrors,
        { optional: true, integer: true, min: 0, max: 0xff });
      const calibrationMissingCell = parseBoolean(
        cell(row, `${prefix}calibration_missing`), label(`${prefix}calibration_missing`), rowErrors, true,
      );
      const diagnosticRawMm = parseFinite(
        cell(row, `${prefix}diagnostic_raw_mm`), label(`${prefix}diagnostic_raw_mm`), rowErrors, { optional: true },
      );
      const diagnosticFppDbm = parseFinite(
        cell(row, `${prefix}diagnostic_fpp_dbm`), label(`${prefix}diagnostic_fpp_dbm`), rowErrors, { optional: true },
      );
      const modeCell = cell(row, `${prefix}range_mode`).trim().toUpperCase();
      const rangingMode: ReplayRangingMode = modeCell === 'SS' || modeCell === 'DS'
        || modeCell === 'SS_FALLBACK' || modeCell === 'UNKNOWN'
        ? modeCell
        : 'UNKNOWN';
      if (modeCell && rangingMode === 'UNKNOWN' && modeCell !== 'UNKNOWN') {
        rowErrors.push(`${label(`${prefix}range_mode`)} has unsupported mode ${modeCell}.`);
      }
      if (valid === true && (correctedRawMm === null || filteredMm === null)) {
        rowErrors.push(`${label(prefix)}valid record must contain raw_mm and filter_mm.`);
      }
      const resolvedStatus = status ?? 0;
      const calibrationMissing = calibrationMissingCell
        ?? ((resolvedStatus & TAG_ST_CALIBRATION_MISSING) !== 0);
      const measurementTimeMs = explicitMeasurementTime
        ?? (timeMs !== null && ageMs !== null && ageMs < 0xffff ? uint32Subtract(timeMs, ageMs) : null);
      records.push({
        id,
        valid: valid ?? false,
        status: resolvedStatus,
        ageMs: ageMs ?? 0xffff,
        measurementTimeMs,
        correctedRawMm,
        filteredMm,
        fppDbm,
        rangingMode,
        calibrationMissing,
        diagnosticRawMm,
        diagnosticFppDbm,
      });
    }

    if (rowErrors.length > 0) {
      errors.push(...rowErrors);
      if (errors.length >= MAX_PARSE_ERRORS) break;
      continue;
    }
    frames.push({
      replayFormatVersion,
      telemetrySchemaVersion: schemaVersion,
      pipelineConfigVersion,
      profileFingerprint,
      calibrationProfileId,
      radioProfileId,
      rangeFilterMode,
      connectionMode,
      seq: seq as number,
      transportSeq: transportSeq as number,
      timeMs: timeMs as number,
      clientTimeMs: clientTimeMs as number,
      ...(cycleDurationUs === null ? {} : { cycleDurationUs }),
      records,
      anchorLayoutJson,
      anchorLayoutFingerprint,
      positionRangeSource,
    });
  }

  if (errors.length > 0) return { ok: false, errors: errors.slice(0, MAX_PARSE_ERRORS) };
  const metadata = consistentMetadata(frames, errors);
  if (!metadata || errors.length > 0) return { ok: false, errors: errors.slice(0, MAX_PARSE_ERRORS) };
  return {
    ok: true,
    dataset: {
      metadata,
      anchorIds,
      frames,
      warnings: [...warnings],
    },
  };
}
