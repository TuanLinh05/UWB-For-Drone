export interface AnchorSample {
  id: number;
  valid: boolean;
  ageMs: number;        // 65535 = never measured
  /** Canonical values are null whenever valid=false. */
  rawMm: number | null;
  filtMm: number | null;
  fppDbm: number | null;
  /** Invalid raw evidence: calibration-missing (0x20) or C9 filter reject (0x40). */
  diagnosticRawMm?: number;
  diagnosticFppDbm?: number;
  status?: number;       // error bitmask — WiFi only, undefined on USB
}

export interface RangeSample {
  seq: number;
  /** Transport sequence: source seq on USB, ws_seq on WiFi. */
  transportSeq: number;
  timeMs: number;        // MCU clock (ms)
  clientTime: number;    // Date.now() when GUI received — for chart & CSV
  anchors: AnchorSample[];
  /** Canonical identity boundary. Consumers must join data by anchor ID. */
  anchorsById: Readonly<Record<number, AnchorSample>>;
}

export interface StatsSample {
  pollSent: number;
  responseOk: number;
  rxTimeout: number;
  rxError: number;
  cycleOverrun: number;
  uartOverflow: number;
  cycHz: number;
  opsHz: number;
  uartGapCount?: number;   // WiFi only
  crcErrorCount?: number;  // WiFi only
  /** Actual bridge range-frame emission rate, measured by ESP32 (WiFi only). */
  wifiRangeHz?: number;
  /** Range broadcasts skipped by the ESP32 low-heap fail-safe (WiFi only). */
  wifiRangeDropCount?: number;
  /** Configured ESP32 WebSocket range interval (WiFi only). */
  wifiRangeIntervalMs?: number;
}

export interface LinkHealth {
  lastSeq: number | null;
  rangeGapCount: number;
}

export type ConnectionMode = 'usb' | 'wifi' | 'disconnected';
export type TelemetryState = 'disconnected' | 'waiting' | 'fresh' | 'stale';
export type LegacyAdaptiveMode = 'off' | 'shadow' | 'active' | 'unknown';
export type C9_2MotionMode = 'off' | 'shadow' | 'active' | 'unknown';
export type C9_2GlobalMotionState = 'reacquire' | 'static' | 'slow' | 'fast' | 'settling' | 'degraded' | 'unknown';

export interface FirmwareInfo {
  schemaVersion: number;
  calibrationProfile: 'legacy' | 'residual-hw' | 'ds' | 'unknown';
  rangingMode: 'ss' | 'ds' | 'unknown';
  activeOffsetsM: Record<number, number>;
  hardwareAntennaDelay: boolean;
  legacyOffsetEnabled: boolean;
  dsCalibratedMask: number;
  rangeFilterMode: number;
  /** Compile-time C9.1 mode from INFO flags bits 3..4. */
  legacyAdaptiveMode: LegacyAdaptiveMode;
  /** Schema-2 INFO: compile-time C9.2 controller mode. */
  c9_2MotionMode: C9_2MotionMode;
  /** Schema-2 INFO: aggregate diagnostic state; filtering remains per-anchor. */
  c9_2GlobalMotionState: C9_2GlobalMotionState;
  phyProfileId: number;
  phyProfile: 'legacy-1024' | 'fast-256' | 'unknown';
  spiClockMhz: number;
  receivedAt: number;
  firmwareBuildId?: string;
}

export interface ChartPoint {
  time: number;
  cyc: number; ops: number;
  a1Raw: number | null; a1Filter: number | null;
  a2Raw: number | null; a2Filter: number | null;
  a3Raw: number | null; a3Filter: number | null;
  a4Raw: number | null; a4Filter: number | null;
}

export interface AnchorPosition { id: number; x: number; y: number; }

/** Anchors exposed by every GUI workflow, even before telemetry is received. */
export const SUPPORTED_ANCHOR_IDS = [1, 2, 3, 4] as const;

export const DEFAULT_ANCHOR_LAYOUT: AnchorPosition[] = [
  { id: 1, x: 0, y: 0 },
  { id: 2, x: 5, y: 0 },
  { id: 3, x: 2.5, y: 4 },
  { id: 4, x: 5, y: 4 },
];
