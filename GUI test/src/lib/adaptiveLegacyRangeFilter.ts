/**
 * Deterministic replay model of the optional Adaptive Legacy firmware path.
 *
 * It is intentionally independent from scalarRangeFilter.ts: that component
 * is a visual tuning aid with a fixed 20 ms assumption, while this model uses
 * per-anchor MCU measurement time, stale handling and the firmware candidate
 * state machine. Keep its numeric seeds aligned with uwb_calibration.h.
 */

export type AdaptiveLegacyMode = 'off' | 'active';
export type AdaptiveLegacyState = 'stable' | 'candidate' | 'tracking' | 'stale-reacquire';
export type AdaptiveLegacyDecision =
  | 'accepted'
  | 'candidate-rejected'
  | 'tracking'
  | 'tracking-rejected'
  | 'stale-rejected'
  | 'stale-reacquired';

export interface AdaptiveLegacyConfig {
  baseProcessNoise: number;
  motionEnterMm: number;
  settleResidualMm: number;
  candidateClusterMm: number;
  maxRadialSpeedMmS: number;
  gateMarginUpMm: number;
  gateMarginDownMm: number;
  reacquireMinFppDbm: number;
  motionConfirmSamples: number;
  staleReacquireSamples: number;
  settleSamples: number;
  staleResetMs: number;
  candidateMaxGapMs: number;
  nominalSampleMs: number;
  minTrackingDtMs: number;
  maxTrackingDtMs: number;
  trackingGain: number;
}

/** Mirrors the ACTIVE seed configuration in uwb_calibration.h. */
export const DEFAULT_ADAPTIVE_LEGACY_CONFIG: Readonly<AdaptiveLegacyConfig> = {
  baseProcessNoise: 0.05,
  motionEnterMm: 250,
  settleResidualMm: 120,
  candidateClusterMm: 250,
  maxRadialSpeedMmS: 10_000,
  gateMarginUpMm: 100,
  gateMarginDownMm: 250,
  reacquireMinFppDbm: -105,
  motionConfirmSamples: 4,
  staleReacquireSamples: 3,
  settleSamples: 5,
  staleResetMs: 500,
  candidateMaxGapMs: 100,
  nominalSampleMs: 20,
  minTrackingDtMs: 10,
  maxTrackingDtMs: 100,
  trackingGain: 0.15,
};

export interface AdaptiveLegacyInput {
  rawMm: number;
  fppDbm: number;
  /** MCU timestamp for this anchor's actual measurement, unsigned 32-bit. */
  nowMs: number;
}

export interface AdaptiveLegacyOutput {
  publishValid: boolean;
  filteredMm?: number;
  decision: AdaptiveLegacyDecision;
  state: AdaptiveLegacyState;
  innovationMm: number;
  gain: number;
  trackEnterCount: number;
  trackExitCount: number;
  trueRejectCount: number;
  staleReacquireCount: number;
}

interface MedianState {
  values: [number, number, number];
  head: number;
  count: number;
}

interface KalmanState {
  initialized: boolean;
  q: number;
  r: number;
  x: number;
  p: number;
  outlierCount: number;
  lastMeasMs: number;
}

interface CandidateState {
  count: number;
  direction: -1 | 0 | 1;
  meanMm: number;
  lastMs: number;
}

interface TrackingState {
  settleCount: number;
  hasLastMeasurement: boolean;
  lastMeasurementMm: number;
  lastMeasurementMs: number;
  hasUpdate: boolean;
  lastUpdateMs: number;
  startedMs: number;
}

function uint32(value: number): number {
  return value >>> 0;
}

function elapsedMs(nowMs: number, thenMs: number): number {
  return (uint32(nowMs) - uint32(thenMs)) >>> 0;
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(maximum, Math.max(minimum, value));
}

function sign(value: number): -1 | 0 | 1 {
  return value > 0 ? 1 : value < 0 ? -1 : 0;
}

function medianState(): MedianState {
  return { values: [0, 0, 0], head: 0, count: 0 };
}

function pushMedian(state: MedianState, value: number): number {
  state.values[state.head] = value;
  state.head = (state.head + 1) % 3;
  state.count = Math.min(3, state.count + 1);
  if (state.count < 3) return value;
  const sorted = [...state.values].sort((left, right) => left - right);
  return sorted[1];
}

function resetMedian(state: MedianState): void {
  state.values = [0, 0, 0];
  state.head = 0;
  state.count = 0;
}

function kalmanState(): KalmanState {
  return {
    initialized: false,
    q: DEFAULT_ADAPTIVE_LEGACY_CONFIG.baseProcessNoise,
    r: 0,
    x: 0,
    p: 0,
    outlierCount: 0,
    lastMeasMs: 0,
  };
}

function candidateState(): CandidateState {
  return { count: 0, direction: 0, meanMm: 0, lastMs: 0 };
}

function trackingState(): TrackingState {
  return {
    settleCount: 0,
    hasLastMeasurement: false,
    lastMeasurementMm: 0,
    lastMeasurementMs: 0,
    hasUpdate: false,
    lastUpdateMs: 0,
    startedMs: 0,
  };
}

function measurementNoise(fppDbm: number): number {
  if (fppDbm <= -95) return 10_000;
  if (fppDbm > -75) return 50;
  if (fppDbm > -82) return 200;
  return 1_000;
}

function stablePosteriorCovariance(config: AdaptiveLegacyConfig, r: number): number {
  const q = config.baseProcessNoise;
  return (Math.sqrt(q * q + 4 * q * r) - q) / 2;
}

function trackingProcessNoise(config: AdaptiveLegacyConfig, r: number, dtMs: number): number {
  const k = config.trackingGain;
  const nominalQ = r * k * k / (1 - k);
  return nominalQ * (dtMs / config.nominalSampleMs);
}

function stateLabel(state: AdaptiveLegacyState): AdaptiveLegacyState {
  return state;
}

/**
 * One stateful filter belongs to exactly one physical Anchor ID. The replay
 * runner owns the ID->instance map; this class intentionally has no index or
 * record ordering dependency.
 */
export class AdaptiveLegacyRangeFilter {
  private readonly config: AdaptiveLegacyConfig;
  private readonly mode: AdaptiveLegacyMode;
  private median = medianState();
  private kf = kalmanState();
  private candidate = candidateState();
  private tracking = trackingState();
  private state: AdaptiveLegacyState = 'stable';
  private hasLastSample = false;
  private lastSampleMs = 0;
  private trackEnterCount = 0;
  private trackExitCount = 0;
  private trueRejectCount = 0;
  private staleReacquireCount = 0;

  constructor(mode: AdaptiveLegacyMode = 'active', config: Partial<AdaptiveLegacyConfig> = {}) {
    this.mode = mode;
    this.config = { ...DEFAULT_ADAPTIVE_LEGACY_CONFIG, ...config };
    if (!(this.config.trackingGain > 0 && this.config.trackingGain < 1)) {
      throw new Error('Adaptive Legacy trackingGain must be in (0, 1).');
    }
  }

  reset(): void {
    this.median = medianState();
    this.kf = kalmanState();
    this.candidate = candidateState();
    this.tracking = trackingState();
    this.state = 'stable';
    this.hasLastSample = false;
    this.lastSampleMs = 0;
    this.trackEnterCount = 0;
    this.trackExitCount = 0;
    this.trueRejectCount = 0;
    this.staleReacquireCount = 0;
  }

  process(input: AdaptiveLegacyInput): AdaptiveLegacyOutput {
    if (!Number.isFinite(input.rawMm) || !Number.isFinite(input.fppDbm) || !Number.isFinite(input.nowMs)) {
      return this.output(false, 'candidate-rejected', 0, 0);
    }
    return this.mode === 'off' ? this.processLegacy(input) : this.processActive(input);
  }

  private output(
    publishValid: boolean,
    decision: AdaptiveLegacyDecision,
    innovationMm: number,
    gain: number,
    filteredMm?: number,
  ): AdaptiveLegacyOutput {
    return {
      publishValid,
      ...(filteredMm === undefined ? {} : { filteredMm }),
      decision,
      state: stateLabel(this.state),
      innovationMm,
      gain,
      trackEnterCount: this.trackEnterCount,
      trackExitCount: this.trackExitCount,
      trueRejectCount: this.trueRejectCount,
      staleReacquireCount: this.staleReacquireCount,
    };
  }

  private initializeKalman(measurementMm: number, nowMs: number): void {
    this.kf.initialized = true;
    this.kf.q = this.config.baseProcessNoise;
    this.kf.r = 0;
    this.kf.x = measurementMm;
    this.kf.p = 1;
    this.kf.outlierCount = 0;
    this.kf.lastMeasMs = uint32(nowMs);
  }

  private kalmanUpdate(measurementMm: number, fppDbm: number): { filteredMm: number; gain: number } {
    this.kf.r = measurementNoise(fppDbm);
    this.kf.p += this.kf.q;
    const gain = this.kf.p / (this.kf.p + this.kf.r);
    this.kf.x += gain * (measurementMm - this.kf.x);
    this.kf.p = (1 - gain) * this.kf.p;
    return { filteredMm: Math.trunc(this.kf.x), gain };
  }

  private legacyGate(measurementMm: number, nowMs: number, allowSnap: boolean): boolean {
    const dtMs = clamp(elapsedMs(nowMs, this.kf.lastMeasMs), 20, 500);
    const up = this.config.maxRadialSpeedMmS * dtMs / 1000 + this.config.gateMarginUpMm;
    const down = -(this.config.maxRadialSpeedMmS * dtMs / 1000 + this.config.gateMarginDownMm);
    const jump = measurementMm - this.kf.x;
    if (jump > up || jump < down) {
      this.kf.outlierCount = Math.min(0xff, this.kf.outlierCount + 1);
      if (!allowSnap || this.kf.outlierCount < 15) return true;
      this.kf.x = measurementMm;
      this.kf.outlierCount = 0;
      this.kf.lastMeasMs = uint32(nowMs);
      return false;
    }
    this.kf.outlierCount = 0;
    this.kf.lastMeasMs = uint32(nowMs);
    return false;
  }

  private processLegacy(input: AdaptiveLegacyInput): AdaptiveLegacyOutput {
    const medianMm = pushMedian(this.median, input.rawMm);
    if (!this.kf.initialized) this.initializeKalman(medianMm, input.nowMs);
    const gateRejected = this.legacyGate(medianMm, input.nowMs, true);
    const updated = this.kalmanUpdate(medianMm, gateRejected ? -100 : input.fppDbm);
    return this.output(true, 'accepted', medianMm - this.kf.x, updated.gain, updated.filteredMm);
  }

  private beginSample(nowMs: number): boolean {
    const stale = this.hasLastSample
      && elapsedMs(nowMs, this.lastSampleMs) > this.config.staleResetMs;
    this.hasLastSample = true;
    this.lastSampleMs = uint32(nowMs);
    if (!stale) return false;

    this.state = 'stale-reacquire';
    this.resetCandidate();
    this.tracking = trackingState();
    this.staleReacquireCount++;
    resetMedian(this.median);
    return true;
  }

  private resetCandidate(): void {
    this.candidate = candidateState();
  }

  private candidateCompatible(measurementMm: number, direction: -1 | 0 | 1, nowMs: number,
                              requireDirection: boolean): boolean {
    if (this.candidate.count === 0) return false;
    if (requireDirection && direction !== this.candidate.direction) return false;
    const dtMs = elapsedMs(nowMs, this.candidate.lastMs);
    if (dtMs > this.config.candidateMaxGapMs) return false;
    const allowed = this.config.candidateClusterMm + this.config.maxRadialSpeedMmS * dtMs / 1000;
    return Math.abs(measurementMm - this.candidate.meanMm) <= allowed;
  }

  private startCandidate(measurementMm: number, direction: -1 | 0 | 1, nowMs: number): void {
    this.candidate = {
      count: 1,
      direction,
      meanMm: measurementMm,
      lastMs: uint32(nowMs),
    };
  }

  private addCandidate(measurementMm: number, nowMs: number): void {
    this.candidate.count++;
    this.candidate.meanMm += (measurementMm - this.candidate.meanMm) / this.candidate.count;
    this.candidate.lastMs = uint32(nowMs);
  }

  private observeMotionCandidate(measurementMm: number, estimateMm: number, nowMs: number): boolean {
    const innovationMm = measurementMm - estimateMm;
    const direction = sign(innovationMm);
    if (direction === 0 || Math.abs(innovationMm) < this.config.motionEnterMm) {
      if (this.state === 'candidate') this.state = 'stable';
      this.resetCandidate();
      return false;
    }
    if (!this.candidateCompatible(measurementMm, direction, nowMs, true)) {
      this.startCandidate(measurementMm, direction, nowMs);
      this.state = 'candidate';
      return false;
    }
    this.addCandidate(measurementMm, nowMs);
    this.state = 'candidate';
    if (this.candidate.count < this.config.motionConfirmSamples) return false;

    this.state = 'tracking';
    this.tracking = {
      ...trackingState(),
      startedMs: uint32(nowMs),
      lastUpdateMs: uint32(nowMs),
    };
    this.trackEnterCount++;
    this.resetCandidate();
    return true;
  }

  private observeStaleReacquire(measurementMm: number, fppDbm: number, nowMs: number): number | null {
    if (fppDbm < this.config.reacquireMinFppDbm) {
      this.resetCandidate();
      return null;
    }
    if (!this.candidateCompatible(measurementMm, 0, nowMs, false)) {
      this.startCandidate(measurementMm, 0, nowMs);
    } else {
      this.addCandidate(measurementMm, nowMs);
    }
    if (this.candidate.count < this.config.staleReacquireSamples) return null;
    const reacquiredMm = this.candidate.meanMm;
    this.state = 'stable';
    this.resetCandidate();
    return reacquiredMm;
  }

  private trackingDtMs(nowMs: number): number {
    if (!this.tracking.hasUpdate) return this.config.nominalSampleMs;
    return clamp(elapsedMs(nowMs, this.tracking.lastUpdateMs),
      this.config.minTrackingDtMs, this.config.maxTrackingDtMs);
  }

  private acceptTrackingMeasurement(measurementMm: number, nowMs: number): boolean {
    if (!this.tracking.hasLastMeasurement) {
      this.tracking.hasLastMeasurement = true;
      this.tracking.lastMeasurementMm = measurementMm;
      this.tracking.lastMeasurementMs = uint32(nowMs);
      return true;
    }
    const dtMs = clamp(elapsedMs(nowMs, this.tracking.lastMeasurementMs),
      this.config.minTrackingDtMs, this.config.maxTrackingDtMs);
    const deltaMm = measurementMm - this.tracking.lastMeasurementMm;
    const up = this.config.maxRadialSpeedMmS * dtMs / 1000 + this.config.gateMarginUpMm;
    const down = -(this.config.maxRadialSpeedMmS * dtMs / 1000 + this.config.gateMarginDownMm);
    if (deltaMm > up || deltaMm < down) {
      this.trueRejectCount++;
      return false;
    }
    this.tracking.lastMeasurementMm = measurementMm;
    this.tracking.lastMeasurementMs = uint32(nowMs);
    return true;
  }

  private noteTrackingUpdate(nowMs: number): void {
    this.tracking.lastUpdateMs = uint32(nowMs);
    this.tracking.hasUpdate = true;
  }

  private observeSettled(measurementMm: number): boolean {
    if (Math.abs(measurementMm - this.kf.x) > this.config.settleResidualMm) {
      this.tracking.settleCount = 0;
      return false;
    }
    this.tracking.settleCount = Math.min(0xff, this.tracking.settleCount + 1);
    if (this.tracking.settleCount < this.config.settleSamples) return false;
    this.state = 'stable';
    this.tracking.hasLastMeasurement = false;
    this.tracking.hasUpdate = false;
    this.tracking.settleCount = 0;
    this.trackExitCount++;
    return true;
  }

  private seedTrackingGain(r: number): void {
    this.kf.r = r;
    this.kf.q = trackingProcessNoise(this.config, r, this.config.nominalSampleMs);
    this.kf.p = r * this.config.trackingGain;
  }

  private processActive(input: AdaptiveLegacyInput): AdaptiveLegacyOutput {
    this.beginSample(input.nowMs);
    const medianMm = pushMedian(this.median, input.rawMm);

    if (this.state === 'stale-reacquire') {
      const reacquiredMm = this.observeStaleReacquire(medianMm, input.fppDbm, input.nowMs);
      if (reacquiredMm === null) {
        return this.output(false, 'stale-rejected', medianMm - this.kf.x, 0);
      }
      this.kf.initialized = true;
      this.kf.x = reacquiredMm;
      this.kf.q = this.config.baseProcessNoise;
      this.kf.r = measurementNoise(input.fppDbm);
      this.kf.p = stablePosteriorCovariance(this.config, this.kf.r);
      this.kf.outlierCount = 0;
      this.kf.lastMeasMs = uint32(input.nowMs);
      return this.output(true, 'stale-reacquired', 0, 1, Math.trunc(reacquiredMm));
    }

    if (!this.kf.initialized) this.initializeKalman(medianMm, input.nowMs);
    const innovationMm = medianMm - this.kf.x;

    if (this.state === 'tracking') {
      const dtMs = this.trackingDtMs(input.nowMs);
      if (!this.acceptTrackingMeasurement(medianMm, input.nowMs)) {
        return this.output(false, 'tracking-rejected', innovationMm, 0);
      }
      this.kf.q = trackingProcessNoise(this.config, measurementNoise(input.fppDbm), dtMs);
      const updated = this.kalmanUpdate(medianMm, input.fppDbm);
      this.noteTrackingUpdate(input.nowMs);
      if (this.observeSettled(medianMm)) {
        this.kf.q = this.config.baseProcessNoise;
        this.kf.p = stablePosteriorCovariance(this.config, this.kf.r);
      }
      return this.output(true, 'tracking', innovationMm, updated.gain, updated.filteredMm);
    }

    const gateRejected = this.legacyGate(medianMm, input.nowMs, false);
    const enteredTracking = this.observeMotionCandidate(medianMm, this.kf.x, input.nowMs);
    if (enteredTracking) {
      this.seedTrackingGain(measurementNoise(input.fppDbm));
      this.kf.outlierCount = 0;
      this.kf.lastMeasMs = uint32(input.nowMs);
      if (!this.acceptTrackingMeasurement(medianMm, input.nowMs)) {
        this.trueRejectCount++;
        return this.output(false, 'tracking-rejected', innovationMm, 0);
      }
      const updated = this.kalmanUpdate(medianMm, input.fppDbm);
      this.noteTrackingUpdate(input.nowMs);
      return this.output(true, 'tracking', innovationMm, updated.gain, updated.filteredMm);
    }
    if (gateRejected) {
      this.trueRejectCount++;
      return this.output(false, 'candidate-rejected', innovationMm, 0);
    }

    this.kf.q = this.config.baseProcessNoise;
    const updated = this.kalmanUpdate(medianMm, input.fppDbm);
    return this.output(true, 'accepted', innovationMm, updated.gain, updated.filteredMm);
  }
}

/** A stable identifier written into replay reports; not a firmware build ID. */
export const ADAPTIVE_LEGACY_REPLAY_CONFIG_FINGERPRINT = [
  'legacy-adaptive-v1',
  `q:${DEFAULT_ADAPTIVE_LEGACY_CONFIG.baseProcessNoise}`,
  `enter:${DEFAULT_ADAPTIVE_LEGACY_CONFIG.motionEnterMm}`,
  `confirm:${DEFAULT_ADAPTIVE_LEGACY_CONFIG.motionConfirmSamples}`,
  `k:${DEFAULT_ADAPTIVE_LEGACY_CONFIG.trackingGain}`,
  `stale:${DEFAULT_ADAPTIVE_LEGACY_CONFIG.staleResetMs}`,
].join('|');
