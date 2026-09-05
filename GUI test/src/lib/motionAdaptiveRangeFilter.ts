/**
 * Deterministic replay model for the optional C9.2 motion-regime range
 * controller. Keep the numeric seed values and transition order aligned with
 * motion_adaptive_range.h and tag_ranging.c. It has no dependency on browser
 * time: every input uses the MCU timestamp of its physical Anchor ID.
 */

export type MotionRangeState = 'reacquire' | 'static' | 'slow' | 'fast' | 'settling' | 'degraded';
export type MotionRangeDecision = 'accepted' | 'reacquired' | 'candidate-rejected' | 'kinematic-rejected';

export interface MotionAdaptiveRangeConfig {
  baseProcessNoise: number;
  slowEnterZ: number;
  slowExitZ: number;
  fastEnterZ: number;
  fastExitZ: number;
  cusumDriftZ: number;
  slowSpeedMmS: number;
  fastSpeedMmS: number;
  slowGain: number;
  fastGain: number;
  settlingGain: number;
  minimumSigmaMm: number;
  reacquireMinFppDbm: number;
  reacquireClusterMm: number;
  maxRadialSpeedMmS: number;
  gateMarginUpMm: number;
  gateMarginDownMm: number;
  slowConfirmSamples: number;
  fastConfirmSamples: number;
  settleDwellSamples: number;
  staticDwellSamples: number;
  reacquireSamples: number;
  degradeAfterRejects: number;
  staleResetMs: number;
  candidateMaxGapMs: number;
  minDtMs: number;
  maxDtMs: number;
}

/** Mirrors UWB_C9_2_* in uwb_calibration.h. */
export const DEFAULT_MOTION_ADAPTIVE_RANGE_CONFIG: Readonly<MotionAdaptiveRangeConfig> = {
  baseProcessNoise: 0.05,
  slowEnterZ: 2.5,
  slowExitZ: 1.5,
  fastEnterZ: 5,
  fastExitZ: 3,
  cusumDriftZ: 0.5,
  slowSpeedMmS: 250,
  fastSpeedMmS: 1000,
  slowGain: 0.12,
  fastGain: 0.26,
  settlingGain: 0.10,
  minimumSigmaMm: 50,
  reacquireMinFppDbm: -105,
  reacquireClusterMm: 250,
  maxRadialSpeedMmS: 10_000,
  gateMarginUpMm: 100,
  gateMarginDownMm: 250,
  slowConfirmSamples: 3,
  fastConfirmSamples: 2,
  settleDwellSamples: 10,
  staticDwellSamples: 25,
  reacquireSamples: 3,
  degradeAfterRejects: 3,
  staleResetMs: 500,
  candidateMaxGapMs: 100,
  minDtMs: 10,
  maxDtMs: 500,
};

export const MOTION_ADAPTIVE_RANGE_REPLAY_CONFIG_FINGERPRINT =
  'c9.2-motion-v1-z2.5-5-g0.12-0.26-settle0.10';

export interface MotionAdaptiveRangeInput {
  rawMm: number;
  fppDbm: number;
  nowMs: number;
}

export interface MotionAdaptiveRangeOutput {
  publishValid: boolean;
  filteredMm?: number;
  decision: MotionRangeDecision;
  state: MotionRangeState;
  gain: number;
  slopeMmS: number;
  motionScore: number;
  staticEnterCount: number;
  slowEnterCount: number;
  fastEnterCount: number;
  settlingEnterCount: number;
  degradedEnterCount: number;
  staleReacquireCount: number;
  trueRejectCount: number;
}

interface MedianState { values: [number, number, number]; head: number; count: number; }
interface KalmanState { initialized: boolean; q: number; r: number; x: number; p: number; }

function uint32(value: number): number { return value >>> 0; }
function elapsedMs(nowMs: number, thenMs: number): number { return (uint32(nowMs) - uint32(thenMs)) >>> 0; }
function clamp(value: number, min: number, max: number): number { return Math.min(max, Math.max(min, value)); }
function abs(value: number): number { return value < 0 ? -value : value; }
function median(values: number[]): number {
  const sorted = [...values].sort((left, right) => left - right);
  const centre = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[centre] : (sorted[centre - 1] + sorted[centre]) / 2;
}

function measurementNoise(fppDbm: number): number {
  if (fppDbm <= -95) return 10_000;
  if (fppDbm > -75) return 50;
  if (fppDbm > -82) return 200;
  return 1_000;
}

function freshMedian(): MedianState { return { values: [0, 0, 0], head: 0, count: 0 }; }
function pushMedian(state: MedianState, value: number): number {
  state.values[state.head] = value;
  state.head = (state.head + 1) % 3;
  state.count = Math.min(3, state.count + 1);
  return state.count < 3 ? value : median(state.values);
}

/** One instance belongs to exactly one Anchor ID in the replay runner. */
export class MotionAdaptiveRangeFilter {
  private readonly config: MotionAdaptiveRangeConfig;
  private median = freshMedian();
  private kf: KalmanState = { initialized: false, q: 0.05, r: 0, x: 0, p: 0 };
  private state: MotionRangeState = 'reacquire';
  private initialized = false;
  private lastSampleMs = 0;
  private lastMedianMm = 0;
  private lastMedianMs: number | null = null;
  private slopes: number[] = [];
  private positiveEvidence = 0;
  private negativeEvidence = 0;
  private lastSlopeMmS = 0;
  private lastMotionScore = 0;
  private slowConfirmCount = 0;
  private fastConfirmCount = 0;
  private quietCount = 0;
  private reacquireCount = 0;
  private reacquireMeanMm = 0;
  private reacquireLastMs: number | null = null;
  private rejectStreak = 0;
  private staticEnterCount = 0;
  private slowEnterCount = 0;
  private fastEnterCount = 0;
  private settlingEnterCount = 0;
  private degradedEnterCount = 0;
  private staleReacquireCount = 0;
  private trueRejectCount = 0;

  constructor(config: Partial<MotionAdaptiveRangeConfig> = {}) {
    this.config = { ...DEFAULT_MOTION_ADAPTIVE_RANGE_CONFIG, ...config };
  }

  reset(): void {
    this.median = freshMedian();
    this.kf = { initialized: false, q: this.config.baseProcessNoise, r: 0, x: 0, p: 0 };
    this.state = 'reacquire';
    this.initialized = false;
    this.lastSampleMs = 0;
    this.lastMedianMm = 0;
    this.lastMedianMs = null;
    this.slopes = [];
    this.positiveEvidence = 0;
    this.negativeEvidence = 0;
    this.lastSlopeMmS = 0;
    this.lastMotionScore = 0;
    this.slowConfirmCount = 0;
    this.fastConfirmCount = 0;
    this.quietCount = 0;
    this.reacquireCount = 0;
    this.reacquireMeanMm = 0;
    this.reacquireLastMs = null;
    this.rejectStreak = 0;
    this.staticEnterCount = 0;
    this.slowEnterCount = 0;
    this.fastEnterCount = 0;
    this.settlingEnterCount = 0;
    this.degradedEnterCount = 0;
    this.staleReacquireCount = 0;
    this.trueRejectCount = 0;
  }

  process(input: MotionAdaptiveRangeInput): MotionAdaptiveRangeOutput {
    if (!Number.isFinite(input.rawMm) || !Number.isFinite(input.fppDbm) || !Number.isFinite(input.nowMs)) {
      return this.output(false, 'candidate-rejected', 0);
    }
    const nowMs = uint32(input.nowMs);
    if (this.initialized && elapsedMs(nowMs, this.lastSampleMs) > this.config.staleResetMs) {
      this.enter('reacquire');
      this.resetEvidence();
      this.resetReacquire();
      this.rejectStreak = 0;
      this.lastMedianMm = 0;
      this.lastMedianMs = null;
      this.slopes = [];
      this.lastSlopeMmS = 0;
      this.lastMotionScore = 0;
      this.median = freshMedian();
      this.kf.initialized = false;
      this.staleReacquireCount++;
    }
    this.initialized = true;
    this.lastSampleMs = nowMs;
    const medianMm = pushMedian(this.median, input.rawMm);

    if (this.state === 'reacquire' || this.state === 'degraded') {
      const reacquired = this.observeReacquire(medianMm, input.fppDbm, nowMs);
      if (reacquired === null) return this.output(false, 'candidate-rejected', 0);
      this.initializeKalman(reacquired, input.fppDbm);
      this.noteAcceptedMedian(reacquired, nowMs);
      return this.output(true, 'reacquired', 1, Math.trunc(reacquired));
    }
    if (!this.kf.initialized) {
      this.enter('reacquire');
      return this.output(false, 'candidate-rejected', 0);
    }

    const innovation = medianMm - this.kf.x;
    if (!this.acceptKinematic(medianMm, nowMs)) {
      const becameDegraded = this.noteReject();
      if (becameDegraded) {
        /* Match firmware: a rejected median window cannot bootstrap the next
         * physical location after controlled degradation. */
        this.median = freshMedian();
        this.kf.initialized = false;
        this.lastMedianMm = 0;
        this.lastMedianMs = null;
        this.slopes = [];
        this.lastSlopeMmS = 0;
        this.lastMotionScore = 0;
      }
      return this.output(false, 'kinematic-rejected', 0);
    }

    const r = measurementNoise(input.fppDbm);
    const sigma = Math.sqrt(r);
    const dtMs = this.lastMedianMs === null
      ? this.config.minDtMs
      : clamp(elapsedMs(nowMs, this.lastMedianMs), this.config.minDtMs, this.config.maxDtMs);
    this.observeAccepted(medianMm, innovation, sigma, nowMs);

    const gainTarget = this.targetGain();
    if (gainTarget > 0) this.seedDynamicGain(r, gainTarget, dtMs);
    else {
      this.kf.q = this.config.baseProcessNoise;
      this.kf.r = r;
      this.kf.p = this.stablePosteriorCovariance(r);
    }
    const { filteredMm, gain } = this.kalmanUpdate(medianMm, input.fppDbm);
    return this.output(true, 'accepted', gain, Math.trunc(filteredMm));
  }

  private output(publishValid: boolean, decision: MotionRangeDecision, gain: number, filteredMm?: number): MotionAdaptiveRangeOutput {
    return {
      publishValid,
      ...(filteredMm === undefined ? {} : { filteredMm }),
      decision,
      state: this.state,
      gain,
      slopeMmS: this.lastSlopeMmS,
      motionScore: this.lastMotionScore,
      staticEnterCount: this.staticEnterCount,
      slowEnterCount: this.slowEnterCount,
      fastEnterCount: this.fastEnterCount,
      settlingEnterCount: this.settlingEnterCount,
      degradedEnterCount: this.degradedEnterCount,
      staleReacquireCount: this.staleReacquireCount,
      trueRejectCount: this.trueRejectCount,
    };
  }

  private enter(next: MotionRangeState): void {
    if (this.state === next) return;
    this.state = next;
    if (next === 'static') this.staticEnterCount++;
    else if (next === 'slow') this.slowEnterCount++;
    else if (next === 'fast') this.fastEnterCount++;
    else if (next === 'settling') this.settlingEnterCount++;
    else if (next === 'degraded') this.degradedEnterCount++;
  }

  private resetEvidence(): void {
    this.slowConfirmCount = 0;
    this.fastConfirmCount = 0;
    this.quietCount = 0;
    this.positiveEvidence = 0;
    this.negativeEvidence = 0;
  }

  private resetReacquire(): void {
    this.reacquireCount = 0;
    this.reacquireMeanMm = 0;
    this.reacquireLastMs = null;
  }

  private observeReacquire(medianMm: number, fppDbm: number, nowMs: number): number | null {
    if (fppDbm < this.config.reacquireMinFppDbm) {
      this.resetReacquire();
      return null;
    }
    const compatible = this.reacquireCount > 0 && this.reacquireLastMs !== null
      && elapsedMs(nowMs, this.reacquireLastMs) <= this.config.candidateMaxGapMs
      && abs(medianMm - this.reacquireMeanMm) <= this.config.reacquireClusterMm;
    if (!compatible) {
      this.reacquireCount = 1;
      this.reacquireMeanMm = medianMm;
    } else {
      this.reacquireCount++;
      this.reacquireMeanMm += (medianMm - this.reacquireMeanMm) / this.reacquireCount;
    }
    this.reacquireLastMs = nowMs;
    if (this.reacquireCount < this.config.reacquireSamples) return null;
    const result = this.reacquireMeanMm;
    this.enter('static');
    this.resetEvidence();
    this.resetReacquire();
    this.rejectStreak = 0;
    return result;
  }

  private initializeKalman(measurementMm: number, fppDbm: number): void {
    const r = measurementNoise(fppDbm);
    this.kf = {
      initialized: true,
      q: this.config.baseProcessNoise,
      r,
      x: measurementMm,
      p: this.stablePosteriorCovariance(r),
    };
  }

  private stablePosteriorCovariance(r: number): number {
    const q = this.config.baseProcessNoise;
    return (Math.sqrt(q * q + 4 * q * r) - q) / 2;
  }

  private noteAcceptedMedian(medianMm: number, nowMs: number): void {
    if (this.lastMedianMs !== null) {
      const dtMs = clamp(elapsedMs(nowMs, this.lastMedianMs), this.config.minDtMs, this.config.maxDtMs);
      this.slopes.push((medianMm - this.lastMedianMm) * 1000 / dtMs);
      if (this.slopes.length > 5) this.slopes.shift();
    }
    this.lastMedianMm = medianMm;
    this.lastMedianMs = nowMs;
    this.lastSlopeMmS = this.slopes.length === 0 ? 0 : median(this.slopes);
  }

  private acceptKinematic(medianMm: number, nowMs: number): boolean {
    if (this.lastMedianMs === null) return true;
    const dtMs = clamp(elapsedMs(nowMs, this.lastMedianMs), this.config.minDtMs, this.config.maxDtMs);
    const limitUp = this.config.maxRadialSpeedMmS * dtMs / 1000 + this.config.gateMarginUpMm;
    const limitDown = -(this.config.maxRadialSpeedMmS * dtMs / 1000 + this.config.gateMarginDownMm);
    const jump = medianMm - this.lastMedianMm;
    return jump <= limitUp && jump >= limitDown;
  }

  private noteReject(): boolean {
    this.rejectStreak = Math.min(0xff, this.rejectStreak + 1);
    this.trueRejectCount++;
    if (this.rejectStreak >= this.config.degradeAfterRejects) {
      this.enter('degraded');
      this.resetEvidence();
      this.resetReacquire();
    }
    return this.state === 'degraded';
  }

  private observeAccepted(medianMm: number, innovation: number, sigmaMm: number, nowMs: number): void {
    const sigma = Math.max(sigmaMm, this.config.minimumSigmaMm);
    const z = innovation / sigma;
    if (z >= 0) {
      this.positiveEvidence = clamp(this.positiveEvidence + z - this.config.cusumDriftZ, 0, 32);
      this.negativeEvidence = clamp(this.negativeEvidence - z - this.config.cusumDriftZ, 0, 32);
    } else {
      this.negativeEvidence = clamp(this.negativeEvidence - z - this.config.cusumDriftZ, 0, 32);
      this.positiveEvidence = clamp(this.positiveEvidence + z - this.config.cusumDriftZ, 0, 32);
    }
    this.lastMotionScore = Math.max(this.positiveEvidence, this.negativeEvidence);
    this.noteAcceptedMedian(medianMm, nowMs);
    const slowEvidence = this.lastMotionScore >= this.config.slowEnterZ
      && abs(this.lastSlopeMmS) >= this.config.slowSpeedMmS;
    const fastEvidence = this.lastMotionScore >= this.config.fastEnterZ
      && abs(this.lastSlopeMmS) >= this.config.fastSpeedMmS;
    const fastStill = this.lastMotionScore >= this.config.fastExitZ
      && abs(this.lastSlopeMmS) >= this.config.fastSpeedMmS;
    const quiet = this.lastMotionScore <= this.config.slowExitZ
      && abs(this.lastSlopeMmS) <= this.config.slowSpeedMmS;

    this.fastConfirmCount = fastEvidence ? Math.min(0xff, this.fastConfirmCount + 1) : 0;
    if (slowEvidence) this.slowConfirmCount = Math.min(0xff, this.slowConfirmCount + 1);
    else if (this.state !== 'fast') this.slowConfirmCount = 0;
    this.quietCount = quiet ? Math.min(0xff, this.quietCount + 1) : 0;

    if (this.state === 'static') {
      if (this.fastConfirmCount >= this.config.fastConfirmSamples) this.enter('fast');
      else if (this.slowConfirmCount >= this.config.slowConfirmSamples) this.enter('slow');
    } else if (this.state === 'slow') {
      if (this.fastConfirmCount >= this.config.fastConfirmSamples) this.enter('fast');
      else if (this.quietCount >= this.config.settleDwellSamples) this.enter('settling');
    } else if (this.state === 'fast') {
      if (!fastStill && slowEvidence) this.enter('slow');
      else if (this.quietCount >= this.config.settleDwellSamples) this.enter('settling');
    } else if (this.state === 'settling') {
      if (this.fastConfirmCount >= this.config.fastConfirmSamples) this.enter('fast');
      else if (this.slowConfirmCount >= this.config.slowConfirmSamples) this.enter('slow');
      else if (this.quietCount >= this.config.staticDwellSamples) this.enter('static');
    }
    this.rejectStreak = 0;
  }

  private targetGain(): number {
    if (this.state === 'fast') return this.config.fastGain;
    if (this.state === 'slow') return this.config.slowGain;
    if (this.state === 'settling') return this.config.settlingGain;
    return 0;
  }

  private seedDynamicGain(r: number, targetGain: number, dtMs: number): void {
    this.kf.r = r;
    this.kf.q = r * targetGain * targetGain / (1 - targetGain)
      * (dtMs / 20);
    this.kf.p = r * targetGain;
  }

  private kalmanUpdate(measurementMm: number, fppDbm: number): { filteredMm: number; gain: number } {
    this.kf.r = measurementNoise(fppDbm);
    this.kf.p += this.kf.q;
    const gain = this.kf.p / (this.kf.p + this.kf.r);
    this.kf.x += gain * (measurementMm - this.kf.x);
    this.kf.p = (1 - gain) * this.kf.p;
    return { filteredMm: this.kf.x, gain };
  }
}
