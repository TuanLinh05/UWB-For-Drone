import type { ReplayAnchorRecord, ReplayDataset, ReplayRangeFrame } from './replayDataset';
import {
  ADAPTIVE_LEGACY_REPLAY_CONFIG_FINGERPRINT,
  AdaptiveLegacyRangeFilter,
} from './adaptiveLegacyRangeFilter';
import {
  MOTION_ADAPTIVE_RANGE_REPLAY_CONFIG_FINGERPRINT,
  MotionAdaptiveRangeFilter,
} from './motionAdaptiveRangeFilter';

export interface ReplayObservation {
  anchorId: number;
  rangeMm: number;
  ageMs: number;
  measurementTimeMs: number | null;
  fppDbm: number | null;
  rangingMode: ReplayAnchorRecord['rangingMode'];
}

export interface ReplayVariantFrameOutput {
  observations: ReplayObservation[];
  rejectedAnchorIds: number[];
  position?: { x: number; y: number };
  status?: string;
}

export interface ReplayVariant {
  readonly id: string;
  readonly label: string;
  process(frame: ReplayRangeFrame): ReplayVariantFrameOutput;
}

export type ReplayVariantFactory = () => ReplayVariant;

export interface ReplayAnchorMetrics {
  anchorId: number;
  usableSamples: number;
  availabilityPct: number;
  meanRangeMm: number | null;
  /** Smoothing/lag proxy against the corrected raw sample in the same frame. */
  meanAbsRawDeltaMm: number | null;
  /** 95th percentile of the smoothing/lag proxy; this is not an accuracy metric. */
  p95AbsRawDeltaMm: number | null;
}

export interface ReplayVariantReport {
  id: string;
  label: string;
  totalFrames: number;
  emittedFrames: number;
  positionFrames: number;
  insufficientFrames: number;
  nonFiniteOutputs: number;
  meanUsableAnchorCount: number;
  rejectedObservations: number;
  sourceGapCount: number;
  transportGapCount: number;
  statusCounts: Record<string, number>;
  perAnchor: ReplayAnchorMetrics[];
}

export interface ReplayRunReport {
  replayFormatVersion: string;
  pipelineConfigVersion: string;
  profileFingerprint: string;
  provisional: true;
  limitations: string[];
  variants: ReplayVariantReport[];
}

function missingPackets(current: number, previous: number | null): number {
  if (previous === null) return 0;
  const delta = (current - previous) >>> 0;
  return delta > 1 && delta < 0x8000_0000 ? delta - 1 : 0;
}

/**
 * A four-anchor snapshot is captured after sequential ranging slots. Per-anchor
 * filters must use the MCU time of their actual measurement, not the later
 * snapshot/client time; otherwise dynamic gates are silently widened.
 */
function measurementNowMs(frame: ReplayRangeFrame, record: ReplayAnchorRecord): number {
  return record.measurementTimeMs ?? frame.timeMs;
}

function createRecordedRangeVariant(
  id: string,
  label: string,
  source: 'correctedRawMm' | 'filteredMm',
): ReplayVariant {
  return {
    id,
    label,
    process(frame) {
      const observations: ReplayObservation[] = [];
      const rejectedAnchorIds: number[] = [];
      for (const record of frame.records) {
        const value = record[source];
        if (!record.valid || record.calibrationMissing || !Number.isFinite(value) || (value as number) < 0) {
          rejectedAnchorIds.push(record.id);
          continue;
        }
        observations.push({
          anchorId: record.id,
          rangeMm: value as number,
          ageMs: record.ageMs,
          measurementTimeMs: record.measurementTimeMs,
          fppDbm: record.fppDbm,
          rangingMode: record.rangingMode,
        });
      }
      return {
        observations,
        rejectedAnchorIds,
        status: observations.length >= 3 ? 'usable' : 'insufficient-observations',
      };
    },
  };
}

/** C5 baseline variants. They exercise identical frames and differ only by range source. */
export const recordedRawVariant: ReplayVariantFactory = () => createRecordedRangeVariant(
  'recorded-corrected-raw',
  'Recorded corrected raw range',
  'correctedRawMm',
);

export const recordedFilteredVariant: ReplayVariantFactory = () => createRecordedRangeVariant(
  'recorded-firmware-filtered',
  'Recorded firmware-filtered range',
  'filteredMm',
);

/* Mirrors the deployed C9 MEDIAN_GATE firmware profile. It is intentionally
 * replay-only: no solver/Kalman parameters are changed in this variant. */
const C9_RADIO_FAILURE_MASK = 0x2f;
const C9_MIN_RANGE_MM = 1;
const C9_MAX_RANGE_MM = 50_000;
const C9_MAX_RADIAL_SPEED_MM_S = 10_000;
const C9_GATE_MARGIN_UP_MM = 100;
const C9_GATE_MARGIN_DOWN_MM = 250;
const C9_STALE_RESET_MS = 500;
const C9_REACQUIRE_MIN_SAMPLES = 3;
const C9_REACQUIRE_CLUSTER_MM = 250;
const C9_REACQUIRE_MIN_FPP_DBM = -105;

interface C9AnchorState {
  initialized: boolean;
  distanceMm: number;
  lastAcceptedMs: number;
  medianWindow: [number, number, number];
  medianCount: number;
  medianHead: number;
  candidateCount: number;
  candidateMeanMm: number;
}

interface C9Result {
  valid: boolean;
  rangeMm?: number;
  status: 'accepted' | 'reacquired' | 'rejected-physical' | 'rejected-dynamic';
}

function c9State(): C9AnchorState {
  return {
    initialized: false,
    distanceMm: 0,
    lastAcceptedMs: 0,
    medianWindow: [0, 0, 0],
    medianCount: 0,
    medianHead: 0,
    candidateCount: 0,
    candidateMeanMm: 0,
  };
}

function uint32Elapsed(nowMs: number, thenMs: number): number {
  return (nowMs - thenMs) >>> 0;
}

function median3(left: number, middle: number, right: number): number {
  return [left, middle, right].sort((a, b) => a - b)[1];
}

function c9Initialize(state: C9AnchorState, distanceMm: number, nowMs: number): void {
  state.initialized = true;
  state.distanceMm = distanceMm;
  state.lastAcceptedMs = nowMs;
  state.medianWindow = [distanceMm, distanceMm, distanceMm];
  state.medianCount = 1;
  state.medianHead = 1;
  state.candidateCount = 0;
  state.candidateMeanMm = 0;
}

function c9CandidateMedian(state: C9AnchorState, rawMm: number): {
  medianMm: number;
  window: [number, number, number];
  count: number;
  head: number;
} {
  const window: [number, number, number] = [...state.medianWindow] as [number, number, number];
  const head = state.medianHead;
  window[head] = rawMm;
  const count = Math.min(3, state.medianCount + 1);
  const nextHead = (head + 1) % 3;
  return {
    medianMm: count < 3 ? rawMm : median3(window[0], window[1], window[2]),
    window,
    count,
    head: nextHead,
  };
}

function c9UpdateReacquireCandidate(state: C9AnchorState, rawMm: number, fppDbm: number): boolean {
  if (fppDbm < C9_REACQUIRE_MIN_FPP_DBM) {
    state.candidateCount = 0;
    return false;
  }
  if (state.candidateCount === 0 || Math.abs(rawMm - state.candidateMeanMm) > C9_REACQUIRE_CLUSTER_MM) {
    state.candidateMeanMm = rawMm;
    state.candidateCount = 1;
  } else {
    state.candidateCount++;
    state.candidateMeanMm += (rawMm - state.candidateMeanMm) / state.candidateCount;
  }
  return state.candidateCount >= C9_REACQUIRE_MIN_SAMPLES;
}

function c9Process(
  state: C9AnchorState,
  record: ReplayAnchorRecord,
  nowMs: number,
): C9Result {
  const rawMm = record.correctedRawMm;
  const fppDbm = record.fppDbm;
  if (!record.valid || record.calibrationMissing || rawMm === null || fppDbm === null
    || !Number.isFinite(rawMm) || !Number.isFinite(fppDbm)
    || rawMm < C9_MIN_RANGE_MM || rawMm > C9_MAX_RANGE_MM
    || (record.status & C9_RADIO_FAILURE_MASK) !== 0) {
    return { valid: false, status: 'rejected-physical' };
  }

  if (!state.initialized) {
    c9Initialize(state, rawMm, nowMs);
    return { valid: true, rangeMm: rawMm, status: 'accepted' };
  }

  if (uint32Elapsed(nowMs, state.lastAcceptedMs) > C9_STALE_RESET_MS) {
    c9Initialize(state, rawMm, nowMs);
    return { valid: true, rangeMm: rawMm, status: 'reacquired' };
  }

  const candidate = c9CandidateMedian(state, rawMm);
  const acceptedDtMs = Math.min(uint32Elapsed(nowMs, state.lastAcceptedMs), C9_STALE_RESET_MS);
  const motionMm = Math.ceil(C9_MAX_RADIAL_SPEED_MM_S * acceptedDtMs / 1000);
  const jumpMm = candidate.medianMm - state.distanceMm;
  if (jumpMm > motionMm + C9_GATE_MARGIN_UP_MM
    || jumpMm < -(motionMm + C9_GATE_MARGIN_DOWN_MM)) {
    if (c9UpdateReacquireCandidate(state, rawMm, fppDbm)) {
      const reacquiredMm = Math.round(state.candidateMeanMm);
      c9Initialize(state, reacquiredMm, nowMs);
      return { valid: true, rangeMm: reacquiredMm, status: 'reacquired' };
    }
    return { valid: false, status: 'rejected-dynamic' };
  }

  state.medianWindow = candidate.window;
  state.medianCount = candidate.count;
  state.medianHead = candidate.head;
  state.distanceMm = candidate.medianMm;
  state.lastAcceptedMs = nowMs;
  state.candidateCount = 0;
  state.candidateMeanMm = 0;
  return { valid: true, rangeMm: candidate.medianMm, status: 'accepted' };
}

/**
 * C9 range A/B candidate. Each anchor owns independent median/gate/reacquire
 * state and all data is joined by Anchor ID, never by record position.
 */
export const c9MedianGateReplayVariant: ReplayVariantFactory = () => {
  const states = new Map<number, C9AnchorState>();
  return {
    id: 'c9-median-gate-raw',
    label: 'C9 MEDIAN_GATE replay candidate (corrected raw)',
    process(frame) {
      const observations: ReplayObservation[] = [];
      const rejectedAnchorIds: number[] = [];
      const statuses: string[] = [];
      for (const record of frame.records) {
        const state = states.get(record.id) ?? c9State();
        states.set(record.id, state);
        const result = c9Process(state, record, measurementNowMs(frame, record));
        statuses.push(result.status);
        if (!result.valid || result.rangeMm === undefined) {
          rejectedAnchorIds.push(record.id);
          continue;
        }
        observations.push({
          anchorId: record.id,
          rangeMm: result.rangeMm,
          ageMs: record.ageMs,
          measurementTimeMs: record.measurementTimeMs,
          fppDbm: record.fppDbm,
          rangingMode: record.rangingMode,
        });
      }
      const status = statuses.includes('reacquired')
        ? 'c9:reacquired'
        : statuses.includes('rejected-dynamic')
          ? 'c9:rejected-dynamic'
          : statuses.includes('rejected-physical')
            ? 'c9:rejected-physical'
            : 'c9:accepted';
      return { observations, rejectedAnchorIds, status };
    },
  };
};

/**
 * C9.2 is replay/shadow-only until an explicitly enabled firmware build has
 * passed the same dataset plus a held-out physical flight test. Its per-ID
 * state remains independent even when radio records arrive in another order.
 */
export const c9_2MotionAdaptiveReplayVariant: ReplayVariantFactory = () => {
  const states = new Map<number, MotionAdaptiveRangeFilter>();
  return {
    id: 'c9.2-motion-adaptive-shadow-raw-v1',
    label: `C9.2 motion-regime replay candidate (${MOTION_ADAPTIVE_RANGE_REPLAY_CONFIG_FINGERPRINT})`,
    process(frame) {
      const observations: ReplayObservation[] = [];
      const rejectedAnchorIds: number[] = [];
      const statuses: string[] = [];
      for (const record of frame.records) {
        const rawMm = record.correctedRawMm;
        const fppDbm = record.fppDbm;
        if (!record.valid || record.calibrationMissing || rawMm === null || fppDbm === null
          || !Number.isFinite(rawMm) || !Number.isFinite(fppDbm)) {
          rejectedAnchorIds.push(record.id);
          statuses.push('invalid-input');
          continue;
        }
        const state = states.get(record.id) ?? new MotionAdaptiveRangeFilter();
        states.set(record.id, state);
        const result = state.process({ rawMm, fppDbm, nowMs: measurementNowMs(frame, record) });
        statuses.push(`${result.state}:${result.decision}`);
        if (!result.publishValid || result.filteredMm === undefined) {
          rejectedAnchorIds.push(record.id);
          continue;
        }
        observations.push({
          anchorId: record.id,
          rangeMm: result.filteredMm,
          ageMs: record.ageMs,
          measurementTimeMs: record.measurementTimeMs,
          fppDbm: record.fppDbm,
          rangingMode: record.rangingMode,
        });
      }
      const activeState = statuses.find(status => status.startsWith('fast:'))
        ? 'fast'
        : statuses.find(status => status.startsWith('slow:'))
          ? 'slow'
          : statuses.find(status => status.startsWith('settling:'))
            ? 'settling'
            : statuses.find(status => status.startsWith('degraded:'))
              ? 'degraded'
              : statuses.find(status => status.startsWith('reacquire:'))
                ? 'reacquire'
                : 'static';
      return {
        observations,
        rejectedAnchorIds,
        status: `c9.2:${activeState}`,
      };
    },
  };
};

/**
 * Adaptive Legacy is a replay/shadow candidate for the deployed Legacy
 * conditioner, not a replacement for C9. Each physical Anchor ID owns its own
 * median/Kalman/candidate state even if record order changes between frames.
 */
export const adaptiveLegacyReplayVariant: ReplayVariantFactory = () => {
  const states = new Map<number, AdaptiveLegacyRangeFilter>();
  return {
    id: 'legacy-adaptive-shadow-raw-v1',
    label: `Adaptive Legacy replay candidate (${ADAPTIVE_LEGACY_REPLAY_CONFIG_FINGERPRINT})`,
    process(frame) {
      const observations: ReplayObservation[] = [];
      const rejectedAnchorIds: number[] = [];
      const statuses: string[] = [];
      for (const record of frame.records) {
        const rawMm = record.correctedRawMm;
        const fppDbm = record.fppDbm;
        if (!record.valid || record.calibrationMissing || rawMm === null || fppDbm === null
          || !Number.isFinite(rawMm) || !Number.isFinite(fppDbm)
          || rawMm < C9_MIN_RANGE_MM || rawMm > C9_MAX_RANGE_MM
          || (record.status & C9_RADIO_FAILURE_MASK) !== 0) {
          rejectedAnchorIds.push(record.id);
          statuses.push('rejected-physical');
          continue;
        }
        const state = states.get(record.id) ?? new AdaptiveLegacyRangeFilter('active');
        states.set(record.id, state);
        const result = state.process({
          rawMm,
          fppDbm,
          nowMs: measurementNowMs(frame, record),
        });
        statuses.push(result.decision);
        if (!result.publishValid || result.filteredMm === undefined) {
          rejectedAnchorIds.push(record.id);
          continue;
        }
        observations.push({
          anchorId: record.id,
          rangeMm: result.filteredMm,
          ageMs: record.ageMs,
          measurementTimeMs: record.measurementTimeMs,
          fppDbm: record.fppDbm,
          rangingMode: record.rangingMode,
        });
      }
      const status = statuses.includes('stale-reacquired')
        ? 'legacy-adaptive:stale-reacquired'
        : statuses.includes('tracking-rejected')
          ? 'legacy-adaptive:tracking-rejected'
          : statuses.includes('candidate-rejected') || statuses.includes('stale-rejected')
            ? 'legacy-adaptive:candidate-rejected'
            : statuses.includes('tracking')
              ? 'legacy-adaptive:tracking'
              : 'legacy-adaptive:accepted';
      return { observations, rejectedAnchorIds, status };
    },
  };
};

/**
 * Run every variant through the exact same immutable frame sequence. C5 reports
 * structural/availability metrics only; accuracy and threshold acceptance stay
 * provisional until ground-truth tuning and holdout datasets exist.
 */
export function runReplayVariants(
  dataset: ReplayDataset,
  factories: readonly ReplayVariantFactory[],
): ReplayRunReport {
  const variants = factories.map(factory => factory());
  const ids = new Set<string>();
  variants.forEach(variant => {
    if (!variant.id || ids.has(variant.id)) throw new Error(`Replay variant ID must be unique: ${variant.id || '<empty>'}`);
    ids.add(variant.id);
  });

  const sourceGapCount = dataset.frames.reduce((state, frame) => {
    state.gaps += missingPackets(frame.seq, state.previous);
    state.previous = frame.seq;
    return state;
  }, { previous: null as number | null, gaps: 0 }).gaps;
  const transportGapCount = dataset.frames.reduce((state, frame) => {
    state.gaps += missingPackets(frame.transportSeq, state.previous);
    state.previous = frame.transportSeq;
    return state;
  }, { previous: null as number | null, gaps: 0 }).gaps;

  const reports = variants.map<ReplayVariantReport>(variant => {
    let emittedFrames = 0;
    let positionFrames = 0;
    let insufficientFrames = 0;
    let nonFiniteOutputs = 0;
    let observationCount = 0;
    let rejectedObservations = 0;
    const statusCounts: Record<string, number> = {};
    const perAnchor = new Map<number, {
      count: number;
      sumMm: number;
      rawDeltaMm: number[];
    }>();

    for (const frame of dataset.frames) {
      const output = variant.process(frame);
      const status = output.status ?? 'unspecified';
      statusCounts[status] = (statusCounts[status] ?? 0) + 1;
      if (output.observations.length >= 3) emittedFrames++;
      else insufficientFrames++;
      rejectedObservations += output.rejectedAnchorIds.length;
      observationCount += output.observations.length;
      for (const observation of output.observations) {
        if (!Number.isFinite(observation.rangeMm)) {
          nonFiniteOutputs++;
          continue;
        }
        const metrics = perAnchor.get(observation.anchorId) ?? {
          count: 0,
          sumMm: 0,
          rawDeltaMm: [],
        };
        metrics.count++;
        metrics.sumMm += observation.rangeMm;
        const rawMm = frame.records.find(record => record.id === observation.anchorId)?.correctedRawMm;
        if (typeof rawMm === 'number' && Number.isFinite(rawMm)) {
          metrics.rawDeltaMm.push(Math.abs(observation.rangeMm - rawMm));
        }
        perAnchor.set(observation.anchorId, metrics);
      }
      if (output.position && (!Number.isFinite(output.position.x) || !Number.isFinite(output.position.y))) {
        nonFiniteOutputs++;
      } else if (output.position) {
        positionFrames++;
      }
    }

    return {
      id: variant.id,
      label: variant.label,
      totalFrames: dataset.frames.length,
      emittedFrames,
      positionFrames,
      insufficientFrames,
      nonFiniteOutputs,
      meanUsableAnchorCount: dataset.frames.length > 0 ? observationCount / dataset.frames.length : 0,
      rejectedObservations,
      sourceGapCount,
      transportGapCount,
      statusCounts,
      perAnchor: dataset.anchorIds.map(anchorId => {
        const metrics = perAnchor.get(anchorId);
        const rawDeltaMm = metrics?.rawDeltaMm ?? [];
        const sortedRawDeltaMm = [...rawDeltaMm].sort((left, right) => left - right);
        const p95Index = Math.max(0, Math.ceil(sortedRawDeltaMm.length * 0.95) - 1);
        return {
          anchorId,
          usableSamples: metrics?.count ?? 0,
          availabilityPct: dataset.frames.length > 0 ? ((metrics?.count ?? 0) / dataset.frames.length) * 100 : 0,
          meanRangeMm: metrics && metrics.count > 0 ? metrics.sumMm / metrics.count : null,
          meanAbsRawDeltaMm: rawDeltaMm.length > 0
            ? rawDeltaMm.reduce((sum, value) => sum + value, 0) / rawDeltaMm.length
            : null,
          p95AbsRawDeltaMm: sortedRawDeltaMm.length > 0
            ? sortedRawDeltaMm[p95Index]
            : null,
        };
      }),
    };
  });

  return {
    replayFormatVersion: dataset.metadata.replayFormatVersion,
    pipelineConfigVersion: dataset.metadata.pipelineConfigVersion,
    profileFingerprint: dataset.metadata.profileFingerprint,
    provisional: true,
    limitations: [
      'No independent ground truth is attached; accuracy, lag and acceptance gates are not evaluated.',
      'meanAbsRawDeltaMm and p95AbsRawDeltaMm are smoothing/lag proxies against raw, not accuracy metrics.',
      'C5 baseline variants compare recorded range sources only; A0-A4 solver variants are added in C6-C9.',
      ...dataset.warnings,
    ],
    variants: reports,
  };
}
