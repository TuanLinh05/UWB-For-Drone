import { useRef, useState, useCallback, useEffect } from 'react';
import {
  solveTrilateration2DDetailed,
  type TrilaterationFailureReason,
} from '../lib/trilateration';
import { KalmanFilter2D } from '../lib/kalman2d';
import { elapsedMcuMilliseconds, normalizeMcuTimestamp } from '../lib/positionTiming';
import { RingBuffer2D } from '../lib/ringBuffer2d';
import { SHADOW_POSITION_CONFIG, POSITION_PIPELINE_CONFIG_VERSION } from '../lib/position/config';
import { provisionalCalibrationRegistry } from '../lib/position/observationBuilder';
import { PositionPipeline } from '../lib/position/positionPipeline';
import type { PositionOutputMode } from '../lib/position/positionPipeline';
import type {
  ExcludedObservation,
  KalmanState2D,
  KalmanTrackingMode,
  PositionSolveResult2D,
  SolveFailureReason,
} from '../lib/position/types';
import type { AnchorPosition, FirmwareInfo, RangeSample } from '../lib/types';

export const MAX_POSITION_ANCHOR_AGE_MS = 200;
const DEFAULT_DT_SECONDS = 0.02;
const MIN_DT_SECONDS = 0.005;
const MAX_DT_SECONDS = 0.2;

export type PositionEstimateFailureReason =
  | TrilaterationFailureReason
  | 'waiting-for-data'
  | 'invalid-layout-anchor'
  | 'duplicate-layout-anchor-id'
  | 'invalid-sample-anchor-id'
  | 'duplicate-sample-anchor-id'
  | 'insufficient-valid-anchors'
  | 'invalid-mcu-timestamp'
  | 'out-of-order-mcu-timestamp';

export interface PositionEstimateQuality {
  status: 'idle' | 'valid' | 'invalid';
  reason: PositionEstimateFailureReason | null;
  usedAnchorIds: number[];
  conditionNumber: number | null;
  residualRmsM: number | null;
  dtSeconds: number | null;
}

const INITIAL_QUALITY: PositionEstimateQuality = {
  status: 'idle',
  reason: 'waiting-for-data',
  usedAnchorIds: [],
  conditionNumber: null,
  residualRmsM: null,
  dtSeconds: null,
};

export interface ShadowPositionQuality {
  status: 'idle' | 'valid' | 'invalid';
  reason: SolveFailureReason | 'observation-input-invalid' | null;
  configVersion: string;
  usableAnchorIds: number[];
  excludedAnchorIds: number[];
  excluded: ExcludedObservation[];
  result: PositionSolveResult2D | null;
  estimate: KalmanState2D | null;
  trackingMode: KalmanTrackingMode;
  measurementAccepted: boolean;
  nis: number | null;
  outputMode: PositionOutputMode;
}

const INITIAL_SHADOW_QUALITY: ShadowPositionQuality = {
  status: 'idle',
  reason: null,
  configVersion: POSITION_PIPELINE_CONFIG_VERSION,
  usableAnchorIds: [],
  excludedAnchorIds: [],
  excluded: [],
  result: null,
  estimate: null,
  trackingMode: 'uninitialized',
  measurementAccepted: false,
  nis: null,
  outputMode: 'none',
};

/**
 * Real-time 2D position estimation from UWB ranges.
 *
 * Observations are joined to the configured layout by anchor ID. Invalid,
 * stale, missing and non-finite observations are excluded; at least three
 * usable anchors are required before either solver is updated.
 */
export function usePositionEstimate(
  anchorLayout: AnchorPosition[],
  useRaw = false,
  firmwareInfo: FirmwareInfo | null = null,
) {
  const kfRef = useRef(new KalmanFilter2D());
  const lastMcuTimeRef = useRef<number | null>(null);
  const trilatTrailRef = useRef(new RingBuffer2D());
  const kalmanTrailRef = useRef(new RingBuffer2D());

  const [trilatPoint, setTrilatPoint] = useState<{ x: number; y: number } | null>(null);
  const [kalmanPoint, setKalmanPoint] = useState<{ x: number; y: number; vx: number; vy: number } | null>(null);
  const [quality, setQuality] = useState<PositionEstimateQuality>(INITIAL_QUALITY);
  const [shadowQuality, setShadowQuality] = useState<ShadowPositionQuality>(INITIAL_SHADOW_QUALITY);
  const shadowPipelineRef = useRef(new PositionPipeline(SHADOW_POSITION_CONFIG));
  const shadowLatestRef = useRef<ShadowPositionQuality>(INITIAL_SHADOW_QUALITY);
  const shadowLastPublishMsRef = useRef(0);

  // PositionMap registers an imperative drawing callback to keep the trails
  // outside React state at telemetry rate.
  const onFrameRef = useRef<((trilat: RingBuffer2D, kalman: RingBuffer2D) => void) | null>(null);
  const registerFrameCallback = useCallback(
    (cb: typeof onFrameRef.current) => { onFrameRef.current = cb; },
    [],
  );

  const markInvalid = useCallback((
    reason: PositionEstimateFailureReason,
    usedAnchorIds: number[] = [],
    conditionNumber: number | null = null,
  ) => {
    // Never leave an old point looking like a current measurement after a
    // rejected sample. Trails are retained for diagnostics and resume.
    setTrilatPoint(null);
    setKalmanPoint(null);
    setQuality({
      status: 'invalid',
      reason,
      usedAnchorIds,
      conditionNumber,
      residualRmsM: null,
      dtSeconds: null,
    });
  }, []);

  const feed = useCallback((sample: RangeSample) => {
    // C6 executes in shadow only. Its output is diagnostic/replay data and must
    // not feed the production trilateration or Kalman path until C4/C6 gates.
    const publishShadow = (next: ShadowPositionQuality) => {
      const previous = shadowLatestRef.current;
      shadowLatestRef.current = next;
      const statusChanged = previous.status !== next.status
        || previous.reason !== next.reason
        || previous.trackingMode !== next.trackingMode;
      if (statusChanged || sample.clientTime - shadowLastPublishMsRef.current >= 100) {
        shadowLastPublishMsRef.current = sample.clientTime;
        setShadowQuality(next);
      }
    };

    const shadowFrame = shadowPipelineRef.current.process({
      sample,
      layout: anchorLayout,
      calibration: provisionalCalibrationRegistry(firmwareInfo),
      observation: {
        rangeSource: useRaw ? 'raw' : 'filtered',
        defaultRangingMode: firmwareInfo?.rangingMode === 'ds' ? 'DS' : 'SS',
      },
    });
    const shadowIds = shadowFrame.usableAnchorIds;
    const shadowExcludedIds = shadowFrame.excluded
      .flatMap(exclusion => exclusion.id === null ? [] : [exclusion.id]);
    if (!shadowFrame.measurement) {
      publishShadow({
        ...INITIAL_SHADOW_QUALITY,
        status: 'invalid',
        reason: shadowFrame.failureReason,
        usableAnchorIds: shadowIds,
        excludedAnchorIds: shadowExcludedIds,
        excluded: shadowFrame.excluded,
        estimate: shadowFrame.estimate,
        trackingMode: shadowFrame.correction.mode,
        measurementAccepted: shadowFrame.correction.accepted,
        nis: shadowFrame.correction.nis,
        outputMode: shadowFrame.outputMode,
      });
    } else {
      publishShadow({
        status: 'valid',
        reason: null,
        configVersion: POSITION_PIPELINE_CONFIG_VERSION,
        usableAnchorIds: shadowIds,
        excludedAnchorIds: shadowExcludedIds,
        excluded: shadowFrame.excluded,
        result: shadowFrame.measurement,
        estimate: shadowFrame.estimate,
        trackingMode: shadowFrame.correction.mode,
        measurementAccepted: shadowFrame.correction.accepted,
        nis: shadowFrame.correction.nis,
        outputMode: shadowFrame.outputMode,
      });
    }

    const mcuTime = normalizeMcuTimestamp(sample.timeMs);
    if (mcuTime === null) {
      markInvalid('invalid-mcu-timestamp');
      return;
    }

    const layoutById = new Map<number, AnchorPosition>();
    for (const anchor of anchorLayout) {
      if (!Number.isFinite(anchor.id) || !Number.isFinite(anchor.x) || !Number.isFinite(anchor.y)) {
        markInvalid('invalid-layout-anchor');
        return;
      }
      if (layoutById.has(anchor.id)) {
        markInvalid('duplicate-layout-anchor-id', [anchor.id]);
        return;
      }
      layoutById.set(anchor.id, anchor);
    }

    const sampleIds = new Set<number>();
    for (const anchor of sample.anchors) {
      if (!Number.isFinite(anchor.id)) {
        markInvalid('invalid-sample-anchor-id');
        return;
      }
      if (sampleIds.has(anchor.id)) {
        markInvalid('duplicate-sample-anchor-id', [anchor.id]);
        return;
      }
      sampleIds.add(anchor.id);
    }

    const solverAnchors: AnchorPosition[] = [];
    const distancesM: number[] = [];
    for (const layoutAnchor of anchorLayout) {
      const observation = sample.anchorsById[layoutAnchor.id];
      if (!observation?.valid) continue;
      if (!Number.isFinite(observation.ageMs) || observation.ageMs < 0
        || observation.ageMs > MAX_POSITION_ANCHOR_AGE_MS) continue;

      const distanceMm = useRaw ? observation.rawMm : observation.filtMm;
      if (typeof distanceMm !== 'number' || !Number.isFinite(distanceMm) || distanceMm <= 0) continue;
      solverAnchors.push(layoutAnchor);
      distancesM.push(distanceMm / 1000);
    }

    const usedAnchorIds = solverAnchors.map(anchor => anchor.id);
    if (solverAnchors.length < 3) {
      markInvalid('insufficient-valid-anchors', usedAnchorIds);
      return;
    }

    const solution = solveTrilateration2DDetailed(solverAnchors, distancesM);
    if (!solution.ok) {
      markInvalid(solution.reason, solution.usedAnchorIds, solution.conditionNumber ?? null);
      return;
    }

    let dtSeconds: number | null = null;
    let filteredPoint: { x: number; y: number; vx: number; vy: number };
    if (lastMcuTimeRef.current === null || !kfRef.current.isInitialized) {
      kfRef.current.initialize(solution.point.x, solution.point.y);
      filteredPoint = kfRef.current.getState();
    } else {
      const elapsedMs = elapsedMcuMilliseconds(mcuTime, lastMcuTimeRef.current);
      if (elapsedMs === null) {
        markInvalid('out-of-order-mcu-timestamp', solution.quality.usedAnchorIds);
        return;
      }
      dtSeconds = Math.max(MIN_DT_SECONDS, Math.min(MAX_DT_SECONDS, elapsedMs / 1000));
      filteredPoint = kfRef.current.update(solution.point.x, solution.point.y, dtSeconds);
    }
    lastMcuTimeRef.current = mcuTime;

    trilatTrailRef.current.push(solution.point.x, solution.point.y);
    kalmanTrailRef.current.push(filteredPoint.x, filteredPoint.y);
    setTrilatPoint(solution.point);
    setKalmanPoint(filteredPoint);
    setQuality({
      status: 'valid',
      reason: null,
      usedAnchorIds: solution.quality.usedAnchorIds,
      conditionNumber: solution.quality.conditionNumber,
      residualRmsM: solution.quality.residualRmsM,
      dtSeconds: dtSeconds ?? DEFAULT_DT_SECONDS,
    });

    onFrameRef.current?.(trilatTrailRef.current, kalmanTrailRef.current);
  }, [anchorLayout, firmwareInfo, markInvalid, useRaw]);

  const reset = useCallback(() => {
    kfRef.current.reset();
    lastMcuTimeRef.current = null;
    trilatTrailRef.current.clear();
    kalmanTrailRef.current.clear();
    setTrilatPoint(null);
    setKalmanPoint(null);
    setQuality(INITIAL_QUALITY);
    shadowPipelineRef.current.reset();
    shadowLatestRef.current = INITIAL_SHADOW_QUALITY;
    shadowLastPublishMsRef.current = 0;
    setShadowQuality(INITIAL_SHADOW_QUALITY);
    onFrameRef.current?.(trilatTrailRef.current, kalmanTrailRef.current);
  }, []);

  // Reset estimator state for semantic configuration changes. Sorting makes a
  // harmless array reorder ID-invariant while still detecting coordinate edits.
  const layoutSignature = anchorLayout
    .map(anchor => `${anchor.id}:${anchor.x}:${anchor.y}`)
    .sort()
    .join('|');
  const profileSignature = firmwareInfo
    ? [
      firmwareInfo.schemaVersion,
      firmwareInfo.firmwareBuildId ?? 'unknown',
      firmwareInfo.rangingMode,
      firmwareInfo.calibrationProfile,
      firmwareInfo.dsCalibratedMask,
      firmwareInfo.rangeFilterMode,
      firmwareInfo.legacyAdaptiveMode,
      firmwareInfo.c9_2MotionMode,
      firmwareInfo.phyProfileId,
      ...Object.entries(firmwareInfo.activeOffsetsM)
        .sort(([left], [right]) => Number(left) - Number(right))
        .flat(),
    ].join('|')
    : 'firmware-info-unavailable';
  useEffect(() => {
    reset();
  }, [layoutSignature, profileSignature, reset, useRaw]);

  return {
    trilatPoint,
    kalmanPoint,
    quality,
    shadowQuality,
    feed,
    reset,
    registerFrameCallback,
  };
}
