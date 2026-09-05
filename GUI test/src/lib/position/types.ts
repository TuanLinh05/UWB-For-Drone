import type { AnchorPosition, RangeSample } from '../types';

export type Mat2 = [[number, number], [number, number]];

export interface Vec2 {
  x: number;
  y: number;
}

export type PositionRangingMode = 'SS' | 'DS' | 'SS_FALLBACK';
export type CalibrationMode = 'SS' | 'DS';

export interface RangeCalibrationProfile {
  calibrated: boolean;
  rawSigmaM: number;
  filteredSigmaM: number;
}

export type CalibrationRegistry = Readonly<Record<
  number,
  Partial<Record<CalibrationMode, RangeCalibrationProfile>>
>>;

export interface RangeObservation2D {
  id: number;
  x: number;
  y: number;
  rangeM: number;
  rawRangeM: number;
  sigmaM: number;
  ageMs: number;
  fppDbm: number | null;
  status: number;
  rangingMode: PositionRangingMode;
}

export type ObservationExcludeReason =
  | 'invalid-flag'
  | 'filter-rejected'
  | 'stale'
  | 'non-finite'
  | 'non-positive'
  | 'over-physical-limit'
  | 'duplicate-id'
  | 'unknown-anchor-id'
  | 'missing-layout'
  | 'missing-calibration'
  | 'wrong-calibration-mode';

export interface ExcludedObservation {
  id: number | null;
  reason: ObservationExcludeReason;
}

export interface ObservationBuildResult {
  usable: RangeObservation2D[];
  excluded: ExcludedObservation[];
  ageSpreadMs: number;
  fatal: boolean;
}

export interface ObservationConfig {
  rangeSource: 'raw' | 'filtered';
  defaultRangingMode: 'SS' | 'DS';
  maxRangeM: number;
  maxAgeMs: number;
  sigmaFloorM: number;
  sigmaCeilingM: number;
  maxFppVarianceMultiplier: number;
  maxAgeVarianceMultiplier: number;
  fallbackVarianceMultiplier: number;
}

export interface SolverConfig {
  maxIterations: number;
  huberK: number;
  jacobianEpsilonM: number;
  stepToleranceM: number;
  costTolerance: number;
  gradientTolerance: number;
  initialLambda: number;
  minLambda: number;
  maxLambda: number;
  lambdaUpFactor: number;
  lambdaDownFactor: number;
  maxConditionNumber: number;
  geometryEigenFloor: number;
  covarianceEigenFloorM2: number;
  covarianceEigenCeilingM2: number;
  looTriggerZ: number;
  looRejectZ: number;
  looWinnerMargin: number;
  maxReducedChi2: number;
}

export type SolveMode = 'four-anchor' | 'three-valid-fallback' | 'three-after-loo';

export interface AnchorResidual {
  id: number;
  residualM: number;
  standardizedResidual: number;
  effectiveWeightPerM2: number;
}

export interface PositionSolveResult2D extends Vec2 {
  covarianceM2: Mat2;
  rmsResidualM: number;
  normalizedChi2: number;
  gdop: number;
  conditionNumber: number;
  geometryScore: number;
  usedAnchorIds: number[];
  rejectedAnchorIds: number[];
  residuals: AnchorResidual[];
  mode: SolveMode;
  iterations: number;
  ageSpreadMs: number;
}

export type SolveFailureReason =
  | 'insufficient-observations'
  | 'bad-geometry'
  | 'non-convergent'
  | 'ambiguous-outlier'
  | 'non-finite';

export type PositionSolveOutcome =
  | { ok: true; result: PositionSolveResult2D }
  | { ok: false; reason: SolveFailureReason; diagnostics?: Record<string, unknown> };

export interface PositionPipelineConfig {
  version: string;
  observation: Omit<ObservationConfig, 'rangeSource' | 'defaultRangingMode'>;
  solver: SolverConfig;
  kalman: AdaptiveKalmanConfig;
}

export interface ObservationBuildInput {
  sample: RangeSample;
  layout: readonly AnchorPosition[];
  calibration: CalibrationRegistry;
  config: ObservationConfig;
}

export interface AdaptiveKalmanConfig {
  processAccelerationSigmaMps2: number;
  initialVelocitySigmaMps: number;
  measurementSigmaFloorM: number;
  measurementCovarianceEigenCeilingM2: number;
  stateCovarianceFloor: number;
  stateCovarianceCeiling: number;
  minDtSec: number;
  maxTrackingDtSec: number;
  maxPredictionOnlySec: number;
  nisSoftGate2D: number;
  nisHardGate2D: number;
  softGateVarianceInflation: number;
  reacquireConsistentSamples: number;
  reacquireMaxDistanceM: number;
}

export interface PositionMeasurement2D extends Vec2 {
  covarianceM2: Mat2;
}

export type KalmanTrackingMode =
  | 'uninitialized'
  | 'initialized'
  | 'tracking'
  | 'coasting'
  | 'nis-rejected'
  | 'reacquiring'
  | 'stale';

export interface KalmanState2D extends Vec2 {
  vx: number;
  vy: number;
  positionCovarianceM2: Mat2;
}

export interface KalmanCorrection {
  state: KalmanState2D | null;
  accepted: boolean;
  nis: number | null;
  mode: KalmanTrackingMode;
  rejectStreak: number;
  predictionOnlySec: number;
}
