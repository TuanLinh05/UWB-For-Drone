import type { PositionPipelineConfig } from './types';

/** Shadow-only until C4 holdout validation and C6 A/B gates are complete. */
export const POSITION_PIPELINE_CONFIG_VERSION = 'c6-shadow-v0-tune-required';

export const SHADOW_POSITION_CONFIG: PositionPipelineConfig = {
  version: POSITION_PIPELINE_CONFIG_VERSION,
  observation: {
    maxRangeM: 50,
    maxAgeMs: 200,
    sigmaFloorM: 0.02,
    sigmaCeilingM: 1.0,
    maxFppVarianceMultiplier: 4,
    maxAgeVarianceMultiplier: 4,
    fallbackVarianceMultiplier: 1.5,
  },
  solver: {
    maxIterations: 12,
    huberK: 1.5,
    jacobianEpsilonM: 1e-6,
    stepToleranceM: 1e-4,
    costTolerance: 1e-9,
    gradientTolerance: 1e-8,
    initialLambda: 1e-3,
    minLambda: 1e-8,
    maxLambda: 1e8,
    lambdaUpFactor: 10,
    lambdaDownFactor: 0.3,
    maxConditionNumber: 1e4,
    geometryEigenFloor: 1e-8,
    covarianceEigenFloorM2: 1e-6,
    covarianceEigenCeilingM2: 25,
    looTriggerZ: 3.5,
    looRejectZ: 4.0,
    looWinnerMargin: 1.0,
    maxReducedChi2: 9.0,
  },
  kalman: {
    processAccelerationSigmaMps2: 4.0,
    initialVelocitySigmaMps: 5.0,
    measurementSigmaFloorM: 0.03,
    measurementCovarianceEigenCeilingM2: 25,
    stateCovarianceFloor: 1e-9,
    stateCovarianceCeiling: 1e6,
    minDtSec: 0.005,
    maxTrackingDtSec: 0.2,
    maxPredictionOnlySec: 0.2,
    nisSoftGate2D: 5.991,
    nisHardGate2D: 9.2103,
    softGateVarianceInflation: 2.0,
    reacquireConsistentSamples: 3,
    reacquireMaxDistanceM: 0.5,
  },
};
