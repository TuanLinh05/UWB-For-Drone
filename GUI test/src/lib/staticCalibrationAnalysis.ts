/**
 * Held-out static calibration evaluation.
 *
 * A range filter can reduce noise but cannot prove/remove a deterministic
 * distance-dependent bias. This module evaluates one scalar offset using
 * leave-one-distance-out (LODO): a held-out distance is never used to fit its
 * correction. It is therefore an acceptance gate for the <= 5 cm static-bias
 * target, not a claim about flight dynamics or unobserved multipath.
 */

export interface StaticCalibrationPointInput {
  trueDistanceM: number;
  rawMm: number[];
}

export interface StaticCalibrationHoldoutPoint {
  trueDistanceM: number;
  sampleCount: number;
  trainingOffsetM: number;
  holdoutBiasMm: number;
  holdoutStdMm: number;
}

export type StaticCalibrationVerdict = 'pass' | 'review' | 'insufficient-data';

export interface StaticCalibrationHoldoutResult {
  points: StaticCalibrationHoldoutPoint[];
  distanceCount: number;
  totalSamples: number;
  meanAbsHoldoutBiasMm: number | null;
  rmsHoldoutBiasMm: number | null;
  p95AbsHoldoutBiasMm: number | null;
  maxAbsHoldoutBiasMm: number | null;
  medianWithinCaptureStdMm: number | null;
  verdict: StaticCalibrationVerdict;
  verdictText: string;
}

export const STATIC_TARGET_BIAS_MM = 50;
const MIN_POINTS = 3;
const MIN_SAMPLES_PER_POINT = 100;

function mean(values: number[]): number {
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}

function standardDeviation(values: number[]): number {
  const average = mean(values);
  return Math.sqrt(mean(values.map(value => (value - average) ** 2)));
}

function percentile(values: number[], fraction: number): number {
  const sorted = [...values].sort((left, right) => left - right);
  return sorted[Math.max(0, Math.ceil(sorted.length * fraction) - 1)];
}

function validPoint(point: StaticCalibrationPointInput): boolean {
  return Number.isFinite(point.trueDistanceM) && point.trueDistanceM > 0
    && point.rawMm.length >= MIN_SAMPLES_PER_POINT
    && point.rawMm.every(value => Number.isFinite(value) && value >= 0);
}

function scalarOffsetM(point: StaticCalibrationPointInput): number {
  return mean(point.rawMm) / 1000 - point.trueDistanceM;
}

export function analyzeStaticCalibrationHoldout(
  input: StaticCalibrationPointInput[],
): StaticCalibrationHoldoutResult {
  const points = input.filter(validPoint).sort((left, right) => left.trueDistanceM - right.trueDistanceM);
  if (points.length < MIN_POINTS) {
    return {
      points: [], distanceCount: points.length,
      totalSamples: points.reduce((count, point) => count + point.rawMm.length, 0),
      meanAbsHoldoutBiasMm: null, rmsHoldoutBiasMm: null,
      p95AbsHoldoutBiasMm: null, maxAbsHoldoutBiasMm: null,
      medianWithinCaptureStdMm: null,
      verdict: 'insufficient-data',
      verdictText: `Need at least ${MIN_POINTS} distinct distances with ${MIN_SAMPLES_PER_POINT}+ valid samples each for held-out static validation.`,
    };
  }

  const holdout = points.map((point, index): StaticCalibrationHoldoutPoint => {
    const trainingOffsetM = mean(points
      .filter((_, candidateIndex) => candidateIndex !== index)
      .map(scalarOffsetM));
    /* This intentionally matches the wizard's unweighted mean-of-distances
     * scalar fit, so a long capture cannot dominate the correction. */
    const residualsMm = point.rawMm.map(rawMm => rawMm - trainingOffsetM * 1000 - point.trueDistanceM * 1000);
    return {
      trueDistanceM: point.trueDistanceM,
      sampleCount: point.rawMm.length,
      trainingOffsetM,
      holdoutBiasMm: mean(residualsMm),
      holdoutStdMm: standardDeviation(residualsMm),
    };
  });

  const absBias = holdout.map(point => Math.abs(point.holdoutBiasMm));
  const withinStd = holdout.map(point => point.holdoutStdMm);
  const p95AbsHoldoutBiasMm = percentile(absBias, 0.95);
  const maxAbsHoldoutBiasMm = Math.max(...absBias);
  const pass = p95AbsHoldoutBiasMm <= STATIC_TARGET_BIAS_MM
    && maxAbsHoldoutBiasMm <= STATIC_TARGET_BIAS_MM;

  return {
    points: holdout,
    distanceCount: points.length,
    totalSamples: points.reduce((count, point) => count + point.rawMm.length, 0),
    meanAbsHoldoutBiasMm: mean(absBias),
    rmsHoldoutBiasMm: Math.sqrt(mean(holdout.map(point => point.holdoutBiasMm ** 2))),
    p95AbsHoldoutBiasMm,
    maxAbsHoldoutBiasMm,
    medianWithinCaptureStdMm: percentile(withinStd, 0.5),
    verdict: pass ? 'pass' : 'review',
    verdictText: pass
      ? `Held-out scalar-offset bias is within the ${STATIC_TARGET_BIAS_MM} mm target at every captured distance. This validates static systematic bias only; it does not certify flight dynamics or multipath outside these conditions.`
      : `Held-out bias exceeds the ${STATIC_TARGET_BIAS_MM} mm static target. Do not add a firmware bias correction yet: repeat outlying distances, check geometry/LOS, and use the FPP analysis as evidence before proposing a bounded correction table.`,
  };
}
