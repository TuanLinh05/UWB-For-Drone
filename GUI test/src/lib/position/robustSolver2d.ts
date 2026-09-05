import { clampSymmetricEigenvalues, invertSymmetric2 } from './matrix2';
import { geometryAtPosition, precheckAnchorGeometry } from './geometry2d';
import type {
  AnchorResidual,
  PositionSolveOutcome,
  PositionSolveResult2D,
  RangeObservation2D,
  SolverConfig,
  Vec2,
} from './types';

export interface NormalEquation2D {
  h00: number;
  h01: number;
  h11: number;
  g0: number;
  g1: number;
  robustCost: number;
}

export function huberWeight(z: number, k: number): number {
  const absolute = Math.abs(z);
  return absolute <= k ? 1 : k / absolute;
}

export function huberLoss(z: number, k: number): number {
  const absolute = Math.abs(z);
  return absolute <= k ? 0.5 * z * z : k * (absolute - 0.5 * k);
}

function directionAt(point: Vec2, observation: RangeObservation2D, epsilon: number) {
  let dx = point.x - observation.x;
  let dy = point.y - observation.y;
  let predictedM = Math.hypot(dx, dy);
  if (predictedM < epsilon) {
    const angle = (Math.abs(observation.id) + 1) * 2.399963229728653;
    dx = epsilon * Math.cos(angle);
    dy = epsilon * Math.sin(angle);
    predictedM = epsilon;
  }
  return { dx, dy, predictedM };
}

export function accumulateNormalEquation(
  point: Vec2,
  observations: readonly RangeObservation2D[],
  config: SolverConfig,
): NormalEquation2D | null {
  let h00 = 0;
  let h01 = 0;
  let h11 = 0;
  let g0 = 0;
  let g1 = 0;
  let robustCost = 0;
  for (const observation of observations) {
    if (![observation.x, observation.y, observation.rangeM, observation.sigmaM].every(Number.isFinite)
      || observation.rangeM <= 0 || observation.sigmaM <= 0) return null;
    const { dx, dy, predictedM } = directionAt(point, observation, config.jacobianEpsilonM);
    if (!Number.isFinite(predictedM)) return null;
    const residualM = predictedM - observation.rangeM;
    const z = residualM / observation.sigmaM;
    const weight = huberWeight(z, config.huberK) / (observation.sigmaM * observation.sigmaM);
    const jx = dx / predictedM;
    const jy = dy / predictedM;
    h00 += weight * jx * jx;
    h01 += weight * jx * jy;
    h11 += weight * jy * jy;
    g0 += weight * jx * residualM;
    g1 += weight * jy * residualM;
    robustCost += huberLoss(z, config.huberK);
  }
  return [h00, h01, h11, g0, g1, robustCost].every(Number.isFinite)
    ? { h00, h01, h11, g0, g1, robustCost }
    : null;
}

export function robustCost(
  point: Vec2,
  observations: readonly RangeObservation2D[],
  config: SolverConfig,
): number {
  let cost = 0;
  for (const observation of observations) {
    const predictedM = Math.hypot(point.x - observation.x, point.y - observation.y);
    if (!Number.isFinite(predictedM) || !Number.isFinite(observation.sigmaM) || observation.sigmaM <= 0) {
      return Number.NaN;
    }
    cost += huberLoss((predictedM - observation.rangeM) / observation.sigmaM, config.huberK);
  }
  return cost;
}

function weightedCentroidSeed(observations: readonly RangeObservation2D[]): Vec2 | null {
  let sumWeight = 0;
  let x = 0;
  let y = 0;
  for (const observation of observations) {
    const weight = 1 / (observation.sigmaM * observation.sigmaM);
    if (!Number.isFinite(weight)) return null;
    sumWeight += weight;
    x += weight * observation.x;
    y += weight * observation.y;
  }
  return sumWeight > 0 ? { x: x / sumWeight, y: y / sumWeight } : null;
}

export function algebraicSeed2D(observations: readonly RangeObservation2D[], config: SolverConfig): Vec2 | null {
  if (observations.length < 3 || !precheckAnchorGeometry(observations, config)) return null;
  const meanX = observations.reduce((sum, observation) => sum + observation.x, 0) / observations.length;
  const meanY = observations.reduce((sum, observation) => sum + observation.y, 0) / observations.length;
  const q = observations.map(observation => {
    const x = observation.x - meanX;
    const y = observation.y - meanY;
    return x * x + y * y - observation.rangeM * observation.rangeM;
  });
  const meanQ = q.reduce((sum, value) => sum + value, 0) / q.length;
  let a00 = 0;
  let a01 = 0;
  let a11 = 0;
  let b0 = 0;
  let b1 = 0;
  observations.forEach((observation, index) => {
    const ax = 2 * (observation.x - meanX);
    const ay = 2 * (observation.y - meanY);
    const b = q[index] - meanQ;
    a00 += ax * ax;
    a01 += ax * ay;
    a11 += ay * ay;
    b0 += ax * b;
    b1 += ay * b;
  });
  const inverse = invertSymmetric2(a00, a01, a11, config.geometryEigenFloor, config.maxConditionNumber);
  if (!inverse) return null;
  const x = meanX + inverse[0][0] * b0 + inverse[0][1] * b1;
  const y = meanY + inverse[1][0] * b0 + inverse[1][1] * b1;
  return Number.isFinite(x) && Number.isFinite(y) ? { x, y } : null;
}

function finalizeSolve(
  point: Vec2,
  observations: readonly RangeObservation2D[],
  iterations: number,
  config: SolverConfig,
): PositionSolveOutcome {
  const geometry = geometryAtPosition(point, observations, config);
  if (!geometry) return { ok: false, reason: 'bad-geometry' };
  let h00 = 0;
  let h01 = 0;
  let h11 = 0;
  let residualSquared = 0;
  let standardizedSquared = 0;
  let inlierStandardizedSquared = 0;
  let inlierCount = 0;
  const residuals: AnchorResidual[] = [];
  for (const observation of observations) {
    const { dx, dy, predictedM } = directionAt(point, observation, config.jacobianEpsilonM);
    const residualM = predictedM - observation.rangeM;
    const z = residualM / observation.sigmaM;
    const effectiveWeightPerM2 = huberWeight(z, config.huberK)
      / (observation.sigmaM * observation.sigmaM);
    const jx = dx / predictedM;
    const jy = dy / predictedM;
    h00 += effectiveWeightPerM2 * jx * jx;
    h01 += effectiveWeightPerM2 * jx * jy;
    h11 += effectiveWeightPerM2 * jy * jy;
    residualSquared += residualM * residualM;
    standardizedSquared += z * z;
    if (Math.abs(z) <= config.huberK) {
      inlierStandardizedSquared += z * z;
      inlierCount++;
    }
    residuals.push({ id: observation.id, residualM, standardizedResidual: z, effectiveWeightPerM2 });
  }
  const weightedInverse = invertSymmetric2(
    h00, h01, h11, config.geometryEigenFloor, config.maxConditionNumber,
  );
  if (!weightedInverse) return { ok: false, reason: 'bad-geometry' };
  const dof = Math.max(1, inlierCount - 2);
  const residualScale = Math.max(1, inlierStandardizedSquared / dof);
  const covariance = clampSymmetricEigenvalues([
    [weightedInverse[0][0] * residualScale, weightedInverse[0][1] * residualScale],
    [weightedInverse[1][0] * residualScale, weightedInverse[1][1] * residualScale],
  ], config.covarianceEigenFloorM2, config.covarianceEigenCeilingM2);
  if (!covariance) return { ok: false, reason: 'non-finite' };
  const ages = observations.map(observation => observation.ageMs);
  const result: PositionSolveResult2D = {
    ...point,
    covarianceM2: covariance,
    rmsResidualM: Math.sqrt(residualSquared / observations.length),
    normalizedChi2: standardizedSquared / Math.max(1, observations.length - 2),
    gdop: geometry.gdop,
    conditionNumber: geometry.conditionNumber,
    geometryScore: geometry.geometryScore,
    usedAnchorIds: observations.map(observation => observation.id).sort((a, b) => a - b),
    rejectedAnchorIds: [],
    residuals: residuals.sort((left, right) => left.id - right.id),
    mode: observations.length === 4 ? 'four-anchor' : 'three-valid-fallback',
    iterations,
    ageSpreadMs: ages.length > 0 ? Math.max(...ages) - Math.min(...ages) : 0,
  };
  return Object.values(point).every(Number.isFinite)
    ? { ok: true, result }
    : { ok: false, reason: 'non-finite' };
}

export function solveRobustSubset2D(
  observations: readonly RangeObservation2D[],
  requestedSeed: Vec2 | undefined,
  config: SolverConfig,
): PositionSolveOutcome {
  if (observations.length < 3) return { ok: false, reason: 'insufficient-observations' };
  if (!precheckAnchorGeometry(observations, config)) return { ok: false, reason: 'bad-geometry' };
  const fallback = algebraicSeed2D(observations, config) ?? weightedCentroidSeed(observations);
  const pointSeed = requestedSeed && Number.isFinite(requestedSeed.x) && Number.isFinite(requestedSeed.y)
    ? requestedSeed
    : fallback;
  if (!pointSeed) return { ok: false, reason: 'non-finite' };

  let point = { ...pointSeed };
  let lambda = config.initialLambda;
  let acceptedSteps = 0;
  for (let iteration = 0; iteration < config.maxIterations; iteration++) {
    const system = accumulateNormalEquation(point, observations, config);
    if (!system) return { ok: false, reason: 'non-finite' };
    if (Math.hypot(system.g0, system.g1) <= config.gradientTolerance) {
      return finalizeSolve(point, observations, iteration + 1, config);
    }
    const damped00 = system.h00 + lambda * Math.max(system.h00, config.geometryEigenFloor);
    const damped11 = system.h11 + lambda * Math.max(system.h11, config.geometryEigenFloor);
    const inverse = invertSymmetric2(
      damped00, system.h01, damped11, config.geometryEigenFloor, config.maxConditionNumber,
    );
    if (!inverse) {
      lambda = Math.min(config.maxLambda, lambda * config.lambdaUpFactor);
      continue;
    }
    const dx = -(inverse[0][0] * system.g0 + inverse[0][1] * system.g1);
    const dy = -(inverse[1][0] * system.g0 + inverse[1][1] * system.g1);
    if (!Number.isFinite(dx) || !Number.isFinite(dy)) return { ok: false, reason: 'non-finite' };
    const candidate = { x: point.x + dx, y: point.y + dy };
    const candidateCost = robustCost(candidate, observations, config);
    if (!Number.isFinite(candidateCost)) return { ok: false, reason: 'non-finite' };
    const improvement = system.robustCost - candidateCost;
    if (improvement > config.costTolerance) {
      point = candidate;
      acceptedSteps++;
      lambda = Math.max(config.minLambda, lambda * config.lambdaDownFactor);
      if (Math.hypot(dx, dy) <= config.stepToleranceM || improvement <= config.costTolerance * 10) {
        return finalizeSolve(point, observations, iteration + 1, config);
      }
    } else if (Math.hypot(dx, dy) <= config.stepToleranceM
      && candidateCost <= system.robustCost + config.costTolerance) {
      return finalizeSolve(point, observations, iteration + 1, config);
    } else {
      lambda = Math.min(config.maxLambda, lambda * config.lambdaUpFactor);
    }
  }
  return {
    ok: false,
    reason: 'non-convergent',
    diagnostics: { acceptedSteps, finalCost: robustCost(point, observations, config) },
  };
}
