import { robustCost, solveRobustSubset2D } from './robustSolver2d';
import type {
  PositionSolveOutcome,
  PositionSolveResult2D,
  RangeObservation2D,
  SolverConfig,
  Vec2,
} from './types';

interface LooCandidate {
  removedId: number;
  result: PositionSolveResult2D;
  heldOutAbsZ: number;
  subsetMaxAbsZ: number;
  evidenceScore: number;
  commonCost: number;
}

function maxAbsoluteResidual(result: PositionSolveResult2D): number {
  return result.residuals.reduce(
    (maximum, residual) => Math.max(maximum, Math.abs(residual.standardizedResidual)), 0,
  );
}

function fullPassesQuality(result: PositionSolveResult2D, config: SolverConfig): boolean {
  return maxAbsoluteResidual(result) <= config.looTriggerZ
    && result.normalizedChi2 <= config.maxReducedChi2;
}

function heldOutStandardizedResidual(
  result: PositionSolveResult2D,
  heldOut: RangeObservation2D,
): number {
  return (Math.hypot(result.x - heldOut.x, result.y - heldOut.y) - heldOut.rangeM) / heldOut.sigmaM;
}

/**
 * Conservative C6 LOO. It evaluates all four subsets during shadow validation,
 * requires the winner to match the full-fit largest residual, and returns an
 * explicit ambiguous failure whenever the evidence margin is insufficient.
 */
export function solveWithConservativeLoo(
  observations: readonly RangeObservation2D[],
  seed: Vec2 | undefined,
  config: SolverConfig,
): PositionSolveOutcome {
  const full = solveRobustSubset2D(observations, seed, config);
  if (full.ok && fullPassesQuality(full.result, config)) return full;
  if (observations.length !== 4) {
    return full.ok ? { ok: false, reason: 'ambiguous-outlier' } : full;
  }

  const suspected = full.ok
    ? [...full.result.residuals]
      .sort((left, right) => Math.abs(right.standardizedResidual) - Math.abs(left.standardizedResidual))[0]
    : undefined;
  if (full.ok && (!suspected || Math.abs(suspected.standardizedResidual) < config.looTriggerZ)) {
    return { ok: false, reason: 'ambiguous-outlier' };
  }

  const candidates: LooCandidate[] = [];
  for (const heldOut of observations) {
    const subset = observations.filter(observation => observation.id !== heldOut.id);
    const solved = solveRobustSubset2D(subset, full.ok ? full.result : seed, config);
    if (!solved.ok || solved.result.normalizedChi2 > config.maxReducedChi2) continue;
    const heldOutAbsZ = Math.abs(heldOutStandardizedResidual(solved.result, heldOut));
    const subsetMaxAbsZ = maxAbsoluteResidual(solved.result);
    const evidenceScore = heldOutAbsZ - subsetMaxAbsZ;
    const commonCost = robustCost(solved.result, observations, config);
    if (![heldOutAbsZ, subsetMaxAbsZ, evidenceScore, commonCost].every(Number.isFinite)) continue;
    if (heldOutAbsZ < config.looRejectZ) continue;
    if (subsetMaxAbsZ > config.looTriggerZ) continue;
    candidates.push({
      removedId: heldOut.id,
      result: solved.result,
      heldOutAbsZ,
      subsetMaxAbsZ,
      evidenceScore,
      commonCost,
    });
  }

  candidates.sort((left, right) => right.evidenceScore - left.evidenceScore
    || left.commonCost - right.commonCost || left.removedId - right.removedId);
  const winner = candidates[0];
  const runnerUp = candidates[1];
  if (!winner || (suspected && winner.removedId !== suspected.id)) {
    return { ok: false, reason: 'ambiguous-outlier' };
  }
  if (runnerUp && winner.evidenceScore - runnerUp.evidenceScore < config.looWinnerMargin) {
    return { ok: false, reason: 'ambiguous-outlier' };
  }
  if (full.ok) {
    const fullMaxAbsZ = maxAbsoluteResidual(full.result);
    if (fullMaxAbsZ - winner.subsetMaxAbsZ < config.looWinnerMargin) {
      return { ok: false, reason: 'ambiguous-outlier' };
    }
  }

  return {
    ok: true,
    result: {
      ...winner.result,
      mode: 'three-after-loo',
      rejectedAnchorIds: [winner.removedId],
    },
  };
}
