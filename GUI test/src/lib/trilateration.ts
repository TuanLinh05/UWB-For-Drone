import type { AnchorPosition } from './types';

export const DEFAULT_MAX_TRILATERATION_CONDITION = 10_000;

export type TrilaterationFailureReason =
  | 'insufficient-observations'
  | 'length-mismatch'
  | 'non-finite-input'
  | 'invalid-distance'
  | 'duplicate-anchor-id'
  | 'duplicate-anchor-position'
  | 'degenerate-geometry'
  | 'ill-conditioned-geometry'
  | 'non-finite-solution';

export interface TrilaterationQuality {
  /** Condition number of the centered geometry matrix A (not A^T A). */
  conditionNumber: number;
  /** RMS error between the solved point and the measured ranges. */
  residualRmsM: number;
  usedAnchorIds: number[];
}

export type TrilaterationResult =
  | {
      ok: true;
      point: { x: number; y: number };
      quality: TrilaterationQuality;
    }
  | {
      ok: false;
      reason: TrilaterationFailureReason;
      usedAnchorIds: number[];
      conditionNumber?: number;
    };

export interface TrilaterationOptions {
  maxConditionNumber?: number;
}

/**
 * Solve a 2D multilateration problem using all observations.
 *
 * Subtracting the mean range equation removes the common x^2 + y^2 term
 * without choosing one anchor as a privileged reference. The resulting
 * overdetermined system is solved by least squares, so N > 3 anchors are
 * supported naturally.
 */
export function solveTrilateration2DDetailed(
  anchors: AnchorPosition[],
  distancesM: number[],
  options: TrilaterationOptions = {},
): TrilaterationResult {
  const usedAnchorIds = anchors.map(anchor => anchor.id);

  if (anchors.length < 3 || distancesM.length < 3) {
    return { ok: false, reason: 'insufficient-observations', usedAnchorIds };
  }
  if (anchors.length !== distancesM.length) {
    return { ok: false, reason: 'length-mismatch', usedAnchorIds };
  }

  const seenIds = new Set<number>();
  for (let index = 0; index < anchors.length; index++) {
    const anchor = anchors[index];
    const distance = distancesM[index];
    if (
      !Number.isFinite(anchor.id)
      || !Number.isFinite(anchor.x)
      || !Number.isFinite(anchor.y)
      || !Number.isFinite(distance)
    ) {
      return { ok: false, reason: 'non-finite-input', usedAnchorIds };
    }
    if (distance <= 0) {
      return { ok: false, reason: 'invalid-distance', usedAnchorIds };
    }
    if (seenIds.has(anchor.id)) {
      return { ok: false, reason: 'duplicate-anchor-id', usedAnchorIds };
    }
    seenIds.add(anchor.id);
  }

  // Coordinate centering below makes the solve translation-invariant. Use a
  // scale-aware tolerance here so coincident anchors cannot masquerade as
  // independent observations due to floating-point noise.
  const xs = anchors.map(anchor => anchor.x);
  const ys = anchors.map(anchor => anchor.y);
  const coordinateScale = Math.max(
    1,
    Math.max(...xs) - Math.min(...xs),
    Math.max(...ys) - Math.min(...ys),
  );
  const duplicateTolerance = coordinateScale * 1e-9;
  for (let i = 0; i < anchors.length; i++) {
    for (let j = i + 1; j < anchors.length; j++) {
      if (Math.hypot(anchors[i].x - anchors[j].x, anchors[i].y - anchors[j].y) <= duplicateTolerance) {
        return { ok: false, reason: 'duplicate-anchor-position', usedAnchorIds };
      }
    }
  }

  const count = anchors.length;
  let meanX = 0;
  let meanY = 0;
  for (const anchor of anchors) {
    meanX += anchor.x;
    meanY += anchor.y;
  }
  meanX /= count;
  meanY /= count;

  // Work in coordinates relative to the anchor centroid. Besides making the
  // equations symmetric, this avoids catastrophic cancellation for layouts
  // expressed in a large global coordinate system.
  let meanQ = 0;
  const localX = new Array<number>(count);
  const localY = new Array<number>(count);
  const q = new Array<number>(count);
  for (let i = 0; i < count; i++) {
    localX[i] = anchors[i].x - meanX;
    localY[i] = anchors[i].y - meanY;
    q[i] = localX[i] * localX[i] + localY[i] * localY[i] - distancesM[i] * distancesM[i];
    meanQ += q[i];
  }
  meanQ /= count;

  let ata00 = 0;
  let ata01 = 0;
  let ata11 = 0;
  let atb0 = 0;
  let atb1 = 0;
  for (let i = 0; i < count; i++) {
    const a0 = 2 * localX[i];
    const a1 = 2 * localY[i];
    const b = q[i] - meanQ;
    ata00 += a0 * a0;
    ata01 += a0 * a1;
    ata11 += a1 * a1;
    atb0 += a0 * b;
    atb1 += a1 * b;
  }

  // Eigenvalues of the symmetric 2x2 normal matrix give a stable rank and
  // conditioning check before inversion.
  const trace = ata00 + ata11;
  const discriminant = Math.hypot(ata00 - ata11, 2 * ata01);
  const lambdaMax = (trace + discriminant) / 2;
  const lambdaMin = (trace - discriminant) / 2;
  if (!Number.isFinite(lambdaMax) || !Number.isFinite(lambdaMin)) {
    return { ok: false, reason: 'non-finite-input', usedAnchorIds };
  }
  if (lambdaMax <= Number.EPSILON || lambdaMin <= lambdaMax * Number.EPSILON * 32) {
    return { ok: false, reason: 'degenerate-geometry', usedAnchorIds };
  }

  const conditionNumber = Math.sqrt(lambdaMax / lambdaMin);
  const configuredLimit = options.maxConditionNumber ?? DEFAULT_MAX_TRILATERATION_CONDITION;
  const maxConditionNumber = Number.isFinite(configuredLimit) && configuredLimit > 1
    ? configuredLimit
    : DEFAULT_MAX_TRILATERATION_CONDITION;
  if (conditionNumber > maxConditionNumber) {
    return {
      ok: false,
      reason: 'ill-conditioned-geometry',
      usedAnchorIds,
      conditionNumber,
    };
  }

  const determinant = ata00 * ata11 - ata01 * ata01;
  if (!Number.isFinite(determinant) || determinant <= 0) {
    return { ok: false, reason: 'degenerate-geometry', usedAnchorIds, conditionNumber };
  }

  const x = meanX + (ata11 * atb0 - ata01 * atb1) / determinant;
  const y = meanY + (ata00 * atb1 - ata01 * atb0) / determinant;
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    return { ok: false, reason: 'non-finite-solution', usedAnchorIds, conditionNumber };
  }

  let residualSquaredSum = 0;
  for (let i = 0; i < count; i++) {
    const predictedDistance = Math.hypot(x - anchors[i].x, y - anchors[i].y);
    const residual = predictedDistance - distancesM[i];
    residualSquaredSum += residual * residual;
  }

  return {
    ok: true,
    point: { x, y },
    quality: {
      conditionNumber,
      residualRmsM: Math.sqrt(residualSquaredSum / count),
      usedAnchorIds,
    },
  };
}

/**
 * Backward-compatible point-only API. Invalid or unsafe geometry now returns
 * null instead of fabricating the anchor centroid.
 */
export function solveTrilateration2D(
  anchors: AnchorPosition[],
  distancesM: number[],
): { x: number; y: number } | null {
  const result = solveTrilateration2DDetailed(anchors, distancesM);
  return result.ok ? result.point : null;
}
