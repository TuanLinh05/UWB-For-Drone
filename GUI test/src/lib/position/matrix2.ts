import type { Mat2 } from './types';

export interface SymmetricEigen2 {
  min: number;
  max: number;
  angleRad: number;
}

export function symmetricEigen2(a: number, b: number, d: number): SymmetricEigen2 | null {
  if (![a, b, d].every(Number.isFinite)) return null;
  const trace = a + d;
  const delta = Math.hypot(a - d, 2 * b);
  const max = (trace + delta) / 2;
  const min = (trace - delta) / 2;
  if (!Number.isFinite(min) || !Number.isFinite(max)) return null;
  return { min, max, angleRad: 0.5 * Math.atan2(2 * b, a - d) };
}

export function invertSymmetric2(
  a: number,
  b: number,
  d: number,
  eigenFloor: number,
  maxConditionNumber: number,
): Mat2 | null {
  const eigen = symmetricEigen2(a, b, d);
  if (!eigen || eigen.min <= eigenFloor || eigen.max <= 0
    || eigen.max / eigen.min > maxConditionNumber) return null;
  const determinant = a * d - b * b;
  if (!Number.isFinite(determinant) || determinant <= 0) return null;
  return [[d / determinant, -b / determinant], [-b / determinant, a / determinant]];
}

export function clampSymmetricEigenvalues(
  matrix: Mat2,
  floor: number,
  ceiling: number,
): Mat2 | null {
  const a = matrix[0][0];
  const b = (matrix[0][1] + matrix[1][0]) / 2;
  const d = matrix[1][1];
  const eigen = symmetricEigen2(a, b, d);
  if (!eigen || floor <= 0 || ceiling < floor) return null;
  const lambdaMax = Math.max(floor, Math.min(ceiling, eigen.max));
  const lambdaMin = Math.max(floor, Math.min(ceiling, eigen.min));
  const c = Math.cos(eigen.angleRad);
  const s = Math.sin(eigen.angleRad);
  const outA = c * c * lambdaMax + s * s * lambdaMin;
  const outB = c * s * (lambdaMax - lambdaMin);
  const outD = s * s * lambdaMax + c * c * lambdaMin;
  return [
    [outA, outB],
    [outB, outD],
  ];
}

export function isFiniteSymmetricPsd(matrix: Mat2, tolerance = 1e-12): boolean {
  const b = (matrix[0][1] + matrix[1][0]) / 2;
  if (![matrix[0][0], matrix[0][1], matrix[1][0], matrix[1][1]].every(Number.isFinite)) return false;
  if (Math.abs(matrix[0][1] - matrix[1][0]) > tolerance) return false;
  const eigen = symmetricEigen2(matrix[0][0], b, matrix[1][1]);
  return eigen !== null && eigen.min >= -tolerance;
}
