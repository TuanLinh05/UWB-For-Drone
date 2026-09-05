import { invertSymmetric2, symmetricEigen2 } from './matrix2';
import type { RangeObservation2D, SolverConfig, Vec2 } from './types';

export interface GeometryDiagnostics2D {
  g00: number;
  g01: number;
  g11: number;
  conditionNumber: number;
  gdop: number;
  geometryScore: number;
}

export function precheckAnchorGeometry(observations: readonly RangeObservation2D[], config: SolverConfig): boolean {
  if (observations.length < 3) return false;
  const meanX = observations.reduce((sum, observation) => sum + observation.x, 0) / observations.length;
  const meanY = observations.reduce((sum, observation) => sum + observation.y, 0) / observations.length;
  let a = 0;
  let b = 0;
  let d = 0;
  for (const observation of observations) {
    const dx = observation.x - meanX;
    const dy = observation.y - meanY;
    a += dx * dx;
    b += dx * dy;
    d += dy * dy;
  }
  const eigen = symmetricEigen2(a, b, d);
  return eigen !== null && eigen.min > config.geometryEigenFloor
    && eigen.max / eigen.min <= config.maxConditionNumber;
}

export function geometryAtPosition(
  point: Vec2,
  observations: readonly RangeObservation2D[],
  config: SolverConfig,
): GeometryDiagnostics2D | null {
  let g00 = 0;
  let g01 = 0;
  let g11 = 0;
  for (const observation of observations) {
    const dx = point.x - observation.x;
    const dy = point.y - observation.y;
    const distance = Math.hypot(dx, dy);
    if (!Number.isFinite(distance) || distance < config.jacobianEpsilonM) return null;
    const jx = dx / distance;
    const jy = dy / distance;
    g00 += jx * jx;
    g01 += jx * jy;
    g11 += jy * jy;
  }
  const eigen = symmetricEigen2(g00, g01, g11);
  const inverse = invertSymmetric2(
    g00, g01, g11, config.geometryEigenFloor, config.maxConditionNumber,
  );
  if (!eigen || !inverse) return null;
  const conditionNumber = eigen.max / eigen.min;
  const gdop = Math.sqrt(Math.max(0, inverse[0][0] + inverse[1][1]));
  return {
    g00,
    g01,
    g11,
    conditionNumber,
    gdop,
    geometryScore: 1 / (1 + Math.log10(Math.max(1, conditionNumber))),
  };
}
