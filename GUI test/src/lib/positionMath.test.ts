import { KalmanFilter2D } from './kalman2d';
import { elapsedMcuMilliseconds } from './positionTiming';
import { solveTrilateration2DDetailed } from './trilateration';
import type { AnchorPosition } from './types';

function assert(condition: boolean, message: string): asserts condition {
  if (!condition) throw new Error(`Position math self-test failed: ${message}`);
}

function assertNear(actual: number, expected: number, tolerance: number, message: string) {
  assert(Math.abs(actual - expected) <= tolerance, `${message}: expected ${expected}, got ${actual}`);
}

function rangesFrom(point: { x: number; y: number }, anchors: AnchorPosition[]) {
  return anchors.map(anchor => Math.hypot(point.x - anchor.x, point.y - anchor.y));
}

export function runPositionMathSelfTests() {
  const target = { x: 2.1, y: 1.3 };
  const anchors: AnchorPosition[] = [
    { id: 4, x: 5, y: 4 },
    { id: 1, x: 0, y: 0 },
    { id: 3, x: 0, y: 4 },
    { id: 2, x: 5, y: 0 },
  ];
  const fourAnchorResult = solveTrilateration2DDetailed(anchors, rangesFrom(target, anchors));
  assert(fourAnchorResult.ok, 'four-anchor least-squares solution should succeed');
  assertNear(fourAnchorResult.point.x, target.x, 1e-9, 'four-anchor x');
  assertNear(fourAnchorResult.point.y, target.y, 1e-9, 'four-anchor y');
  assert(fourAnchorResult.quality.usedAnchorIds.join(',') === '4,1,3,2', 'solver must preserve used IDs');

  const translatedAnchors = anchors.map(anchor => ({
    ...anchor,
    x: anchor.x + 1_000_000_000,
    y: anchor.y - 1_000_000_000,
  }));
  const translatedTarget = { x: target.x + 1_000_000_000, y: target.y - 1_000_000_000 };
  const translatedResult = solveTrilateration2DDetailed(
    translatedAnchors,
    rangesFrom(translatedTarget, translatedAnchors),
  );
  assert(translatedResult.ok, 'large translated coordinates should remain numerically safe');
  assertNear(translatedResult.point.x, translatedTarget.x, 1e-6, 'translated x');
  assertNear(translatedResult.point.y, translatedTarget.y, 1e-6, 'translated y');

  const collinear: AnchorPosition[] = [
    { id: 1, x: 0, y: 0 },
    { id: 2, x: 2, y: 0 },
    { id: 3, x: 4, y: 0 },
  ];
  const collinearResult = solveTrilateration2DDetailed(collinear, [1, 1, 3]);
  assert(!collinearResult.ok && collinearResult.reason === 'degenerate-geometry',
    'collinear geometry must fail instead of returning a centroid');

  const duplicateId = solveTrilateration2DDetailed(
    [{ id: 1, x: 0, y: 0 }, { id: 1, x: 4, y: 0 }, { id: 3, x: 0, y: 4 }],
    [1, 3, 3],
  );
  assert(!duplicateId.ok && duplicateId.reason === 'duplicate-anchor-id', 'duplicate IDs must fail');

  const nearCollinear: AnchorPosition[] = [
    { id: 1, x: 0, y: 0 },
    { id: 2, x: 2, y: 0 },
    { id: 3, x: 4, y: 0.00001 },
  ];
  const conditionedResult = solveTrilateration2DDetailed(nearCollinear, [1, 1, 3]);
  assert(!conditionedResult.ok && conditionedResult.reason === 'ill-conditioned-geometry',
    'unsafe geometry condition must fail');

  assert(elapsedMcuMilliseconds(0x10, 0xfffffff0) === 32, 'uint32 timestamp wrap must be handled');
  assert(elapsedMcuMilliseconds(90, 100) === null, 'out-of-order timestamp must be rejected');

  const filter = new KalmanFilter2D();
  const first = filter.update(target.x, target.y);
  assertNear(first.x, target.x, 1e-12, 'Kalman must initialize at first x measurement');
  assertNear(first.y, target.y, 1e-12, 'Kalman must initialize at first y measurement');
  const second = filter.update(target.x + 0.1, target.y + 0.1, 0.04);
  assert(Number.isFinite(second.x) && Number.isFinite(second.y), 'Kalman update with dynamic dt must stay finite');
  const dynamics = filter as unknown as {
    F: { get(row: number, column: number): number };
    Q: { get(row: number, column: number): number };
  };
  assertNear(dynamics.F.get(0, 2), 0.04, 1e-12, 'F must be rebuilt for a new dt');
  assertNear(dynamics.Q.get(2, 2), 0.04 * 0.04 * filter.processNoise, 1e-12,
    'Q must be rebuilt for the same new dt');
}
