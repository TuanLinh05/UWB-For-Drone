import { describe, expect, it } from 'vitest';
import { analyzeStaticCalibrationHoldout } from './staticCalibrationAnalysis';

function samples(meanMm: number, count = 120): number[] {
  return Array.from({ length: count }, (_, index) => meanMm + [-12, -4, 0, 5, 11][index % 5]);
}

describe('static calibration holdout analysis', () => {
  it('passes a stable scalar offset on held-out distances', () => {
    const result = analyzeStaticCalibrationHoldout([
      { trueDistanceM: 1, rawMm: samples(1150) },
      { trueDistanceM: 3, rawMm: samples(3150) },
      { trueDistanceM: 5, rawMm: samples(5150) },
    ]);
    expect(result.verdict).toBe('pass');
    expect(result.maxAbsHoldoutBiasMm).toBeLessThanOrEqual(1);
    expect(result.points).toHaveLength(3);
  });

  it('flags distance-dependent bias instead of hiding it in an average offset', () => {
    const result = analyzeStaticCalibrationHoldout([
      { trueDistanceM: 1, rawMm: samples(1010) },
      { trueDistanceM: 3, rawMm: samples(3100) },
      { trueDistanceM: 5, rawMm: samples(5250) },
    ]);
    expect(result.verdict).toBe('review');
    expect(result.maxAbsHoldoutBiasMm).toBeGreaterThan(50);
  });

  it('does not claim an accuracy result with fewer than three distances', () => {
    const result = analyzeStaticCalibrationHoldout([
      { trueDistanceM: 1, rawMm: samples(1100) },
      { trueDistanceM: 3, rawMm: samples(3100) },
    ]);
    expect(result.verdict).toBe('insufficient-data');
  });
});
