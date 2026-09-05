import { describe, expect, it } from 'vitest';
import { SHADOW_POSITION_CONFIG } from './position/config';
import { isFiniteSymmetricPsd } from './position/matrix2';
import { buildObservations } from './position/observationBuilder';
import { solveWithConservativeLoo } from './position/leaveOneOut';
import {
  accumulateNormalEquation,
  robustCost,
  solveRobustSubset2D,
} from './position/robustSolver2d';
import type {
  CalibrationRegistry,
  ObservationConfig,
  RangeObservation2D,
} from './position/types';
import type { AnchorPosition, AnchorSample, RangeSample } from './types';

const layout: AnchorPosition[] = [
  { id: 1, x: 0, y: 0 },
  { id: 2, x: 5, y: 0 },
  { id: 3, x: 0, y: 4 },
  { id: 4, x: 5, y: 4 },
];

const calibration: CalibrationRegistry = Object.fromEntries(layout.map(anchor => [anchor.id, {
  DS: { calibrated: true, rawSigmaM: 0.05, filteredSigmaM: 0.04 },
}]));

const observationConfig: ObservationConfig = {
  ...SHADOW_POSITION_CONFIG.observation,
  rangeSource: 'raw',
  defaultRangingMode: 'DS',
};

function ranges(point: { x: number; y: number }, anchors = layout): number[] {
  return anchors.map(anchor => Math.hypot(point.x - anchor.x, point.y - anchor.y));
}

function rangeSample(point = { x: 2.0, y: 1.5 }): RangeSample {
  const records: AnchorSample[] = layout.map((anchor, index) => {
    const mm = ranges(point)[index] * 1000;
    return { id: anchor.id, valid: true, ageMs: index * 4, rawMm: mm, filtMm: mm - 10, fppDbm: -80, status: 0 };
  });
  return {
    seq: 1,
    transportSeq: 1,
    timeMs: 100,
    clientTime: 1000,
    anchors: records,
    anchorsById: Object.fromEntries(records.map(record => [record.id, record])),
  };
}

function observations(point: { x: number; y: number }, sigmaM = 0.05): RangeObservation2D[] {
  return layout.map((anchor, index) => ({
    id: anchor.id,
    x: anchor.x,
    y: anchor.y,
    rangeM: ranges(point)[index],
    rawRangeM: ranges(point)[index],
    sigmaM,
    ageMs: index * 4,
    fppDbm: -80,
    status: 0,
    rangingMode: 'DS',
  }));
}

describe('C6 observation builder', () => {
  it('is ID/order invariant and reports age spread', () => {
    const sample = rangeSample();
    sample.anchors.reverse();
    const result = buildObservations({ sample, layout: [...layout].reverse(), calibration, config: observationConfig });
    expect(result.fatal).toBe(false);
    expect(result.usable.map(observation => observation.id)).toEqual([1, 2, 3, 4]);
    expect(result.ageSpreadMs).toBe(12);
  });

  it('keeps three usable observations when one anchor is invalid', () => {
    const sample = rangeSample();
    sample.anchors[3] = { ...sample.anchors[3], valid: false, rawMm: null, filtMm: null };
    const result = buildObservations({ sample, layout, calibration, config: observationConfig });
    expect(result.usable).toHaveLength(3);
    expect(result.excluded).toContainEqual({ id: 4, reason: 'invalid-flag' });
  });

  it('fails duplicate IDs explicitly instead of last-write-wins', () => {
    const sample = rangeSample();
    sample.anchors.push({ ...sample.anchors[0] });
    const result = buildObservations({ sample, layout, calibration, config: observationConfig });
    expect(result.fatal).toBe(true);
    expect(result.excluded.some(item => item.reason === 'duplicate-id')).toBe(true);
  });

  it('excludes stale, non-positive, over-limit and missing-layout records independently', () => {
    const sample = rangeSample();
    sample.anchors[0] = { ...sample.anchors[0], ageMs: 201 };
    sample.anchors[1] = { ...sample.anchors[1], rawMm: 0 };
    sample.anchors[2] = { ...sample.anchors[2], rawMm: 60_000 };
    const reducedLayout = layout.filter(anchor => anchor.id !== 4);
    const result = buildObservations({ sample, layout: reducedLayout, calibration, config: observationConfig });
    expect(result.excluded.map(item => item.reason)).toEqual([
      'stale', 'non-positive', 'over-physical-limit', 'missing-layout',
    ]);
  });

  it('excludes an uncalibrated A4 and SS fallback without an SS profile', () => {
    const sample = rangeSample();
    sample.anchors[2] = { ...sample.anchors[2], status: 0x10 };
    const registry: CalibrationRegistry = {
      ...calibration,
      4: { DS: { calibrated: false, rawSigmaM: 0.05, filteredSigmaM: 0.04 } },
    };
    const result = buildObservations({ sample, layout, calibration: registry, config: observationConfig });
    expect(result.excluded).toContainEqual({ id: 3, reason: 'wrong-calibration-mode' });
    expect(result.excluded).toContainEqual({ id: 4, reason: 'missing-calibration' });
  });

  it('selects the configured field and keeps sigma multipliers bounded', () => {
    const sample = rangeSample();
    sample.anchors[0] = { ...sample.anchors[0], ageMs: 200, fppDbm: -110 };
    const filtered = buildObservations({
      sample,
      layout,
      calibration,
      config: { ...observationConfig, rangeSource: 'filtered' },
    });
    expect(filtered.usable[0].rangeM).toBeCloseTo((sample.anchorsById[1].filtMm as number) / 1000, 12);
    expect(filtered.usable[0].sigmaM).toBeLessThanOrEqual(observationConfig.sigmaCeilingM);
    expect(filtered.usable[0].sigmaM).toBeGreaterThanOrEqual(observationConfig.sigmaFloorM);
  });
});

describe('C6 robust LM solver', () => {
  const config = SHADOW_POSITION_CONFIG.solver;
  const target = { x: 2.1, y: 1.3 };

  it('solves perfect four-anchor and every valid three-anchor combination', () => {
    const full = observations(target);
    const solved = solveRobustSubset2D(full, undefined, config);
    expect(solved.ok).toBe(true);
    if (solved.ok) {
      expect(solved.result.x).toBeCloseTo(target.x, 8);
      expect(solved.result.y).toBeCloseTo(target.y, 8);
      expect(isFiniteSymmetricPsd(solved.result.covarianceM2)).toBe(true);
      expect(solved.result.covarianceM2[0][0]).toBeGreaterThan(0);
    }
    for (const removed of full) {
      const subset = full.filter(observation => observation.id !== removed.id);
      const result = solveRobustSubset2D(subset, undefined, config);
      expect(result.ok, `subset without A${removed.id}`).toBe(true);
      if (result.ok) {
        expect(result.result.x).toBeCloseTo(target.x, 7);
        expect(result.result.y).toBeCloseTo(target.y, 7);
      }
    }
  });

  it('is permutation and translation invariant', () => {
    const original = solveRobustSubset2D(observations(target), undefined, config);
    const translated = observations(target).map(observation => ({
      ...observation,
      x: observation.x + 1000,
      y: observation.y - 500,
    })).reverse();
    const moved = solveRobustSubset2D(translated, undefined, config);
    expect(original.ok && moved.ok).toBe(true);
    if (original.ok && moved.ok) {
      expect(moved.result.x - 1000).toBeCloseTo(original.result.x, 7);
      expect(moved.result.y + 500).toBeCloseTo(original.result.y, 7);
      expect(moved.result.covarianceM2[0][0]).toBeCloseTo(original.result.covarianceM2[0][0], 8);
    }
  });

  it('returns bad geometry for collinear anchors', () => {
    const collinear = observations(target).slice(0, 3).map((observation, index) => ({
      ...observation,
      x: index * 2,
      y: 0,
    }));
    expect(solveRobustSubset2D(collinear, undefined, config)).toMatchObject({ ok: false, reason: 'bad-geometry' });
  });

  it('LOO rejects one gross outlier and refuses two ambiguous outliers', () => {
    const oneBad = observations(target);
    oneBad[3] = { ...oneBad[3], rangeM: oneBad[3].rangeM + 1.0 };
    const recovered = solveWithConservativeLoo(oneBad, undefined, config);
    expect(recovered.ok).toBe(true);
    if (recovered.ok) {
      expect(recovered.result.rejectedAnchorIds).toEqual([4]);
      expect(recovered.result.x).toBeCloseTo(target.x, 5);
      expect(recovered.result.y).toBeCloseTo(target.y, 5);
    }

    const twoBad = observations(target);
    twoBad[2] = { ...twoBad[2], rangeM: twoBad[2].rangeM + 0.8 };
    twoBad[3] = { ...twoBad[3], rangeM: twoBad[3].rangeM - 0.8 };
    const ambiguous = solveWithConservativeLoo(twoBad, undefined, config);
    expect(ambiguous).toMatchObject({ ok: false, reason: 'ambiguous-outlier' });
  });

  it('has an analytic gradient consistent with finite difference', () => {
    const obs = observations(target);
    const point = { x: 2.3, y: 1.1 };
    const system = accumulateNormalEquation(point, obs, config);
    expect(system).not.toBeNull();
    if (!system) return;
    const epsilon = 1e-6;
    const gradientX = (robustCost({ x: point.x + epsilon, y: point.y }, obs, config)
      - robustCost({ x: point.x - epsilon, y: point.y }, obs, config)) / (2 * epsilon);
    const gradientY = (robustCost({ x: point.x, y: point.y + epsilon }, obs, config)
      - robustCost({ x: point.x, y: point.y - epsilon }, obs, config)) / (2 * epsilon);
    expect(system.g0).toBeCloseTo(gradientX, 4);
    expect(system.g1).toBeCloseTo(gradientY, 4);
  });
});
