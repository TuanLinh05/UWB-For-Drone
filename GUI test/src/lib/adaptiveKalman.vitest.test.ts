import { describe, expect, it } from 'vitest';
import { AdaptiveKalmanFilter2D, whiteAccelerationQ } from './position/adaptiveKalman2d';
import { SHADOW_POSITION_CONFIG } from './position/config';
import { McuClock } from './position/mcuClock';
import { PositionPipeline } from './position/positionPipeline';
import type {
  AdaptiveKalmanConfig,
  CalibrationRegistry,
  Mat2,
  PositionMeasurement2D,
} from './position/types';
import type { AnchorPosition, AnchorSample, RangeSample } from './types';

const config: AdaptiveKalmanConfig = SHADOW_POSITION_CONFIG.kalman;
const smallCovariance: Mat2 = [[0.0025, 0], [0, 0.0025]];

const measurement = (
  x: number,
  y: number,
  covarianceM2: Mat2 = smallCovariance,
): PositionMeasurement2D => ({ x, y, covarianceM2 });

function matrixIsFiniteSymmetric(matrix: number[][]): boolean {
  return matrix.every((row, rowIndex) => row.every((value, columnIndex) =>
    Number.isFinite(value) && Math.abs(value - matrix[columnIndex][rowIndex]) < 1e-8));
}

function positiveSemidefiniteByCholesky(matrix: number[][], tolerance = 1e-8): boolean {
  const lower = matrix.map(row => row.map(() => 0));
  for (let row = 0; row < matrix.length; row++) {
    for (let column = 0; column <= row; column++) {
      let value = matrix[row][column];
      for (let index = 0; index < column; index++) {
        value -= lower[row][index] * lower[column][index];
      }
      if (row === column) {
        if (value < -tolerance) return false;
        lower[row][column] = Math.sqrt(Math.max(value, tolerance));
      } else {
        lower[row][column] = value / lower[column][column];
      }
    }
  }
  return true;
}

function correctionAtNis(targetNis: number) {
  const filter = new AdaptiveKalmanFilter2D(config);
  filter.correct(measurement(0, 0));
  filter.predict(0.02);
  const covariance = filter.getCovariance4() as number[][];
  const measurementVariance = smallCovariance[0][0]
    + config.measurementSigmaFloorM * config.measurementSigmaFloorM;
  const innovationVariance = covariance[0][0] + measurementVariance;
  return filter.correct(measurement(Math.sqrt(targetNis * innovationVariance), 0));
}

describe('C8 adaptive 2D Kalman', () => {
  it('initializes from the first measurement instead of converging from the origin', () => {
    const filter = new AdaptiveKalmanFilter2D(config);
    const initialized = filter.correct(measurement(12.5, -3.25));
    expect(initialized.mode).toBe('initialized');
    expect(initialized.accepted).toBe(true);
    expect(initialized.state?.x).toBeCloseTo(12.5, 12);
    expect(initialized.state?.y).toBeCloseTo(-3.25, 12);
    expect(initialized.state?.vx).toBe(0);
  });

  it('rebuilds white-acceleration Q from the actual dt', () => {
    const short = whiteAccelerationQ(0.02, 4);
    const long = whiteAccelerationQ(0.04, 4);
    expect(long[0][0] / short[0][0]).toBeCloseTo(16, 12);
    expect(long[0][2] / short[0][2]).toBeCloseTo(8, 12);
    expect(long[2][2] / short[2][2]).toBeCloseTo(4, 12);
    expect(short[0][1]).toBe(0);
    expect(short[1][3]).toBeCloseTo(short[0][2], 12);
  });

  it('tracks a constant-velocity trajectory using actual prediction intervals', () => {
    const relaxed = { ...config, nisSoftGate2D: 1e9, nisHardGate2D: 1e10 };
    const filter = new AdaptiveKalmanFilter2D(relaxed);
    filter.correct(measurement(0, 0));
    const velocity = 2.0;
    for (let index = 1; index <= 300; index++) {
      filter.predict(0.02);
      const corrected = filter.correct(measurement(velocity * index * 0.02, 0));
      expect(corrected.accepted).toBe(true);
    }
    const state = filter.getState();
    expect(state?.x).toBeCloseTo(12, 2);
    expect(state?.vx).toBeCloseTo(velocity, 1);
  });

  it('preserves anisotropic and correlated measurement covariance', () => {
    const covariance: Mat2 = [[0.01, 0.006], [0.006, 0.09]];
    const filter = new AdaptiveKalmanFilter2D(config);
    const initialized = filter.correct(measurement(1, 2, covariance));
    expect(initialized.state?.positionCovarianceM2[0][1]).not.toBe(0);
    expect(initialized.state?.positionCovarianceM2[0][0])
      .toBeLessThan(initialized.state?.positionCovarianceM2[1][1] as number);
  });

  it('applies the configured soft and hard NIS gates', () => {
    const belowSoft = correctionAtNis(config.nisSoftGate2D * 0.99);
    const aboveSoft = correctionAtNis(config.nisSoftGate2D * 1.01);
    const aboveHard = correctionAtNis(config.nisHardGate2D * 1.01);
    expect(belowSoft.accepted).toBe(true);
    expect(aboveSoft.accepted).toBe(true);
    expect(aboveSoft.nis).toBeGreaterThan(config.nisSoftGate2D);
    expect(aboveHard.accepted).toBe(false);
    expect(aboveHard.mode).toBe('nis-rejected');
  });

  it('hard rejection leaves the predicted state untouched and increases uncertainty', () => {
    const filter = new AdaptiveKalmanFilter2D(config);
    const initialized = filter.correct(measurement(0, 0));
    const beforeVariance = initialized.state?.positionCovarianceM2[0][0] as number;
    const predicted = filter.predict(0.02);
    const rejected = filter.correct(measurement(100, 100));
    expect(rejected.accepted).toBe(false);
    expect(rejected.state?.x).toBeCloseTo(predicted?.x as number, 12);
    expect(rejected.state?.y).toBeCloseTo(predicted?.y as number, 12);
    expect(rejected.state?.positionCovarianceM2[0][0]).toBeGreaterThan(beforeVariance);
  });

  it('coasts for a bounded TTL and then reports stale without hiding the long dt', () => {
    const filter = new AdaptiveKalmanFilter2D(config);
    filter.correct(measurement(0, 0));
    const coasting = filter.markMissingMeasurement(0.1);
    const stale = filter.markMissingMeasurement(0.15);
    expect(coasting.mode).toBe('coasting');
    expect(stale.mode).toBe('stale');
    expect(stale.predictionOnlySec).toBeCloseTo(0.25, 12);
    expect(stale.state?.positionCovarianceM2[0][0])
      .toBeGreaterThan(coasting.state?.positionCovarianceM2[0][0] as number);
  });

  it('reacquires only after the configured number of consistent rejected measurements', () => {
    const filter = new AdaptiveKalmanFilter2D(config);
    filter.correct(measurement(0, 0));
    const outcomes = [5.0, 5.05, 4.98].map(value => {
      filter.predict(0.02);
      return filter.correct(measurement(value, 0));
    });
    expect(outcomes[0].accepted).toBe(false);
    expect(outcomes[1].accepted).toBe(false);
    expect(outcomes[2].accepted).toBe(true);
    expect(outcomes[2].mode).toBe('reacquiring');
    expect(outcomes[2].state?.x).toBeCloseTo((5 + 5.05 + 4.98) / 3, 8);
  });

  it('does not reacquire from one isolated spike', () => {
    const filter = new AdaptiveKalmanFilter2D(config);
    filter.correct(measurement(0, 0));
    filter.predict(0.02);
    expect(filter.correct(measurement(20, 0)).accepted).toBe(false);
    filter.predict(0.02);
    const recovered = filter.correct(measurement(0.01, 0));
    expect(recovered.accepted).toBe(true);
    expect(recovered.mode).toBe('tracking');
    expect(recovered.state?.x).toBeLessThan(0.1);
  });

  it('keeps Joseph covariance finite, symmetric and PSD over 10,000 updates', () => {
    const filter = new AdaptiveKalmanFilter2D({
      ...config,
      nisSoftGate2D: 1e9,
      nisHardGate2D: 1e10,
    });
    filter.correct(measurement(0, 0, [[0.01, 0.003], [0.003, 0.02]]));
    for (let index = 1; index <= 10_000; index++) {
      filter.predict(index % 3 === 0 ? 0.025 : 0.02);
      filter.correct(measurement(
        Math.sin(index / 1000) * 0.1,
        Math.cos(index / 1000) * 0.1,
        [[0.01, 0.003], [0.003, 0.02]],
      ));
    }
    const covariance = filter.getCovariance4() as number[][];
    expect(matrixIsFiniteSymmetric(covariance)).toBe(true);
    expect(positiveSemidefiniteByCholesky(covariance)).toBe(true);
    expect(filter.getState()?.positionCovarianceM2[0][0]).toBeGreaterThan(0);
  });

  it('rejects an invalid dt without corrupting the state', () => {
    const filter = new AdaptiveKalmanFilter2D(config);
    filter.correct(measurement(3, 4));
    const before = filter.getState();
    expect(() => filter.predict(Number.NaN)).toThrow(RangeError);
    expect(filter.getState()).toEqual(before);
  });
});

describe('C8 MCU clock', () => {
  it('accepts uint32 wrap, reports source gaps, resets on backward/reset time', () => {
    const clock = new McuClock();
    expect(clock.advance(100, 0xffff_fff0).dtSec).toBeNull();
    const wrapped = clock.advance(102, 0x0000_0018);
    expect(wrapped.dtSec).toBeCloseTo(0.04, 12);
    expect(wrapped.sourceGapCount).toBe(1);
    const reset = clock.advance(1, 10);
    expect(reset.resetDetected).toBe(true);
    expect(reset.dtSec).toBeNull();
  });

  it('returns a plausible long gap in full instead of clamping it to 0.2 seconds', () => {
    const clock = new McuClock();
    clock.advance(1, 1000);
    const gap = clock.advance(2, 2500);
    expect(gap.resetDetected).toBe(false);
    expect(gap.dtSec).toBe(1.5);
  });
});

const pipelineLayout: AnchorPosition[] = [
  { id: 1, x: 0, y: 0 },
  { id: 2, x: 5, y: 0 },
  { id: 3, x: 0, y: 4 },
  { id: 4, x: 5, y: 4 },
];
const pipelineCalibration: CalibrationRegistry = Object.fromEntries(
  pipelineLayout.map(anchor => [anchor.id, {
    DS: { calibrated: true, rawSigmaM: 0.05, filteredSigmaM: 0.04 },
  }]),
);

function pipelineSample(
  seq: number,
  timeMs: number,
  point: { x: number; y: number },
  validIds = new Set([1, 2, 3, 4]),
): RangeSample {
  const anchors: AnchorSample[] = pipelineLayout.map(anchor => {
    const valid = validIds.has(anchor.id);
    const rangeMm = Math.hypot(point.x - anchor.x, point.y - anchor.y) * 1000;
    return {
      id: anchor.id,
      valid,
      ageMs: valid ? 0 : 0xffff,
      rawMm: valid ? rangeMm : null,
      filtMm: valid ? rangeMm : null,
      fppDbm: valid ? -80 : null,
      status: valid ? 0 : 1,
    };
  });
  return {
    seq,
    transportSeq: seq,
    timeMs,
    clientTime: timeMs,
    anchors,
    anchorsById: Object.fromEntries(anchors.map(anchor => [anchor.id, anchor])),
  };
}

describe('C8 pure position pipeline', () => {
  const process = (
    pipeline: PositionPipeline,
    sample: RangeSample,
    layout: readonly AnchorPosition[] = pipelineLayout,
  ) => pipeline.process({
    sample,
    layout,
    calibration: pipelineCalibration,
    observation: { rangeSource: 'raw', defaultRangingMode: 'DS' },
  });

  it('runs observation, robust solve and adaptive Kalman without React state', () => {
    const pipeline = new PositionPipeline(SHADOW_POSITION_CONFIG);
    const first = process(pipeline, pipelineSample(1, 100, { x: 2, y: 1.5 }));
    const second = process(pipeline, pipelineSample(2, 120, { x: 2.01, y: 1.5 }));
    expect(first.outputMode).toBe('measured');
    expect(first.correction.mode).toBe('initialized');
    expect(first.measurement?.usedAnchorIds).toEqual([1, 2, 3, 4]);
    expect(second.measurement).not.toBeNull();
    expect(second.estimate).not.toBeNull();
    expect(second.timing.dtSec).toBeCloseTo(0.02, 12);
  });

  it('predicts through short missing geometry then suppresses stale output after TTL', () => {
    const pipeline = new PositionPipeline(SHADOW_POSITION_CONFIG);
    process(pipeline, pipelineSample(1, 100, { x: 2, y: 1.5 }));
    const onlyTwo = new Set([1, 2]);
    const coast1 = process(pipeline, pipelineSample(2, 200, { x: 2, y: 1.5 }, onlyTwo));
    const coast2 = process(pipeline, pipelineSample(3, 300, { x: 2, y: 1.5 }, onlyTwo));
    const stale = process(pipeline, pipelineSample(4, 400, { x: 2, y: 1.5 }, onlyTwo));
    expect(coast1.outputMode).toBe('predicted');
    expect(coast2.outputMode).toBe('predicted');
    expect(stale.outputMode).toBe('stale');
    expect(stale.estimate).toBeNull();
    expect(stale.failureReason).toBe('insufficient-observations');
  });

  it('fails closed on duplicate layout IDs', () => {
    const pipeline = new PositionPipeline(SHADOW_POSITION_CONFIG);
    const duplicateLayout = [...pipelineLayout, { ...pipelineLayout[0] }];
    const frame = process(
      pipeline,
      pipelineSample(1, 100, { x: 2, y: 1.5 }),
      duplicateLayout,
    );
    expect(frame.outputMode).toBe('none');
    expect(frame.failureReason).toBe('observation-input-invalid');
  });

  it('reinitializes transparently after a long gap instead of clamping dt', () => {
    const pipeline = new PositionPipeline(SHADOW_POSITION_CONFIG);
    process(pipeline, pipelineSample(1, 100, { x: 2, y: 1.5 }));
    const afterGap = process(pipeline, pipelineSample(2, 1100, { x: 3, y: 2 }));
    expect(afterGap.timing.dtSec).toBe(1);
    expect(afterGap.correction.mode).toBe('initialized');
    expect(afterGap.estimate?.x).toBeCloseTo(3, 8);
    expect(afterGap.estimate?.y).toBeCloseTo(2, 8);
  });
});
