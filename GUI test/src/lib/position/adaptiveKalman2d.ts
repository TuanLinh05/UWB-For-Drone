import { clampSymmetricEigenvalues, invertSymmetric2 } from './matrix2';
import type {
  AdaptiveKalmanConfig,
  KalmanCorrection,
  KalmanState2D,
  KalmanTrackingMode,
  Mat2,
  PositionMeasurement2D,
} from './types';

type Vec4 = [number, number, number, number];
type Mat4 = [
  [number, number, number, number],
  [number, number, number, number],
  [number, number, number, number],
  [number, number, number, number],
];

interface ReacquireCandidate {
  x: number;
  y: number;
  count: number;
  covarianceM2: Mat2;
}

const cloneMat2 = (matrix: Mat2): Mat2 => [
  [matrix[0][0], matrix[0][1]],
  [matrix[1][0], matrix[1][1]],
];

const zeroMat4 = (): Mat4 => [
  [0, 0, 0, 0],
  [0, 0, 0, 0],
  [0, 0, 0, 0],
  [0, 0, 0, 0],
];

function identityMat4(): Mat4 {
  const matrix = zeroMat4();
  for (let index = 0; index < 4; index++) matrix[index][index] = 1;
  return matrix;
}

function transpose4(matrix: Mat4): Mat4 {
  const result = zeroMat4();
  for (let row = 0; row < 4; row++) {
    for (let column = 0; column < 4; column++) result[row][column] = matrix[column][row];
  }
  return result;
}

function multiply4(left: Mat4, right: Mat4): Mat4 {
  const result = zeroMat4();
  for (let row = 0; row < 4; row++) {
    for (let column = 0; column < 4; column++) {
      for (let index = 0; index < 4; index++) {
        result[row][column] += left[row][index] * right[index][column];
      }
    }
  }
  return result;
}

function add4(left: Mat4, right: Mat4): Mat4 {
  const result = zeroMat4();
  for (let row = 0; row < 4; row++) {
    for (let column = 0; column < 4; column++) result[row][column] = left[row][column] + right[row][column];
  }
  return result;
}

function finiteMat4(matrix: Mat4): boolean {
  return matrix.every(row => row.every(Number.isFinite));
}

function sanitizeStateCovariance(matrix: Mat4, config: AdaptiveKalmanConfig): Mat4 | null {
  const result = zeroMat4();
  for (let row = 0; row < 4; row++) {
    for (let column = 0; column < 4; column++) {
      const symmetric = 0.5 * (matrix[row][column] + matrix[column][row]);
      if (!Number.isFinite(symmetric)) return null;
      result[row][column] = symmetric;
    }
  }
  for (let index = 0; index < 4; index++) {
    result[index][index] = Math.max(
      config.stateCovarianceFloor,
      Math.min(config.stateCovarianceCeiling, result[index][index]),
    );
  }
  return finiteMat4(result) ? result : null;
}

function sanitizeMeasurementCovariance(
  covariance: Mat2,
  config: AdaptiveKalmanConfig,
): Mat2 | null {
  const floorM2 = config.measurementSigmaFloorM * config.measurementSigmaFloorM;
  const withFloor: Mat2 = [
    [covariance[0][0] + floorM2, covariance[0][1]],
    [covariance[1][0], covariance[1][1] + floorM2],
  ];
  return clampSymmetricEigenvalues(
    withFloor,
    floorM2,
    config.measurementCovarianceEigenCeilingM2,
  );
}

function positionCovariance(matrix: Mat4): Mat2 {
  return [
    [matrix[0][0], matrix[0][1]],
    [matrix[1][0], matrix[1][1]],
  ];
}

function snapshotFrom(
  state: Vec4,
  covariance: Mat4,
): KalmanState2D {
  return {
    x: state[0],
    y: state[1],
    vx: state[2],
    vy: state[3],
    positionCovarianceM2: positionCovariance(covariance),
  };
}

function correction(
  state: Vec4 | null,
  covariance: Mat4 | null,
  accepted: boolean,
  nis: number | null,
  mode: KalmanTrackingMode,
  rejectStreak: number,
  predictionOnlySec: number,
): KalmanCorrection {
  return {
    state: state && covariance ? snapshotFrom(state, covariance) : null,
    accepted,
    nis,
    mode,
    rejectStreak,
    predictionOnlySec,
  };
}

export function whiteAccelerationQ(
  dtSec: number,
  accelerationSigmaMps2: number,
): Mat4 {
  const variance = accelerationSigmaMps2 * accelerationSigmaMps2;
  const dt2 = dtSec * dtSec;
  const dt3 = dt2 * dtSec;
  const dt4 = dt2 * dt2;
  const position = variance * dt4 / 4;
  const cross = variance * dt3 / 2;
  const velocity = variance * dt2;
  return [
    [position, 0, cross, 0],
    [0, position, 0, cross],
    [cross, 0, velocity, 0],
    [0, cross, 0, velocity],
  ];
}

export class AdaptiveKalmanFilter2D {
  private state: Vec4 | null = null;
  private covariance: Mat4 | null = null;
  private rejectionCount = 0;
  private predictionOnlySeconds = 0;
  private pendingPredictionSeconds = 0;
  private reacquireCandidate: ReacquireCandidate | null = null;

  constructor(private readonly config: AdaptiveKalmanConfig) {}

  get initialized(): boolean {
    return this.state !== null && this.covariance !== null;
  }

  reset(): void {
    this.state = null;
    this.covariance = null;
    this.rejectionCount = 0;
    this.predictionOnlySeconds = 0;
    this.pendingPredictionSeconds = 0;
    this.reacquireCandidate = null;
  }

  private initializeFromMeasurement(
    measurement: PositionMeasurement2D,
    covarianceM2: Mat2,
  ): KalmanCorrection {
    const velocityVariance = this.config.initialVelocitySigmaMps
      * this.config.initialVelocitySigmaMps;
    this.state = [measurement.x, measurement.y, 0, 0];
    this.covariance = [
      [covarianceM2[0][0], covarianceM2[0][1], 0, 0],
      [covarianceM2[1][0], covarianceM2[1][1], 0, 0],
      [0, 0, velocityVariance, 0],
      [0, 0, 0, velocityVariance],
    ];
    this.rejectionCount = 0;
    this.predictionOnlySeconds = 0;
    this.pendingPredictionSeconds = 0;
    this.reacquireCandidate = null;
    return correction(this.state, this.covariance, true, null, 'initialized', 0, 0);
  }

  getState(): KalmanState2D | null {
    return this.state && this.covariance ? snapshotFrom(this.state, this.covariance) : null;
  }

  /** Test/debug snapshot. The returned matrix cannot mutate estimator state. */
  getCovariance4(): Mat4 | null {
    return this.covariance
      ? this.covariance.map(row => [...row]) as Mat4
      : null;
  }

  predict(dtSec: number): KalmanState2D | null {
    if (!Number.isFinite(dtSec) || dtSec <= 0) throw new RangeError('Kalman dt must be finite and positive.');
    if (!this.state || !this.covariance) return null;
    const dt = dtSec;
    const transition: Mat4 = [
      [1, 0, dt, 0],
      [0, 1, 0, dt],
      [0, 0, 1, 0],
      [0, 0, 0, 1],
    ];
    const previous = this.state;
    const predicted: Vec4 = [
      previous[0] + dt * previous[2],
      previous[1] + dt * previous[3],
      previous[2],
      previous[3],
    ];
    const predictedCovariance = sanitizeStateCovariance(
      add4(
        multiply4(multiply4(transition, this.covariance), transpose4(transition)),
        whiteAccelerationQ(dt, this.config.processAccelerationSigmaMps2),
      ),
      this.config,
    );
    if (!predicted.every(Number.isFinite) || !predictedCovariance) {
      this.reset();
      return null;
    }
    this.state = predicted;
    this.covariance = predictedCovariance;
    this.pendingPredictionSeconds += dt;
    return snapshotFrom(predicted, predictedCovariance);
  }

  private snapshot(
    accepted: boolean,
    nis: number | null,
    mode: KalmanTrackingMode,
  ): KalmanCorrection {
    return correction(
      this.state,
      this.covariance,
      accepted,
      nis,
      mode,
      this.rejectionCount,
      this.predictionOnlySeconds,
    );
  }

  private consumePredictionAsCoast(): KalmanTrackingMode {
    this.predictionOnlySeconds += this.pendingPredictionSeconds;
    this.pendingPredictionSeconds = 0;
    return this.predictionOnlySeconds > this.config.maxPredictionOnlySec ? 'stale' : 'coasting';
  }

  markPredictedMeasurementMissing(): KalmanCorrection {
    if (!this.initialized) {
      return correction(null, null, false, null, 'uninitialized', this.rejectionCount, 0);
    }
    const mode = this.consumePredictionAsCoast();
    return this.snapshot(false, null, mode);
  }

  markMissingMeasurement(dtSec: number): KalmanCorrection {
    if (this.initialized) this.predict(dtSec);
    return this.markPredictedMeasurementMissing();
  }

  private updateReacquireCandidate(
    measurement: PositionMeasurement2D,
    covarianceM2: Mat2,
  ): boolean {
    const candidate = this.reacquireCandidate;
    if (!candidate || Math.hypot(measurement.x - candidate.x, measurement.y - candidate.y)
      > this.config.reacquireMaxDistanceM) {
      this.reacquireCandidate = {
        x: measurement.x,
        y: measurement.y,
        count: 1,
        covarianceM2: cloneMat2(covarianceM2),
      };
      return false;
    }
    const count = candidate.count + 1;
    candidate.x += (measurement.x - candidate.x) / count;
    candidate.y += (measurement.y - candidate.y) / count;
    candidate.count = count;
    candidate.covarianceM2 = cloneMat2(covarianceM2);
    return count >= this.config.reacquireConsistentSamples;
  }

  correct(measurement: PositionMeasurement2D): KalmanCorrection {
    if (![measurement.x, measurement.y].every(Number.isFinite)) {
      return this.snapshot(false, null, 'nis-rejected');
    }
    const measurementCovariance = sanitizeMeasurementCovariance(
      measurement.covarianceM2,
      this.config,
    );
    if (!measurementCovariance) return this.snapshot(false, null, 'nis-rejected');
    if (!this.state || !this.covariance) {
      return this.initializeFromMeasurement(measurement, measurementCovariance);
    }

    const innovation: [number, number] = [
      measurement.x - this.state[0],
      measurement.y - this.state[1],
    ];
    const innovationCovariance: Mat2 = [
      [this.covariance[0][0] + measurementCovariance[0][0],
        this.covariance[0][1] + measurementCovariance[0][1]],
      [this.covariance[1][0] + measurementCovariance[1][0],
        this.covariance[1][1] + measurementCovariance[1][1]],
    ];
    const innovationInverse = invertSymmetric2(
      innovationCovariance[0][0],
      0.5 * (innovationCovariance[0][1] + innovationCovariance[1][0]),
      innovationCovariance[1][1],
      this.config.stateCovarianceFloor,
      this.config.stateCovarianceCeiling / this.config.stateCovarianceFloor,
    );
    if (!innovationInverse) {
      this.rejectionCount++;
      const coastMode = this.consumePredictionAsCoast();
      return this.snapshot(false, null, coastMode === 'stale' ? 'stale' : 'nis-rejected');
    }
    const nis = innovation[0] * (
      innovationInverse[0][0] * innovation[0] + innovationInverse[0][1] * innovation[1]
    ) + innovation[1] * (
      innovationInverse[1][0] * innovation[0] + innovationInverse[1][1] * innovation[1]
    );
    if (!Number.isFinite(nis) || nis > this.config.nisHardGate2D) {
      this.rejectionCount++;
      const coastMode = this.consumePredictionAsCoast();
      if (this.updateReacquireCandidate(measurement, measurementCovariance)) {
        const candidate = this.reacquireCandidate as ReacquireCandidate;
        const reacquired = this.initializeFromMeasurement(
          { x: candidate.x, y: candidate.y, covarianceM2: candidate.covarianceM2 },
          candidate.covarianceM2,
        );
        return { ...reacquired, mode: 'reacquiring', nis };
      }
      return this.snapshot(false, nis, coastMode === 'stale' ? 'stale' : 'nis-rejected');
    }

    const effectiveR = nis > this.config.nisSoftGate2D
      ? [
        [
          measurementCovariance[0][0] * this.config.softGateVarianceInflation,
          measurementCovariance[0][1] * this.config.softGateVarianceInflation,
        ],
        [
          measurementCovariance[1][0] * this.config.softGateVarianceInflation,
          measurementCovariance[1][1] * this.config.softGateVarianceInflation,
        ],
      ] as Mat2
      : measurementCovariance;
    const effectiveInnovationCovariance: Mat2 = [
      [this.covariance[0][0] + effectiveR[0][0], this.covariance[0][1] + effectiveR[0][1]],
      [this.covariance[1][0] + effectiveR[1][0], this.covariance[1][1] + effectiveR[1][1]],
    ];
    const effectiveInverse = invertSymmetric2(
      effectiveInnovationCovariance[0][0],
      0.5 * (effectiveInnovationCovariance[0][1] + effectiveInnovationCovariance[1][0]),
      effectiveInnovationCovariance[1][1],
      this.config.stateCovarianceFloor,
      this.config.stateCovarianceCeiling / this.config.stateCovarianceFloor,
    );
    if (!effectiveInverse) {
      this.rejectionCount++;
      const coastMode = this.consumePredictionAsCoast();
      return this.snapshot(false, nis, coastMode === 'stale' ? 'stale' : 'nis-rejected');
    }

    const gain: [[number, number], [number, number], [number, number], [number, number]] = [
      [0, 0], [0, 0], [0, 0], [0, 0],
    ];
    for (let row = 0; row < 4; row++) {
      gain[row][0] = this.covariance[row][0] * effectiveInverse[0][0]
        + this.covariance[row][1] * effectiveInverse[1][0];
      gain[row][1] = this.covariance[row][0] * effectiveInverse[0][1]
        + this.covariance[row][1] * effectiveInverse[1][1];
      this.state[row] += gain[row][0] * innovation[0] + gain[row][1] * innovation[1];
    }

    const identityMinusKh = identityMat4();
    for (let row = 0; row < 4; row++) {
      identityMinusKh[row][0] -= gain[row][0];
      identityMinusKh[row][1] -= gain[row][1];
    }
    const gainR = gain.map(row => [
      row[0] * effectiveR[0][0] + row[1] * effectiveR[1][0],
      row[0] * effectiveR[0][1] + row[1] * effectiveR[1][1],
    ]);
    const krkt = zeroMat4();
    for (let row = 0; row < 4; row++) {
      for (let column = 0; column < 4; column++) {
        krkt[row][column] = gainR[row][0] * gain[column][0]
          + gainR[row][1] * gain[column][1];
      }
    }
    const joseph = add4(
      multiply4(multiply4(identityMinusKh, this.covariance), transpose4(identityMinusKh)),
      krkt,
    );
    const sanitized = sanitizeStateCovariance(joseph, this.config);
    if (!this.state.every(Number.isFinite) || !sanitized) {
      this.reset();
      return correction(null, null, false, nis, 'stale', 0, 0);
    }
    this.covariance = sanitized;
    this.rejectionCount = 0;
    this.predictionOnlySeconds = 0;
    this.pendingPredictionSeconds = 0;
    this.reacquireCandidate = null;
    return this.snapshot(true, nis, 'tracking');
  }
}
