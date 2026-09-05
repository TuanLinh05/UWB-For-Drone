import { AdaptiveKalmanFilter2D } from './adaptiveKalman2d';
import { solveWithConservativeLoo } from './leaveOneOut';
import { McuClock, type McuClockAdvance } from './mcuClock';
import { buildObservations } from './observationBuilder';
import type {
  CalibrationRegistry,
  ExcludedObservation,
  KalmanCorrection,
  KalmanState2D,
  ObservationConfig,
  PositionPipelineConfig,
  RangeObservation2D,
  PositionSolveResult2D,
  SolveFailureReason,
  Vec2,
} from './types';
import type { AnchorPosition, RangeSample } from '../types';

export type PositionOutputMode = 'measured' | 'predicted' | 'stale' | 'none';

export interface PositionPipelineFrame {
  measurement: PositionSolveResult2D | null;
  estimate: KalmanState2D | null;
  outputMode: PositionOutputMode;
  failureReason: SolveFailureReason | 'observation-input-invalid' | null;
  correction: KalmanCorrection;
  excluded: ExcludedObservation[];
  usableAnchorIds: number[];
  observations: RangeObservation2D[];
  timing: McuClockAdvance;
}

export interface PositionPipelineInput {
  sample: RangeSample;
  layout: readonly AnchorPosition[];
  calibration: CalibrationRegistry;
  observation: Pick<ObservationConfig, 'rangeSource' | 'defaultRangingMode'>;
}

function uninitializedCorrection(): KalmanCorrection {
  return {
    state: null,
    accepted: false,
    nis: null,
    mode: 'uninitialized',
    rejectStreak: 0,
    predictionOnlySec: 0,
  };
}

export class PositionPipeline {
  private readonly kalman: AdaptiveKalmanFilter2D;
  private readonly clock = new McuClock();
  private lastSolverPoint: Vec2 | undefined;

  constructor(private readonly config: PositionPipelineConfig) {
    this.kalman = new AdaptiveKalmanFilter2D(config.kalman);
  }

  reset(): void {
    this.kalman.reset();
    this.clock.reset();
    this.lastSolverPoint = undefined;
  }

  private missingCorrection(predictedThisFrame: boolean): KalmanCorrection {
    if (!this.kalman.initialized) return uninitializedCorrection();
    return predictedThisFrame
      ? this.kalman.markPredictedMeasurementMissing()
      : {
        state: this.kalman.getState(),
        accepted: false,
        nis: null,
        mode: 'coasting',
        rejectStreak: 0,
        predictionOnlySec: 0,
      };
  }

  private frameFromCorrection(
    correction: KalmanCorrection,
    timing: McuClockAdvance,
    measurement: PositionSolveResult2D | null,
    failureReason: PositionPipelineFrame['failureReason'],
    excluded: ExcludedObservation[],
    usableAnchorIds: number[],
    observations: RangeObservation2D[],
  ): PositionPipelineFrame {
    const stale = correction.mode === 'stale';
    return {
      measurement,
      estimate: stale ? null : correction.state,
      outputMode: correction.accepted
        ? 'measured'
        : stale
          ? 'stale'
          : correction.state
            ? 'predicted'
            : 'none',
      failureReason,
      correction,
      excluded,
      usableAnchorIds,
      observations,
      timing,
    };
  }

  process(input: PositionPipelineInput): PositionPipelineFrame {
    const timing = this.clock.advance(input.sample.seq, input.sample.timeMs);
    if (timing.resetDetected) {
      this.kalman.reset();
      this.lastSolverPoint = undefined;
    }

    let prior = this.kalman.getState();
    let predictedThisFrame = false;
    if (timing.dtSec !== null && this.kalman.initialized) {
      if (timing.dtSec > this.config.kalman.maxTrackingDtSec) {
        // Reset on a long semantic gap. Never replace the gap with a clamped dt.
        this.kalman.reset();
        this.lastSolverPoint = undefined;
        prior = null;
      } else {
        prior = this.kalman.predict(timing.dtSec);
        predictedThisFrame = prior !== null;
      }
    }

    const built = buildObservations({
      sample: input.sample,
      layout: input.layout,
      calibration: input.calibration,
      config: {
        ...this.config.observation,
        ...input.observation,
      },
    });
    const usableAnchorIds = built.usable.map(observation => observation.id);
    if (built.fatal) {
      return this.frameFromCorrection(
        this.missingCorrection(predictedThisFrame),
        timing,
        null,
        'observation-input-invalid',
        built.excluded,
        usableAnchorIds,
        built.usable,
      );
    }

    const solved = solveWithConservativeLoo(
      built.usable,
      prior ?? this.lastSolverPoint,
      this.config.solver,
    );
    if (!solved.ok) {
      return this.frameFromCorrection(
        this.missingCorrection(predictedThisFrame),
        timing,
        null,
        solved.reason,
        built.excluded,
        usableAnchorIds,
        built.usable,
      );
    }

    this.lastSolverPoint = { x: solved.result.x, y: solved.result.y };
    const corrected = this.kalman.correct({
      x: solved.result.x,
      y: solved.result.y,
      covarianceM2: solved.result.covarianceM2,
    });
    return this.frameFromCorrection(
      corrected,
      timing,
      solved.result,
      null,
      built.excluded,
      usableAnchorIds,
      built.usable,
    );
  }
}
