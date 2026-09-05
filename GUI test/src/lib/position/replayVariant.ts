import type { AnchorSample, RangeSample } from '../types';
import type {
  ReplayDataset,
  ReplayRangeFrame,
} from '../replayDataset';
import type {
  ReplayObservation,
  ReplayVariantFactory,
  ReplayVariantFrameOutput,
} from '../replayRunner';
import { SHADOW_POSITION_CONFIG } from './config';
import { solveWithConservativeLoo } from './leaveOneOut';
import { buildObservations } from './observationBuilder';
import { PositionPipeline } from './positionPipeline';
import type {
  CalibrationMode,
  CalibrationRegistry,
  RangeCalibrationProfile,
  Vec2,
} from './types';

const TAG_ST_DS_FALLBACK = 0x10;
const PROVISIONAL_RAW_SIGMA_M = 0.15;
const PROVISIONAL_FILTERED_SIGMA_M = 0.10;

const provisionalProfile = (calibrated: boolean): RangeCalibrationProfile => ({
  calibrated,
  rawSigmaM: PROVISIONAL_RAW_SIGMA_M,
  filteredSigmaM: PROVISIONAL_FILTERED_SIGMA_M,
});

function sourceMode(frame: ReplayRangeFrame): 'SS' | 'DS' {
  return frame.records.some(record => record.rangingMode === 'DS') ? 'DS' : 'SS';
}

function calibrationForFrame(frame: ReplayRangeFrame): CalibrationRegistry {
  return Object.fromEntries(frame.records.map(record => {
    const mode: CalibrationMode = record.rangingMode === 'DS' ? 'DS' : 'SS';
    return [record.id, { [mode]: provisionalProfile(!record.calibrationMissing) }];
  }));
}

function rangeSampleFromReplay(frame: ReplayRangeFrame): RangeSample {
  const anchors: AnchorSample[] = frame.records.map(record => ({
    id: record.id,
    valid: record.valid,
    ageMs: record.ageMs,
    rawMm: record.correctedRawMm,
    filtMm: record.filteredMm,
    fppDbm: record.fppDbm,
    status: record.rangingMode === 'SS_FALLBACK'
      ? record.status | TAG_ST_DS_FALLBACK
      : record.status,
    ...(record.diagnosticRawMm === null ? {} : { diagnosticRawMm: record.diagnosticRawMm }),
    ...(record.diagnosticFppDbm === null ? {} : { diagnosticFppDbm: record.diagnosticFppDbm }),
  }));
  return {
    seq: frame.seq,
    transportSeq: frame.transportSeq,
    timeMs: frame.timeMs,
    clientTime: frame.clientTimeMs,
    anchors,
    anchorsById: Object.fromEntries(anchors.map(anchor => [anchor.id, anchor])),
  };
}

function rejectedIds(
  frame: ReplayRangeFrame,
  usableIds: ReadonlySet<number>,
): number[] {
  return frame.records
    .map(record => record.id)
    .filter(id => !usableIds.has(id))
    .sort((left, right) => left - right);
}

/**
 * C6 replay variant. Thresholds and per-anchor sigma remain provisional until
 * C4 supplies multi-distance ground truth; therefore this variant is shadow
 * evidence only and never qualifies a production gate by itself.
 */
export function createC6RobustReplayVariant(
  dataset: ReplayDataset,
  requestedSource?: 'raw' | 'filtered',
): ReplayVariantFactory {
  const source = requestedSource
    ?? (dataset.metadata.positionRangeSource === 'unknown'
      ? null
      : dataset.metadata.positionRangeSource);
  const layout = dataset.metadata.anchorLayout.map(anchor => ({ ...anchor }));

  return () => {
    let seed: Vec2 | undefined;
    return {
      id: `c6-robust-shadow-${source ?? 'missing-source'}`,
      label: `C6 robust LM/IRLS shadow (${source ?? 'missing source metadata'})`,
      process(frame): ReplayVariantFrameOutput {
        if (!source || layout.length < 3) {
          return {
            observations: [],
            rejectedAnchorIds: frame.records.map(record => record.id),
            status: !source ? 'missing-range-source-metadata' : 'missing-anchor-layout-metadata',
          };
        }

        const built = buildObservations({
          sample: rangeSampleFromReplay(frame),
          layout,
          calibration: calibrationForFrame(frame),
          config: {
            ...SHADOW_POSITION_CONFIG.observation,
            rangeSource: source,
            defaultRangingMode: sourceMode(frame),
          },
        });
        const usableIds = new Set(built.usable.map(observation => observation.id));
        const observations: ReplayObservation[] = built.usable.map(observation => ({
          anchorId: observation.id,
          rangeMm: observation.rangeM * 1000,
          ageMs: observation.ageMs,
          measurementTimeMs: frame.records.find(record => record.id === observation.id)
            ?.measurementTimeMs ?? null,
          fppDbm: observation.fppDbm,
          rangingMode: observation.rangingMode,
        }));
        const rejectedAnchorIds = rejectedIds(frame, usableIds);
        if (built.fatal) {
          return { observations, rejectedAnchorIds, status: 'invalid-observation-input' };
        }

        const solved = solveWithConservativeLoo(
          built.usable,
          seed,
          SHADOW_POSITION_CONFIG.solver,
        );
        if (!solved.ok) {
          return {
            observations,
            rejectedAnchorIds,
            status: `withheld:${solved.reason}`,
          };
        }
        seed = { x: solved.result.x, y: solved.result.y };
        return {
          observations,
          rejectedAnchorIds: [...new Set([
            ...rejectedAnchorIds,
            ...solved.result.rejectedAnchorIds,
          ])].sort((left, right) => left - right),
          position: seed,
          status: `valid:${solved.result.mode}`,
        };
      },
    };
  };
}

/** C8 replay variant: C6 measurement followed by adaptive full-R/NIS Kalman. */
export function createC8AdaptiveReplayVariant(
  dataset: ReplayDataset,
  requestedSource?: 'raw' | 'filtered',
): ReplayVariantFactory {
  const source = requestedSource
    ?? (dataset.metadata.positionRangeSource === 'unknown'
      ? null
      : dataset.metadata.positionRangeSource);
  const layout = dataset.metadata.anchorLayout.map(anchor => ({ ...anchor }));

  return () => {
    const pipeline = new PositionPipeline(SHADOW_POSITION_CONFIG);
    return {
      id: `c8-adaptive-shadow-${source ?? 'missing-source'}`,
      label: `C8 adaptive full-R/NIS Kalman shadow (${source ?? 'missing source metadata'})`,
      process(frame): ReplayVariantFrameOutput {
        if (!source || layout.length < 3) {
          return {
            observations: [],
            rejectedAnchorIds: frame.records.map(record => record.id),
            status: !source ? 'missing-range-source-metadata' : 'missing-anchor-layout-metadata',
          };
        }
        const pipelineFrame = pipeline.process({
          sample: rangeSampleFromReplay(frame),
          layout,
          calibration: calibrationForFrame(frame),
          observation: {
            rangeSource: source,
            defaultRangingMode: sourceMode(frame),
          },
        });
        const observations: ReplayObservation[] = pipelineFrame.observations.map(observation => ({
          anchorId: observation.id,
          rangeMm: observation.rangeM * 1000,
          ageMs: observation.ageMs,
          measurementTimeMs: frame.records.find(record => record.id === observation.id)
            ?.measurementTimeMs ?? null,
          fppDbm: observation.fppDbm,
          rangingMode: observation.rangingMode,
        }));
        const usedIds = new Set(observations.map(observation => observation.anchorId));
        const rejectedAnchorIds = [...new Set([
          ...rejectedIds(frame, usedIds),
          ...(pipelineFrame.measurement?.rejectedAnchorIds ?? []),
        ])].sort((left, right) => left - right);
        return {
          observations,
          rejectedAnchorIds,
          ...(pipelineFrame.estimate
            ? { position: { x: pipelineFrame.estimate.x, y: pipelineFrame.estimate.y } }
            : {}),
          status: `c8:${pipelineFrame.outputMode}:${pipelineFrame.correction.mode}`
            + (pipelineFrame.failureReason ? `:${pipelineFrame.failureReason}` : ''),
        };
      },
    };
  };
}
