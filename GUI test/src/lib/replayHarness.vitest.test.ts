// @ts-expect-error Node runtime types are intentionally not shipped with the browser bundle.
import { execFileSync } from 'node:child_process';
// @ts-expect-error Node runtime types are intentionally not shipped with the browser bundle.
import { mkdtempSync, rmSync, writeFileSync } from 'node:fs';
// @ts-expect-error Node runtime types are intentionally not shipped with the browser bundle.
import { tmpdir } from 'node:os';
// @ts-expect-error Node runtime types are intentionally not shipped with the browser bundle.
import { join } from 'node:path';
// @ts-expect-error Node runtime types are intentionally not shipped with the browser bundle.
import process from 'node:process';
// @ts-expect-error Node runtime types are intentionally not shipped with the browser bundle.
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { buildReplayCsv, createReplayLogContext, REPLAY_CSV_FORMAT_VERSION } from './replayLog';
import { parseReplayCsv } from './replayDataset';
import {
  recordedFilteredVariant,
  recordedRawVariant,
  c9MedianGateReplayVariant,
  adaptiveLegacyReplayVariant,
  c9_2MotionAdaptiveReplayVariant,
  runReplayVariants,
} from './replayRunner';
import {
  createC6RobustReplayVariant,
  createC8AdaptiveReplayVariant,
} from './position/replayVariant';
import type { AnchorSample, FirmwareInfo, RangeSample } from './types';

const firmwareInfo: FirmwareInfo = {
  schemaVersion: 2,
  calibrationProfile: 'ds',
  rangingMode: 'ds',
  activeOffsetsM: { 1: 154.09, 2: 154.03, 3: 154.10, 4: 154.19 },
  hardwareAntennaDelay: false,
  legacyOffsetEnabled: true,
  dsCalibratedMask: 0x0f,
  rangeFilterMode: 0,
  legacyAdaptiveMode: 'off',
  c9_2MotionMode: 'off',
  c9_2GlobalMotionState: 'static',
  phyProfileId: 2,
  phyProfile: 'fast-256',
  spiClockMhz: 16,
  receivedAt: 123,
  firmwareBuildId: 'fast-50-ds',
};

const anchorLayout = [
  { id: 1, x: 0, y: 0 },
  { id: 2, x: 5, y: 0 },
  { id: 3, x: 0, y: 4 },
  { id: 4, x: 5, y: 4 },
];

function anchor(id: number, rangeMm: number, valid = true): AnchorSample {
  return {
    id,
    valid,
    ageMs: valid ? id * 2 : 0xffff,
    rawMm: valid ? rangeMm : null,
    filtMm: valid ? rangeMm - 5 : null,
    fppDbm: valid ? -80 - id : null,
    status: valid ? 0 : 1,
  };
}

function sample(seq: number, transportSeq: number, clientTime: number): RangeSample {
  const anchors = [anchor(4, 1400), anchor(1, 1000), anchor(3, 1300), anchor(2, 1200)];
  return {
    seq,
    transportSeq,
    timeMs: (1000 + seq) >>> 0,
    clientTime,
    anchors,
    anchorsById: Object.fromEntries(anchors.map(record => [record.id, record])),
  };
}

function buildDatasetCsv(): string {
  return buildReplayCsv(
    [sample(7, 10, 1_000), sample(9, 13, 1_020)],
    createReplayLogContext({
      startedAtMs: 900,
      connectionMode: 'usb',
      firmwareInfo,
      anchorLayout,
      positionRangeSource: 'raw',
    }),
  );
}

describe('C5 replay dataset', () => {
  it('round-trips deterministic metadata and canonical anchor IDs', () => {
    const result = parseReplayCsv(buildDatasetCsv());
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    expect(result.dataset.metadata.replayFormatVersion).toBe(REPLAY_CSV_FORMAT_VERSION);
    expect(result.dataset.metadata.radioProfileId).toBe('fast-256');
    expect(result.dataset.metadata.anchorLayout).toEqual(anchorLayout);
    expect(result.dataset.metadata.positionRangeSource).toBe('raw');
    expect(result.dataset.anchorIds).toEqual([1, 2, 3, 4]);
    expect(result.dataset.frames).toHaveLength(2);
    expect(result.dataset.frames[0].records.map(record => record.id)).toEqual([1, 2, 3, 4]);
    expect(result.dataset.frames[0].records[3].correctedRawMm).toBe(1400);
  });

  it('preserves quoted firmware metadata', () => {
    const quotedCsv = buildReplayCsv(
      [sample(7, 10, 1_000)],
      createReplayLogContext({
        startedAtMs: 900,
        connectionMode: 'usb',
        firmwareInfo: { ...firmwareInfo, firmwareBuildId: 'fast-50,ds' },
      }),
    );
    const result = parseReplayCsv(quotedCsv);
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    expect(result.dataset.metadata.profileFingerprint).toContain('build:fast-50,ds');
  });

  it('fails closed on a profile change inside one session', () => {
    const firstCsv = buildDatasetCsv().split('\n');
    const changedProfileCsv = buildReplayCsv(
      [sample(9, 13, 1_020)],
      createReplayLogContext({
        startedAtMs: 900,
        connectionMode: 'usb',
        firmwareInfo: { ...firmwareInfo, firmwareBuildId: 'different-profile' },
        anchorLayout,
        positionRangeSource: 'raw',
      }),
    ).split('\n');
    const result = parseReplayCsv([firstCsv[0], firstCsv[1], changedProfileCsv[1]].join('\n'));
    expect(result.ok).toBe(false);
    if (result.ok) return;
    expect(result.errors.some(error => error.includes('profileFingerprint'))).toBe(true);
  });

  it('rejects a valid anchor record with a missing range', () => {
    const invalid = sample(7, 10, 1_000);
    invalid.anchorsById[1].rawMm = null;
    const result = parseReplayCsv(buildReplayCsv(
      [invalid],
      createReplayLogContext({
        startedAtMs: 900,
        connectionMode: 'usb',
        firmwareInfo,
        anchorLayout,
        positionRangeSource: 'raw',
      }),
    ));
    expect(result.ok).toBe(false);
  });
});

describe('C5 replay runner', () => {
  it('runs the same immutable frames through multiple variants', () => {
    const parsed = parseReplayCsv(buildDatasetCsv());
    expect(parsed.ok).toBe(true);
    if (!parsed.ok) return;
    const report = runReplayVariants(parsed.dataset, [
      recordedRawVariant,
      recordedFilteredVariant,
      c9MedianGateReplayVariant,
      adaptiveLegacyReplayVariant,
      c9_2MotionAdaptiveReplayVariant,
    ]);
    expect(report.provisional).toBe(true);
    expect(report.variants.map(variant => variant.id)).toEqual([
      'recorded-corrected-raw',
      'recorded-firmware-filtered',
      'c9-median-gate-raw',
      'legacy-adaptive-shadow-raw-v1',
      'c9.2-motion-adaptive-shadow-raw-v1',
    ]);
    expect(report.variants[0].totalFrames).toBe(2);
    expect(report.variants[0].meanUsableAnchorCount).toBe(4);
    expect(report.variants[0].sourceGapCount).toBe(1);
    expect(report.variants[0].transportGapCount).toBe(2);
    expect(report.variants[0].nonFiniteOutputs).toBe(0);
    expect(report.variants[2].emittedFrames).toBe(2);
    expect(report.variants[3].emittedFrames).toBe(2);
    expect(report.variants[4].emittedFrames).toBe(0); // controlled 3-sample boot candidate
    expect(report.variants[1].perAnchor[0].meanAbsRawDeltaMm).toBe(5);
    expect(report.variants[2].perAnchor[0].meanAbsRawDeltaMm).toBe(0);
    expect(report.variants[0].perAnchor[0].meanRangeMm).toBe(1000);
    expect(report.variants[1].perAnchor[0].meanRangeMm).toBe(995);
  });

  it('runs the C6 robust solver as a provisional shadow variant', () => {
    const target = { x: 2, y: 1.5 };
    const solverSample = sample(7, 10, 1_000);
    for (const anchorPosition of anchorLayout) {
      const record = solverSample.anchorsById[anchorPosition.id];
      const rangeMm = Math.hypot(
        target.x - anchorPosition.x,
        target.y - anchorPosition.y,
      ) * 1000;
      record.rawMm = rangeMm;
      record.filtMm = rangeMm;
    }
    const parsed = parseReplayCsv(buildReplayCsv(
      [solverSample],
      createReplayLogContext({
        startedAtMs: 900,
        connectionMode: 'usb',
        firmwareInfo,
        anchorLayout,
        positionRangeSource: 'raw',
      }),
    ));
    expect(parsed.ok).toBe(true);
    if (!parsed.ok) return;
    const report = runReplayVariants(parsed.dataset, [
      createC6RobustReplayVariant(parsed.dataset),
      createC8AdaptiveReplayVariant(parsed.dataset),
    ]);
    expect(report.variants[0].positionFrames).toBe(1);
    expect(report.variants[0].statusCounts['valid:four-anchor']).toBe(1);
    expect(report.variants[0].nonFiniteOutputs).toBe(0);
    expect(report.variants[1].positionFrames).toBe(1);
    expect(report.variants[1].statusCounts['c8:measured:initialized']).toBe(1);
    expect(report.variants[1].nonFiniteOutputs).toBe(0);
  });

  it('runs the standalone replay report CLI with the C6 variant', () => {
    const tempDirectory = mkdtempSync(join(tmpdir(), 'uwb-replay-cli-test-'));
    try {
      const inputPath = join(tempDirectory, 'input.csv');
      writeFileSync(inputPath, buildDatasetCsv(), 'utf8');
      const scriptPath = fileURLToPath(new URL('../../scripts/replay-report.mjs', import.meta.url));
      const output = execFileSync(process.execPath, [scriptPath, inputPath], {
        encoding: 'utf8',
        windowsHide: true,
      });
      const parsed = JSON.parse(output) as {
        ok: boolean;
        report: { variants: { id: string }[] };
      };
      expect(parsed.ok).toBe(true);
      expect(parsed.report.variants.map(variant => variant.id)).toContain('c6-robust-shadow-raw');
      expect(parsed.report.variants.map(variant => variant.id)).toContain('c8-adaptive-shadow-raw');
      expect(parsed.report.variants.map(variant => variant.id)).toContain('c9-median-gate-raw');
      expect(parsed.report.variants.map(variant => variant.id)).toContain('legacy-adaptive-shadow-raw-v1');
    } finally {
      rmSync(tempDirectory, { recursive: true, force: true });
    }
  });

  it('rejects duplicate variant IDs', () => {
    const parsed = parseReplayCsv(buildDatasetCsv());
    expect(parsed.ok).toBe(true);
    if (!parsed.ok) return;
    expect(() => runReplayVariants(parsed.dataset, [recordedRawVariant, recordedRawVariant])).toThrow(/unique/);
  });
});
