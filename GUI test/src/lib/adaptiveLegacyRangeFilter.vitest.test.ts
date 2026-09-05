import { describe, expect, it } from 'vitest';
import {
  AdaptiveLegacyRangeFilter,
  DEFAULT_ADAPTIVE_LEGACY_CONFIG,
} from './adaptiveLegacyRangeFilter';
import {
  adaptiveLegacyReplayVariant,
  c9MedianGateReplayVariant,
} from './replayRunner';
import type { ReplayVariantFrameOutput } from './replayRunner';
import type { ReplayRangeFrame } from './replayDataset';

function process(filter: AdaptiveLegacyRangeFilter, rawMm: number, nowMs: number, fppDbm = -70) {
  return filter.process({ rawMm, fppDbm, nowMs });
}

function frame(rawMm: number, timeMs: number, measurementTimeMs: number | null): ReplayRangeFrame {
  return {
    replayFormatVersion: 'uwb-replay-csv-v2',
    telemetrySchemaVersion: 1,
    pipelineConfigVersion: 'test',
    profileFingerprint: 'test',
    calibrationProfileId: 'ds',
    radioProfileId: 'fast-256',
    rangeFilterMode: '0',
    connectionMode: 'usb',
    seq: timeMs >>> 0,
    transportSeq: timeMs >>> 0,
    timeMs,
    clientTimeMs: timeMs,
    records: [{
      id: 1,
      valid: true,
      status: 0,
      ageMs: 0,
      measurementTimeMs,
      correctedRawMm: rawMm,
      filteredMm: rawMm,
      fppDbm: -80,
      rangingMode: 'DS',
      calibrationMissing: false,
      diagnosticRawMm: null,
      diagnosticFppDbm: null,
    }],
    anchorLayoutJson: '',
    anchorLayoutFingerprint: '',
    positionRangeSource: 'filtered',
  };
}

function record(id: number, rawMm: number, measurementTimeMs: number) {
  return {
    ...frame(rawMm, measurementTimeMs, measurementTimeMs).records[0],
    id,
  };
}

function frameWithRecords(timeMs: number, records: ReplayRangeFrame['records']): ReplayRangeFrame {
  return { ...frame(0, timeMs, timeMs), records };
}

function rangesById(output: ReplayVariantFrameOutput) {
  return Object.fromEntries(output.observations
    .map(observation => [observation.anchorId, observation.rangeMm])
    .sort(([left], [right]) => left - right));
}

describe('Adaptive Legacy replay model', () => {
  it('matches the independently frozen Legacy golden vector when disabled', () => {
    const filter = new AdaptiveLegacyRangeFilter('off');
    const input = [1000, 1100, 1100, 1000, 1000, 5000, 1000, 1000];
    const expected = [1000, 1002, 1004, 1006, 1006, 1006, 1005, 1005];

    input.forEach((rawMm, index) => {
      const output = process(filter, rawMm, 20 + index * 20);
      expect(output.publishValid).toBe(true);
      expect(output.filteredMm).toBe(expected[index]);
    });
  });

  it('keeps static weak-FPP output bit-compatible with Legacy and never false-tracks', () => {
    const baseline = new AdaptiveLegacyRangeFilter('off');
    const candidate = new AdaptiveLegacyRangeFilter('active');
    const noise = [-90, 60, -35, 95, -70, 40, 0, -60, 80, -20];

    for (let index = 0; index < 3000; index++) {
      const nowMs = 20 + index * 20;
      const rawMm = 2000 + noise[index % noise.length];
      const expected = process(baseline, rawMm, nowMs, -97);
      const output = process(candidate, rawMm, nowMs, -97);
      expect(output.publishValid).toBe(true);
      expect(output.filteredMm).toBe(expected.filteredMm);
      expect(output.state).not.toBe('tracking');
      expect(output.trackEnterCount).toBe(0);
    }
  });

  it('does not false-track alternating static multipath after the baseline has settled', () => {
    const filter = new AdaptiveLegacyRangeFilter('active');
    for (let index = 0; index < 10; index++) process(filter, 2000, 20 + index * 20, -97);

    for (let index = 0; index < 300; index++) {
      const output = process(filter, index % 2 === 0 ? 1725 : 2275, 220 + index * 20, -97);
      expect(output.publishValid).toBe(true);
      expect(output.state).not.toBe('tracking');
      expect(output.trackEnterCount).toBe(0);
    }
  });

  it('confirms a fresh step, reaches 90% in the seed budget, then restores stable mode', () => {
    const filter = new AdaptiveLegacyRangeFilter('active');
    for (let index = 0; index < 10; index++) process(filter, 1000, 20 + index * 20, -97);

    const stepStartMs = 220;
    let t90Ms: number | null = null;
    let latest = 1000;
    let last = process(filter, 3000, stepStartMs, -97);
    if (last.publishValid) latest = last.filteredMm ?? latest;
    for (let index = 1; index < 50; index++) {
      last = process(filter, 3000, stepStartMs + index * 20, -97);
      if (last.publishValid) latest = last.filteredMm ?? latest;
      if (t90Ms === null && latest >= 2800) t90Ms = index * 20;
    }

    expect(last.trackEnterCount).toBe(1);
    expect(t90Ms).not.toBeNull();
    expect(t90Ms).toBeLessThanOrEqual(500);
    expect(latest).toBeGreaterThanOrEqual(2900);
    expect(last.state).toBe('stable');
  });

  it('does not mix a stale median window with the new location', () => {
    const filter = new AdaptiveLegacyRangeFilter('active');
    process(filter, 1000, 20, -97);
    process(filter, 1000, 40, -97);
    process(filter, 1000, 60, -97);
    expect(process(filter, 3000, 700, -97).publishValid).toBe(false);
    expect(process(filter, 3010, 720, -97).publishValid).toBe(false);
    const reacquired = process(filter, 2990, 740, -97);
    expect(reacquired.decision).toBe('stale-reacquired');
    expect(reacquired.filteredMm).toBeGreaterThanOrEqual(2980);
    expect(reacquired.filteredMm).toBeLessThanOrEqual(3020);
  });

  it('does not use below-floor FPP to seed a stale reacquire, but accepts weak valid DS FPP', () => {
    const filter = new AdaptiveLegacyRangeFilter('active');
    process(filter, 1000, 20, -97);
    process(filter, 1000, 40, -97);
    process(filter, 1000, 60, -97);

    expect(process(filter, 3000, 700, -106).decision).toBe('stale-rejected');
    expect(process(filter, 3000, 720, -97).decision).toBe('stale-rejected');
    expect(process(filter, 3010, 740, -97).decision).toBe('stale-rejected');
    expect(process(filter, 2990, 760, -97).decision).toBe('stale-reacquired');
  });

  it('enters TRACK for an 8 m/s ramp and retains real per-sample timing', () => {
    const filter = new AdaptiveLegacyRangeFilter('active');
    for (let index = 0; index < 10; index++) process(filter, 1000, 20 + index * 20, -97);

    let latest = 1000;
    let output = process(filter, 1000, 200, -97);
    for (let index = 1; index <= 30; index++) {
      output = process(filter, 1000 + index * 160, 220 + index * 20, -97);
      if (output.publishValid) latest = output.filteredMm ?? latest;
    }

    expect(output.state).toBe('tracking');
    expect(output.trackEnterCount).toBe(1);
    expect(latest).toBeGreaterThan(4000);
  });

  it('handles a uint32 MCU tick wrap as a 20 ms interval, not a stale gap', () => {
    const filter = new AdaptiveLegacyRangeFilter('active');
    process(filter, 1000, 0xfffffff0, -97);
    const output = process(filter, 1000, 0x00000004, -97);
    expect(output.decision).toBe('accepted');
    expect(output.staleReacquireCount).toBe(0);
  });

  it('uses each anchor measurement timestamp for C9 and Adaptive Legacy gates', () => {
    const c9 = c9MedianGateReplayVariant();
    const adaptive = adaptiveLegacyReplayVariant();
    const first = frame(1000, 1000, 1000);
    /* Snapshot is at 1080 ms, but A1 was actually measured 20 ms after its
     * previous observation. 700 mm cannot pass the real dynamic gate. */
    const second = frame(1700, 1080, 1020);

    c9.process(first);
    adaptive.process(first);
    expect(c9.process(second).rejectedAnchorIds).toEqual([1]);
    expect(adaptive.process(second).rejectedAnchorIds).toEqual([1]);
  });

  it('falls back to frame MCU time only when measurement time is unavailable', () => {
    const c9 = c9MedianGateReplayVariant();
    c9.process(frame(1000, 1000, null));
    const output = c9.process(frame(1700, 1020, null));
    expect(output.rejectedAnchorIds).toEqual([1]);
  });

  it('keeps independent candidate state by Anchor ID when record order changes', () => {
    const ordered = adaptiveLegacyReplayVariant();
    const reversed = adaptiveLegacyReplayVariant();
    const sequences = [
      { timeMs: 1000, a1: 1000, a2: 2000 },
      { timeMs: 1020, a1: 1000, a2: 2000 },
      { timeMs: 1040, a1: 1700, a2: 2000 },
      { timeMs: 1060, a1: 1700, a2: 2000 },
      { timeMs: 1080, a1: 1700, a2: 2000 },
      { timeMs: 1100, a1: 1700, a2: 2000 },
    ];

    for (const sample of sequences) {
      const normalOrder = frameWithRecords(sample.timeMs, [
        record(1, sample.a1, sample.timeMs),
        record(2, sample.a2, sample.timeMs),
      ]);
      const reverseOrder = frameWithRecords(sample.timeMs, [
        record(2, sample.a2, sample.timeMs),
        record(1, sample.a1, sample.timeMs),
      ]);
      expect(rangesById(ordered.process(normalOrder))).toEqual(rangesById(reversed.process(reverseOrder)));
    }
  });

  it('keeps the documented seed configuration finite and valid', () => {
    expect(DEFAULT_ADAPTIVE_LEGACY_CONFIG.trackingGain).toBeGreaterThan(0);
    expect(DEFAULT_ADAPTIVE_LEGACY_CONFIG.trackingGain).toBeLessThan(1);
  });
});
