import { describe, expect, it } from 'vitest';
import {
  DEFAULT_MOTION_ADAPTIVE_RANGE_CONFIG,
  MotionAdaptiveRangeFilter,
} from './motionAdaptiveRangeFilter';

function process(filter: MotionAdaptiveRangeFilter, rawMm: number, nowMs: number, fppDbm = -97) {
  return filter.process({ rawMm, fppDbm, nowMs });
}

describe('C9.2 motion-regime replay model', () => {
  it('requires a compatible three-sample boot candidate before publishing', () => {
    const filter = new MotionAdaptiveRangeFilter();
    expect(process(filter, 2000, 20).publishValid).toBe(false);
    expect(process(filter, 2010, 40).publishValid).toBe(false);
    const output = process(filter, 1990, 60);
    expect(output.publishValid).toBe(true);
    expect(output.decision).toBe('reacquired');
    expect(output.state).toBe('static');
    expect(output.filteredMm).toBeGreaterThanOrEqual(1990);
    expect(output.filteredMm).toBeLessThanOrEqual(2010);
  });

  it('enters FAST for a coherent 1 m/s ramp, then returns through SETTLING to STATIC', () => {
    const filter = new MotionAdaptiveRangeFilter();
    process(filter, 1000, 20);
    process(filter, 1000, 40);
    process(filter, 1000, 60);

    let output = process(filter, 1000, 80);
    for (let index = 1; index <= 30; index++) {
      output = process(filter, 1000 + index * 20, 80 + index * 20);
    }
    expect(output.fastEnterCount).toBeGreaterThan(0);
    expect(['fast', 'settling', 'static']).toContain(output.state);
    expect(output.filteredMm).toBeGreaterThan(1400);

    const stationaryMm = 1600;
    for (let index = 1; index <= 65; index++) {
      output = process(filter, stationaryMm, 700 + index * 20);
    }
    expect(output.settlingEnterCount).toBeGreaterThan(0);
    expect(output.state).toBe('static');
  });

  it('does not false-enter SLOW or FAST during a 60 second weak-FPP static capture', () => {
    const filter = new MotionAdaptiveRangeFilter();
    const noise = [-90, 60, -35, 95, -70, 40, 0, -60, 80, -20];
    let output = process(filter, 2000, 20);
    for (let index = 1; index < 3000; index++) {
      output = process(filter, 2000 + noise[index % noise.length], 20 + index * 20);
      expect(output.state).not.toBe('slow');
      expect(output.state).not.toBe('fast');
    }
    expect(output.slowEnterCount).toBe(0);
    expect(output.fastEnterCount).toBe(0);
    expect(output.trueRejectCount).toBe(0);
  });

  it('uses SLOW before FAST for coherent 0.5 m/s radial motion', () => {
    const filter = new MotionAdaptiveRangeFilter();
    process(filter, 1000, 20);
    process(filter, 1000, 40);
    process(filter, 1000, 60);
    let output = process(filter, 1000, 80);
    for (let index = 1; index <= 60; index++) {
      output = process(filter, 1000 + index * 10, 80 + index * 20);
    }
    expect(output.slowEnterCount).toBeGreaterThan(0);
    expect(output.fastEnterCount).toBe(0);
  });

  it('rejects an impossible single spike, degrades after repeated rejects, then cleanly reacquires', () => {
    const filter = new MotionAdaptiveRangeFilter();
    process(filter, 1000, 20);
    process(filter, 1000, 40);
    process(filter, 1000, 60);
    /* The median-3 deliberately absorbs the first lone spike. Subsequent
     * persistent impossible medians count toward the controlled degrade. */
    expect(process(filter, 5000, 80).decision).toBe('accepted');
    expect(process(filter, 5000, 100).decision).toBe('kinematic-rejected');
    expect(process(filter, 5000, 120).decision).toBe('kinematic-rejected');
    const degraded = process(filter, 5000, 140);
    expect(degraded.state).toBe('degraded');
    expect(degraded.degradedEnterCount).toBe(1);
    expect(process(filter, 3000, 160).publishValid).toBe(false);
    expect(process(filter, 3010, 180).publishValid).toBe(false);
    const reacquired = process(filter, 2990, 200);
    expect(reacquired.decision).toBe('reacquired');
    expect(reacquired.filteredMm).toBeGreaterThanOrEqual(2980);
    expect(reacquired.filteredMm).toBeLessThanOrEqual(3020);
  });

  it('treats tick wrap as fresh and a real long gap as an explicit controlled reacquire', () => {
    const filter = new MotionAdaptiveRangeFilter();
    process(filter, 1000, 0xfffffff0);
    process(filter, 1000, 0x00000004);
    const normal = process(filter, 1000, 0x00000018);
    expect(normal.staleReacquireCount).toBe(0);
    expect(process(filter, 3000, 0x00000280).publishValid).toBe(false);
    expect(process(filter, 3010, 0x00000294).publishValid).toBe(false);
    const reacquired = process(filter, 2990, 0x000002a8);
    expect(reacquired.staleReacquireCount).toBe(1);
    expect(reacquired.decision).toBe('reacquired');
  });

  it('keeps the documented seed gains bounded', () => {
    expect(DEFAULT_MOTION_ADAPTIVE_RANGE_CONFIG.slowGain).toBeGreaterThan(0);
    expect(DEFAULT_MOTION_ADAPTIVE_RANGE_CONFIG.fastGain).toBeLessThan(1);
    expect(DEFAULT_MOTION_ADAPTIVE_RANGE_CONFIG.settlingGain).toBeLessThan(
      DEFAULT_MOTION_ADAPTIVE_RANGE_CONFIG.fastGain,
    );
  });
});
