const UINT32_HALF = 0x8000_0000;

export interface McuClockAdvance {
  dtSec: number | null;
  resetDetected: boolean;
  sourceGapCount: number;
}

export class McuClock {
  private previousSequence: number | null = null;
  private previousTimeMs: number | null = null;

  constructor(private readonly maximumPlausibleDeltaMs = 60_000) {}

  reset(): void {
    this.previousSequence = null;
    this.previousTimeMs = null;
  }

  advance(sequence: number, timeMs: number): McuClockAdvance {
    if (!Number.isInteger(sequence) || sequence < 0 || sequence > 0xffff_ffff
      || !Number.isInteger(timeMs) || timeMs < 0 || timeMs > 0xffff_ffff) {
      this.reset();
      return { dtSec: null, resetDetected: true, sourceGapCount: 0 };
    }
    const currentSequence = sequence >>> 0;
    const currentTime = timeMs >>> 0;
    if (this.previousSequence === null || this.previousTimeMs === null) {
      this.previousSequence = currentSequence;
      this.previousTimeMs = currentTime;
      return { dtSec: null, resetDetected: false, sourceGapCount: 0 };
    }

    const sequenceDelta = (currentSequence - this.previousSequence) >>> 0;
    const timeDeltaMs = (currentTime - this.previousTimeMs) >>> 0;
    const invalid = sequenceDelta === 0 || sequenceDelta >= UINT32_HALF
      || timeDeltaMs === 0 || timeDeltaMs >= UINT32_HALF
      || timeDeltaMs > this.maximumPlausibleDeltaMs;
    this.previousSequence = currentSequence;
    this.previousTimeMs = currentTime;
    if (invalid) {
      return { dtSec: null, resetDetected: true, sourceGapCount: 0 };
    }
    return {
      dtSec: timeDeltaMs / 1000,
      resetDetected: false,
      sourceGapCount: Math.max(0, sequenceDelta - 1),
    };
  }
}
