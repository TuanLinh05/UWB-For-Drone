import { describe, it } from 'vitest';
import { runPositionMathSelfTests } from './positionMath.test';
import { runReplayLogSelfTests } from './replayLog.test';

describe('existing core self-tests', () => {
  it('keeps position math invariants', () => {
    runPositionMathSelfTests();
  });

  it('keeps replay export invariants', () => {
    runReplayLogSelfTests();
  });
});
