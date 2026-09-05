const UINT32_MODULUS = 0x1_0000_0000;
const UINT32_HALF_RANGE = 0x8000_0000;
const UINT32_MAX = 0xffff_ffff;

export function normalizeMcuTimestamp(timeMs: number): number | null {
  if (!Number.isInteger(timeMs) || timeMs < 0 || timeMs > UINT32_MAX) return null;
  return timeMs;
}

/**
 * Return elapsed milliseconds for a wrapping uint32 MCU clock.
 * A delta larger than half the uint32 range is treated as an out-of-order
 * packet instead of a forward jump of more than 24 days.
 */
export function elapsedMcuMilliseconds(currentMs: number, previousMs: number): number | null {
  const current = normalizeMcuTimestamp(currentMs);
  const previous = normalizeMcuTimestamp(previousMs);
  if (current === null || previous === null) return null;

  const elapsed = current >= previous
    ? current - previous
    : UINT32_MODULUS - previous + current;
  return elapsed < UINT32_HALF_RANGE ? elapsed : null;
}
