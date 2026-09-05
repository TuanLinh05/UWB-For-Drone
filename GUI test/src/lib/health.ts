import type { RangeSample } from './types';

export const DEFAULT_ANCHOR_FRESHNESS_MS = 200;

export function computeAnchorUptime(
  history: RangeSample[],
  anchorId: number,
  maxAgeMs = DEFAULT_ANCHOR_FRESHNESS_MS,
): number {
  if (history.length === 0) return 0;
  const validCount = history.filter(sample => {
    const anchor = sample.anchorsById[anchorId];
    return anchor?.valid === true && anchor.ageMs <= maxAgeMs;
  }).length;
  return (validCount / history.length) * 100;
}
