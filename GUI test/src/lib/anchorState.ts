import type { AnchorSample } from './types';

export const MEASUREMENT_FRESH_MS = 200;
export const ANCHOR_ONLINE_HOLD_MS = 60;
export const ANCHOR_OFFLINE_AFTER_MS = 750;
export const TAG_ST_CALIBRATION_MISSING = 0x20;

export interface AnchorLinkState {
  isOnline: boolean;
  isStale: boolean;
  isOffline: boolean;
}

/**
 * Link state is based on time since the last successful range, not only the
 * valid bit of the latest radio slot. At a 20ms cycle, one or two missed slots
 * remain ONLINE; a longer interruption becomes STALE before OFFLINE.
 */
export function deriveAnchorLinkState(
  isDataFresh: boolean,
  ageMs: number,
  calibrationMissing: boolean,
): AnchorLinkState {
  const isOnline = isDataFresh
    && !calibrationMissing
    && ageMs <= ANCHOR_ONLINE_HOLD_MS;
  const isStale = isDataFresh
    && !calibrationMissing
    && ageMs > ANCHOR_ONLINE_HOLD_MS
    && ageMs <= ANCHOR_OFFLINE_AFTER_MS;
  return {
    isOnline,
    isStale,
    isOffline: !isOnline && !isStale && !calibrationMissing,
  };
}

/**
 * Missing calibration belongs to the active profile, so a radio timeout must
 * not erase it. A later valid production sample is the explicit clear signal.
 */
export function updateCalibrationMissingIds(
  previous: ReadonlySet<number>,
  anchorIds: readonly number[],
  anchorsById: Readonly<Record<number, AnchorSample>>,
): ReadonlySet<number> {
  const next = new Set(previous);
  for (const id of anchorIds) {
    const sample = anchorsById[id];
    if (sample?.valid) {
      next.delete(id);
    } else if (sample?.status !== undefined
      && (sample.status & TAG_ST_CALIBRATION_MISSING) !== 0) {
      next.add(id);
    }
  }

  if (next.size === previous.size && [...next].every(id => previous.has(id))) {
    return previous;
  }
  return next;
}
