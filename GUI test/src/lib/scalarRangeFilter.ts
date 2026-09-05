export interface MedianState { buf: [number, number, number]; idx: number; count: number; }
export const createMedianState = (): MedianState => ({ buf: [0,0,0], idx: 0, count: 0 });

export function applyMedianFilter(s: MedianState, newVal: number): number {
  s.buf[s.idx] = newVal;
  s.idx = ((s.idx + 1) % 3) as 0|1|2;
  if (s.count < 3) s.count++;
  if (s.count < 3) return newVal;
  return [...s.buf].sort((a, b) => a - b)[1];
}

export interface ScalarKalmanState { Q: number; R: number; x: number; P: number; initialized: boolean; }
export const createScalarKalman = (Q = 0.05): ScalarKalmanState =>
  ({ Q, R: 1000, x: 0, P: 1.0, initialized: false });

export function rForFpp(fppDbm: number): number {
  if (fppDbm <= -95) return 10000;
  if (fppDbm > -75) return 50;
  if (fppDbm > -82) return 200;
  return 1000;
}

export function updateScalarKalman(kf: ScalarKalmanState, meas: number, fppDbm: number): number {
  if (!kf.initialized) { kf.x = meas; kf.initialized = true; }
  kf.R = rForFpp(fppDbm);
  kf.P = kf.P + kf.Q;
  const K = kf.P / (kf.P + kf.R);
  kf.x = kf.x + K * (meas - kf.x);
  kf.P = (1 - K) * kf.P;
  return kf.x;
}

export interface OutlierGateParams { jumpUpMm: number; jumpDownMm: number; snapAfter: number; }
// Firmware (tag_ranging.c) uses dt-aware thresholds that scale by elapsed time since last
// accepted sample: up = VMAX*dt + MARGIN_UP, down = -(VMAX*dt + MARGIN_DOWN).
// These static values match exactly 1 × 20ms cycle (dt=0.02s, VMAX=10000mm/s):
//   up  = 10000*0.02 + 100 = 300mm
//   down= -(10000*0.02 + 250) = -450mm
//   snap= 15 samples (0.3s, was 50=1s)
export const DEFAULT_GATE: OutlierGateParams = { jumpUpMm: 300, jumpDownMm: -450, snapAfter: 15 };

export function applyOutlierGate(
  kf: ScalarKalmanState, outlierCountRef: { count: number }, meas: number, p = DEFAULT_GATE
): boolean {
  const jump = meas - kf.x;
  if (jump > p.jumpUpMm || jump < p.jumpDownMm) {
    outlierCountRef.count++;
    if (outlierCountRef.count < p.snapAfter) return true;
    kf.x = meas; outlierCountRef.count = 0; return false;
  }
  outlierCountRef.count = 0;
  return false;
}

export function runFilterPipeline(
  samples: { rawMm: number; fppDbm: number }[],
  Q = 0.05, gate: OutlierGateParams = DEFAULT_GATE
): number[] {
  const median = createMedianState();
  const kf = createScalarKalman(Q);
  const outlierRef = { count: 0 };
  return samples.map(s => {
    const med = applyMedianFilter(median, s.rawMm);
    const isOutlier = applyOutlierGate(kf, outlierRef, med, gate);
    const fppForKalman = isOutlier ? -100 : s.fppDbm;
    return updateScalarKalman(kf, med, fppForKalman);
  });
}
