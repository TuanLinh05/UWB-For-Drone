/**
 * fppBiasAnalysis.ts — Đối chiếu First-Path Power (FPP) với sai số khoảng cách,
 * để xác nhận/loại trừ giả thuyết "DW1000 RX-power-dependent range bias" ngay
 * trong GUI, thay cho việc thu log thô rồi chạy tools/analyze_fpp_bias.py.
 *
 * Cùng phương pháp thống kê với analyze_fpp_bias.py (đã kiểm chứng bằng dữ liệu
 * giả lập ở đó): 2 mức bằng chứng — across-distance (yếu hơn) và within-capture
 * (mạnh hơn, vì khoảng cách thật cố định trong mỗi lần đo nên tương quan chỉ có
 * thể do thiên lệch phần cứng theo công suất tín hiệu).
 */

export function mean(xs: number[]): number {
  return xs.reduce((a, b) => a + b, 0) / xs.length;
}

export function pstdev(xs: number[]): number {
  const m = mean(xs);
  const variance = xs.reduce((s, v) => s + (v - m) ** 2, 0) / xs.length;
  return Math.sqrt(variance);
}

export function pearsonCorrelation(xs: number[], ys: number[]): number | null {
  const n = xs.length;
  if (n < 3 || ys.length !== n) return null;
  const mx = mean(xs), my = mean(ys);
  let sxy = 0, sxx = 0, syy = 0;
  for (let i = 0; i < n; i++) {
    const dx = xs[i] - mx, dy = ys[i] - my;
    sxy += dx * dy;
    sxx += dx * dx;
    syy += dy * dy;
  }
  if (sxx === 0 || syy === 0) return null;
  return sxy / Math.sqrt(sxx * syy);
}

export function linearRegression(xs: number[], ys: number[]): { slope: number; intercept: number } | null {
  const n = xs.length;
  if (n < 2 || ys.length !== n) return null;
  const mx = mean(xs), my = mean(ys);
  let sxy = 0, sxx = 0;
  for (let i = 0; i < n; i++) {
    const dx = xs[i] - mx;
    sxy += dx * (ys[i] - my);
    sxx += dx * dx;
  }
  if (sxx === 0) return null;
  const slope = sxy / sxx;
  return { slope, intercept: my - slope * mx };
}

export interface FppBiasPointInput {
  trueDistanceM: number;
  rawMm: number[];
  fppDbm: number[];
}

export interface FppBiasDistanceResult {
  trueDistanceM: number;
  n: number;
  meanRawMm: number;
  stdRawMm: number;
  meanFppDbm: number;
  stdFppDbm: number;
  offsetM: number;
}

export type FppBiasVerdict = 'confirmed' | 'inconclusive' | 'no-evidence' | 'insufficient-data';

export interface FppBiasAnalysisResult {
  perDistance: FppBiasDistanceResult[];
  rAcrossDistance: number | null;   // r(mean FPP theo cự ly, offset theo cự ly)
  rDistanceVsFpp: number | null;    // r(khoảng cách thật, mean FPP) — kỳ vọng âm mạnh (tín hiệu yếu dần)
  rWithinCapture: number | null;    // r(fpp đã demean, raw_mm đã demean), dồn tất cả cự ly
  withinSlopeMmPerDb: number | null;
  withinIntercept: number | null;
  verdict: FppBiasVerdict;
  verdictText: string;
}

const MIN_SAMPLES_PER_DISTANCE = 10;
const MIN_DISTANCES = 3;
const MEDIUM_R = 0.25;

/**
 * @param pointsIn  1 phần tử / cự ly đã đo cho MỘT anchor (rawMm và fppDbm phải
 *                  cùng chỉ số mẫu — lấy trực tiếp từ dữ liệu Calibration Wizard).
 */
export function analyzeFppBias(pointsIn: FppBiasPointInput[]): FppBiasAnalysisResult {
  const perDistance: FppBiasDistanceResult[] = [];
  const pooledResidRaw: number[] = [];
  const pooledResidFpp: number[] = [];

  for (const p of pointsIn) {
    if (p.rawMm.length < MIN_SAMPLES_PER_DISTANCE || p.fppDbm.length < MIN_SAMPLES_PER_DISTANCE) continue;
    const meanRawMm = mean(p.rawMm);
    const stdRawMm = pstdev(p.rawMm);
    const meanFppDbm = mean(p.fppDbm);
    const stdFppDbm = pstdev(p.fppDbm);
    const offsetM = meanRawMm / 1000 - p.trueDistanceM;
    perDistance.push({ trueDistanceM: p.trueDistanceM, n: p.rawMm.length, meanRawMm, stdRawMm, meanFppDbm, stdFppDbm, offsetM });

    const n = Math.min(p.rawMm.length, p.fppDbm.length);
    for (let i = 0; i < n; i++) {
      pooledResidRaw.push(p.rawMm[i] - meanRawMm);
      pooledResidFpp.push(p.fppDbm[i] - meanFppDbm);
    }
  }

  perDistance.sort((a, b) => a.trueDistanceM - b.trueDistanceM);

  if (perDistance.length < MIN_DISTANCES) {
    return {
      perDistance, rAcrossDistance: null, rDistanceVsFpp: null,
      rWithinCapture: null, withinSlopeMmPerDb: null, withinIntercept: null,
      verdict: 'insufficient-data',
      verdictText: `Cần ≥${MIN_DISTANCES} cự ly hợp lệ (≥${MIN_SAMPLES_PER_DISTANCE} mẫu/cự ly) để kết luận — hiện có ${perDistance.length}. Thêm cự ly ở Step 4 ("Add Another Distance") rồi quay lại đây.`,
    };
  }

  const dists = perDistance.map(p => p.trueDistanceM);
  const fpps = perDistance.map(p => p.meanFppDbm);
  const offsets = perDistance.map(p => p.offsetM);

  const rAcrossDistance = pearsonCorrelation(fpps, offsets);
  const rDistanceVsFpp = pearsonCorrelation(dists, fpps);
  const rWithinCapture = pearsonCorrelation(pooledResidFpp, pooledResidRaw);
  const reg = rWithinCapture !== null ? linearRegression(pooledResidFpp, pooledResidRaw) : null;

  const strongWithin = rWithinCapture !== null && Math.abs(rWithinCapture) >= MEDIUM_R;
  const strongAcross = rAcrossDistance !== null && Math.abs(rAcrossDistance) >= MEDIUM_R;

  let verdict: FppBiasVerdict;
  let verdictText: string;

  if (strongWithin) {
    verdict = 'confirmed';
    verdictText =
      'XÁC NHẬN: raw_mm phụ thuộc vào FPP ngay cả khi khoảng cách thật KHÔNG đổi trong từng lần đo. ' +
      'Đây là bằng chứng trực tiếp cho DW1000 RX-power-dependent range bias. ' +
      'Hướng sửa: thêm bảng bù range-bias theo FPP vào compute_distance_mm() (DW1000 User Manual Table 4.3 / Decawave APS011), ' +
      'áp dụng SAU khi trừ calibration_offset_m, TRƯỚC khi clamp dist_m >= 0.';
  } else if (strongAcross) {
    verdict = 'inconclusive';
    verdictText =
      'CÓ DẤU HIỆU nhưng chưa chắc chắn: chỉ thấy tương quan giữa các cự ly (across-distance), ' +
      'không thấy rõ trong từng cự ly cố định (within-capture yếu). Có thể lẫn giữa RX-power-bias thật ' +
      'và sai số đo/vị trí giữa các lần đo. Nên đo lại 1-2 cự ly với nhiều mẫu hơn (≥1000) trước khi sửa firmware.';
  } else {
    verdict = 'no-evidence';
    verdictText =
      'KHÔNG thấy bằng chứng rõ ràng cho RX-power bias ở dữ liệu này. Khả năng cao nguyên nhân là vật lý/môi trường ' +
      '(LOS không sạch, vị trí đặt lệch giữa các lần đo, vật cản/kim loại tăng dần theo cự ly...), không phải do thiếu bù trong code.';
  }

  return {
    perDistance, rAcrossDistance, rDistanceVsFpp, rWithinCapture,
    withinSlopeMmPerDb: reg?.slope ?? null, withinIntercept: reg?.intercept ?? null,
    verdict, verdictText,
  };
}

export function verdictStrength(r: number | null): string {
  if (r === null) return 'không đủ dữ liệu';
  const ar = Math.abs(r);
  if (ar >= 0.5) return 'MẠNH';
  if (ar >= MEDIUM_R) return 'TRUNG BÌNH';
  return 'YẾU / không rõ';
}
