#!/usr/bin/env python3
"""
analyze_fpp_bias.py — Đối chiếu First-Path Power (FPP) với sai số khoảng cách,
để xác nhận/loại trừ giả thuyết "DW1000 RX-power-dependent range bias" khi
offset calibration của một anchor tăng/giảm đơn điệu theo khoảng cách thật.

Bối cảnh: calibration_offset_m hiện chỉ là 1 hằng số cộng dồn — nếu sai số thật
phụ thuộc vào công suất tín hiệu thu (FPP), một hằng số duy nhất không thể sửa
đúng ở mọi cự ly. Script này dùng log THÔ (không phải CSV Wizard, vì Wizard
không xuất cột FPP) để kiểm tra 2 mức bằng chứng:

  1. Across-distance: FPP trung bình mỗi cự ly có tương quan với offset đo được
     ở cự ly đó không? (yếu hơn — có thể lẫn với chính sai số khoảng cách thật)
  2. Within-capture (bằng chứng MẠNH hơn): ở CÙNG 1 cự ly thật cố định, mẫu nào
     có FPP khác biệt thì raw_mm có lệch theo không? Vì khoảng cách thật không
     đổi trong 1 file, bất kỳ tương quan nào ở đây chắc chắn là do thiên lệch
     phần cứng theo công suất tín hiệu, không phải do hình học/khoảng cách.

Cách dùng:
    1. Thu log thô (giống calibration_from_log.py) tại CÙNG các cự ly đã dùng
       để calib (Wizard), cho 1 anchor cụ thể — mỗi cự ly 1 file, >= 200 mẫu.
    2. Chạy:

       python analyze_fpp_bias.py --anchor 2 \\
           3.00=anchor2_300.log 3.60=anchor2_360.log \\
           4.80=anchor2_480.log 6.00=anchor2_600.log

       Mỗi tham số vị trí có dạng  <true_dist_m>=<đường_dẫn_file_log>.
       (Dùng dấu "=", không dùng ":", để tránh xung đột với "C:\\..." trên Windows.)

Định dạng dòng "R" — xem STM32_UWB/TAG/Core/Src/telemetry.c:
    R,seq,time_ms,<a_id,a_valid,a_age_ms,a_raw_mm,a_filt_mm,a_fpp_cdbm> x3

    fpp_cdbm là FPP tính bằng centi-dBm (dBm * 100) — script tự chia 100.
"""

import argparse
import statistics
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")


def parse_log(path, anchor_id):
    """Đọc file log, trả về list (raw_mm, fpp_dbm) của đúng anchor_id, chỉ lấy mẫu valid=1."""
    samples = []
    total_lines = 0
    r_lines = 0
    skipped_invalid = 0

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            total_lines += 1
            line = line.strip()
            if not line.startswith("R,"):
                continue
            parts = line.split(",")
            if len(parts) < 21:
                continue
            r_lines += 1

            for i in range(3):
                base = 3 + i * 6
                try:
                    a_id = int(parts[base])
                    a_valid = parts[base + 1] == "1"
                    a_raw_mm = int(parts[base + 3])
                    a_fpp_cdbm = int(parts[base + 5])
                except (ValueError, IndexError):
                    continue

                if a_id != anchor_id:
                    continue
                if not a_valid:
                    skipped_invalid += 1
                    continue
                samples.append((a_raw_mm, a_fpp_cdbm / 100.0))

    return samples, {
        "total_lines": total_lines,
        "r_lines": r_lines,
        "skipped_invalid": skipped_invalid,
    }


def parse_capture_arg(arg):
    if "=" not in arg:
        raise argparse.ArgumentTypeError(
            f"'{arg}' sai định dạng — cần <true_dist_m>=<file_log> (vd: 3.60=anchor2_360.log)"
        )
    dist_str, path = arg.split("=", 1)
    try:
        dist = float(dist_str)
    except ValueError:
        raise argparse.ArgumentTypeError(f"'{dist_str}' không phải số (true_dist_m)")
    return dist, path


def safe_corr(xs, ys):
    """Pearson r, trả về None nếu không đủ dữ liệu hoặc phương sai = 0."""
    if len(xs) < 3:
        return None
    try:
        return statistics.correlation(xs, ys)
    except statistics.StatisticsError:
        return None


def main():
    ap = argparse.ArgumentParser(
        description="Đối chiếu FPP vs sai số khoảng cách để xác nhận DW1000 RX-power bias.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--anchor", type=int, required=True, choices=[1, 2, 3], help="ID anchor (1/2/3)")
    ap.add_argument(
        "captures",
        nargs="+",
        type=parse_capture_arg,
        help="Danh sách <true_dist_m>=<file_log>, mỗi cự ly 1 file",
    )
    args = ap.parse_args()

    captures = sorted(args.captures, key=lambda c: c[0])

    print(f"=== Anchor {args.anchor}: đối chiếu FPP vs sai số khoảng cách ===\n")

    per_distance = []       # (true_dist, mean_fpp, offset_m, n)
    pooled_resid_mm = []    # raw_mm - mean_mm (đã trừ trung bình riêng của từng cự ly)
    pooled_fpp_dm = []      # fpp - mean_fpp (đã trừ trung bình riêng của từng cự ly)

    for true_dist, path in captures:
        samples, stats = parse_log(path, args.anchor)
        n = len(samples)
        if n < 10:
            print(f"⚠️  {path} (true_dist={true_dist}m): chỉ {n} mẫu hợp lệ — QUÁ ÍT, bỏ qua file này.")
            continue

        raw_vals = [s[0] for s in samples]
        fpp_vals = [s[1] for s in samples]

        mean_mm = statistics.mean(raw_vals)
        std_mm = statistics.pstdev(raw_vals)
        mean_fpp = statistics.mean(fpp_vals)
        std_fpp = statistics.pstdev(fpp_vals)
        offset_m = (mean_mm / 1000.0) - true_dist

        print(f"--- {path}  (true_dist = {true_dist:.2f}m, n = {n}) ---")
        print(f"  raw_mm:  mean = {mean_mm:.1f}mm   std = {std_mm:.1f}mm")
        print(f"  FPP:     mean = {mean_fpp:.2f}dBm  std = {std_fpp:.2f}dB")
        print(f"  offset (mean_mm/1000 - true_dist) = {offset_m:.4f} m")

        r_within = safe_corr(fpp_vals, raw_vals)
        if r_within is not None:
            print(f"  → Tương quan NỘI BỘ (fpp vs raw_mm, cùng cự ly cố định): r = {r_within:+.3f}")
        print()

        per_distance.append((true_dist, mean_fpp, offset_m, n))
        for rv, fv in zip(raw_vals, fpp_vals):
            pooled_resid_mm.append(rv - mean_mm)
            pooled_fpp_dm.append(fv - mean_fpp)

    if len(per_distance) < 3:
        print("⚠️  Cần >= 3 cự ly hợp lệ để kết luận đáng tin — hiện chưa đủ.")
        return

    print("=" * 70)
    print("BẰNG CHỨNG 1 — Across-distance: FPP trung bình vs offset theo cự ly")
    print("=" * 70)
    for true_dist, mean_fpp, offset_m, n in per_distance:
        print(f"  {true_dist:6.2f}m | FPP={mean_fpp:7.2f}dBm | offset={offset_m:+.4f}m | n={n}")

    dists = [p[0] for p in per_distance]
    fpps = [p[1] for p in per_distance]
    offsets = [p[2] for p in per_distance]

    r_fpp_vs_offset = safe_corr(fpps, offsets)
    r_dist_vs_fpp = safe_corr(dists, fpps)
    print()
    if r_fpp_vs_offset is not None:
        print(f"  r(FPP trung bình, offset)     = {r_fpp_vs_offset:+.3f}")
    if r_dist_vs_fpp is not None:
        print(f"  r(khoảng cách thật, FPP)      = {r_dist_vs_fpp:+.3f}  (âm mạnh = tín hiệu yếu dần theo cự ly, đúng vật lý)")

    print()
    print("=" * 70)
    print("BẰNG CHỨNG 2 — Within-capture (MẠNH HƠN): fpp vs residual, dồn tất cả cự ly")
    print("=" * 70)
    print("Ở đây khoảng cách thật CỐ ĐỊNH trong mỗi file → residual chỉ có thể do")
    print("nhiễu ngẫu nhiên hoặc do thiên lệch phần cứng theo công suất tín hiệu.")
    print("(Cả 2 trục đều đã trừ trung bình RIÊNG của từng cự ly trước khi gộp, để")
    print(" tránh chênh lệch FPP giữa các cự ly pha loãng tương quan nội bộ thật.)\n")

    r_pooled = safe_corr(pooled_fpp_dm, pooled_resid_mm)
    slope = intercept = None
    if r_pooled is not None:
        try:
            slope, intercept = statistics.linear_regression(pooled_fpp_dm, pooled_resid_mm)
        except statistics.StatisticsError:
            pass
        print(f"  r(fpp đã demean, raw_mm đã demean) dồn {len(pooled_fpp_dm)} mẫu = {r_pooled:+.3f}")
        if slope is not None:
            print(f"  Hồi quy tuyến tính: residual_mm ≈ {slope:+.2f} * (fpp - mean_fpp_cự_ly)")
            print(f"  → Cứ 1dB FPP lệch khỏi trung bình cự ly đó, raw_mm lệch khoảng {slope:+.2f}mm.")

    print()
    print("=" * 70)
    print("KẾT LUẬN")
    print("=" * 70)

    def verdict(r):
        if r is None:
            return "không đủ dữ liệu"
        ar = abs(r)
        if ar >= 0.5:
            return "MẠNH"
        if ar >= 0.25:
            return "TRUNG BÌNH"
        return "YẾU / không rõ"

    print(f"  Tương quan across-distance (FPP vs offset):     {verdict(r_fpp_vs_offset)}"
          + (f"  (r={r_fpp_vs_offset:+.3f})" if r_fpp_vs_offset is not None else ""))
    print(f"  Tương quan within-capture (fpp vs residual):    {verdict(r_pooled)}"
          + (f"  (r={r_pooled:+.3f})" if r_pooled is not None else ""))
    print()

    strong_within = r_pooled is not None and abs(r_pooled) >= 0.25
    strong_across = r_fpp_vs_offset is not None and abs(r_fpp_vs_offset) >= 0.25

    if strong_within:
        print("→ XÁC NHẬN: raw_mm phụ thuộc vào FPP ngay cả khi khoảng cách thật KHÔNG đổi.")
        print("  Đây là bằng chứng trực tiếp cho DW1000 RX-power-dependent range bias.")
        print("  Hướng sửa: thêm bảng bù range-bias theo FPP vào compute_distance_mm()")
        print("  (giá trị chuẩn: DW1000 User Manual Table 4.3 / Decawave APS011),")
        print("  áp dụng SAU khi trừ calibration_offset_m, TRƯỚC khi clamp dist_m >= 0.")
    elif strong_across:
        print("→ CÓ DẤU HIỆU nhưng chưa chắc chắn: chỉ thấy tương quan across-distance,")
        print("  không thấy rõ trong từng cự ly cố định (within-capture yếu).")
        print("  Có thể lẫn giữa RX-power-bias thật và sai số đo/vị trí giữa các lần đo.")
        print("  Khuyến nghị: đo lại 1-2 cự ly với NHIỀU mẫu hơn (>=1000) để within-capture")
        print("  rõ hơn, trước khi sửa firmware.")
    else:
        print("→ KHÔNG thấy bằng chứng rõ ràng cho RX-power bias ở dữ liệu này.")
        print("  Khả năng cao nguyên nhân là VẬT LÝ/MÔI TRƯỜNG (LOS không sạch, vị trí")
        print("  đặt lệch giữa các lần đo, vật cản/kim loại tăng dần theo cự ly...),")
        print("  không phải do thiếu bù trong code. Kiểm tra lại setup đo trước khi sửa code.")


if __name__ == "__main__":
    main()
