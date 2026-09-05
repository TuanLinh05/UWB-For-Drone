#!/usr/bin/env python3
"""
calibration_from_log.py — Tính offset calibration từ file log serial (ASCII).

Cách dùng:
    1. Đặt TAG cách 1 Anchor đúng khoảng cách biết trước (đo bằng thước/laser).
    2. Mở serial terminal (PuTTY/CoolTerm/Tera Term...), bật tính năng "Log to file"
       (hoặc dùng lệnh `python -m serial.tools.miniterm` / `screen ... | tee log.txt`),
       thu ít nhất ~10-20 giây (@50Hz ~ 500-1000 dòng "R,...").
    3. Chạy script này trỏ vào file log vừa thu:

           python calibration_from_log.py log.txt --anchor 1 --true-dist 2.000

       --anchor       : ID anchor cần tính (1, 2, hoặc 3)
       --true-dist    : khoảng cách THẬT đã đo bằng thước (mét)
       --trim-sigma   : (tuỳ chọn) loại mẫu lệch quá N lần std-dev trước khi tính lại
                         mean (mặc định: không loại, chỉ báo cáo)

    4. Script in ra: số mẫu hợp lệ, mean/std-dev/min/max raw_mm, và giá trị
       calibration_offset_m MỚI để dán vào tag_ranging.c.

Định dạng dòng "R" (ASCII, TELEM_ASCII=1 — xem STM32_UWB/TAG/Core/Src/telemetry.c):
    R,<seq>,<time_ms>,<a1_id>,<a1_valid>,<a1_age_ms>,<a1_raw_mm>,<a1_filt_mm>,<a1_fpp_cdbm>,
      <a2_id>,<a2_valid>,<a2_age_ms>,<a2_raw_mm>,<a2_filt_mm>,<a2_fpp_cdbm>,
      <a3_id>,<a3_valid>,<a3_age_ms>,<a3_raw_mm>,<a3_filt_mm>,<a3_fpp_cdbm>

File log có thể lẫn banner/dòng "S,..."/rác từ terminal — script tự bỏ qua, chỉ
xử lý dòng bắt đầu bằng "R," và đủ 21 trường.
"""

import argparse
import statistics
import sys

# Windows console mặc định dùng codepage cp1252, không encode được tiếng Việt
# Unicode (vd "ổ", "ệ") — ép stdout/stderr sang UTF-8 để tránh crash khi in.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")


def parse_log(path, anchor_id):
    """Đọc file log, trả về list raw_mm (mm, int) của đúng anchor_id, chỉ lấy mẫu valid=1."""
    raw_values = []
    total_lines = 0
    r_lines = 0
    skipped_invalid = 0
    skipped_wrong_anchor = 0

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

            # 3 anchor, mỗi anchor 6 field, bắt đầu từ index 3
            found = False
            for i in range(3):
                base = 3 + i * 6
                try:
                    a_id = int(parts[base])
                    a_valid = parts[base + 1] == "1"
                    a_raw_mm = int(parts[base + 3])
                except (ValueError, IndexError):
                    continue

                if a_id != anchor_id:
                    continue
                found = True
                if not a_valid:
                    skipped_invalid += 1
                    continue
                raw_values.append(a_raw_mm)

            if not found:
                skipped_wrong_anchor += 1

    return raw_values, {
        "total_lines": total_lines,
        "r_lines": r_lines,
        "skipped_invalid": skipped_invalid,
        "skipped_wrong_anchor": skipped_wrong_anchor,
    }


def trim_outliers(values, n_sigma):
    """Loại các giá trị lệch quá n_sigma lần std-dev so với mean. Lặp 1 lần (không đệ quy)."""
    if len(values) < 2:
        return values
    mean = statistics.mean(values)
    stdev = statistics.pstdev(values)
    if stdev == 0:
        return values
    return [v for v in values if abs(v - mean) <= n_sigma * stdev]


def main():
    ap = argparse.ArgumentParser(description="Tính calibration offset từ file log serial ASCII của TAG.")
    ap.add_argument("logfile", help="Đường dẫn file log đã capture (text, chứa các dòng 'R,...')")
    ap.add_argument("--anchor", type=int, required=True, choices=[1, 2, 3], help="ID anchor (1/2/3)")
    ap.add_argument("--true-dist", type=float, required=True, help="Khoảng cách thật đã đo (mét)")
    ap.add_argument("--trim-sigma", type=float, default=None,
                     help="Loại mẫu lệch quá N lần std-dev trước khi tính lại (tuỳ chọn, vd 3.0)")
    args = ap.parse_args()

    raw_values, stats = parse_log(args.logfile, args.anchor)

    print(f"=== File: {args.logfile} ===")
    print(f"Tổng số dòng đọc:        {stats['total_lines']}")
    print(f"Số dòng 'R,...' hợp lệ:  {stats['r_lines']}")
    print(f"Số mẫu Anchor {args.anchor} bị invalid (bỏ qua): {stats['skipped_invalid']}")
    print()

    if len(raw_values) < 10:
        print(f"⚠️  Chỉ có {len(raw_values)} mẫu hợp lệ cho Anchor {args.anchor} — QUÁ ÍT để tin cậy.")
        print("    Thu log lâu hơn (khuyến nghị >= 200-500 mẫu, ~5-10 giây @50Hz) rồi chạy lại.")
        if len(raw_values) == 0:
            sys.exit(1)

    n = len(raw_values)
    mean_mm = statistics.mean(raw_values)
    stdev_mm = statistics.pstdev(raw_values) if n > 1 else 0.0
    min_mm = min(raw_values)
    max_mm = max(raw_values)

    print(f"--- Anchor {args.anchor}: {n} mẫu raw (chưa lọc outlier) ---")
    print(f"  Mean:   {mean_mm:.2f} mm")
    print(f"  StdDev: {stdev_mm:.2f} mm")
    print(f"  Min/Max: {min_mm} / {max_mm} mm")

    offset_m_raw = (mean_mm / 1000.0) - args.true_dist
    print(f"\n  → calibration_offset_m[{args.anchor - 1}] (chưa lọc outlier) = {offset_m_raw:.10f}")

    if args.trim_sigma is not None:
        trimmed = trim_outliers(raw_values, args.trim_sigma)
        removed = n - len(trimmed)
        if trimmed:
            mean_t = statistics.mean(trimmed)
            stdev_t = statistics.pstdev(trimmed) if len(trimmed) > 1 else 0.0
            offset_m_trimmed = (mean_t / 1000.0) - args.true_dist
            print(f"\n--- Sau khi loại outlier (>{args.trim_sigma}σ): {len(trimmed)} mẫu, đã bỏ {removed} ---")
            print(f"  Mean:   {mean_t:.2f} mm")
            print(f"  StdDev: {stdev_t:.2f} mm")
            print(f"  → calibration_offset_m[{args.anchor - 1}] (đã lọc)     = {offset_m_trimmed:.10f}")
        else:
            print("\n⚠️  Sau khi lọc outlier không còn mẫu nào — bỏ qua bước lọc.")

    print(f"\nĐộ lệch chuẩn {stdev_mm:.1f}mm cho biết sai số ngẫu nhiên còn lại sau khi trung bình hoá.")
    print("Nếu StdDev lớn (>> vài chục mm), kiểm tra LOS/nhiễu trước khi tin giá trị offset.")


if __name__ == "__main__":
    main()
