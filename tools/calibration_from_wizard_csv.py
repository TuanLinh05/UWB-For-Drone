#!/usr/bin/env python3
"""
calibration_from_wizard_csv.py — Tổng hợp calibration từ NHIỀU file CSV export
của GUI Calibration Wizard (nhiều cự ly x nhiều anchor), tính offset cuối cùng
và cảnh báo nếu dữ liệu không ổn định.

CHỈ dùng đúng cách khi `calibration_offset_m[3] = {0.0, 0.0, 0.0}` trong
firmware lúc thu dữ liệu (xem CALIBRATION_PROCEDURE.md) — khi đó cột `raw_mm`
trong CSV là khoảng cách THÔ chưa hiệu chỉnh, offset tính ra là giá trị
TUYỆT ĐỐI, không cần cộng dồn với offset cũ.

Cách dùng:
    py calibration_from_wizard_csv.py file1.csv file2.csv file3.csv ...
    py calibration_from_wizard_csv.py calib_session/*.csv

Mỗi file CSV phải đúng format wizard xuất ra:
    anchor_id,true_distance_m,sample_index,raw_mm,mean_mm,offset_m

Script sẽ:
  1. Gộp theo (anchor_id, true_distance_m) — nếu 1 anchor+cự ly có NHIỀU file
     (đo lặp lại), gộp tất cả mẫu lại trước khi tính trung bình.
  2. Tính offset riêng cho từng cự ly của từng anchor.
  3. Cảnh báo nếu offset giữa các cự ly CÙNG 1 anchor lệch nhau > ngưỡng
     (mặc định 5cm) — dấu hiệu NLOS/multipath/vị trí đo không cố định.
  4. Tính offset cuối = trung bình các điểm cự ly hợp lệ cho từng anchor.
  5. In sẵn dòng code `calibration_offset_m[3] = {...};` để dán vào
     tag_ranging.c.
"""

import argparse
import csv
import statistics
import sys
from collections import defaultdict

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")


def load_files(paths):
    """Trả về dict: {anchor_id: {true_distance_m: [raw_mm, ...]}}"""
    data = defaultdict(lambda: defaultdict(list))
    for path in paths:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    anchor_id = int(row["anchor_id"])
                    true_dist = float(row["true_distance_m"])
                    raw_mm = int(row["raw_mm"])
                except (KeyError, ValueError):
                    continue
                data[anchor_id][true_dist].append(raw_mm)
    return data


def main():
    ap = argparse.ArgumentParser(description="Tổng hợp calibration từ nhiều CSV wizard (đa cự ly).")
    ap.add_argument("csv_files", nargs="+", help="Danh sách file CSV wizard (hỗ trợ glob qua shell)")
    ap.add_argument("--max-anchor-id", type=int, default=3, help="Số anchor tối đa (mặc định 3)")
    ap.add_argument("--warn-spread-mm", type=float, default=50.0,
                     help="Ngưỡng cảnh báo (mm) nếu offset giữa các cự ly cùng 1 anchor lệch quá mức này (mặc định 50mm = 5cm)")
    args = ap.parse_args()

    data = load_files(args.csv_files)

    if not data:
        print("Không đọc được dữ liệu hợp lệ từ các file đã cho.")
        sys.exit(1)

    final_offsets = {}

    for anchor_id in sorted(data.keys()):
        print(f"\n=== Anchor {anchor_id} ===")
        dist_offsets = []  # list of (true_dist, offset_m, n, stdev_mm)

        for true_dist in sorted(data[anchor_id].keys()):
            vals = data[anchor_id][true_dist]
            n = len(vals)
            mean_mm = statistics.mean(vals)
            stdev_mm = statistics.pstdev(vals) if n > 1 else 0.0
            offset_m = mean_mm / 1000.0 - true_dist
            dist_offsets.append((true_dist, offset_m, n, stdev_mm))
            print(f"  true_dist={true_dist:>6.2f}m | n={n:>5} | mean={mean_mm:>10.2f}mm "
                  f"| std={stdev_mm:>6.2f}mm | offset={offset_m:.6f}m")

        if len(dist_offsets) == 1:
            print(f"  ⚠️  CHỈ CÓ 1 CỰ LY — không kiểm chứng chéo được. Nên đo thêm ít nhất 1 cự ly khác xa hẳn.")

        offsets_only = [o for (_, o, _, _) in dist_offsets]
        spread_m = max(offsets_only) - min(offsets_only)
        spread_mm = spread_m * 1000.0

        final_offset = statistics.mean(offsets_only)
        final_offsets[anchor_id] = final_offset

        print(f"  → Offset trung bình qua {len(dist_offsets)} cự ly: {final_offset:.10f} m")
        print(f"  → Chênh lệch offset giữa các cự ly: {spread_mm:.1f} mm", end="")

        if spread_mm > args.warn_spread_mm:
            print(f"  ⚠️⚠️⚠️  VƯỢT NGƯỠNG {args.warn_spread_mm:.0f}mm — DỮ LIỆU KHÔNG ỔN ĐỊNH!")
            print(f"       Không nên tin giá trị trung bình này. Nguyên nhân khả dĩ: NLOS/multipath,")
            print(f"       vị trí đo không cố định giữa các lần, hoặc anchor có vấn đề phần cứng.")
            print(f"       → Đo lại ở vị trí LOS sạch, đánh dấu cố định điểm đặt TAG/Anchor.")
        else:
            print("  (ổn định, trong ngưỡng chấp nhận được)")

    # In sẵn dòng code để dán
    print("\n" + "=" * 70)
    n_anchors = args.max_anchor_id
    values = []
    for i in range(1, n_anchors + 1):
        if i in final_offsets:
            values.append(f"{final_offsets[i]:.10f}")
        else:
            values.append("0.0 /* CHƯA CÓ DỮ LIỆU cho anchor này */")
    print(f"volatile double calibration_offset_m[{n_anchors}] = {{{', '.join(values)}}};")
    print("=" * 70)
    print("\n⚠️  Chỉ dán dòng trên nếu KHÔNG có cảnh báo 'DỮ LIỆU KHÔNG ỔN ĐỊNH' ở trên.")
    print("    Nếu có, xử lý nguyên nhân rồi đo lại trước khi cập nhật firmware.")


if __name__ == "__main__":
    main()
