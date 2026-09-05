/**
 ******************************************************************************
 * @file    uwb_calibration.h
 * @brief   Nguồn cấu hình calibration DUY NHẤT (Phase 3) — TAG & Anchor.
 *
 * NGUYÊN TẮC AN TOÀN: mọi flag mặc định GIỮ hành vi hiện tại (đang chạy đúng).
 * Chỉ bật path mới SAU KHI đo calibration trên phần cứng, rồi A/B test.
 *
 *   Workflow chuyển sang antenna delay phần cứng (khuyến nghị của plan):
 *     1. Đặt UWB_USE_HW_ANTENNA_DELAY = 1, UWB_USE_LEGACY_OFFSET = 0.
 *     2. Flash CẢ TAG lẫn Anchor.
 *     3. Đặt khoảng cách thật, dùng auto-calibrate để lấy residual offset nhỏ.
 *     4. Đo nhiều cự ly, tinh chỉnh UWB_TX/RX_ANT_DLY tới khi mean error ~0.
 *     5. Nếu tệ hơn → đặt lại 2 flag về (0,1) để rollback ngay.
 ******************************************************************************
 */

#ifndef UWB_CALIBRATION_H
#define UWB_CALIBRATION_H

/* ========================================================================== */
/*                 FEATURE FLAGS (mặc định = hành vi hiện tại)                 */
/* ========================================================================== */

/** 1: ghi antenna delay vào register DW1000 (F-01);
 *  0: để antenna delay = 0, bù toàn bộ bằng software offset (~157m) như hiện tại. */
#ifndef UWB_USE_HW_ANTENNA_DELAY
#define UWB_USE_HW_ANTENNA_DELAY   0
#endif

/** 1: dùng offset lớn hiện tại (calibration_offset_m ~157m);
 *  0: dùng residual offset nhỏ (residual_offset_m) — chỉ khi đã bật HW antenna delay. */
#ifndef UWB_USE_LEGACY_OFFSET
#define UWB_USE_LEGACY_OFFSET      1
#endif

/** 1: bật bù clock drift bằng carrier integrator (F-02);
 *  0: tắt (như hiện tại — chấp nhận trôi nhiệt lúc mới bật nguồn). */
#ifndef UWB_USE_CLOCK_CORRECTION
#define UWB_USE_CLOCK_CORRECTION   0
#endif

#if (UWB_USE_HW_ANTENNA_DELAY != 0) && (UWB_USE_HW_ANTENNA_DELAY != 1)
#error "UWB_USE_HW_ANTENNA_DELAY must be 0 or 1"
#endif
#if (UWB_USE_LEGACY_OFFSET != 0) && (UWB_USE_LEGACY_OFFSET != 1)
#error "UWB_USE_LEGACY_OFFSET must be 0 or 1"
#endif
#if (UWB_USE_CLOCK_CORRECTION != 0) && (UWB_USE_CLOCK_CORRECTION != 1)
#error "UWB_USE_CLOCK_CORRECTION must be 0 or 1"
#endif
#if UWB_USE_HW_ANTENNA_DELAY == UWB_USE_LEGACY_OFFSET
#error "Select exactly one range compensation path: HW antenna delay or legacy offset"
#endif

/** 1: DS-TWR 4-message (POLL/RESP/FINAL/REPORT) — Phase 4.
 *  0: SS-TWR 2-message như hiện tại (mặc định — KHÔNG đổi hành vi). */
#ifndef UWB_USE_DS_TWR
#define UWB_USE_DS_TWR             1
#endif

#if (UWB_USE_DS_TWR != 0) && (UWB_USE_DS_TWR != 1)
#error "UWB_USE_DS_TWR must be 0 or 1"
#endif

/* ========================================================================== */
/*                 OUTLIER GATE ĐỘNG HỌC (Task 1, Phase 4)                    */
/* ========================================================================== */

/** Vận tốc tối đa của drone (mm/s). User bay tới 30km/h=8333mm/s → chọn 10000 có margin. */
#define UWB_DRONE_VMAX_MM_S     10000.0
/** Margin nhiễu cộng thêm vào ngưỡng (mm). Hướng dương chặt (multipath dương). */
#define UWB_GATE_MARGIN_UP_MM   100.0
#define UWB_GATE_MARGIN_DOWN_MM 250.0
/** Số outlier liên tiếp trước khi snap về giá trị đo (backstop). 15 × 20ms = 0,3s
 *  (cũ là 50 = 1s — quá lâu cho drone đang bay). */
#define UWB_GATE_SNAP_AFTER     15

/* ========================================================================== */
/*                 ANTENNA DELAY (DTU) — khi HW antenna delay bật              */
/* ========================================================================== */

/** Điểm khởi đầu Decawave (~DW_DEFAULT_ANT_DLY). Hiệu chỉnh per-device (APS014).
 *  Mỗi thiết bị nên có giá trị riêng nếu cần độ chính xác cao. */
#ifndef UWB_TX_ANT_DLY
#define UWB_TX_ANT_DLY   16436
#endif
#ifndef UWB_RX_ANT_DLY
#define UWB_RX_ANT_DLY   16436
#endif

/* ========================================================================== */
/*                 HẰNG SỐ VẬT LÝ (named — không magic number)                 */
/* ========================================================================== */

/** DWT time unit (giây). GIỮ 15.65e-12 để KHÔNG đổi scale của legacy offset
 *  (~157m khuếch đại sai số scale ~78mm nếu đổi). Khi đã dùng HW antenna delay,
 *  có thể đổi sang giá trị chính xác 1.0/(499.2e6*128.0) = 1.56576e-11. */
#define UWB_DWT_TIME_UNIT_S   15.65e-12

/** Tốc độ ánh sáng (m/s). Giữ giá trị chân không hiện tại (calibration hấp thụ
 *  chênh lệch nhỏ với tốc độ trong không khí 299702547.0). */
#define UWB_SPEED_OF_LIGHT    299792458.0

/** Hệ số chuyển carrier integrator → clock offset ratio (Ch5, PRF16, 6.8Mbps).
 *  Ratio = CI_ema * hệ_số. DẤU phải VERIFY bằng warm-up test: nếu bật correction
 *  làm drift TỆ hơn thì đảo dấu hệ số này. */
#define UWB_CLOCK_OFFSET_MULT   5.7312e-10

/** Hằng số A cho công thức First-Path Power (DW1000 UM 4.7.2).
 *  PRF16 = 113.77, PRF64 = 121.74. PHY thực tế là PRF16.
 *  (Code cũ ghi nhầm 115.72 gán cho "PRF64" — sai cả hai.) */
#define UWB_FPP_A_CONST   113.77f

#endif /* UWB_CALIBRATION_H */
