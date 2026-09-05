/**
 ******************************************************************************
 * @file    uwb_calibration.h
 * @brief   Nguồn cấu hình calibration DUY NHẤT (Phase 3/C0) — TAG & Anchor.
 *
 * NGUYÊN TẮC AN TOÀN: mọi flag mặc định GIỮ hành vi hiện tại (đang chạy đúng).
 * Chỉ bật path mới SAU KHI đo calibration trên phần cứng, rồi A/B test.
 *
 * C0 (2026-07-22): Thêm UwbRangingMode_t enum để phân biệt SS/DS/SS_FALLBACK
 *   trong apply_offset_and_clamp — sửa lỗi P0 mục 2.3 ANCHOR4 plan.
 *   Thêm UWB_DS_CALIBRATED_MASK để guard calibration riêng từng anchor.
 *   Thêm UWB_RANGE_FILTER_MODE feature flag theo §8.2 ANCHOR4 plan.
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

#if (UWB_USE_HW_ANTENNA_DELAY != 0) && (UWB_USE_HW_ANTENNA_DELAY != 1)
#error "UWB_USE_HW_ANTENNA_DELAY must be 0 or 1"
#endif

#if (UWB_USE_LEGACY_OFFSET != 0) && (UWB_USE_LEGACY_OFFSET != 1)
#error "UWB_USE_LEGACY_OFFSET must be 0 or 1"
#endif

/* Exactly one SS compensation profile must be active. This also protects the
 * SS fallback path in a DS build from double or missing compensation. */
#if UWB_USE_HW_ANTENNA_DELAY == UWB_USE_LEGACY_OFFSET
#error "Select exactly one calibration profile: legacy SW offset or HW antenna delay + residual"
#endif
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

/* Exactly one compensation path must be active. (0,0) publishes uncorrected
 * range; (1,1) double-compensates and can collapse the range to zero. */
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

/** Calibration validity theo từng anchor, không dùng boolean toàn cục.
 *  Bit 0/1/2/3 tương ứng A1/A2/A3/A4. Chỉ set một bit sau khi đúng anchor đó
 *  đã được calibration + validation ở radio profile hiện tại.
 *
 *  Có thể override sạch từ build configuration, ví dụ:
 *    -DUWB_DS_CALIBRATED_MASK=0x01U  (chỉ A1 đã calibration)
 *
 *  Mặc định 0: mọi DS result chỉ được lưu làm calibration diagnostic và valid=0. */
#define UWB_DS_CAL_A1_BIT          (1U << 0)
#define UWB_DS_CAL_A2_BIT          (1U << 1)
#define UWB_DS_CAL_A3_BIT          (1U << 2)
#define UWB_DS_CAL_A4_BIT          (1U << 3)
#define UWB_DS_CAL_KNOWN_MASK      (UWB_DS_CAL_A1_BIT | \
                                    UWB_DS_CAL_A2_BIT | \
                                    UWB_DS_CAL_A3_BIT | \
                                    UWB_DS_CAL_A4_BIT)

#ifndef UWB_DS_CALIBRATED_MASK
/* C4 recalibration session: A1-A4 now have validated DS captures. */
#define UWB_DS_CALIBRATED_MASK     (UWB_DS_CAL_A1_BIT | UWB_DS_CAL_A2_BIT | \
                                    UWB_DS_CAL_A3_BIT | UWB_DS_CAL_A4_BIT)
#endif

#if (UWB_DS_CALIBRATED_MASK & ~UWB_DS_CAL_KNOWN_MASK) != 0U
#error "UWB_DS_CALIBRATED_MASK contains an unknown anchor bit"
#endif

/** SS calibration validity is separated by compensation profile.
 *  Legacy offsets A1-A3 are already validated; A4 remains TODO_CALIBRATE.
 *  Residual offsets default to none because they require a separate HW-delay
 *  calibration and must never inherit the legacy validity mask. */
#define UWB_SS_CAL_A1_BIT          (1U << 0)
#define UWB_SS_CAL_A2_BIT          (1U << 1)
#define UWB_SS_CAL_A3_BIT          (1U << 2)
#define UWB_SS_CAL_A4_BIT          (1U << 3)
#define UWB_SS_CAL_KNOWN_MASK      (UWB_SS_CAL_A1_BIT | \
                                    UWB_SS_CAL_A2_BIT | \
                                    UWB_SS_CAL_A3_BIT | \
                                    UWB_SS_CAL_A4_BIT)

#ifndef UWB_SS_LEGACY_CALIBRATED_MASK
#define UWB_SS_LEGACY_CALIBRATED_MASK \
    (UWB_SS_CAL_A1_BIT | UWB_SS_CAL_A2_BIT | UWB_SS_CAL_A3_BIT)
#endif

#ifndef UWB_SS_RESIDUAL_CALIBRATED_MASK
#define UWB_SS_RESIDUAL_CALIBRATED_MASK 0U
#endif

#if (UWB_SS_LEGACY_CALIBRATED_MASK & ~UWB_SS_CAL_KNOWN_MASK) != 0U
#error "UWB_SS_LEGACY_CALIBRATED_MASK contains an unknown anchor bit"
#endif
#if (UWB_SS_RESIDUAL_CALIBRATED_MASK & ~UWB_SS_CAL_KNOWN_MASK) != 0U
#error "UWB_SS_RESIDUAL_CALIBRATED_MASK contains an unknown anchor bit"
#endif

#if UWB_USE_LEGACY_OFFSET
#define UWB_SS_ACTIVE_CALIBRATED_MASK UWB_SS_LEGACY_CALIBRATED_MASK
#else
#define UWB_SS_ACTIVE_CALIBRATED_MASK UWB_SS_RESIDUAL_CALIBRATED_MASK
#endif

/* ========================================================================== */
/*           RANGE FILTER MODE — Feature Flag (§8.2 ANCHOR4 plan)            */
/* ========================================================================== */

/** Profile bộ lọc range tại firmware.
 *  LEGACY_KALMAN remains the deployed profile because the current radio noise
 *  needs its smoothing. MEDIAN_GATE is a C9 replay/hardware A/B candidate:
 *  median-3, physical/dynamic gate and controlled reacquire, without a
 *  range-domain Kalman.
 *  0 = UWB_RANGE_FILTER_LEGACY_KALMAN (deployed baseline)
 *  1 = UWB_RANGE_FILTER_MEDIAN_GATE   (candidate production)
 *  2 = UWB_RANGE_FILTER_CV_KALMAN_V2  (thử nghiệm) */
#define UWB_RANGE_FILTER_LEGACY_KALMAN  0U
#define UWB_RANGE_FILTER_MEDIAN_GATE    1U
#define UWB_RANGE_FILTER_CV_KALMAN_V2   2U

#ifndef UWB_RANGE_FILTER_MODE
#define UWB_RANGE_FILTER_MODE   UWB_RANGE_FILTER_LEGACY_KALMAN
#endif

#if (UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_LEGACY_KALMAN) && \
    (UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_MEDIAN_GATE) && \
    (UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_CV_KALMAN_V2)
#error "Unknown UWB_RANGE_FILTER_MODE"
#endif

/* C9: mode 1 is opt-in for a controlled replay/hardware A/B only. Set this
 * macro to UWB_RANGE_FILTER_MEDIAN_GATE (1U) to test it; CV_KALMAN_V2 remains
 * experimental until it independently passes A/B. */

/* ========================================================================== */
/*            ADAPTIVE LEGACY TRACKER — separate, reversible A/B path        */
/* ========================================================================== */

/**
 * Adaptive Legacy changes only the temporal response of the deployed legacy
 * conditioner. It is intentionally NOT another C9 profile and it never
 * changes the production range path by default:
 *
 *   OFF    : byte-for-byte Legacy output path (deployed baseline).
 *   SHADOW : run an independent candidate state for Live Expressions only.
 *   ACTIVE : use the confirmed candidate for the published Legacy range.
 *
 * Start with OFF, record a baseline, then use SHADOW/replay before an ACTIVE
 * firmware A/B. Do not set ACTIVE while switching DS/SS, PHY or calibration.
 */
#define UWB_LEGACY_ADAPTIVE_OFF     0U
#define UWB_LEGACY_ADAPTIVE_SHADOW  1U
#define UWB_LEGACY_ADAPTIVE_ACTIVE  2U

#ifndef UWB_LEGACY_ADAPTIVE_MODE
#define UWB_LEGACY_ADAPTIVE_MODE UWB_LEGACY_ADAPTIVE_SHADOW
#endif

#if (UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_OFF) && \
    (UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_SHADOW) && \
    (UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_ACTIVE)
#error "Unknown UWB_LEGACY_ADAPTIVE_MODE"
#endif

/* The adaptive tracker reuses the Legacy median + scalar Kalman path only. */
#if (UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_OFF) && \
    (UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_LEGACY_KALMAN)
#error "Adaptive Legacy requires UWB_RANGE_FILTER_LEGACY_KALMAN"
#endif

/* These are deliberately named seeds, not calibration constants. The values
 * are chosen to retain the current static behaviour and need replay/hardware
 * A/B before an ACTIVE build is flight-qualified. */
#define UWB_LEGACY_BASE_PROCESS_NOISE             0.05
#define UWB_LEGACY_ADAPTIVE_MOTION_ENTER_MM       250.0  /* TUNE_REQUIRED */
#define UWB_LEGACY_ADAPTIVE_SETTLE_RESIDUAL_MM    120.0  /* TUNE_REQUIRED */
#define UWB_LEGACY_ADAPTIVE_CANDIDATE_CLUSTER_MM  250.0  /* TUNE_REQUIRED */
#define UWB_LEGACY_ADAPTIVE_REACQUIRE_MIN_FPP_DBM (-105.0f)
#define UWB_LEGACY_ADAPTIVE_MOTION_CONFIRM_SAMPLES 4U
#define UWB_LEGACY_ADAPTIVE_STALE_REACQUIRE_SAMPLES 3U
#define UWB_LEGACY_ADAPTIVE_SETTLE_SAMPLES        5U
#define UWB_LEGACY_ADAPTIVE_STALE_RESET_MS        500U
#define UWB_LEGACY_ADAPTIVE_CANDIDATE_MAX_GAP_MS  100U
#define UWB_LEGACY_ADAPTIVE_NOMINAL_SAMPLE_MS     20U
#define UWB_LEGACY_ADAPTIVE_TRACK_MIN_DT_MS       10U
#define UWB_LEGACY_ADAPTIVE_TRACK_MAX_DT_MS       100U
#define UWB_LEGACY_ADAPTIVE_TRACK_GAIN            0.15  /* TUNE_REQUIRED: 0.10..0.20 */

/* ========================================================================== */
/*          C9.2 MOTION-ADAPTIVE RANGE REGIMES — independent A/B path        */
/* ========================================================================== */

/**
 * C9.2 deliberately remains a separate feature switch from C9.1 Adaptive
 * Legacy. SHADOW owns fully independent median/Kalman/motion state and can
 * never mutate the canonical Legacy state. ACTIVE is only permitted after
 * replay and hardware A/B acceptance.
 */
#define UWB_C9_2_MOTION_OFF     0U
#define UWB_C9_2_MOTION_SHADOW  1U
#define UWB_C9_2_MOTION_ACTIVE  2U

#ifndef UWB_C9_2_MOTION_MODE
#define UWB_C9_2_MOTION_MODE UWB_C9_2_MOTION_OFF
#endif

#if (UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_OFF) && \
    (UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_SHADOW) && \
    (UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_ACTIVE)
#error "Unknown UWB_C9_2_MOTION_MODE"
#endif

#if (UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_OFF) && \
    (UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_LEGACY_KALMAN)
#error "C9.2 motion controller requires UWB_RANGE_FILTER_LEGACY_KALMAN"
#endif

/* Two candidate controllers must never both own production output. */
#if (UWB_C9_2_MOTION_MODE == UWB_C9_2_MOTION_ACTIVE) && \
    (UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_ACTIVE)
#error "Only one adaptive range controller may be ACTIVE"
#endif

/* C9.2 seed values: replay/hardware tune required before an ACTIVE build. */
#define UWB_C9_2_SLOW_ENTER_Z                  2.5f
#define UWB_C9_2_SLOW_EXIT_Z                   1.5f
#define UWB_C9_2_FAST_ENTER_Z                  5.0f
#define UWB_C9_2_FAST_EXIT_Z                   3.0f
#define UWB_C9_2_CUSUM_DRIFT_Z                 0.5f
#define UWB_C9_2_SLOW_SPEED_MM_S               250.0f
#define UWB_C9_2_FAST_SPEED_MM_S               1000.0f
#define UWB_C9_2_SLOW_GAIN                     0.12f
#define UWB_C9_2_FAST_GAIN                     0.26f
#define UWB_C9_2_SETTLING_GAIN                 0.10f
#define UWB_C9_2_MINIMUM_SIGMA_MM              50.0f
#define UWB_C9_2_REACQUIRE_MIN_FPP_DBM        (-105.0f)
#define UWB_C9_2_REACQUIRE_CLUSTER_MM          250.0f
#define UWB_C9_2_SLOW_CONFIRM_SAMPLES          3U
#define UWB_C9_2_FAST_CONFIRM_SAMPLES          2U
#define UWB_C9_2_SETTLE_DWELL_SAMPLES          10U
#define UWB_C9_2_STATIC_DWELL_SAMPLES          25U
#define UWB_C9_2_REACQUIRE_SAMPLES             3U
#define UWB_C9_2_DEGRADE_AFTER_REJECTS         3U
#define UWB_C9_2_STALE_RESET_MS                500U
#define UWB_C9_2_CANDIDATE_MAX_GAP_MS          100U
#define UWB_C9_2_MIN_DT_MS                     10U
#define UWB_C9_2_MAX_DT_MS                     500U

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

/* ========================================================================== */
/*      C0 — RANGING MODE ENUM (sửa P0 calibration SS/DS/fallback)           */
/* ========================================================================== */

/**
 * @brief  Chế độ ranging được dùng để tính khoảng cách.
 *         Dùng để chọn đúng calibration offset profile theo mode.
 *
 * UWB_MODE_SS          = SS-TWR thuần (default)
 * UWB_MODE_DS          = DS-TWR thuần (4-message, chỉ khi UWB_USE_DS_TWR=1)
 * UWB_MODE_SS_FALLBACK = SS-TWR fallback khi REPORT timeout trong DS cycle
 *                        → phải dùng offset SS (không phải DS)
 */
typedef enum {
    UWB_MODE_SS          = 0,  /**< Single-Sided TWR (2-message) */
    UWB_MODE_DS          = 1,  /**< Double-Sided TWR (4-message) */
    UWB_MODE_SS_FALLBACK = 2,  /**< SS fallback trong DS cycle — dùng profile SS */
} UwbRangingMode_t;

/**
 * @brief  DS-TWR calibration offset (mét) cho từng anchor.
 *         Chỉ có hiệu lực khi bit tương ứng trong UWB_DS_CALIBRATED_MASK = 1.
 *         Giá trị TODO_CALIBRATE = 0.0 cho đến khi đo xong.
 *         KHÔNG copy giá trị offset SS sang đây.
 */
#ifndef UWB_DS_OFFSET_A1_M
#define UWB_DS_OFFSET_A1_M   154.091571  /* Fast-256 DS: 1.000 m, 2000 samples */
#endif
#ifndef UWB_DS_OFFSET_A2_M
#define UWB_DS_OFFSET_A2_M   154.243357  /* Fast-256 DS: 5.410 m, 2000 samples (2026-08-02) */
#endif
#ifndef UWB_DS_OFFSET_A3_M
#define UWB_DS_OFFSET_A3_M   154.235083  /* Fast-256 DS: 5.410 m, 2000 samples (2026-08-02) */
#endif
#ifndef UWB_DS_OFFSET_A4_M
#define UWB_DS_OFFSET_A4_M   154.186475  /* Fast-256 DS: 1.000 m, 2000 samples */
#endif

#endif /* UWB_CALIBRATION_H */
