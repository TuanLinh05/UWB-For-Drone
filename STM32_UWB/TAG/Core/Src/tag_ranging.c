/**
 ******************************************************************************
 * @file    tag_ranging.c
 * @brief   TAG SS-TWR Ranging — Interrupt-driven State Machine @ 50Hz
 *
 * Protocol: Single-Sided Two-Way Ranging (SS-TWR)
 *   For each Anchor (1, 2, 3, 4):
 *     1. TAG  ──[POLL]──>  Anchor
 *     2. Anchor ──[RESP]──> TAG   (T_reply embedded in payload)
 *     3. TAG computes: ToF = (T_round - T_reply) / 2
 *                      dist = ToF × 15.65ps × c
 *
 * State Machine (50Hz = 20ms cycle):
 *   TAG_STATE_IDLE
 *     │  (every 20ms: HAL_GetTick() - last_cycle >= TAG_CYCLE_MS)
 *     ▼
 *   TAG_STATE_TX_POLL  ── send POLL to current anchor
 *     │  (IRQ: TXFRS bit set) ── TX done
 *     ▼
 *   TAG_STATE_WAIT_RESP  ── enable RX, wait for RESP
 *     │  (IRQ: RXFCG + valid frame)  ── compute distance, advance anchor
 *     │  (timeout after TAG_RESP_TIMEOUT_US) ── skip anchor, advance
 *     ▼
 *   [if more anchors] → TAG_STATE_TX_POLL
 *   [if all done]     → TAG_STATE_IDLE
 *
 * Key design:
 *   - No HAL_Delay anywhere.
 *   - No blocking while() loops.
 *   - EXTI3 ISR only sets dw1000_irq_flag = 1.
 *   - All SPI and logic runs in Tag_Task() from main while(1).
 ******************************************************************************
 */

#include "tag_ranging.h"
#include "legacy_adaptive_tracking.h"
#include "motion_adaptive_range.h"
#include "range_filter.h"
#include <limits.h>
#include <math.h>
#include <string.h>

/* ========================================================================== */
/*                     CALIBRATION                                             */
/* ========================================================================== */

/* SS-TWR legacy offsets. A1-A3 are validated; A4 is deliberately left at
 * TODO_CALIBRATE and is guarded by UWB_SS_LEGACY_CALIBRATED_MASK. */
volatile double calibration_offset_m[TAG_NUM_ANCHORS] = {
    156.7284371582,
    156.4618,
    155.9958,
    0.0
};
volatile double auto_calibrate_actual_m = 0.0;
volatile uint16_t auto_calibrate_anchor_id = 0U;

/* Residual offset (mét) khi dùng HW antenna delay — nhỏ (cỡ cm). */
volatile double residual_offset_m[TAG_NUM_ANCHORS] = {0.0, 0.0, 0.0, 0.0};

/* ========================================================================== */
/*                     PRIVATE DEFINES                                         */
/* ========================================================================== */

#define MAX_RX_FRAME_LEN    20
#define POLL_FRAME_LEN      10

#define RANGE_COMPUTE_ERROR       (-1)
#define RANGE_CALIBRATION_MISSING (-2)

#define LED_TOGGLE()        HAL_GPIO_TogglePin(LED_PORT, LED_PIN)

typedef struct {
    uint16_t id;
    uint32_t ss_calibration_bit;
} TagAnchorConfig_t;

/** ID is the data key. Array index is only the sequential radio slot. */
static const TagAnchorConfig_t s_anchor_cfg[TAG_NUM_ANCHORS] = {
    { 0x0001U, UWB_SS_CAL_A1_BIT },
    { 0x0002U, UWB_SS_CAL_A2_BIT },
    { 0x0003U, UWB_SS_CAL_A3_BIT },
    { 0x0004U, UWB_SS_CAL_A4_BIT },
};

static uint32_t s_ss_calibrated_mask = UWB_SS_ACTIVE_CALIBRATED_MASK;

static inline uint16_t anchor_id_at(uint8_t anchor_index)
{
    return s_anchor_cfg[anchor_index].id;
}

#if UWB_USE_DS_TWR
/**
 * @brief  Resolve DS calibration by anchor ID, not by array position.
 *         This prevents a future ANCHOR_LIST reorder from silently applying
 *         A1's validity bit/offset to another physical anchor.
 * @retval 0 on success, RANGE_CALIBRATION_MISSING when the known anchor has
 *         not been calibrated, RANGE_COMPUTE_ERROR for bad input/config.
 */
static int32_t ds_calibration_offset_for_index(uint8_t anchor_index,
                                               double *offset_m)
{
    uint32_t calibration_bit;

    if (offset_m == NULL || anchor_index >= TAG_NUM_ANCHORS)
        return RANGE_COMPUTE_ERROR;

    switch (anchor_id_at(anchor_index))
    {
        case 0x0001U:
            calibration_bit = UWB_DS_CAL_A1_BIT;
            *offset_m = UWB_DS_OFFSET_A1_M;
            break;

        case 0x0002U:
            calibration_bit = UWB_DS_CAL_A2_BIT;
            *offset_m = UWB_DS_OFFSET_A2_M;
            break;

        case 0x0003U:
            calibration_bit = UWB_DS_CAL_A3_BIT;
            *offset_m = UWB_DS_OFFSET_A3_M;
            break;

        case 0x0004U:
            calibration_bit = UWB_DS_CAL_A4_BIT;
            *offset_m = UWB_DS_OFFSET_A4_M;
            break;

        default:
            return RANGE_COMPUTE_ERROR;
    }

    if ((UWB_DS_CALIBRATED_MASK & calibration_bit) == 0U)
        return RANGE_CALIBRATION_MISSING;

    return isfinite(*offset_m) ? 0 : RANGE_COMPUTE_ERROR;
}
#endif

/* ========================================================================== */
/*                     STATE MACHINE                                           */
/* ========================================================================== */

typedef enum {
    TAG_STATE_IDLE      = 0,  /* Waiting for next 20ms cycle */
    TAG_STATE_TX_POLL   = 1,  /* POLL sent, waiting for TXFRS IRQ */
    TAG_STATE_WAIT_RESP = 2,  /* RX active, waiting for RESP from anchor */
    TAG_STATE_INTER_ANCHOR_GUARD = 3, /* Let all anchors return to RX before next POLL */
#if UWB_USE_DS_TWR
    TAG_STATE_TX_FINAL    = 4, /* FINAL sent (immediate TX), waiting for TXFRS to read T5 */
    TAG_STATE_WAIT_REPORT = 5, /* RX active, waiting for REPORT from anchor */
#endif
} TagState_t;

/* ========================================================================== */
/*                     PRIVATE DATA                                            */
/* ========================================================================== */

static TagState_t          s_state          = TAG_STATE_IDLE;
static uint32_t            s_cycle_tick     = 0;  /* Tick of last cycle start */
static McuCycleStamp_t      s_state_cycle    = 0;  /* FIX-02: mốc cycle thô (thay s_state_tick/s_state_tick_us) */
static McuCycleStamp_t      s_cycle_end_cycle = 0; /* End of previous full cycle; overload recovery */
static uint8_t             s_current_anchor = 0;  /* Sequential slot in s_anchor_cfg */
static uint8_t             s_seq_num        = 0;
static uint32_t            s_anchor_next_probe_cycle[TAG_NUM_ANCHORS] = {0};
static uint8_t             s_backoff_probe_used = 0;

/* Poll TX timestamp for current POLL (used in SS-TWR calc) */
static uint8_t             s_poll_tx_ts[5]  = {0};

static DW1000_RangingResult s_result        = {0};

#if UWB_USE_DS_TWR
/* DS-TWR per-exchange state — valid only during TX_FINAL / WAIT_REPORT */
static uint8_t             s_resp_rx_ts[5]   = {0};  /* T4: RESP RX timestamp */
static uint32_t            s_da_from_resp    = 0;    /* Da from RESP payload bytes 10-13 */
static uint8_t             s_final_tx_ts[5]  = {0};  /* T5: FINAL TX timestamp (read after TXFRS) */
static DW1000_SignalDiag_t s_resp_diag       = {0};  /* RESP diag (read before FINAL TX) */
static float               s_resp_fpp        = 0.0f; /* FPP from RESP (saved before FINAL TX) */
static int32_t             s_resp_ci         = 0;    /* FIX-04: Carrier integrator of RESP, saved before FINAL TX */
#endif

/* Published per-anchor distances (volatile for Live Expressions / debugger) */
volatile int32_t distance_a1_mm = 0;
volatile int32_t distance_a2_mm = 0;
volatile int32_t distance_a3_mm = 0;
volatile int32_t distance_a4_mm = 0;
volatile int32_t distance_raw_mm[TAG_NUM_ANCHORS] = {0};

volatile int32_t distance_a1_filtered_mm = 0;
volatile int32_t distance_a2_filtered_mm = 0;
volatile int32_t distance_a3_filtered_mm = 0;
volatile int32_t distance_a4_filtered_mm = 0;
volatile int32_t distance_filtered_mm[TAG_NUM_ANCHORS] = {0};

volatile uint32_t tag_cycle_count = 0;

/* ========================================================================== */
/*                     PHASE 1: TELEMETRY STATE & COUNTERS                     */
/* ========================================================================== */

volatile uint8_t  tag_cycle_ready     = 0;
volatile uint32_t tag_sample_seq      = 0;

volatile uint32_t poll_sent_count     = 0;
volatile uint32_t response_ok_count   = 0;
volatile uint32_t rx_timeout_count    = 0;
volatile uint32_t rx_error_count      = 0;
volatile uint32_t cycle_overrun_count = 0;
volatile uint32_t anchor_success_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_timeout_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_rx_error_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_poll_tx_timeout_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_response_timeout_count[TAG_NUM_ANCHORS] = {0};
volatile uint16_t anchor_response_timeout_streak[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_poll_skipped_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_calibration_missing_count[TAG_NUM_ANCHORS] = {0};

/* Adaptive Legacy diagnostics intentionally remain outside the frozen range
 * record. They are available in Live Expressions for SHADOW/ACTIVE tuning. */
volatile uint8_t legacy_adaptive_state[TAG_NUM_ANCHORS] = {0};
volatile uint8_t legacy_adaptive_candidate_count[TAG_NUM_ANCHORS] = {0};
volatile int8_t legacy_adaptive_candidate_direction[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_track_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_track_exit_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_true_reject_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_stale_reacquire_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_track_duration_ms[TAG_NUM_ANCHORS] = {0};
volatile int32_t legacy_adaptive_last_innovation_mm[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_max_abs_innovation_mm[TAG_NUM_ANCHORS] = {0};
volatile double legacy_adaptive_last_q[TAG_NUM_ANCHORS] = {0.0};
volatile double legacy_adaptive_last_r[TAG_NUM_ANCHORS] = {0.0};
volatile double legacy_adaptive_last_gain[TAG_NUM_ANCHORS] = {0.0};
const uint8_t uwb_legacy_adaptive_mode_build = (uint8_t)UWB_LEGACY_ADAPTIVE_MODE;

/* C9.2 diagnostics remain out of the frozen range record. The runtime global
 * regime is a quality hint derived from independently filtered anchor states;
 * it never controls radio scheduling or a single anchor's acceptance gate. */
volatile uint8_t c9_2_motion_state[TAG_NUM_ANCHORS] = {0};
volatile uint8_t c9_2_global_motion_state = 0;
volatile uint32_t c9_2_static_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_slow_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_fast_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_settling_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_degraded_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_stale_reacquire_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_true_reject_count[TAG_NUM_ANCHORS] = {0};
volatile int32_t c9_2_last_slope_mm_s[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_last_motion_score_milli[TAG_NUM_ANCHORS] = {0};
volatile double c9_2_last_gain[TAG_NUM_ANCHORS] = {0.0};
const uint8_t uwb_c9_2_motion_mode_build = (uint8_t)UWB_C9_2_MOTION_MODE;

/* DS-TWR counters (Phase 4) — always defined as 0 when flag is OFF */
volatile uint32_t ds_ok_count             = 0;
volatile uint32_t ds_report_ok_count      = 0;
volatile uint32_t ds_report_timeout_count = 0;
volatile uint32_t ds_fallback_count       = 0;
volatile uint32_t ds_final_tx_timeout_count = 0;
volatile uint32_t ds_report_rx_error_count  = 0;
/** C0.1: Đếm DS result bị reject do anchor hiện tại chưa được calibration. */
volatile uint32_t ds_cal_missing_count    = 0;

const uint8_t uwb_ds_mode_enabled_build = (uint8_t)UWB_USE_DS_TWR;
const uint8_t uwb_ds_calibrated_mask_build = (uint8_t)UWB_DS_CALIBRATED_MASK;

/* Instrumentation chu kỳ (§5.2 Phase 4) */
volatile uint32_t tag_cycle_duration_us     = 0;
volatile uint32_t tag_cycle_duration_max_us = 0;
volatile uint32_t anchor_slot_duration_us[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_slot_duration_max_us[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_poll_tx_duration_max_us[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_response_wait_max_us[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_processing_max_us[TAG_NUM_ANCHORS] = {0};
static McuCycleStamp_t s_full_cycle_start   = 0;
static McuCycleStamp_t s_slot_start         = 0;

/* Theo dõi kết quả gần nhất của từng anchor (valid/age/quality). */
typedef struct {
    int32_t  raw_mm;
    int32_t  filtered_mm;
    int16_t  fpp_cdbm;
    uint8_t  valid;
    uint8_t  status;
    uint32_t last_success_tick;
    uint16_t consecutive_failures;
} AnchorTrack_t;

static AnchorTrack_t s_track[TAG_NUM_ANCHORS];

/**
 * @brief  Ghi nhận kết quả một anchor trong lần thử của chu kỳ hiện tại.
 * @param  idx      chỉ số radio slot (0..TAG_NUM_ANCHORS-1)
 * @param  ok       1 = đo thành công; 0 = thất bại
 * @param  status   TAG_ST_* bit flags
 * @param  raw_mm   khoảng cách thô (chỉ dùng khi ok)
 * @param  filt_mm  khoảng cách đã lọc (chỉ dùng khi ok)
 * @param  fpp_dbm  first-path power dBm (chỉ dùng khi ok)
 */
static void mark_anchor_result(uint8_t idx, uint8_t ok, uint8_t status,
                               int32_t raw_mm, int32_t filt_mm, float fpp_dbm)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    AnchorTrack_t *t = &s_track[idx];
    t->status = status;
    if (ok)
    {
        t->raw_mm            = raw_mm;
        t->filtered_mm       = filt_mm;
        t->fpp_cdbm          = (int16_t)(fpp_dbm * 100.0f);
        t->valid             = 1;
        t->last_success_tick = HAL_GetTick();
        t->consecutive_failures = 0;
    }
    else
    {
        /* Generic radio/compute failures carry no current measurement.
         * Clear raw/FPP so a previous pre-offset calibration diagnostic cannot
         * survive under an unrelated TIMEOUT/RXERR/COMPUTE status. Keep only
         * filtered_mm + last_success_tick as the explicit last-good hold. */
        t->raw_mm  = 0;
        t->fpp_cdbm = 0;
        t->valid   = 0;
        if (t->consecutive_failures < 0xFFFF)
            t->consecutive_failures++;
    }
}

/**
 * @brief  Lưu measurement diagnostic hiện tại nhưng vẫn đánh dấu production invalid.
 *         Dùng khi profile calibration hiện tại còn thiếu: raw/FPP phải còn để
 *         hiệu chỉnh, nhưng solver không được coi đây là một lần đo thành công.
 */
static void mark_anchor_rejected_measurement(uint8_t idx, uint8_t status,
                                             int32_t raw_mm, float fpp_dbm)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    AnchorTrack_t *t = &s_track[idx];
    t->status   = status;
    t->raw_mm   = raw_mm;
    t->fpp_cdbm = (int16_t)(fpp_dbm * 100.0f);
    t->valid    = 0U;

    /* The UWB exchange and ToF computation succeeded; only the calibration
     * profile is missing. Count it as a ranging operation so Link/Stats do not
     * misleadingly report 0 ops/s, but keep production valid=0 so the solver
     * can never consume this pre-offset value. */
    s_result.ranging_count++;
    response_ok_count++;
    anchor_calibration_missing_count[idx]++;

    /* Expose the pre-offset measurement in debugger mirrors. Telemetry still
     * carries it through the explicit diagnostic channel/status 0x20. */
    distance_raw_mm[idx] = raw_mm;
    switch (anchor_id_at(idx))
    {
        case 0x0001U: distance_a1_mm = raw_mm; break;
        case 0x0002U: distance_a2_mm = raw_mm; break;
        case 0x0003U: distance_a3_mm = raw_mm; break;
        case 0x0004U: distance_a4_mm = raw_mm; break;
        default: break;
    }

    if (t->consecutive_failures < 0xFFFFU)
        t->consecutive_failures++;
}

/**
 * @brief Preserve a radio-successful measurement rejected by C9 conditioner.
 *
 * Unlike calibration rejection this must not increment the calibration-missing
 * counter. last_success_tick and the held filtered value are deliberately not
 * updated, so age/valid continue to describe the canonical measurement.
 */
static void mark_anchor_conditioner_reject(uint8_t idx, uint8_t status,
                                           int32_t raw_mm, float fpp_dbm)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    AnchorTrack_t *t = &s_track[idx];
    t->status = status;
    t->raw_mm = raw_mm;
    t->fpp_cdbm = (int16_t)(fpp_dbm * 100.0f);
    t->valid = 0U;
    if (t->consecutive_failures < 0xFFFFU)
        t->consecutive_failures++;

    s_result.ranging_count++;
    s_result.distance_mm = raw_mm;
    response_ok_count++;

    distance_raw_mm[idx] = raw_mm;
    switch (anchor_id_at(idx))
    {
        case 0x0001U: distance_a1_mm = raw_mm; break;
        case 0x0002U: distance_a2_mm = raw_mm; break;
        case 0x0003U: distance_a3_mm = raw_mm; break;
        case 0x0004U: distance_a4_mm = raw_mm; break;
        default: break;
    }
}

/* ========================================================================== */
/*                     KALMAN FILTER                                           */
/* ========================================================================== */

#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_LEGACY_KALMAN
typedef struct {
    double Q; /* Process noise */
    double R; /* Measurement noise */
    double x; /* Estimated value */
    double P; /* Estimation error */
    uint8_t  outlier_count;  /* Consecutive outliers */
    uint32_t last_meas_tick; /* MOỚI (Task 1): HAL_GetTick() của lần đo được CHẤP NHẬN gần nhất */
} KalmanFilter_t;

static KalmanFilter_t s_kf[TAG_NUM_ANCHORS];
static uint8_t s_kf_initialized[TAG_NUM_ANCHORS] = {0};

#if UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_OFF
static const LegacyAdaptiveTrackingConfig_t s_legacy_adaptive_config = {
    .motion_enter_mm = UWB_LEGACY_ADAPTIVE_MOTION_ENTER_MM,
    .settle_residual_mm = UWB_LEGACY_ADAPTIVE_SETTLE_RESIDUAL_MM,
    .candidate_cluster_mm = UWB_LEGACY_ADAPTIVE_CANDIDATE_CLUSTER_MM,
    .max_radial_speed_mm_s = UWB_DRONE_VMAX_MM_S,
    .gate_margin_up_mm = UWB_GATE_MARGIN_UP_MM,
    .gate_margin_down_mm = UWB_GATE_MARGIN_DOWN_MM,
    .reacquire_min_fpp_dbm = UWB_LEGACY_ADAPTIVE_REACQUIRE_MIN_FPP_DBM,
    .motion_confirm_samples = UWB_LEGACY_ADAPTIVE_MOTION_CONFIRM_SAMPLES,
    .stale_reacquire_samples = UWB_LEGACY_ADAPTIVE_STALE_REACQUIRE_SAMPLES,
    .settle_samples = UWB_LEGACY_ADAPTIVE_SETTLE_SAMPLES,
    .stale_reset_ms = UWB_LEGACY_ADAPTIVE_STALE_RESET_MS,
    .candidate_max_gap_ms = UWB_LEGACY_ADAPTIVE_CANDIDATE_MAX_GAP_MS,
    .nominal_sample_ms = UWB_LEGACY_ADAPTIVE_NOMINAL_SAMPLE_MS,
    .min_tracking_dt_ms = UWB_LEGACY_ADAPTIVE_TRACK_MIN_DT_MS,
    .max_tracking_dt_ms = UWB_LEGACY_ADAPTIVE_TRACK_MAX_DT_MS,
};

static LegacyAdaptiveTrackingState_t s_legacy_adaptive_state[TAG_NUM_ANCHORS];

#if UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_SHADOW
/* SHADOW owns independent range state. It can never alter the published
 * Legacy state, even if the candidate has a bug or an extreme observation. */
static KalmanFilter_t s_legacy_adaptive_shadow_kf[TAG_NUM_ANCHORS];
static uint8_t s_legacy_adaptive_shadow_initialized[TAG_NUM_ANCHORS] = {0};
#endif
#endif

/* ========================================================================== */
/*                     MEDIAN FILTER                                           */
/* ========================================================================== */

typedef struct {
    int32_t buf[3];
    uint8_t idx;
    uint8_t count;
} MedianFilter_t;

static MedianFilter_t s_mf[TAG_NUM_ANCHORS];

#if UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_SHADOW
static MedianFilter_t s_legacy_adaptive_shadow_mf[TAG_NUM_ANCHORS];
#endif

#if UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_OFF
/* C9.2 owns an entirely separate candidate state in SHADOW. ACTIVE reuses
 * this state as the only published candidate; it never aliases s_kf/s_mf. */
static MotionAdaptiveRangeState_t s_c9_2_motion[TAG_NUM_ANCHORS];
static KalmanFilter_t s_c9_2_kf[TAG_NUM_ANCHORS];
static uint8_t s_c9_2_kf_initialized[TAG_NUM_ANCHORS] = {0};
static MedianFilter_t s_c9_2_mf[TAG_NUM_ANCHORS];

static const MotionAdaptiveRangeConfig_t s_c9_2_motion_config = {
    .slow_enter_z = UWB_C9_2_SLOW_ENTER_Z,
    .slow_exit_z = UWB_C9_2_SLOW_EXIT_Z,
    .fast_enter_z = UWB_C9_2_FAST_ENTER_Z,
    .fast_exit_z = UWB_C9_2_FAST_EXIT_Z,
    .cusum_drift_z = UWB_C9_2_CUSUM_DRIFT_Z,
    .slow_speed_mm_s = UWB_C9_2_SLOW_SPEED_MM_S,
    .fast_speed_mm_s = UWB_C9_2_FAST_SPEED_MM_S,
    .slow_gain = UWB_C9_2_SLOW_GAIN,
    .fast_gain = UWB_C9_2_FAST_GAIN,
    .settling_gain = UWB_C9_2_SETTLING_GAIN,
    .minimum_sigma_mm = UWB_C9_2_MINIMUM_SIGMA_MM,
    .reacquire_min_fpp_dbm = UWB_C9_2_REACQUIRE_MIN_FPP_DBM,
    .reacquire_cluster_mm = UWB_C9_2_REACQUIRE_CLUSTER_MM,
    .max_radial_speed_mm_s = UWB_DRONE_VMAX_MM_S,
    .gate_margin_up_mm = UWB_GATE_MARGIN_UP_MM,
    .gate_margin_down_mm = UWB_GATE_MARGIN_DOWN_MM,
    .slow_confirm_samples = UWB_C9_2_SLOW_CONFIRM_SAMPLES,
    .fast_confirm_samples = UWB_C9_2_FAST_CONFIRM_SAMPLES,
    .settle_dwell_samples = UWB_C9_2_SETTLE_DWELL_SAMPLES,
    .static_dwell_samples = UWB_C9_2_STATIC_DWELL_SAMPLES,
    .reacquire_samples = UWB_C9_2_REACQUIRE_SAMPLES,
    .degrade_after_rejects = UWB_C9_2_DEGRADE_AFTER_REJECTS,
    .stale_reset_ms = UWB_C9_2_STALE_RESET_MS,
    .candidate_max_gap_ms = UWB_C9_2_CANDIDATE_MAX_GAP_MS,
    .min_dt_ms = UWB_C9_2_MIN_DT_MS,
    .max_dt_ms = UWB_C9_2_MAX_DT_MS,
};
#endif

static int32_t apply_median_filter(MedianFilter_t* mf, int32_t new_val)
{
    /* Add to circular buffer */
    mf->buf[mf->idx] = new_val;
    mf->idx = (mf->idx + 1) % 3;
    if (mf->count < 3) mf->count++;

    /* If not enough samples, just return the raw value */
    if (mf->count < 3) return new_val;

    /* Sort the 3 values to find the median */
    int32_t a = mf->buf[0];
    int32_t b = mf->buf[1];
    int32_t c = mf->buf[2];

    if (a > b) { int32_t tmp = a; a = b; b = tmp; }
    if (b > c) { int32_t tmp = b; b = c; c = tmp; }
    if (a > b) { int32_t tmp = a; a = b; b = tmp; }

    return b; /* Median is in the middle */
}

/* ========================================================================== */
/*                     OUTLIER GATE                                            */
/* ========================================================================== */

/**
 * @brief  Outlier gate động học theo vận tốc × thời gian thực đã trôi (Task 1, Phase 4).
 *         Ngưỡng tự nới khi drone di chuyển nhanh hoặc khi bị timeout vài chu kỳ.
 *         Vẫn giữ bất đối xứng: multipath chỉ tăng khoảng cách, hướng dương chặt hơn.
 * @retval 1 if measurement is an outlier (rejected)
 * @retval 0 if measurement is normal (or snap backstop triggered)
 */
static uint8_t apply_outlier_gate_at(KalmanFilter_t* kf, double meas,
                                     uint32_t now, uint8_t allow_legacy_snap)
{
    uint32_t dt_ms = now - kf->last_meas_tick;
    /* Clamp: tối thiểu 1 chu kỳ (20ms), tối đa 500ms — quá 500ms ngưỡng rộng
     * gần như tắt gate, backstop snap sẽ xử lý phần còn lại. */
    if (dt_ms < 20)  dt_ms = 20;
    if (dt_ms > 500) dt_ms = 500;
    double dt_s = (double)dt_ms / 1000.0;

    double up_thr   =  UWB_DRONE_VMAX_MM_S * dt_s + UWB_GATE_MARGIN_UP_MM;
    double down_thr = -(UWB_DRONE_VMAX_MM_S * dt_s + UWB_GATE_MARGIN_DOWN_MM);

    double jump = meas - kf->x;
    if (jump > up_thr || jump < down_thr) {
        if (kf->outlier_count < 0xFFU)
            kf->outlier_count++;
        /* KHÔNG cập nhật last_meas_tick khi reject → dt tự lớn dần → ngưỡng tự
         * nới → tự phục hồi nhanh, không cần chờ đủ snap count trong đa số case. */
        if (allow_legacy_snap == 0U || kf->outlier_count < UWB_GATE_SNAP_AFTER) {
            return 1;
        }
        kf->x            = meas;   /* snap backstop */
        kf->outlier_count  = 0;
        kf->last_meas_tick = now;
        return 0;
    }
    kf->outlier_count  = 0;
    kf->last_meas_tick = now;
    return 0;
}

/* Retain the deployed Legacy call path unchanged. Adaptive builds use the
 * explicit-timestamp form above so host tests and offline-anchor timing use
 * the same wrap-safe elapsed-time semantics. */
#if UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_ACTIVE
static uint8_t apply_outlier_gate(KalmanFilter_t* kf, double meas)
{
    return apply_outlier_gate_at(kf, meas, HAL_GetTick(), 1U);
}
#endif

/* ========================================================================== */
/*                     KALMAN UPDATE                                           */
/* ========================================================================== */

static double kalman_measurement_noise(float fpp)
{
    /* FPP is a noise hint, not a validity threshold: calibrated DS captures
     * can legitimately be around -97 dBm. */
    if (fpp <= -95.0f) {
        return 10000.0;
    } else if (fpp > -75.0f) {
        return 50.0;
    } else if (fpp > -82.0f) {
        return 200.0;
    }
    return 1000.0;  /* Medium/noisy signal -> filter heavily. */
}

static double kalman_update_with_gain(KalmanFilter_t* kf, double meas,
                                      float fpp, double *gain_out)
{
    kf->R = kalman_measurement_noise(fpp);

    /* Prediction (static model) */
    kf->P = kf->P + kf->Q;

    /* Update */
    double K = kf->P / (kf->P + kf->R);
    kf->x = kf->x + K * (meas - kf->x);
    kf->P = (1.0 - K) * kf->P;
    if (gain_out != NULL)
        *gain_out = K;

    return kf->x;
}

#if UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_OFF
typedef struct {
    uint8_t publish_valid;
    uint8_t reacquired;
    int32_t filtered_mm;
    double innovation_mm;
    double gain;
} C9_2MotionProcessResult_t;

static double c9_2_static_posterior_covariance(double measurement_noise)
{
    const double q = UWB_LEGACY_BASE_PROCESS_NOISE;
    return (sqrt(q * q + 4.0 * q * measurement_noise) - q) * 0.5;
}

static void c9_2_kalman_initialize(KalmanFilter_t *kf, double measurement_mm,
                                   float fpp, uint32_t now_ms)
{
    const double measurement_noise = kalman_measurement_noise(fpp);

    kf->x = measurement_mm;
    kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
    kf->R = measurement_noise;
    kf->P = c9_2_static_posterior_covariance(measurement_noise);
    kf->outlier_count = 0U;
    kf->last_meas_tick = now_ms;
}

/* Set P/Q so the first update in a new dynamic regime has exactly target K.
 * This makes the response deterministic across FPP/R, rather than relying on
 * a process-noise value tuned only for one signal strength. */
static void c9_2_seed_dynamic_gain(KalmanFilter_t *kf,
                                   double measurement_noise,
                                   double target_gain,
                                   uint32_t dt_ms)
{
    double dt_scale;

    if (target_gain <= 0.0 || target_gain >= 0.95)
        return;
    dt_scale = (double)dt_ms / (double)UWB_LEGACY_ADAPTIVE_NOMINAL_SAMPLE_MS;
    kf->R = measurement_noise;
    kf->Q = measurement_noise * target_gain * target_gain
          / (1.0 - target_gain) * dt_scale;
    /* kalman_update_with_gain adds Q before K.  P=R*K gives K=target_gain
     * for the first dynamic update at nominal dt. */
    kf->P = measurement_noise * target_gain;
}

static int32_t c9_2_to_i32(double value)
{
    if (value > (double)INT32_MAX)
        return INT32_MAX;
    if (value < (double)INT32_MIN)
        return INT32_MIN;
    return (int32_t)value;
}

static uint32_t c9_2_score_to_milli(float score)
{
    double scaled = (double)score * 1000.0;
    if (scaled <= 0.0)
        return 0U;
    if (scaled >= (double)UINT32_MAX)
        return UINT32_MAX;
    return (uint32_t)scaled;
}

static void c9_2_publish_diagnostics(uint8_t anchor_index,
                                     const MotionAdaptiveRangeState_t *motion,
                                     double gain)
{
    if (anchor_index >= TAG_NUM_ANCHORS)
        return;

    c9_2_motion_state[anchor_index] = motion->mode;
    c9_2_static_enter_count[anchor_index] = motion->static_enter_count;
    c9_2_slow_enter_count[anchor_index] = motion->slow_enter_count;
    c9_2_fast_enter_count[anchor_index] = motion->fast_enter_count;
    c9_2_settling_enter_count[anchor_index] = motion->settling_enter_count;
    c9_2_degraded_enter_count[anchor_index] = motion->degraded_enter_count;
    c9_2_stale_reacquire_count[anchor_index] = motion->stale_reacquire_count;
    c9_2_true_reject_count[anchor_index] = motion->true_reject_count;
    c9_2_last_slope_mm_s[anchor_index] = c9_2_to_i32(motion->last_slope_mm_s);
    c9_2_last_motion_score_milli[anchor_index] =
        c9_2_score_to_milli(motion->last_motion_score);
    c9_2_last_gain[anchor_index] = gain;
}

/* Aggregate only for telemetry/UI.  It cannot alter the per-anchor state
 * machines: a single degraded anchor must not slow healthy anchors. */
static void c9_2_refresh_global_motion_state(void)
{
    uint8_t index;
    uint8_t static_count = 0U;
    uint8_t slow_count = 0U;
    uint8_t fast_count = 0U;
    uint8_t settling_count = 0U;
    uint8_t degraded_count = 0U;
    uint8_t reacquire_count = 0U;

    for (index = 0U; index < TAG_NUM_ANCHORS; index++)
    {
        uint8_t mode = c9_2_motion_state[index];
        if (mode == (uint8_t)MOTION_RANGE_FAST)
            fast_count++;
        else if (mode == (uint8_t)MOTION_RANGE_SLOW)
            slow_count++;
        else if (mode == (uint8_t)MOTION_RANGE_SETTLING)
            settling_count++;
        else if (mode == (uint8_t)MOTION_RANGE_STATIC)
            static_count++;
        else if (mode == (uint8_t)MOTION_RANGE_DEGRADED)
            degraded_count++;
        else
            reacquire_count++;
    }

    if (degraded_count != 0U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_DEGRADED;
    else if (fast_count >= 2U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_FAST;
    else if ((slow_count + fast_count) >= 2U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_SLOW;
    else if (settling_count >= 2U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_SETTLING;
    else if (static_count >= 3U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_STATIC;
    else if (reacquire_count != 0U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_REACQUIRE;
    else
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_STATIC;
}

static C9_2MotionProcessResult_t c9_2_motion_process(
    uint8_t anchor_index,
    int32_t raw_mm,
    float fpp,
    uint32_t now_ms)
{
    C9_2MotionProcessResult_t result = {0U, 0U, 0, 0.0, 0.0};
    MotionAdaptiveRangeState_t *motion = &s_c9_2_motion[anchor_index];
    KalmanFilter_t *kf = &s_c9_2_kf[anchor_index];
    int32_t median_mm;
    double measurement_noise;
    double sigma_mm;
    uint32_t dt_ms = UWB_LEGACY_ADAPTIVE_NOMINAL_SAMPLE_MS;

    if (MotionAdaptiveRange_BeginSample(motion, now_ms,
                                        &s_c9_2_motion_config) != 0U)
    {
        /* A discontinuity invalidates both median history and old covariance. */
        memset(&s_c9_2_mf[anchor_index], 0, sizeof(s_c9_2_mf[anchor_index]));
        s_c9_2_kf_initialized[anchor_index] = 0U;
    }

    median_mm = apply_median_filter(&s_c9_2_mf[anchor_index], raw_mm);

    if (motion->last_median_tick != 0U)
    {
        dt_ms = MotionAdaptiveRange_ClampU32(
            MotionAdaptiveRange_ElapsedMs(now_ms, motion->last_median_tick),
            s_c9_2_motion_config.min_dt_ms,
            s_c9_2_motion_config.max_dt_ms);
    }

    if (motion->mode == (uint8_t)MOTION_RANGE_REACQUIRE
        || motion->mode == (uint8_t)MOTION_RANGE_DEGRADED)
    {
        float reacquired_mm = 0.0f;
        if (MotionAdaptiveRange_ObserveReacquire(
                motion, (float)median_mm, fpp, now_ms,
                &s_c9_2_motion_config, &reacquired_mm) == 0U)
        {
            c9_2_publish_diagnostics(anchor_index, motion, 0.0);
            c9_2_refresh_global_motion_state();
            return result;
        }

        c9_2_kalman_initialize(kf, (double)reacquired_mm, fpp, now_ms);
        s_c9_2_kf_initialized[anchor_index] = 1U;
        MotionAdaptiveRange_NoteAcceptedMedian(
            motion, reacquired_mm, now_ms, &s_c9_2_motion_config);
        result.publish_valid = 1U;
        result.reacquired = 1U;
        result.filtered_mm = c9_2_to_i32(reacquired_mm);
        result.gain = 1.0;
        c9_2_publish_diagnostics(anchor_index, motion, result.gain);
        c9_2_refresh_global_motion_state();
        return result;
    }

    if (s_c9_2_kf_initialized[anchor_index] == 0U)
    {
        /* This normally only occurs when mode was changed in a debugger.
         * Restart cleanly rather than mixing a filter from a former profile. */
        MotionAdaptiveRange_Enter(motion, (uint8_t)MOTION_RANGE_REACQUIRE);
        c9_2_publish_diagnostics(anchor_index, motion, 0.0);
        c9_2_refresh_global_motion_state();
        return result;
    }

    result.innovation_mm = (double)median_mm - kf->x;
    if (MotionAdaptiveRange_AcceptKinematic(
            motion, (float)median_mm, now_ms, &s_c9_2_motion_config) == 0U)
    {
        MotionAdaptiveRange_NoteReject(motion, &s_c9_2_motion_config);
        if (motion->mode == (uint8_t)MOTION_RANGE_DEGRADED)
        {
            /* Do not let the outlier that caused degradation contaminate the
             * new physical-location candidate. The next three compatible
             * medians are therefore a bounded 60 ms reacquire at 50 Hz. */
            memset(&s_c9_2_mf[anchor_index], 0, sizeof(s_c9_2_mf[anchor_index]));
            s_c9_2_kf_initialized[anchor_index] = 0U;
            motion->last_median_mm = 0.0f;
            motion->last_median_tick = 0U;
            motion->slope_count = 0U;
            motion->slope_head = 0U;
            motion->last_slope_mm_s = 0.0f;
            motion->last_motion_score = 0.0f;
        }
        c9_2_publish_diagnostics(anchor_index, motion, 0.0);
        c9_2_refresh_global_motion_state();
        return result;
    }

    measurement_noise = kalman_measurement_noise(fpp);
    sigma_mm = sqrt(measurement_noise);
    (void)MotionAdaptiveRange_ObserveAccepted(
        motion, (float)median_mm, (float)result.innovation_mm, (float)sigma_mm,
        now_ms, &s_c9_2_motion_config);

    if (MotionAdaptiveRange_IsDynamic(motion->mode) != 0U)
    {
        const double target_gain = (double)MotionAdaptiveRange_TargetGain(
            motion, &s_c9_2_motion_config);
        c9_2_seed_dynamic_gain(kf, measurement_noise, target_gain, dt_ms);
    }
    else
    {
        kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
        kf->R = measurement_noise;
        kf->P = c9_2_static_posterior_covariance(measurement_noise);
    }

    result.filtered_mm = c9_2_to_i32(kalman_update_with_gain(
        kf, (double)median_mm, fpp, &result.gain));
    result.publish_valid = 1U;
    c9_2_publish_diagnostics(anchor_index, motion, result.gain);
    c9_2_refresh_global_motion_state();
    return result;
}
#endif

#if UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_OFF
typedef struct {
    uint8_t publish_valid;
    int32_t filtered_mm;
    double innovation_mm;
    double gain;
} LegacyAdaptiveProcessResult_t;

static double legacy_stable_posterior_covariance(double measurement_noise)
{
    const double q = UWB_LEGACY_BASE_PROCESS_NOISE;
    return (sqrt(q * q + 4.0 * q * measurement_noise) - q) * 0.5;
}

static void legacy_kalman_initialize(KalmanFilter_t *kf, int32_t measurement_mm,
                                     uint32_t now_ms)
{
    kf->x = (double)measurement_mm;
    kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
    kf->P = 1.0;
    kf->R = 0.0;
    kf->outlier_count = 0U;
    kf->last_meas_tick = now_ms;
}

static double legacy_tracking_process_noise(double measurement_noise,
                                            uint32_t dt_ms)
{
    const double tracking_gain = UWB_LEGACY_ADAPTIVE_TRACK_GAIN;
    const double nominal_q = measurement_noise * tracking_gain * tracking_gain
                           / (1.0 - tracking_gain);
    const double dt_scale = (double)dt_ms
                          / (double)UWB_LEGACY_ADAPTIVE_NOMINAL_SAMPLE_MS;
    return nominal_q * dt_scale;
}

static void legacy_seed_tracking_gain(KalmanFilter_t *kf,
                                      double measurement_noise)
{
    const double tracking_gain = UWB_LEGACY_ADAPTIVE_TRACK_GAIN;
    kf->R = measurement_noise;
    kf->Q = legacy_tracking_process_noise(
        measurement_noise, UWB_LEGACY_ADAPTIVE_NOMINAL_SAMPLE_MS);
    /* kalman_update adds Q before forming K. This P/Q pair gives the first
     * tracking update the requested K_track, even at weak FPP (R=10000). */
    kf->P = measurement_noise * tracking_gain;
}

static int32_t legacy_adaptive_innovation_to_i32(double innovation_mm)
{
    if (innovation_mm > (double)INT32_MAX)
        return INT32_MAX;
    if (innovation_mm < (double)INT32_MIN)
        return INT32_MIN;
    return (int32_t)innovation_mm;
}

static uint32_t legacy_adaptive_abs_innovation_to_u32(double innovation_mm)
{
    const double absolute_mm = innovation_mm < 0.0 ? -innovation_mm : innovation_mm;
    if (absolute_mm > (double)UINT32_MAX)
        return UINT32_MAX;
    return (uint32_t)absolute_mm;
}

static void legacy_adaptive_publish_diagnostics(
    uint8_t anchor_index,
    const LegacyAdaptiveTrackingState_t *adaptive,
    const KalmanFilter_t *kf,
    double innovation_mm,
    double gain,
    uint32_t now_ms)
{
    uint32_t absolute_innovation;

    if (anchor_index >= TAG_NUM_ANCHORS)
        return;

    absolute_innovation = legacy_adaptive_abs_innovation_to_u32(innovation_mm);
    legacy_adaptive_state[anchor_index] = adaptive->state;
    legacy_adaptive_candidate_count[anchor_index] = adaptive->candidate_count;
    legacy_adaptive_candidate_direction[anchor_index] = adaptive->candidate_direction;
    legacy_adaptive_track_enter_count[anchor_index] = adaptive->track_enter_count;
    legacy_adaptive_track_exit_count[anchor_index] = adaptive->track_exit_count;
    legacy_adaptive_true_reject_count[anchor_index] = adaptive->true_reject_count;
    legacy_adaptive_stale_reacquire_count[anchor_index] = adaptive->stale_reacquire_count;
    legacy_adaptive_last_innovation_mm[anchor_index] =
        legacy_adaptive_innovation_to_i32(innovation_mm);
    if (absolute_innovation > legacy_adaptive_max_abs_innovation_mm[anchor_index])
        legacy_adaptive_max_abs_innovation_mm[anchor_index] = absolute_innovation;
    legacy_adaptive_last_q[anchor_index] = kf->Q;
    legacy_adaptive_last_r[anchor_index] = kf->R;
    legacy_adaptive_last_gain[anchor_index] = gain;
    legacy_adaptive_track_duration_ms[anchor_index] =
        adaptive->state == (uint8_t)LEGACY_ADAPTIVE_STATE_TRACKING
            ? LegacyAdaptive_ElapsedMs(now_ms, adaptive->tracking_start_tick)
            : 0U;
}

/**
 * @brief Controlled candidate used by SHADOW and ACTIVE Adaptive Legacy.
 *
 * ACTIVE returns publish_valid=0 during a stale reacquire or an unconfirmed
 * candidate that also breaches the existing dynamic gate, preserving the last
 * canonical range rather than feeding a likely spike into the solver. A
 * candidate still inside the proven Legacy gate remains on the exact baseline
 * path until it has enough evidence to enter TRACK. SHADOW follows the same
 * candidate logic on independent filter state, but its return value is
 * intentionally ignored by production.
 */
static LegacyAdaptiveProcessResult_t legacy_adaptive_process(
    KalmanFilter_t *kf,
    uint8_t *kf_initialized,
    MedianFilter_t *median,
    LegacyAdaptiveTrackingState_t *adaptive,
    int32_t raw_mm,
    float fpp,
    uint32_t now_ms)
{
    LegacyAdaptiveProcessResult_t result = {0U, 0, 0.0, 0.0};
    int32_t median_mm;
    double measurement_noise;
    uint8_t legacy_gate_rejected;
    uint8_t entered_tracking;

    if (LegacyAdaptiveTracking_BeginSample(adaptive, now_ms,
                                           &s_legacy_adaptive_config) != 0U)
    {
        /* Never mix a pre-gap median window with a new physical location. */
        memset(median, 0, sizeof(*median));
    }

    median_mm = apply_median_filter(median, raw_mm);

    if (adaptive->state == (uint8_t)LEGACY_ADAPTIVE_STATE_STALE_REACQUIRE)
    {
        double reacquired_mm = 0.0;
        if (LegacyAdaptiveTracking_ObserveStaleReacquire(
                adaptive, (double)median_mm, fpp, now_ms,
                &s_legacy_adaptive_config, &reacquired_mm) == 0U)
        {
            result.innovation_mm = (double)median_mm - kf->x;
            legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                                result.innovation_mm, 0.0, now_ms);
            return result;
        }

        measurement_noise = kalman_measurement_noise(fpp);
        kf->x = reacquired_mm;
        kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
        kf->R = measurement_noise;
        kf->P = legacy_stable_posterior_covariance(measurement_noise);
        kf->outlier_count = 0U;
        kf->last_meas_tick = now_ms;
        *kf_initialized = 1U;
        result.publish_valid = 1U;
        result.filtered_mm = (int32_t)reacquired_mm;
        result.innovation_mm = 0.0;
        result.gain = 1.0; /* Explicit controlled stale reacquire, not a jump hidden as K. */
        legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                            result.innovation_mm, result.gain, now_ms);
        return result;
    }

    if (*kf_initialized == 0U)
    {
        legacy_kalman_initialize(kf, median_mm, now_ms);
        *kf_initialized = 1U;
    }

    result.innovation_mm = (double)median_mm - kf->x;

    if (adaptive->state == (uint8_t)LEGACY_ADAPTIVE_STATE_TRACKING)
    {
        uint32_t tracking_dt_ms = LegacyAdaptiveTracking_TrackingDtMs(
            adaptive, now_ms, &s_legacy_adaptive_config);
        if (LegacyAdaptiveTracking_AcceptTrackingMeasurement(
                adaptive, (double)median_mm, now_ms,
                &s_legacy_adaptive_config) == 0U)
        {
            legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                                result.innovation_mm, 0.0, now_ms);
            return result;
        }

        measurement_noise = kalman_measurement_noise(fpp);
        kf->Q = legacy_tracking_process_noise(measurement_noise, tracking_dt_ms);
        result.filtered_mm = (int32_t)kalman_update_with_gain(
            kf, (double)median_mm, fpp, &result.gain);
        LegacyAdaptiveTracking_NoteTrackingUpdate(adaptive, now_ms);
        if (LegacyAdaptiveTracking_ObserveSettled(
                adaptive, (double)median_mm, kf->x,
                &s_legacy_adaptive_config) != 0U)
        {
            /* Restore the Legacy steady-state covariance immediately after
             * movement settles, so a prior fast-tracking P cannot leak noise
             * into the same smooth static output the user has approved. */
            kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
            kf->P = legacy_stable_posterior_covariance(kf->R);
        }
        result.publish_valid = 1U;
        legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                            result.innovation_mm, result.gain, now_ms);
        return result;
    }

    /* Outside TRACK, retain the existing dt-aware gate but disable its legacy
     * 15-sample snap. ACTIVE accepts a large shift only after the explicit
     * four-sample candidate; SHADOW records the same decision independently. */
    legacy_gate_rejected = apply_outlier_gate_at(kf, (double)median_mm, now_ms, 0U);
    entered_tracking = LegacyAdaptiveTracking_ObserveMotionCandidate(
        adaptive, (double)median_mm, kf->x, now_ms, &s_legacy_adaptive_config);

    if (entered_tracking != 0U)
    {
        measurement_noise = kalman_measurement_noise(fpp);
        legacy_seed_tracking_gain(kf, measurement_noise);
        kf->outlier_count = 0U;
        kf->last_meas_tick = now_ms;
        (void)LegacyAdaptiveTracking_AcceptTrackingMeasurement(
            adaptive, (double)median_mm, now_ms, &s_legacy_adaptive_config);
        result.filtered_mm = (int32_t)kalman_update_with_gain(
            kf, (double)median_mm, fpp, &result.gain);
        LegacyAdaptiveTracking_NoteTrackingUpdate(adaptive, now_ms);
        result.publish_valid = 1U;
        legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                            result.innovation_mm, result.gain, now_ms);
        return result;
    }

    if (legacy_gate_rejected != 0U)
    {
        adaptive->true_reject_count++;
        legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                            result.innovation_mm, 0.0, now_ms);
        return result;
    }

    /* STABLE/CANDIDATE normal samples retain the original Legacy Q/R model. */
    kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
    result.filtered_mm = (int32_t)kalman_update_with_gain(
        kf, (double)median_mm, fpp, &result.gain);
    result.publish_valid = 1U;
    legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                        result.innovation_mm, result.gain, now_ms);
    return result;
}
#endif
#else
/* Candidate state is global so counters and decisions remain visible in Live
 * Expressions without changing the frozen 16-byte range record. */
RangeFilterState_t range_filter_state[TAG_NUM_ANCHORS];
volatile uint8_t range_filter_last_decision[TAG_NUM_ANCHORS] = {0};
volatile int32_t range_filter_last_innovation_mm[TAG_NUM_ANCHORS] = {0};
volatile uint32_t range_filter_max_abs_innovation_mm[TAG_NUM_ANCHORS] = {0};
#endif

/* ========================================================================== */
/*                     PRIVATE HELPERS                                         */
/* ========================================================================== */

static uint64_t ts_to_u64(const uint8_t *ts)
{
    uint64_t v = 0;
    v  = (uint64_t)ts[0];
    v |= (uint64_t)ts[1] <<  8;
    v |= (uint64_t)ts[2] << 16;
    v |= (uint64_t)ts[3] << 24;
    v |= (uint64_t)ts[4] << 32;
    return v;
}

/**
 * @brief  Build and fire a POLL frame to the specified anchor.
 *         Clears DW1000 status, writes frame, starts TX.
 */
static void send_poll(uint16_t target_anchor)
{
    uint8_t poll[POLL_FRAME_LEN];
    poll[0] = 0x41;
    poll[1] = 0x88;
    poll[2] = s_seq_num++;
    poll[3] = DW_PAN_ID & 0xFF;
    poll[4] = (DW_PAN_ID >> 8) & 0xFF;
    poll[5] = target_anchor & 0xFF;
    poll[6] = (target_anchor >> 8) & 0xFF;
    poll[7] = TAG_ADDR & 0xFF;
    poll[8] = (TAG_ADDR >> 8) & 0xFF;
    poll[9] = FRAME_POLL_FUNC;

    DW1000_ClearAllStatus();
    DW1000_WriteTxData(poll, POLL_FRAME_LEN);
    DW1000_SetTxFrameCtrl(POLL_FRAME_LEN + 2);  /* +2 HW FCS */
    DW1000_StartTx();

    poll_sent_count++;
}

/* Forward declaration — compute_distance_ss_terms_mm gọi apply_offset_and_clamp
 * nhưng hàm đó được định nghĩa sau. Đặt ngoài #if để luôn visible (FIX-04 §8.4).
 * C0: nhận thêm tham số mode để chọn đúng calibration profile (P0 fix). */
static int32_t apply_offset_and_clamp(double dist_m, UwbRangingMode_t mode);
static uint8_t meters_to_mm_checked(double dist_m, int32_t *out_mm);

/**
 * @brief  SS-TWR core calculation shared by SS path and DS fallback.
 *         Single implementation of t_round - t_reply formula with carrier
 *         integrator correction — ensures SS thuần and SS fallback are
 *         always identical in result for the same inputs (FIX-04 §8.2).
 *
 * @param  resp_rx_ts_raw    5-byte RESP RX timestamp
 * @param  t_reply_ticks     Da (anchor's reply delay) from RESP payload (uint32)
 * @param  carrier_integrator Raw CI value read immediately after RESP RX
 * @param  mode              C0: Ranging mode (SS, DS, SS_FALLBACK) — dùng chọn offset
 * @retval Distance in mm (>=0), or -1 if invalid, -2 if DS not calibrated
 */
static int32_t compute_distance_ss_terms_mm(const uint8_t *resp_rx_ts_raw,
                                             uint32_t t_reply_ticks,
                                             int32_t carrier_integrator,
                                             UwbRangingMode_t mode,
                                             int32_t *uncalibrated_raw_mm)
{
    if (uncalibrated_raw_mm == NULL)
        return RANGE_COMPUTE_ERROR;

    *uncalibrated_raw_mm = 0;

    uint64_t poll_tx = ts_to_u64(s_poll_tx_ts);
    uint64_t resp_rx = ts_to_u64(resp_rx_ts_raw);
    uint64_t t_round = (resp_rx - poll_tx) & 0xFFFFFFFFFFULL;
    uint64_t t_reply = (uint64_t)t_reply_ticks;

    if (t_reply == 0U || t_reply >= t_round)
        return -1;

    /* Chỉ một EMA cho mỗi anchor, dùng chung cho mọi đường SS. */
    static double ci_ema[TAG_NUM_ANCHORS] = {0.0};
    if (ci_ema[s_current_anchor] == 0.0)
    {
        ci_ema[s_current_anchor] = (double)carrier_integrator;
    }
    else
    {
        ci_ema[s_current_anchor] = ci_ema[s_current_anchor] * 0.95
                                 + (double)carrier_integrator * 0.05;
    }

#if UWB_USE_CLOCK_CORRECTION
    double clock_offset_ratio = ci_ema[s_current_anchor] * UWB_CLOCK_OFFSET_MULT;
#else
    double clock_offset_ratio = 0.0;
#endif

    double t_reply_corrected = (double)t_reply * (1.0 - clock_offset_ratio);
    double diff = (double)t_round - t_reply_corrected;
    if (diff < 0.0)
        return -1;

    double tof    = diff / 2.0;
    double dist_m = tof * UWB_DWT_TIME_UNIT_S * UWB_SPEED_OF_LIGHT;

    if (!meters_to_mm_checked(dist_m, uncalibrated_raw_mm))
        return RANGE_COMPUTE_ERROR;

    /* C0: SS_FALLBACK dùng profile SS — đúng, không cần đổi.
     * Mode truyền vào là tham số của hàm này (của caller). */
    return apply_offset_and_clamp(dist_m, mode);
}

/**
 * @brief  SS-TWR wrapper: parse T_reply from RESP frame bytes 10-13,
 *         read carrier integrator, then call shared helper.
 *         `compute_distance_mm()` is only called when rx_len >= 14 is
 *         already guaranteed, so reading bytes 10-13 is safe.
 *
 * @param  resp_rx_ts_raw  5-byte RX timestamp of RESP
 * @param  resp_frame      Full RESP frame buffer (bytes 10-13 = T_reply)
 * @retval Distance in mm (>=0), or -1 if invalid
 */
static int32_t compute_distance_mm(const uint8_t *resp_rx_ts_raw,
                                   const uint8_t *resp_frame,
                                   int32_t *uncalibrated_raw_mm)
{
    uint32_t t_reply = (uint32_t)resp_frame[10]
                     | ((uint32_t)resp_frame[11] <<  8)
                     | ((uint32_t)resp_frame[12] << 16)
                     | ((uint32_t)resp_frame[13] << 24);

    int32_t ci = DW1000_ReadCarrierIntegrator();
    return compute_distance_ss_terms_mm(resp_rx_ts_raw, t_reply, ci,
                                        UWB_MODE_SS, uncalibrated_raw_mm);
}

static uint8_t meters_to_mm_checked(double dist_m, int32_t *out_mm)
{
    if (out_mm == NULL || !isfinite(dist_m) || dist_m < 0.0
        || dist_m > ((double)INT32_MAX / 1000.0))
    {
        return 0U;
    }

    *out_mm = (int32_t)(dist_m * 1000.0);
    return 1U;
}

/**
 * @brief  Apply calibration theo mode bằng switch fail-closed.
 *         SS_FALLBACK luôn dùng profile SS. DS resolve validity + offset bằng
 *         anchor ID để không phụ thuộc thứ tự ANCHOR_LIST.
 * @retval Distance in mm (>=0), RANGE_COMPUTE_ERROR, hoặc
 *         RANGE_CALIBRATION_MISSING.
 */
static int32_t apply_offset_and_clamp(double dist_m, UwbRangingMode_t mode)
{
    int32_t dist_mm = 0;

    if (s_current_anchor >= TAG_NUM_ANCHORS || !isfinite(dist_m) || dist_m < 0.0)
        return RANGE_COMPUTE_ERROR;

    switch (mode)
    {
        case UWB_MODE_DS:
#if UWB_USE_DS_TWR
        {
            double ds_offset_m = 0.0;
            int32_t calibration_status = ds_calibration_offset_for_index(
                s_current_anchor, &ds_offset_m);
            if (calibration_status != 0)
                return calibration_status;

            dist_m -= ds_offset_m;
            if (dist_m < 0.0)
                dist_m = 0.0;

            return meters_to_mm_checked(dist_m, &dist_mm)
                ? dist_mm
                : RANGE_COMPUTE_ERROR;
        }
#else
            /* Fail closed: DS mode không được rơi xuống profile SS khi DS compile OFF. */
            return RANGE_COMPUTE_ERROR;
#endif

        case UWB_MODE_SS:
        case UWB_MODE_SS_FALLBACK:
        {
            const TagAnchorConfig_t *cfg = &s_anchor_cfg[s_current_anchor];
#if UWB_USE_LEGACY_OFFSET
            volatile double *offset = &calibration_offset_m[s_current_anchor];
#else
            volatile double *offset = &residual_offset_m[s_current_anchor];
#endif

            /* Auto-calibration is keyed by physical anchor ID. */
            if (auto_calibrate_actual_m > 0.001
                && auto_calibrate_anchor_id == cfg->id)
            {
                *offset = dist_m - auto_calibrate_actual_m;
                s_ss_calibrated_mask |= cfg->ss_calibration_bit;
                auto_calibrate_actual_m = 0.0;
                auto_calibrate_anchor_id = 0U;
            }

            if ((s_ss_calibrated_mask & cfg->ss_calibration_bit) == 0U)
                return RANGE_CALIBRATION_MISSING;

            dist_m -= *offset;
            if (dist_m < 0.0)
                dist_m = 0.0;

            return meters_to_mm_checked(dist_m, &dist_mm)
                ? dist_mm
                : RANGE_COMPUTE_ERROR;
        }

        default:
            return RANGE_COMPUTE_ERROR;
    }
}

/**
 * @brief  Common post-distance pipeline: median → outlier gate → Kalman → publish.
 *         Di chuyển code từ WAIT_RESP valid branch, không đổi logic.
 *         Called by SS path (direct) and DS paths (DS ok + DS fallback).
 * @param  dist_mm   Raw distance (mm) from compute_distance_mm or compute_distance_ds_mm
 * @param  diag      Signal diagnostics pointer (for FPP computation)
 * @param  fpp       First-path power in dBm (pre-computed from diag)
 * @param  st_flags  Extra TAG_ST_* flags to OR into mark_anchor_result (e.g. TAG_ST_DS_FALLBACK)
 */
static void publish_distance(int32_t dist_mm, const DW1000_SignalDiag_t *diag,
                             float fpp, uint8_t st_flags)
{
    (void)diag;  /* diag kept for future use / debug; FPP already extracted */
    float fpp_report = fpp;   /* FPP thật, trước khi outlier gate ép -100 */
    int32_t filtered_dist_mm;
    uint8_t publish_status = (uint8_t)(TAG_ST_OK | st_flags);

#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_LEGACY_KALMAN
#if UWB_C9_2_MOTION_MODE == UWB_C9_2_MOTION_ACTIVE
    {
        C9_2MotionProcessResult_t motion_result = c9_2_motion_process(
            s_current_anchor, dist_mm, fpp_report, HAL_GetTick());
        if (motion_result.publish_valid == 0U)
        {
            /* A controlled candidate/reacquire/reject is not a radio loss.
             * Keep the canonical range's age unchanged and expose raw/FPP
             * evidence for diagnostics and calibration. */
            mark_anchor_conditioner_reject(
                s_current_anchor,
                (uint8_t)(st_flags | TAG_ST_RANGE_REJECT),
                dist_mm,
                fpp_report);
            if (s_current_anchor == (TAG_NUM_ANCHORS - 1U))
                LED_TOGGLE();
            return;
        }
        filtered_dist_mm = motion_result.filtered_mm;
        if (motion_result.reacquired != 0U)
            publish_status = (uint8_t)(publish_status | TAG_ST_FILTER_REACQUIRE);
    }
#else
#if UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_OFF
    /* Apply Median Filter */
    int32_t med_dist_mm = apply_median_filter(&s_mf[s_current_anchor], dist_mm);

    /* Initialize Kalman Filter if needed */
    if (!s_kf_initialized[s_current_anchor]) {
        s_kf[s_current_anchor].x              = (double)med_dist_mm;
        s_kf[s_current_anchor].Q              = 0.05;
        s_kf[s_current_anchor].P              = 1.0;
        s_kf[s_current_anchor].outlier_count  = 0;
        s_kf[s_current_anchor].last_meas_tick = HAL_GetTick();
        s_kf_initialized[s_current_anchor]    = 1;
    }

    /* Apply Outlier Gate */
    uint8_t is_outlier = apply_outlier_gate(&s_kf[s_current_anchor], (double)med_dist_mm);
    if (is_outlier) {
        fpp = -100.0f; /* Force extremely high R in Kalman to reject this spike */
    }

    /* Apply Adaptive Kalman Filter */
    double filtered_mm = kalman_update_with_gain(
        &s_kf[s_current_anchor], (double)med_dist_mm, fpp, NULL);
    filtered_dist_mm = (int32_t)filtered_mm;
#elif UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_SHADOW
    /* Production is deliberately the exact deployed Legacy path in SHADOW. */
    int32_t med_dist_mm = apply_median_filter(&s_mf[s_current_anchor], dist_mm);
    if (!s_kf_initialized[s_current_anchor]) {
        s_kf[s_current_anchor].x              = (double)med_dist_mm;
        s_kf[s_current_anchor].Q              = 0.05;
        s_kf[s_current_anchor].P              = 1.0;
        s_kf[s_current_anchor].outlier_count  = 0;
        s_kf[s_current_anchor].last_meas_tick = HAL_GetTick();
        s_kf_initialized[s_current_anchor]    = 1;
    }
    if (apply_outlier_gate(&s_kf[s_current_anchor], (double)med_dist_mm))
        fpp = -100.0f;
    filtered_dist_mm = (int32_t)kalman_update_with_gain(
        &s_kf[s_current_anchor], (double)med_dist_mm, fpp, NULL);

    /* Run candidate state independently after the published path. It has its
     * own median/Kalman memory and cannot mutate s_kf or s_mf. */
    (void)legacy_adaptive_process(
        &s_legacy_adaptive_shadow_kf[s_current_anchor],
        &s_legacy_adaptive_shadow_initialized[s_current_anchor],
        &s_legacy_adaptive_shadow_mf[s_current_anchor],
        &s_legacy_adaptive_state[s_current_anchor],
        dist_mm, fpp_report, HAL_GetTick());
#else /* UWB_LEGACY_ADAPTIVE_ACTIVE */
    {
        LegacyAdaptiveProcessResult_t adaptive_result = legacy_adaptive_process(
            &s_kf[s_current_anchor],
            &s_kf_initialized[s_current_anchor],
            &s_mf[s_current_anchor],
            &s_legacy_adaptive_state[s_current_anchor],
            dist_mm, fpp_report, HAL_GetTick());
        if (adaptive_result.publish_valid == 0U)
        {
            /* Preserve actual raw/FPP evidence while holding the last good
             * range. This covers an unconfirmed gate-breaching candidate or a
             * controlled stale reacquire; it is not a radio timeout. */
            mark_anchor_conditioner_reject(
                s_current_anchor,
                (uint8_t)(st_flags | TAG_ST_RANGE_REJECT),
                dist_mm,
                fpp_report);
            if (s_current_anchor == (TAG_NUM_ANCHORS - 1U))
                LED_TOGGLE();
            return;
        }
        filtered_dist_mm = adaptive_result.filtered_mm;
    }
#endif
#if UWB_C9_2_MOTION_MODE == UWB_C9_2_MOTION_SHADOW
    /* C9.2 has wholly independent median/Kalman/state memory in SHADOW.
     * This call is intentionally after the canonical path and its result is
     * discarded, so a shadow defect cannot affect flight output. */
    (void)c9_2_motion_process(
        s_current_anchor, dist_mm, fpp_report, HAL_GetTick());
#endif
#endif /* UWB_C9_2_MOTION_MODE == ACTIVE */
#else
    RangeFilterInput_t filter_input = {
        .corrected_raw_mm = dist_mm,
        .fpp_dbm = fpp,
        .now_ms = HAL_GetTick(),
        .radio_status = st_flags
    };
    RangeFilterOutput_t filter_output = RangeFilter_Update(
        &range_filter_state[s_current_anchor],
        &filter_input,
        &g_range_filter_config);
    uint32_t abs_innovation = (uint32_t)(
        filter_output.innovation_mm < 0.0f
            ? -filter_output.innovation_mm
            : filter_output.innovation_mm);

    range_filter_last_decision[s_current_anchor] = (uint8_t)filter_output.decision;
    range_filter_last_innovation_mm[s_current_anchor] =
        (int32_t)filter_output.innovation_mm;
    if (abs_innovation > range_filter_max_abs_innovation_mm[s_current_anchor])
        range_filter_max_abs_innovation_mm[s_current_anchor] = abs_innovation;

    filtered_dist_mm = filter_output.filtered_mm;
    if (filter_output.publish_valid == 0U)
    {
        /* Preserve corrected raw/FPP as diagnostic evidence while keeping the
         * canonical measurement invalid and last-success age unchanged. */
        mark_anchor_conditioner_reject(
            s_current_anchor,
            (uint8_t)(st_flags | TAG_ST_RANGE_REJECT),
            dist_mm,
            fpp_report);
        if (s_current_anchor == (TAG_NUM_ANCHORS - 1U))
            LED_TOGGLE();
        return;
    }
    if (filter_output.decision == RANGE_FILTER_REACQUIRED)
        publish_status = (uint8_t)(publish_status | TAG_ST_FILTER_REACQUIRE);
#endif

    distance_raw_mm[s_current_anchor] = dist_mm;
    distance_filtered_mm[s_current_anchor] = filtered_dist_mm;

    /* Legacy debugger mirrors; the arrays above are the source of truth. */
    switch (anchor_id_at(s_current_anchor))
    {
        case 0x0001U:
            distance_a1_mm = dist_mm;
            distance_a1_filtered_mm = filtered_dist_mm;
            break;
        case 0x0002U:
            distance_a2_mm = dist_mm;
            distance_a2_filtered_mm = filtered_dist_mm;
            break;
        case 0x0003U:
            distance_a3_mm = dist_mm;
            distance_a3_filtered_mm = filtered_dist_mm;
            break;
        case 0x0004U:
            distance_a4_mm = dist_mm;
            distance_a4_filtered_mm = filtered_dist_mm;
            break;
        default:
            break;
    }

    if (s_current_anchor == (TAG_NUM_ANCHORS - 1U))
        LED_TOGGLE();

    s_result.ranging_count++;
    s_result.distance_mm = dist_mm;
    response_ok_count++;
    anchor_success_count[s_current_anchor]++;
    mark_anchor_result(s_current_anchor, 1, publish_status,
                       dist_mm, filtered_dist_mm, fpp_report);
}

/* Forward declaration — finish_with_ss_fallback (DS) gọi hàm này trước khi nó
 * được định nghĩa bên dưới. Đặt ngoài #if để tránh quên khi refactor sau này. */
static void advance_to_next_anchor(void);

#if UWB_USE_DS_TWR
/**
 * @brief  DS-TWR distance calculation using 4-term formula.
 *         Kế hoạch §4.4: tof = (Ra*Rb - Da*Db) / (Ra+Rb+Da+Db)
 * @param  rb_ticks             Rb value from REPORT frame
 * @param  uncalibrated_raw_mm  nhận DS range trước software offset để calibration
 * @retval Distance in mm (>=0), or -1 if invalid
 */
static int32_t compute_distance_ds_mm(uint32_t rb_ticks,
                                      int32_t *uncalibrated_raw_mm)
{
    if (uncalibrated_raw_mm == NULL)
        return RANGE_COMPUTE_ERROR;

    *uncalibrated_raw_mm = 0;

    uint64_t t1 = ts_to_u64(s_poll_tx_ts);
    uint64_t t4 = ts_to_u64(s_resp_rx_ts);
    uint64_t t5 = ts_to_u64(s_final_tx_ts);

    double Ra = (double)((t4 - t1) & 0xFFFFFFFFFFULL);
    double Db = (double)((t5 - t4) & 0xFFFFFFFFFFULL);
    double Da = (double)s_da_from_resp;
    double Rb = (double)rb_ticks;

    double denom = Ra + Rb + Da + Db;
    if (denom <= 0.0) return -1;
    double tof = (Ra * Rb - Da * Db) / denom;
    if (tof <= 0.0) return -1;

    double dist_m = tof * UWB_DWT_TIME_UNIT_S * UWB_SPEED_OF_LIGHT;

    if (!meters_to_mm_checked(dist_m, uncalibrated_raw_mm))
        return RANGE_COMPUTE_ERROR;

    /* C0.1: raw đã được capture; production output vẫn bị guard theo từng anchor. */
    return apply_offset_and_clamp(dist_m, UWB_MODE_DS);
}

/**
 * @brief  Build and send a FINAL frame (10-byte, func=FRAME_FINAL_FUNC, immediate TX).
 *         Structurally identical to send_poll but with FRAME_FINAL_FUNC.
 *         Kế hoạch §4.2 / §4.4.
 */
static void build_and_send_final(uint16_t target_anchor)
{
    uint8_t final_frame[POLL_FRAME_LEN];
    final_frame[0] = 0x41;
    final_frame[1] = 0x88;
    final_frame[2] = s_seq_num++;
    final_frame[3] = DW_PAN_ID & 0xFF;
    final_frame[4] = (DW_PAN_ID >> 8) & 0xFF;
    final_frame[5] = target_anchor & 0xFF;
    final_frame[6] = (target_anchor >> 8) & 0xFF;
    final_frame[7] = TAG_ADDR & 0xFF;
    final_frame[8] = (TAG_ADDR >> 8) & 0xFF;
    final_frame[9] = FRAME_FINAL_FUNC;

    DW1000_ClearAllStatus();
    DW1000_WriteTxData(final_frame, POLL_FRAME_LEN);
    DW1000_SetTxFrameCtrl(POLL_FRAME_LEN + 2);  /* +2 HW FCS */
    DW1000_StartTx();  /* immediate TX */
}

/**
 * @brief  SS fallback using shared helper — ensures same formula as SS thuần
 *         including carrier integrator correction (FIX-04 §8.6).
 *         Called when FINAL TX stuck or REPORT lost/timeout.
 */
static void finish_with_ss_fallback(void)
{
    ds_fallback_count++;
    int32_t ss_uncalibrated_raw_mm = 0;

    /* C0: truyền UWB_MODE_SS_FALLBACK — dùng đúng SS calibration profile.
     * ANCHOR4 plan §2.3 và §7.4: SS_FALLBACK phải resolve về profile SS, không dùng DS. */
    int32_t dist_mm = compute_distance_ss_terms_mm(
        s_resp_rx_ts,
        s_da_from_resp,
        s_resp_ci,
        UWB_MODE_SS_FALLBACK,
        &ss_uncalibrated_raw_mm);

    if (dist_mm == RANGE_CALIBRATION_MISSING)
    {
        mark_anchor_rejected_measurement(
            s_current_anchor,
            (uint8_t)(TAG_ST_DS_FALLBACK | TAG_ST_CALIBRATION_MISSING),
            ss_uncalibrated_raw_mm,
            s_resp_fpp);
        advance_to_next_anchor();
        return;
    }
    else if (dist_mm < 0)
    {
        mark_anchor_result(s_current_anchor, 0, TAG_ST_COMPUTE, 0, 0, 0.0f);
        advance_to_next_anchor();
        return;
    }

    publish_distance(dist_mm, &s_resp_diag, s_resp_fpp, TAG_ST_DS_FALLBACK);
    advance_to_next_anchor();
}
#endif /* UWB_USE_DS_TWR */

static void note_anchor_response_timeout(uint8_t idx)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    if (anchor_response_timeout_streak[idx] < 0xFFFFU)
        anchor_response_timeout_streak[idx]++;

    if (anchor_response_timeout_streak[idx] >= TAG_OFFLINE_AFTER_TIMEOUTS)
    {
        s_anchor_next_probe_cycle[idx] =
            tag_cycle_count + TAG_OFFLINE_PROBE_INTERVAL_CYCLES;
    }
}

static void note_anchor_response_received(uint8_t idx)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    anchor_response_timeout_streak[idx] = 0U;
    s_anchor_next_probe_cycle[idx] = 0U;
}

static uint8_t cycle_deadline_reached(uint32_t now_cycle, uint32_t target_cycle)
{
    return ((int32_t)(now_cycle - target_cycle) >= 0) ? 1U : 0U;
}

/**
 * Healthy anchors are always polled. An anchor confirmed offline is probed at
 * a lower rate, with no more than one such probe in a full TAG cycle.
 */
static uint8_t anchor_should_poll(uint8_t idx)
{
    if (anchor_response_timeout_streak[idx] < TAG_OFFLINE_AFTER_TIMEOUTS)
        return 1U;

    if (!cycle_deadline_reached(tag_cycle_count, s_anchor_next_probe_cycle[idx]))
        return 0U;

    if (s_backoff_probe_used)
        return 0U;

    s_backoff_probe_used = 1U;
    /* Reserve the next probe deadline immediately. A malformed response or a
     * TAG-side TX fault must not let this anchor consume the single probe slot
     * again in every following cycle. A valid response clears the deadline. */
    s_anchor_next_probe_cycle[idx] =
        tag_cycle_count + TAG_OFFLINE_PROBE_INTERVAL_CYCLES;
    return 1U;
}

static uint8_t select_poll_anchor(uint8_t first_idx)
{
    for (uint8_t idx = first_idx; idx < TAG_NUM_ANCHORS; idx++)
    {
        if (anchor_should_poll(idx))
        {
            s_current_anchor = idx;
            return 1U;
        }

        /* Keep telemetry truthful: this cycle has no new measurement for the
         * skipped offline anchor. The skip has its own counter and must not be
         * counted as another UWB timeout. */
        anchor_poll_skipped_count[idx]++;
        mark_anchor_result(idx, 0U, TAG_ST_TIMEOUT, 0, 0, 0.0f);
    }

    return 0U;
}

static void finish_full_cycle(void)
{
    uint32_t dur_us = MCU_ElapsedUs(s_full_cycle_start);
    tag_cycle_duration_us = dur_us;
    if (dur_us > tag_cycle_duration_max_us)
        tag_cycle_duration_max_us = dur_us;

    if (dur_us > ((uint32_t)TAG_CYCLE_MS * 1000U))
        cycle_overrun_count++;

    tag_sample_seq++;
    tag_cycle_ready = 1;
    s_cycle_end_cycle = MCU_CycleNow();
    s_state = TAG_STATE_IDLE;
}

/**
 * @brief  Advance to the next due anchor, or finish the full telemetry cycle.
 *         Backed-off anchors are skipped without consuming a radio timeout.
 */
static void advance_to_next_anchor(void)
{
    /* Record the whole slot, including TX, response wait and DS exchange when
     * enabled. This pinpoints the slot that consumes the cycle budget. */
    uint32_t slot_us = MCU_ElapsedUs(s_slot_start);
    anchor_slot_duration_us[s_current_anchor] = slot_us;
    if (slot_us > anchor_slot_duration_max_us[s_current_anchor])
        anchor_slot_duration_max_us[s_current_anchor] = slot_us;

    if (select_poll_anchor((uint8_t)(s_current_anchor + 1U)))
    {
        DW1000_ForceRxOff();
        DW1000_ClearAllStatus();
        dw1000_irq_flag = 0;
        s_state       = TAG_STATE_INTER_ANCHOR_GUARD;
        s_state_cycle = MCU_CycleNow();
    }
    else
    {
        finish_full_cycle();
    }
}

/* ========================================================================== */
/*                     PUBLIC API                                              */
/* ========================================================================== */

int Tag_Init(void)
{
    if (DW1000_Init() != 0)
        return -1;

    DW1000_Configure();                 /* Also writes SYS_MASK */
    DW1000_SetAddress(DW_PAN_ID, TAG_ADDR);
    if (DW1000_EnableFastSPI() == 0U)
        return -2;
    DW1000_ClearAllStatus();           /* Ensure IRQ pin LOW before EnableIRQ */
    memset(&s_result, 0, sizeof(s_result));
    memset(s_track, 0, sizeof(s_track));
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        legacy_adaptive_state[i] = TAG_LEGACY_ADAPTIVE_STABLE;
        legacy_adaptive_candidate_count[i] = 0U;
        legacy_adaptive_candidate_direction[i] = 0;
        legacy_adaptive_track_enter_count[i] = 0U;
        legacy_adaptive_track_exit_count[i] = 0U;
        legacy_adaptive_true_reject_count[i] = 0U;
        legacy_adaptive_stale_reacquire_count[i] = 0U;
        legacy_adaptive_track_duration_ms[i] = 0U;
        legacy_adaptive_last_innovation_mm[i] = 0;
        legacy_adaptive_max_abs_innovation_mm[i] = 0U;
        legacy_adaptive_last_q[i] = 0.0;
        legacy_adaptive_last_r[i] = 0.0;
        legacy_adaptive_last_gain[i] = 0.0;
        c9_2_motion_state[i] = (uint8_t)MOTION_RANGE_REACQUIRE;
        c9_2_static_enter_count[i] = 0U;
        c9_2_slow_enter_count[i] = 0U;
        c9_2_fast_enter_count[i] = 0U;
        c9_2_settling_enter_count[i] = 0U;
        c9_2_degraded_enter_count[i] = 0U;
        c9_2_stale_reacquire_count[i] = 0U;
        c9_2_true_reject_count[i] = 0U;
        c9_2_last_slope_mm_s[i] = 0;
        c9_2_last_motion_score_milli[i] = 0U;
        c9_2_last_gain[i] = 0.0;
    }
    c9_2_global_motion_state = (uint8_t)MOTION_RANGE_REACQUIRE;
#if UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_LEGACY_KALMAN
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
        RangeFilter_Init(&range_filter_state[i]);
#endif
#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_LEGACY_KALMAN && \
    UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_OFF
    memset(s_legacy_adaptive_state, 0, sizeof(s_legacy_adaptive_state));
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
        LegacyAdaptiveTracking_Init(&s_legacy_adaptive_state[i]);
#if UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_SHADOW
    memset(s_legacy_adaptive_shadow_kf, 0, sizeof(s_legacy_adaptive_shadow_kf));
    memset(s_legacy_adaptive_shadow_initialized, 0,
           sizeof(s_legacy_adaptive_shadow_initialized));
    memset(s_legacy_adaptive_shadow_mf, 0, sizeof(s_legacy_adaptive_shadow_mf));
#endif
#endif
#if UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_OFF
    memset(s_c9_2_kf, 0, sizeof(s_c9_2_kf));
    memset(s_c9_2_kf_initialized, 0, sizeof(s_c9_2_kf_initialized));
    memset(s_c9_2_mf, 0, sizeof(s_c9_2_mf));
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
        MotionAdaptiveRange_Init(&s_c9_2_motion[i]);
#endif

    s_cycle_tick      = HAL_GetTick();
    s_cycle_end_cycle = MCU_CycleNow();
    return 0;
}

void Tag_Task(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_state)
    {
        /* ================================================================== */
        case TAG_STATE_IDLE:
        /* ================================================================== */
        {
            /* Keep the nominal 20ms schedule while healthy. After an overrun,
             * also guarantee a real quiet interval before A1 is polled again. */
            if ((now - s_cycle_tick) < TAG_CYCLE_MS)
                break;
            if (MCU_ElapsedUs(s_cycle_end_cycle) < TAG_INTER_CYCLE_RECOVERY_US)
                break;

            /* Start a new cycle */
            tag_cycle_count++;
            s_cycle_tick     = now;
            s_full_cycle_start = MCU_CycleNow();   /* §5.2 instrumentation: mốc bắt đầu chu kỳ */
            s_backoff_probe_used = 0U;

            /* A cycle can contain only healthy anchors or one periodic offline
             * probe. If no anchor is due, still publish a truthful age/status
             * snapshot without touching the radio. */
            if (!select_poll_anchor(0U))
            {
                finish_full_cycle();
                break;
            }

            s_slot_start = s_full_cycle_start;
            send_poll(anchor_id_at(s_current_anchor));
            s_state      = TAG_STATE_TX_POLL;
            s_state_cycle = MCU_CycleNow();   /* FIX-02 */
            break;
        }

        /* ================================================================== */
        case TAG_STATE_TX_POLL:
        /* ================================================================== */
        {
            /* Guard: TX must complete within TAG_TX_TIMEOUT_US (FIX-02: wrap-safe) */
            uint32_t tx_elapsed_us = MCU_ElapsedUs(s_state_cycle);
            if (tx_elapsed_us > TAG_TX_TIMEOUT_US)
            {
                /* TX stuck — skip this anchor */
                if (tx_elapsed_us > anchor_poll_tx_duration_max_us[s_current_anchor])
                    anchor_poll_tx_duration_max_us[s_current_anchor] = tx_elapsed_us;
                s_result.timeout_count++;
                anchor_timeout_count[s_current_anchor]++;
                anchor_poll_tx_timeout_count[s_current_anchor]++;
                DW1000_ForceRxOff();
                DW1000_ClearAllStatus();
                advance_to_next_anchor();
                break;
            }

            /* Wait for TXFRS IRQ */
            if (!dw1000_irq_flag)
                break;

            dw1000_irq_flag = 0;
            uint32_t status = DW1000_ReadStatus();

            if (status & DW_TXFRS_BIT)
            {
                /* POLL sent — capture TX timestamp, enable RX for RESP */
                tx_elapsed_us = MCU_ElapsedUs(s_state_cycle);
                if (tx_elapsed_us > anchor_poll_tx_duration_max_us[s_current_anchor])
                    anchor_poll_tx_duration_max_us[s_current_anchor] = tx_elapsed_us;
                DW1000_ReadTxTimestamp(s_poll_tx_ts);
                DW1000_ClearAllStatus();
                DW1000_StartRx();
                s_state      = TAG_STATE_WAIT_RESP;
                s_state_cycle = MCU_CycleNow();   /* FIX-02 */
            }
            /* If non-TX IRQ somehow fires here, just keep waiting */
            break;
        }

        /* ================================================================== */
        case TAG_STATE_WAIT_RESP:
        /* ================================================================== */
        {
            uint8_t advance = 0;
            uint8_t processed_irq = 0U;
            McuCycleStamp_t processing_start = 0;

            /* --- FIX-03: xử lý IRQ đã tới TRƯỚC, timeout chỉ xét sau -----
             * Lý do: nếu kiểm tra timeout trước, 1 frame RESP hợp lệ tới
             * đúng lúc gần deadline sẽ bị huỷ bỏ nhầm thay vì được đọc. */
            if (dw1000_irq_flag)
            {
                uint32_t wait_us = MCU_ElapsedUs(s_state_cycle);
                if (wait_us > anchor_response_wait_max_us[s_current_anchor])
                    anchor_response_wait_max_us[s_current_anchor] = wait_us;
                processing_start = MCU_CycleNow();
                processed_irq = 1U;

                dw1000_irq_flag = 0;
                uint32_t status = DW1000_ReadStatus();

                if ((status & DW_ALL_RX_GOOD) != DW_ALL_RX_GOOD)
                {
                    /* RX error (FCS fail, etc.) — skip anchor */
                    DW1000_ForceRxOff();
                    DW1000_ClearAllStatus();
                    s_result.timeout_count++;
                    rx_error_count++;
                    anchor_rx_error_count[s_current_anchor]++;
                    mark_anchor_result(s_current_anchor, 0, TAG_ST_RXERR, 0, 0, 0.0f);
                    advance = 1;
                }
                else
                {
                    /* Good frame received */
                    uint8_t  rx_buf[MAX_RX_FRAME_LEN];
                    uint16_t rx_len = DW1000_ReadRxData(rx_buf, MAX_RX_FRAME_LEN);

                    /* Validate theo thứ tự: length → header → payload (§6.3 FIX-03) */
                    if (rx_len < 10U)
                    {
                        /* Tuyệt đối không đọc rx_buf[5..9] trong nhánh này. */
                        DW1000_ClearAllStatus();
                        s_result.timeout_count++;
                        rx_error_count++;
                        anchor_rx_error_count[s_current_anchor]++;
                        mark_anchor_result(s_current_anchor, 0, TAG_ST_BADFRAME,
                                           0, 0, 0.0f);
                        advance = 1;
                    }
                    else
                    {
                        uint16_t dst  = (uint16_t)rx_buf[5] | ((uint16_t)rx_buf[6] << 8);
                        uint16_t src  = (uint16_t)rx_buf[7] | ((uint16_t)rx_buf[8] << 8);
                        uint8_t  func = rx_buf[9];

                        if (func != FRAME_RESP_FUNC
                            || src != anchor_id_at(s_current_anchor))
                        {
                            /* Stray: giữ deadline gốc (FIX-03). */
                            DW1000_ClearAllStatus();
                            DW1000_StartRx();
                        }
                        else if (dst != TAG_ADDR || rx_len < 14U)
                        {
                            /* Đúng loại/source nhưng frame không dùng được. */
                            DW1000_ClearAllStatus();
                            s_result.timeout_count++;
                            rx_error_count++;
                            anchor_rx_error_count[s_current_anchor]++;
                            mark_anchor_result(s_current_anchor, 0, TAG_ST_BADFRAME,
                                               0, 0, 0.0f);
                            advance = 1;
                        }
                        else
                        {
                            /* Valid RESP — read timestamp + Da */
                            note_anchor_response_received(s_current_anchor);
                            uint8_t resp_rx_ts[5];
                            DW1000_ReadRxTimestamp(resp_rx_ts);

#if UWB_USE_DS_TWR
                            /* DS path: save T4 + Da + diag BEFORE sending FINAL
                             * (diag registers will change after FINAL/REPORT TX). */
                            memcpy(s_resp_rx_ts, resp_rx_ts, 5);
                            s_da_from_resp = (uint32_t)rx_buf[10]
                                           | ((uint32_t)rx_buf[11] <<  8)
                                           | ((uint32_t)rx_buf[12] << 16)
                                           | ((uint32_t)rx_buf[13] << 24);
                            DW1000_ReadSignalDiag(&s_resp_diag);
                            s_resp_fpp = DW1000_GetFirstPathPower(&s_resp_diag);
                            s_resp_ci  = DW1000_ReadCarrierIntegrator();

                            /* Send FINAL immediately (immediate TX, no delayed TX) */
                            build_and_send_final(anchor_id_at(s_current_anchor));
                            s_state       = TAG_STATE_TX_FINAL;
                            s_state_cycle = MCU_CycleNow();
                            /* advance stays 0 — do NOT call advance_to_next_anchor here */
#else
                            /* SS path (default): compute distance now and publish */
                            DW1000_ClearAllStatus();

                            int32_t ss_uncalibrated_raw_mm = 0;
                            int32_t dist_mm = compute_distance_mm(
                                resp_rx_ts, rx_buf, &ss_uncalibrated_raw_mm);
                            if (dist_mm >= 0)
                            {
                                DW1000_SignalDiag_t diag;
                                DW1000_ReadSignalDiag(&diag);
                                float fpp = DW1000_GetFirstPathPower(&diag);
                                publish_distance(dist_mm, &diag, fpp, TAG_ST_OK);
                            }
                            else if (dist_mm == RANGE_CALIBRATION_MISSING)
                            {
                                DW1000_SignalDiag_t diag;
                                DW1000_ReadSignalDiag(&diag);
                                float fpp = DW1000_GetFirstPathPower(&diag);
                                mark_anchor_rejected_measurement(
                                    s_current_anchor,
                                    TAG_ST_CALIBRATION_MISSING,
                                    ss_uncalibrated_raw_mm,
                                    fpp);
                            }
                            else
                            {
                                mark_anchor_result(s_current_anchor, 0, TAG_ST_COMPUTE,
                                                   0, 0, 0.0f);
                            }
                            advance = 1;
#endif /* UWB_USE_DS_TWR */
                        }
                    }
                }
            }
            /* --- Timeout check — CHỈ khi thực sự không có IRQ đang chờ ---
             * FIX-02: MCU_ElapsedUs(s_state_cycle) an toàn qua wrap CYCCNT. */
            else
            {
                uint32_t wait_us = MCU_ElapsedUs(s_state_cycle);
                if (wait_us > TAG_RESP_TIMEOUT_US)
                {
                    if (wait_us > anchor_response_wait_max_us[s_current_anchor])
                        anchor_response_wait_max_us[s_current_anchor] = wait_us;

                    /* Anchor didn't reply in time — skip */
                    DW1000_ForceRxOff();
                    DW1000_ClearAllStatus();
                    s_result.timeout_count++;
                    rx_timeout_count++;
                    anchor_timeout_count[s_current_anchor]++;
                    anchor_response_timeout_count[s_current_anchor]++;
                    note_anchor_response_timeout(s_current_anchor);
                    mark_anchor_result(s_current_anchor, 0, TAG_ST_TIMEOUT, 0, 0, 0.0f);
                    advance = 1;
                }
            }

            if (processed_irq)
            {
                uint32_t processing_us = MCU_ElapsedUs(processing_start);
                if (processing_us > anchor_processing_max_us[s_current_anchor])
                    anchor_processing_max_us[s_current_anchor] = processing_us;
            }

            /* --- Move to next anchor if needed ------------------------- */
            if (advance)
            {
                advance_to_next_anchor();
            }
            break;
        }

        /* ================================================================== */
        case TAG_STATE_INTER_ANCHOR_GUARD:
        /* ================================================================== */
        {
            if (MCU_ElapsedUs(s_state_cycle) < TAG_INTER_ANCHOR_GUARD_US)
                break;

            /* send_poll() clears any residual DW1000 status before TX. */
            dw1000_irq_flag = 0;
            s_slot_start = MCU_CycleNow();
            send_poll(anchor_id_at(s_current_anchor));
            s_state       = TAG_STATE_TX_POLL;
            s_state_cycle = MCU_CycleNow();
            break;
        }

        /* ================================================================== */
        default:
        /* ================================================================== */
            s_state = TAG_STATE_IDLE;
            break;

#if UWB_USE_DS_TWR
        /* ================================================================== */
        case TAG_STATE_TX_FINAL:
        /* ================================================================== */
        {
            /* Guard: FINAL TX must complete within TAG_FINAL_TX_TIMEOUT_US */
            if (MCU_ElapsedUs(s_state_cycle) > TAG_FINAL_TX_TIMEOUT_US)
            {
                /* TX stuck — fallback to SS using data already captured */
                DW1000_ForceRxOff();
                DW1000_ClearAllStatus();
                ds_final_tx_timeout_count++;
                anchor_timeout_count[s_current_anchor]++;
                finish_with_ss_fallback();
                break;
            }

            /* Wait for TXFRS IRQ */
            if (!dw1000_irq_flag)
                break;

            dw1000_irq_flag = 0;
            uint32_t status_final = DW1000_ReadStatus();

            if (status_final & DW_TXFRS_BIT)
            {
                /* FINAL sent — capture T5 (FINAL TX timestamp), arm RX for REPORT */
                DW1000_ReadTxTimestamp(s_final_tx_ts);   /* T5 */
                DW1000_ClearAllStatus();
                DW1000_StartRx();
                s_state       = TAG_STATE_WAIT_REPORT;
                s_state_cycle = MCU_CycleNow();
            }
            /* If non-TX IRQ fires here, keep waiting */
            break;
        }

        /* ================================================================== */
        case TAG_STATE_WAIT_REPORT:
        /* ================================================================== */
        {
            /* FIX-03 pattern: process IRQ first, timeout only via else-if */
            if (dw1000_irq_flag)
            {
                dw1000_irq_flag = 0;
                uint32_t status_rpt = DW1000_ReadStatus();

                if ((status_rpt & DW_ALL_RX_GOOD) != DW_ALL_RX_GOOD)
                {
                    /* RX error — fallback SS, data from RESP still valid */
                    DW1000_ForceRxOff();
                    DW1000_ClearAllStatus();
                    ds_report_rx_error_count++;
                    anchor_rx_error_count[s_current_anchor]++;
                    finish_with_ss_fallback();
                }
                else
                {
                    uint8_t  rx_buf_r[MAX_RX_FRAME_LEN];
                    uint16_t rx_len_r = DW1000_ReadRxData(rx_buf_r, MAX_RX_FRAME_LEN);

                    if (rx_len_r >= 14)
                    {
                        uint16_t dst_r = rx_buf_r[5] | ((uint16_t)rx_buf_r[6] << 8);
                        uint16_t src_r = rx_buf_r[7] | ((uint16_t)rx_buf_r[8] << 8);

                        if (rx_buf_r[9] == FRAME_REPORT_FUNC
                            && dst_r == TAG_ADDR
                            && src_r == anchor_id_at(s_current_anchor))
                        {
                            /* Valid REPORT — extract Rb, compute DS distance */
                            ds_report_ok_count++;
                            uint32_t rb = (uint32_t)rx_buf_r[10]
                                        | ((uint32_t)rx_buf_r[11] <<  8)
                                        | ((uint32_t)rx_buf_r[12] << 16)
                                        | ((uint32_t)rx_buf_r[13] << 24);

                            DW1000_ClearAllStatus();

                            int32_t ds_uncalibrated_raw_mm = 0;
                            int32_t dist_mm = compute_distance_ds_mm(
                                rb, &ds_uncalibrated_raw_mm);
                            if (dist_mm >= 0)
                            {
                                ds_ok_count++;
                                publish_distance(dist_mm, &s_resp_diag, s_resp_fpp, TAG_ST_OK);
                            }
                            else if (dist_mm == RANGE_CALIBRATION_MISSING)
                            {
                                /* Giữ raw/FPP để calibration nhưng production valid=0.
                                 * Không cập nhật last_success_tick hoặc filtered output. */
                                ds_cal_missing_count++;
                                mark_anchor_rejected_measurement(
                                    s_current_anchor,
                                    TAG_ST_CAL_MISSING_DS,
                                    ds_uncalibrated_raw_mm,
                                    s_resp_fpp);
                            }
                            else /* RANGE_COMPUTE_ERROR */
                            {
                                mark_anchor_result(s_current_anchor, 0, TAG_ST_COMPUTE, 0, 0, 0.0f);
                            }
                            advance_to_next_anchor();
                        }
                        else
                        {
                            /* Stray frame — keep listening, DO NOT reset deadline (FIX-03) */
                            DW1000_ClearAllStatus();
                            DW1000_StartRx();
                        }
                    }
                    else
                    {
                        /* Frame quá ngắn — stray, keep listening, GIỮ deadline (FIX-03) */
                        DW1000_ClearAllStatus();
                        DW1000_StartRx();
                    }
                }
            }
            /* --- Timeout: REPORT didn't arrive in time --- */
            else if (MCU_ElapsedUs(s_state_cycle) > TAG_REPORT_TIMEOUT_US)
            {
                /* Fallback SS — RESP data still valid, no sample lost */
                DW1000_ForceRxOff();
                DW1000_ClearAllStatus();
                ds_report_timeout_count++;
                anchor_timeout_count[s_current_anchor]++;
                finish_with_ss_fallback();
            }
            break;
        }
#endif /* UWB_USE_DS_TWR */
    }
}

void Tag_GetSnapshot(TagCycleSnapshot_t *out)
{
    if (out == NULL)
        return;

    uint32_t now = HAL_GetTick();
    out->seq     = tag_sample_seq;
    out->time_ms = now;

    for (uint8_t i = 0; i < TAG_NUM_ANCHORS; i++)
    {
        const AnchorTrack_t *t = &s_track[i];
        TagAnchorSample_t   *a = &out->anchor[i];

        a->anchor_id   = anchor_id_at(i);
        a->valid       = t->valid;
        a->status      = t->status;

        uint32_t age   = now - t->last_success_tick;
        a->age_ms      = (age > 0xFFFF) ? 0xFFFF : (uint16_t)age;

        a->raw_mm      = t->raw_mm;
        a->filtered_mm = t->filtered_mm;
        a->fpp_cdbm    = t->fpp_cdbm;
    }
}

const DW1000_RangingResult* Tag_GetResult(void)
{
    return &s_result;
}
