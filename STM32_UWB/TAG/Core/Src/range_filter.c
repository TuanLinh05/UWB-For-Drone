#include "range_filter.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define RANGE_FILTER_RADIO_FAILURE_MASK 0x2FU

const RangeFilterConfig_t g_range_filter_config = {
    .min_range_mm = 1,
    .max_range_mm = 50000,
    .max_radial_speed_mm_s = 10000U,      /* TUNE_REQUIRED */
    .gate_margin_up_mm = 100U,            /* TUNE_REQUIRED */
    .gate_margin_down_mm = 250U,          /* TUNE_REQUIRED */
    .stale_reset_ms = 500U,               /* TUNE_REQUIRED */
    .process_accel_sigma_mm_s2 = 12000.0f,/* TUNE_REQUIRED */
    .initial_velocity_sigma_mm_s = 10000.0f,
    .nis_gate_1d = 6.635f,
    .min_dt_s = 0.005f,
    .max_tracking_dt_s = 0.5f,
    .reacquire_min_samples = 3U,           /* 60 ms at 50 Hz */
    .reacquire_cluster_mm = 250,
    /* Current calibrated DS links can report below -95 dBm while still being
     * usable. Do not turn a genuine position step into a 500 ms stale reset. */
    .reacquire_min_fpp_dbm = -105.0f,
    .fpp_strong_dbm = -75.0f,
    .fpp_medium_dbm = -82.0f,
    .variance_strong_mm2 = 2500.0f,       /* 50 mm sigma; TUNE_REQUIRED */
    .variance_medium_mm2 = 10000.0f,      /* 100 mm sigma; TUNE_REQUIRED */
    .variance_weak_mm2 = 40000.0f,        /* 200 mm sigma; TUNE_REQUIRED */
    .variance_floor_mm2 = 400.0f,
    .variance_ceiling_mm2 = 250000.0f,
};

static void increment_u32(uint32_t *value)
{
    if (*value < UINT32_MAX)
        (*value)++;
}

static int32_t rounded_mm(float value)
{
    if (value >= (float)INT32_MAX)
        return INT32_MAX;
    if (value <= (float)INT32_MIN)
        return INT32_MIN;
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static float clamp_f32(float value, float low, float high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

static int32_t median3(int32_t a, int32_t b, int32_t c)
{
    int32_t swap;
    if (a > b) { swap = a; a = b; b = swap; }
    if (b > c) { swap = b; b = c; c = swap; }
    if (a > b) { swap = a; a = b; b = swap; }
    return b;
}

static int32_t median_candidate(
    const RangeFilterState_t *state,
    int32_t input_mm,
    int32_t window_out[3],
    uint8_t *count_out,
    uint8_t *head_out)
{
    memcpy(window_out, state->median_window_mm, sizeof(state->median_window_mm));
    *count_out = state->median_count;
    *head_out = state->median_head;
    window_out[*head_out] = input_mm;
    *head_out = (uint8_t)((*head_out + 1U) % 3U);
    if (*count_out < 3U)
        (*count_out)++;
    if (*count_out < 3U)
        return input_mm;
    return median3(window_out[0], window_out[1], window_out[2]);
}

static void commit_median(
    RangeFilterState_t *state,
    const int32_t window[3],
    uint8_t count,
    uint8_t head)
{
    memcpy(state->median_window_mm, window, sizeof(state->median_window_mm));
    state->median_count = count;
    state->median_head = head;
}

static void reset_tracking_keep_counters(RangeFilterState_t *state)
{
    state->distance_mm = 0.0f;
    state->radial_velocity_mm_s = 0.0f;
    state->p00_mm2 = 0.0f;
    state->p01_mm2_s = 0.0f;
    state->p11_mm2_s2 = 0.0f;
    memset(state->median_window_mm, 0, sizeof(state->median_window_mm));
    state->median_count = 0U;
    state->median_head = 0U;
    state->last_predict_ms = 0U;
    state->last_accepted_ms = 0U;
    state->reject_streak = 0U;
    state->candidate_count = 0U;
    state->candidate_mean_mm = 0.0f;
    state->initialized = 0U;
}

void RangeFilter_Init(RangeFilterState_t *state)
{
    if (state != NULL)
        memset(state, 0, sizeof(*state));
}

#if UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_LEGACY_KALMAN
static uint32_t motion_limit_mm(
    uint32_t vmax_mm_s,
    uint32_t dt_ms,
    uint32_t margin_mm)
{
    uint64_t motion_mm =
        ((uint64_t)vmax_mm_s * (uint64_t)dt_ms + 999ULL) / 1000ULL;
    uint64_t total = motion_mm + (uint64_t)margin_mm;
    return total > UINT32_MAX ? UINT32_MAX : (uint32_t)total;
}
#endif

static float measurement_variance(
    float fpp_dbm,
    const RangeFilterConfig_t *config)
{
    float variance;
    if (fpp_dbm > config->fpp_strong_dbm)
        variance = config->variance_strong_mm2;
    else if (fpp_dbm > config->fpp_medium_dbm)
        variance = config->variance_medium_mm2;
    else
        variance = config->variance_weak_mm2;
    return clamp_f32(
        variance,
        config->variance_floor_mm2,
        config->variance_ceiling_mm2);
}

static void initialize_tracking(
    RangeFilterState_t *state,
    int32_t distance_mm,
    uint32_t now_ms,
    float measurement_variance_mm2,
    const RangeFilterConfig_t *config)
{
    int32_t sample_window[3] = { distance_mm, distance_mm, distance_mm };
    memcpy(state->median_window_mm, sample_window, sizeof(sample_window));
    state->median_count = 1U;
    state->median_head = 1U;
    state->distance_mm = (float)distance_mm;
    state->radial_velocity_mm_s = 0.0f;
    state->p00_mm2 = measurement_variance_mm2;
    state->p01_mm2_s = 0.0f;
    state->p11_mm2_s2 =
        config->initial_velocity_sigma_mm_s * config->initial_velocity_sigma_mm_s;
    state->last_predict_ms = now_ms;
    state->last_accepted_ms = now_ms;
    state->reject_streak = 0U;
    state->candidate_count = 0U;
    state->candidate_mean_mm = 0.0f;
    state->initialized = 1U;
}

#if UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_LEGACY_KALMAN
static uint8_t update_reacquire_candidate(
    RangeFilterState_t *state,
    int32_t candidate_mm,
    float fpp_dbm,
    const RangeFilterConfig_t *config)
{
    if (fpp_dbm < config->reacquire_min_fpp_dbm)
    {
        state->candidate_count = 0U;
        return 0U;
    }
    if (state->candidate_count == 0U
        || fabsf((float)candidate_mm - state->candidate_mean_mm)
            > (float)config->reacquire_cluster_mm)
    {
        state->candidate_mean_mm = (float)candidate_mm;
        state->candidate_count = 1U;
    }
    else
    {
        uint16_t next_count = state->candidate_count < UINT16_MAX
            ? (uint16_t)(state->candidate_count + 1U)
            : UINT16_MAX;
        state->candidate_mean_mm +=
            ((float)candidate_mm - state->candidate_mean_mm) / (float)next_count;
        state->candidate_count = next_count;
    }
    return state->candidate_count >= config->reacquire_min_samples ? 1U : 0U;
}
#endif

static RangeFilterOutput_t default_output(const RangeFilterState_t *state)
{
    RangeFilterOutput_t output;
    memset(&output, 0, sizeof(output));
    output.filtered_mm = state != NULL && state->initialized
        ? rounded_mm(state->distance_mm)
        : 0;
    output.decision = RANGE_FILTER_REJECTED_PHYSICAL;
    return output;
}

static uint8_t invalid_input(
    const RangeFilterInput_t *input,
    const RangeFilterConfig_t *config)
{
    return input == NULL || config == NULL
        || !isfinite(input->fpp_dbm)
        || input->corrected_raw_mm < config->min_range_mm
        || input->corrected_raw_mm > config->max_range_mm
        || (input->radio_status & RANGE_FILTER_RADIO_FAILURE_MASK) != 0U;
}

#if UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_LEGACY_KALMAN
static void note_reject(RangeFilterState_t *state)
{
    if (state->reject_streak < UINT16_MAX)
        state->reject_streak++;
}
#endif

#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_CV_KALMAN_V2
static uint8_t cv_predict(
    RangeFilterState_t *state,
    uint32_t now_ms,
    const RangeFilterConfig_t *config)
{
    uint32_t dt_ms = now_ms - state->last_predict_ms;
    float dt = (float)dt_ms / 1000.0f;
    if (dt > config->max_tracking_dt_s)
        return 0U;
    if (dt < config->min_dt_s)
        dt = config->min_dt_s;

    state->distance_mm += state->radial_velocity_mm_s * dt;
    {
        float p00 = state->p00_mm2
            + 2.0f * dt * state->p01_mm2_s
            + dt * dt * state->p11_mm2_s2;
        float p01 = state->p01_mm2_s + dt * state->p11_mm2_s2;
        float p11 = state->p11_mm2_s2;
        float sigma2 = config->process_accel_sigma_mm_s2
            * config->process_accel_sigma_mm_s2;
        float dt2 = dt * dt;
        float dt3 = dt2 * dt;
        float dt4 = dt2 * dt2;
        state->p00_mm2 = p00 + sigma2 * dt4 * 0.25f;
        state->p01_mm2_s = p01 + sigma2 * dt3 * 0.5f;
        state->p11_mm2_s2 = p11 + sigma2 * dt2;
    }
    state->last_predict_ms = now_ms;
    return isfinite(state->distance_mm)
        && isfinite(state->p00_mm2)
        && isfinite(state->p01_mm2_s)
        && isfinite(state->p11_mm2_s2)
        && state->p00_mm2 > 0.0f ? 1U : 0U;
}

static void cv_correct(
    RangeFilterState_t *state,
    float innovation,
    float variance,
    float innovation_variance)
{
    float k0 = state->p00_mm2 / innovation_variance;
    float k1 = state->p01_mm2_s / innovation_variance;
    float prior00 = state->p00_mm2;
    float prior01 = state->p01_mm2_s;
    float prior11 = state->p11_mm2_s2;
    float a00 = 1.0f - k0;
    float a10 = -k1;

    state->distance_mm += k0 * innovation;
    state->radial_velocity_mm_s += k1 * innovation;

    /* Joseph form for H=[1,0]. */
    state->p00_mm2 = a00 * a00 * prior00 + k0 * k0 * variance;
    state->p01_mm2_s = a00 * (a10 * prior00 + prior01)
        + k0 * k1 * variance;
    state->p11_mm2_s2 = a10 * a10 * prior00
        + 2.0f * a10 * prior01 + prior11
        + k1 * k1 * variance;
    if (state->p00_mm2 < 1.0f)
        state->p00_mm2 = 1.0f;
    if (state->p11_mm2_s2 < 1.0f)
        state->p11_mm2_s2 = 1.0f;
}
#endif

RangeFilterOutput_t RangeFilter_Update(
    RangeFilterState_t *state,
    const RangeFilterInput_t *input,
    const RangeFilterConfig_t *config)
{
    RangeFilterOutput_t output = default_output(state);
    int32_t candidate_window[3];
    uint8_t candidate_count;
    uint8_t candidate_head;
    float variance;
#if UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_LEGACY_KALMAN
    uint32_t dt_accepted_ms;
    uint32_t tracking_cap_ms;
    uint32_t up_limit;
    uint32_t down_limit;
    int64_t jump;
#endif

    if (state == NULL)
        return output;
    if (invalid_input(input, config))
    {
        increment_u32(&state->physical_reject_count);
        return output;
    }

    variance = measurement_variance(input->fpp_dbm, config);
    output.measurement_variance_mm2 = variance;
    if (!state->initialized)
    {
        initialize_tracking(state, input->corrected_raw_mm, input->now_ms, variance, config);
        increment_u32(&state->accepted_count);
        output.median_mm = input->corrected_raw_mm;
        output.filtered_mm = input->corrected_raw_mm;
        output.decision = RANGE_FILTER_ACCEPTED;
        output.publish_valid = 1U;
        return output;
    }

    if ((uint32_t)(input->now_ms - state->last_accepted_ms) > config->stale_reset_ms)
    {
        increment_u32(&state->stale_reset_count);
        reset_tracking_keep_counters(state);
        initialize_tracking(state, input->corrected_raw_mm, input->now_ms, variance, config);
        increment_u32(&state->reacquire_count);
        output.median_mm = input->corrected_raw_mm;
        output.filtered_mm = input->corrected_raw_mm;
        output.decision = RANGE_FILTER_REACQUIRED;
        output.publish_valid = 1U;
        return output;
    }

    output.median_mm = median_candidate(
        state,
        input->corrected_raw_mm,
        candidate_window,
        &candidate_count,
        &candidate_head);

#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_LEGACY_KALMAN
    commit_median(state, candidate_window, candidate_count, candidate_head);
    state->distance_mm = (float)output.median_mm;
    state->last_accepted_ms = input->now_ms;
    state->last_predict_ms = input->now_ms;
    increment_u32(&state->accepted_count);
    output.filtered_mm = output.median_mm;
    output.decision = RANGE_FILTER_ACCEPTED;
    output.publish_valid = 1U;
    return output;
#else
    dt_accepted_ms = input->now_ms - state->last_accepted_ms;
    tracking_cap_ms = (uint32_t)(config->max_tracking_dt_s * 1000.0f);
    if (tracking_cap_ms == 0U)
        tracking_cap_ms = 1U;
    if (dt_accepted_ms > tracking_cap_ms)
        dt_accepted_ms = tracking_cap_ms;
    up_limit = motion_limit_mm(
        config->max_radial_speed_mm_s,
        dt_accepted_ms,
        config->gate_margin_up_mm);
    down_limit = motion_limit_mm(
        config->max_radial_speed_mm_s,
        dt_accepted_ms,
        config->gate_margin_down_mm);

#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_CV_KALMAN_V2
    if (!cv_predict(state, input->now_ms, config))
    {
        increment_u32(&state->stale_reset_count);
        reset_tracking_keep_counters(state);
        initialize_tracking(state, input->corrected_raw_mm, input->now_ms, variance, config);
        increment_u32(&state->reacquire_count);
        output.filtered_mm = input->corrected_raw_mm;
        output.decision = RANGE_FILTER_REACQUIRED;
        output.publish_valid = 1U;
        return output;
    }
#endif

    jump = (int64_t)output.median_mm - (int64_t)rounded_mm(state->distance_mm);
    if (jump > (int64_t)up_limit || jump < -(int64_t)down_limit)
    {
        note_reject(state);
        increment_u32(&state->dynamic_reject_count);
        output.innovation_mm = (float)jump;
        output.filtered_mm = rounded_mm(state->distance_mm);
        output.decision = RANGE_FILTER_REJECTED_DYNAMIC;
        if (update_reacquire_candidate(
                state, input->corrected_raw_mm, input->fpp_dbm, config))
        {
            int32_t reacquired_mm = rounded_mm(state->candidate_mean_mm);
            initialize_tracking(state, reacquired_mm, input->now_ms, variance, config);
            increment_u32(&state->reacquire_count);
            output.median_mm = reacquired_mm;
            output.filtered_mm = reacquired_mm;
            output.decision = RANGE_FILTER_REACQUIRED;
            output.publish_valid = 1U;
        }
        return output;
    }

#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_CV_KALMAN_V2
    output.innovation_mm = (float)output.median_mm - state->distance_mm;
    output.innovation_variance_mm2 = state->p00_mm2 + variance;
    if (!isfinite(output.innovation_variance_mm2)
        || output.innovation_variance_mm2 < config->variance_floor_mm2)
    {
        note_reject(state);
        increment_u32(&state->nis_reject_count);
        output.filtered_mm = rounded_mm(state->distance_mm);
        output.decision = RANGE_FILTER_REJECTED_NIS;
        return output;
    }
    output.nis_1d = output.innovation_mm * output.innovation_mm
        / output.innovation_variance_mm2;
    if (!isfinite(output.nis_1d) || output.nis_1d > config->nis_gate_1d)
    {
        note_reject(state);
        increment_u32(&state->nis_reject_count);
        output.filtered_mm = rounded_mm(state->distance_mm);
        output.decision = RANGE_FILTER_REJECTED_NIS;
        if (update_reacquire_candidate(
                state, input->corrected_raw_mm, input->fpp_dbm, config))
        {
            int32_t reacquired_mm = rounded_mm(state->candidate_mean_mm);
            initialize_tracking(state, reacquired_mm, input->now_ms, variance, config);
            increment_u32(&state->reacquire_count);
            output.median_mm = reacquired_mm;
            output.filtered_mm = reacquired_mm;
            output.decision = RANGE_FILTER_REACQUIRED;
            output.publish_valid = 1U;
        }
        return output;
    }
    cv_correct(
        state,
        output.innovation_mm,
        variance,
        output.innovation_variance_mm2);
#else
    state->distance_mm = (float)output.median_mm;
    state->last_predict_ms = input->now_ms;
#endif

    commit_median(state, candidate_window, candidate_count, candidate_head);
    state->last_accepted_ms = input->now_ms;
    state->reject_streak = 0U;
    state->candidate_count = 0U;
    increment_u32(&state->accepted_count);
    output.filtered_mm = rounded_mm(state->distance_mm);
    output.decision = RANGE_FILTER_ACCEPTED;
    output.publish_valid = 1U;
    return output;
#endif
}
