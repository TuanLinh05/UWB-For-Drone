/**
 ******************************************************************************
 * @file    motion_adaptive_range.h
 * @brief   Allocation-free C9.2 range-motion regime controller.
 *
 * This module deliberately owns motion evidence and state transitions only.
 * Median filtering and scalar Kalman arithmetic stay in tag_ranging.c, so an
 * anchor still has exactly one canonical range estimator.  The module has no
 * HAL dependency and is shared by host tests and firmware.
 ******************************************************************************
 */

#ifndef MOTION_ADAPTIVE_RANGE_H
#define MOTION_ADAPTIVE_RANGE_H

#include <stdint.h>

typedef enum {
    MOTION_RANGE_REACQUIRE = 0U,
    MOTION_RANGE_STATIC = 1U,
    MOTION_RANGE_SLOW = 2U,
    MOTION_RANGE_FAST = 3U,
    MOTION_RANGE_SETTLING = 4U,
    MOTION_RANGE_DEGRADED = 5U,
} MotionAdaptiveRangeMode_t;

typedef struct {
    float slow_enter_z;
    float slow_exit_z;
    float fast_enter_z;
    float fast_exit_z;
    float cusum_drift_z;
    float slow_speed_mm_s;
    float fast_speed_mm_s;
    float slow_gain;
    float fast_gain;
    float settling_gain;
    float minimum_sigma_mm;
    float reacquire_min_fpp_dbm;
    float reacquire_cluster_mm;
    float max_radial_speed_mm_s;
    float gate_margin_up_mm;
    float gate_margin_down_mm;
    uint8_t slow_confirm_samples;
    uint8_t fast_confirm_samples;
    uint8_t settle_dwell_samples;
    uint8_t static_dwell_samples;
    uint8_t reacquire_samples;
    uint8_t degrade_after_rejects;
    uint32_t stale_reset_ms;
    uint32_t candidate_max_gap_ms;
    uint32_t min_dt_ms;
    uint32_t max_dt_ms;
} MotionAdaptiveRangeConfig_t;

typedef struct {
    uint8_t initialized;
    uint8_t mode;
    uint8_t slow_confirm_count;
    uint8_t fast_confirm_count;
    uint8_t quiet_count;
    uint8_t reacquire_count;
    uint8_t reject_streak;

    float positive_evidence;
    float negative_evidence;
    float last_median_mm;
    uint32_t last_median_tick;
    uint32_t last_sample_tick;

    float slope_history_mm_s[5];
    uint8_t slope_count;
    uint8_t slope_head;
    float last_slope_mm_s;
    float last_motion_score;

    float reacquire_mean_mm;
    uint32_t reacquire_last_tick;

    uint32_t static_enter_count;
    uint32_t slow_enter_count;
    uint32_t fast_enter_count;
    uint32_t settling_enter_count;
    uint32_t degraded_enter_count;
    uint32_t stale_reacquire_count;
    uint32_t true_reject_count;
} MotionAdaptiveRangeState_t;

typedef struct {
    uint8_t accepted;
    uint8_t publish_valid;
    uint8_t stale;
    uint8_t mode_changed;
    uint8_t previous_mode;
    float slope_mm_s;
    float motion_score;
} MotionAdaptiveRangeDecision_t;

static inline float MotionAdaptiveRange_Abs(float value)
{
    return value < 0.0f ? -value : value;
}

static inline uint32_t MotionAdaptiveRange_ElapsedMs(uint32_t now_ms,
                                                       uint32_t then_ms)
{
    /* Intentional uint32 arithmetic: correct across HAL_GetTick() wrap. */
    return now_ms - then_ms;
}

static inline uint32_t MotionAdaptiveRange_ClampU32(uint32_t value,
                                                      uint32_t minimum,
                                                      uint32_t maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static inline float MotionAdaptiveRange_ClampF32(float value,
                                                  float minimum,
                                                  float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static inline void MotionAdaptiveRange_ResetEvidence(MotionAdaptiveRangeState_t *state)
{
    state->slow_confirm_count = 0U;
    state->fast_confirm_count = 0U;
    state->quiet_count = 0U;
    state->positive_evidence = 0.0f;
    state->negative_evidence = 0.0f;
}

static inline void MotionAdaptiveRange_ResetReacquire(MotionAdaptiveRangeState_t *state)
{
    state->reacquire_count = 0U;
    state->reacquire_mean_mm = 0.0f;
    state->reacquire_last_tick = 0U;
}

static inline void MotionAdaptiveRange_Enter(MotionAdaptiveRangeState_t *state,
                                              uint8_t mode)
{
    if (state->mode == mode)
        return;

    state->mode = mode;
    if (mode == (uint8_t)MOTION_RANGE_STATIC)
        state->static_enter_count++;
    else if (mode == (uint8_t)MOTION_RANGE_SLOW)
        state->slow_enter_count++;
    else if (mode == (uint8_t)MOTION_RANGE_FAST)
        state->fast_enter_count++;
    else if (mode == (uint8_t)MOTION_RANGE_SETTLING)
        state->settling_enter_count++;
    else if (mode == (uint8_t)MOTION_RANGE_DEGRADED)
        state->degraded_enter_count++;
}

static inline void MotionAdaptiveRange_Init(MotionAdaptiveRangeState_t *state)
{
    uint8_t *bytes = (uint8_t *)state;
    uint32_t index;

    for (index = 0U; index < (uint32_t)sizeof(*state); index++)
        bytes[index] = 0U;
    state->mode = (uint8_t)MOTION_RANGE_REACQUIRE;
}

static inline uint8_t MotionAdaptiveRange_IsDynamic(uint8_t mode)
{
    return mode == (uint8_t)MOTION_RANGE_SLOW
        || mode == (uint8_t)MOTION_RANGE_FAST
        || mode == (uint8_t)MOTION_RANGE_SETTLING;
}

static inline float MotionAdaptiveRange_TargetGain(
    const MotionAdaptiveRangeState_t *state,
    const MotionAdaptiveRangeConfig_t *config)
{
    if (state->mode == (uint8_t)MOTION_RANGE_FAST)
        return config->fast_gain;
    if (state->mode == (uint8_t)MOTION_RANGE_SLOW)
        return config->slow_gain;
    if (state->mode == (uint8_t)MOTION_RANGE_SETTLING)
        return config->settling_gain;
    return 0.0f; /* STATIC retains the deployed low-Q Legacy update. */
}

/**
 * @brief Mark arrival of one valid radio distance before median/gate handling.
 * @retval 1 if a real stale gap requires the caller to clear its median state.
 */
static inline uint8_t MotionAdaptiveRange_BeginSample(
    MotionAdaptiveRangeState_t *state,
    uint32_t now_ms,
    const MotionAdaptiveRangeConfig_t *config)
{
    uint8_t stale = 0U;

    if (state->initialized != 0U
        && MotionAdaptiveRange_ElapsedMs(now_ms, state->last_sample_tick)
            > config->stale_reset_ms)
    {
        MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_REACQUIRE);
        MotionAdaptiveRange_ResetEvidence(state);
        MotionAdaptiveRange_ResetReacquire(state);
        state->reject_streak = 0U;
        state->slope_count = 0U;
        state->slope_head = 0U;
        state->last_median_mm = 0.0f;
        state->last_median_tick = 0U;
        state->last_slope_mm_s = 0.0f;
        state->last_motion_score = 0.0f;
        state->stale_reacquire_count++;
        stale = 1U;
    }

    state->initialized = 1U;
    state->last_sample_tick = now_ms;
    return stale;
}

static inline float MotionAdaptiveRange_MedianSlope(const MotionAdaptiveRangeState_t *state)
{
    float values[5];
    uint8_t count = state->slope_count;
    uint8_t i;
    uint8_t j;

    if (count == 0U)
        return 0.0f;
    for (i = 0U; i < count; i++)
        values[i] = state->slope_history_mm_s[i];
    for (i = 0U; i < count; i++)
    {
        for (j = (uint8_t)(i + 1U); j < count; j++)
        {
            if (values[i] > values[j])
            {
                float swap = values[i];
                values[i] = values[j];
                values[j] = swap;
            }
        }
    }
    if ((count & 1U) != 0U)
        return values[count / 2U];
    return (values[(count / 2U) - 1U] + values[count / 2U]) * 0.5f;
}

static inline void MotionAdaptiveRange_NoteAcceptedMedian(
    MotionAdaptiveRangeState_t *state,
    float median_mm,
    uint32_t now_ms,
    const MotionAdaptiveRangeConfig_t *config)
{
    if (state->last_median_tick != 0U)
    {
        uint32_t dt_ms = MotionAdaptiveRange_ElapsedMs(now_ms, state->last_median_tick);
        dt_ms = MotionAdaptiveRange_ClampU32(dt_ms,
                                              config->min_dt_ms,
                                              config->max_dt_ms);
        state->slope_history_mm_s[state->slope_head] =
            (median_mm - state->last_median_mm) * 1000.0f / (float)dt_ms;
        state->slope_head = (uint8_t)((state->slope_head + 1U) % 5U);
        if (state->slope_count < 5U)
            state->slope_count++;
    }
    state->last_median_mm = median_mm;
    state->last_median_tick = now_ms;
    state->last_slope_mm_s = MotionAdaptiveRange_MedianSlope(state);
}

/**
 * @brief Gate against the last accepted median, never a delayed Kalman state.
 */
static inline uint8_t MotionAdaptiveRange_AcceptKinematic(
    const MotionAdaptiveRangeState_t *state,
    float median_mm,
    uint32_t now_ms,
    const MotionAdaptiveRangeConfig_t *config)
{
    uint32_t dt_ms;
    float dt_s;
    float jump;
    float up_limit;
    float down_limit;

    if (state->last_median_tick == 0U)
        return 1U;

    dt_ms = MotionAdaptiveRange_ElapsedMs(now_ms, state->last_median_tick);
    dt_ms = MotionAdaptiveRange_ClampU32(dt_ms,
                                          config->min_dt_ms,
                                          config->max_dt_ms);
    dt_s = (float)dt_ms / 1000.0f;
    jump = median_mm - state->last_median_mm;
    up_limit = config->max_radial_speed_mm_s * dt_s + config->gate_margin_up_mm;
    down_limit = -(config->max_radial_speed_mm_s * dt_s + config->gate_margin_down_mm);
    return jump <= up_limit && jump >= down_limit ? 1U : 0U;
}

static inline void MotionAdaptiveRange_NoteReject(
    MotionAdaptiveRangeState_t *state,
    const MotionAdaptiveRangeConfig_t *config)
{
    if (state->reject_streak < 0xFFU)
        state->reject_streak++;
    state->true_reject_count++;
    if (state->reject_streak >= config->degrade_after_rejects)
    {
        MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_DEGRADED);
        MotionAdaptiveRange_ResetEvidence(state);
        MotionAdaptiveRange_ResetReacquire(state);
    }
}

/**
 * @brief Accumulate compatible medians after boot/stale/degraded state.
 * @retval 1 when a new canonical anchor range may be initialized to the mean.
 */
static inline uint8_t MotionAdaptiveRange_ObserveReacquire(
    MotionAdaptiveRangeState_t *state,
    float median_mm,
    float fpp_dbm,
    uint32_t now_ms,
    const MotionAdaptiveRangeConfig_t *config,
    float *mean_out)
{
    uint8_t compatible = 0U;

    if (fpp_dbm < config->reacquire_min_fpp_dbm)
    {
        MotionAdaptiveRange_ResetReacquire(state);
        return 0U;
    }

    if (state->reacquire_count != 0U
        && MotionAdaptiveRange_ElapsedMs(now_ms, state->reacquire_last_tick)
            <= config->candidate_max_gap_ms
        && MotionAdaptiveRange_Abs(median_mm - state->reacquire_mean_mm)
            <= config->reacquire_cluster_mm)
    {
        compatible = 1U;
    }

    if (compatible == 0U)
    {
        state->reacquire_count = 1U;
        state->reacquire_mean_mm = median_mm;
    }
    else
    {
        state->reacquire_count++;
        state->reacquire_mean_mm +=
            (median_mm - state->reacquire_mean_mm) / (float)state->reacquire_count;
    }
    state->reacquire_last_tick = now_ms;

    if (state->reacquire_count < config->reacquire_samples)
        return 0U;

    if (mean_out != 0)
        *mean_out = state->reacquire_mean_mm;
    MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_STATIC);
    MotionAdaptiveRange_ResetEvidence(state);
    MotionAdaptiveRange_ResetReacquire(state);
    state->reject_streak = 0U;
    return 1U;
}

static inline void MotionAdaptiveRange_UpdateEvidence(
    MotionAdaptiveRangeState_t *state,
    float innovation_mm,
    float sigma_mm,
    const MotionAdaptiveRangeConfig_t *config)
{
    float z;

    sigma_mm = sigma_mm < config->minimum_sigma_mm
        ? config->minimum_sigma_mm : sigma_mm;
    z = innovation_mm / sigma_mm;
    if (z >= 0.0f)
    {
        state->positive_evidence = MotionAdaptiveRange_ClampF32(
            state->positive_evidence + z - config->cusum_drift_z, 0.0f, 32.0f);
        state->negative_evidence = MotionAdaptiveRange_ClampF32(
            state->negative_evidence - z - config->cusum_drift_z, 0.0f, 32.0f);
    }
    else
    {
        state->negative_evidence = MotionAdaptiveRange_ClampF32(
            state->negative_evidence - z - config->cusum_drift_z, 0.0f, 32.0f);
        state->positive_evidence = MotionAdaptiveRange_ClampF32(
            state->positive_evidence + z - config->cusum_drift_z, 0.0f, 32.0f);
    }
    state->last_motion_score = state->positive_evidence > state->negative_evidence
        ? state->positive_evidence : state->negative_evidence;
}

/**
 * @brief Advance STATIC/SLOW/FAST/SETTLING after a kinematically accepted
 *        median.  The caller uses TargetGain() to select its scalar Kalman Q.
 */
static inline MotionAdaptiveRangeDecision_t MotionAdaptiveRange_ObserveAccepted(
    MotionAdaptiveRangeState_t *state,
    float median_mm,
    float innovation_mm,
    float sigma_mm,
    uint32_t now_ms,
    const MotionAdaptiveRangeConfig_t *config)
{
    MotionAdaptiveRangeDecision_t decision;
    uint8_t previous_mode = state->mode;
    uint8_t slow_evidence;
    uint8_t fast_evidence;
    uint8_t fast_still;
    uint8_t quiet;

    decision.accepted = 1U;
    decision.publish_valid = 1U;
    decision.stale = 0U;
    decision.mode_changed = 0U;
    decision.previous_mode = previous_mode;

    MotionAdaptiveRange_UpdateEvidence(state, innovation_mm, sigma_mm, config);
    MotionAdaptiveRange_NoteAcceptedMedian(state, median_mm, now_ms, config);

    slow_evidence = state->last_motion_score >= config->slow_enter_z
        && MotionAdaptiveRange_Abs(state->last_slope_mm_s) >= config->slow_speed_mm_s;
    fast_evidence = state->last_motion_score >= config->fast_enter_z
        && MotionAdaptiveRange_Abs(state->last_slope_mm_s) >= config->fast_speed_mm_s;
    fast_still = state->last_motion_score >= config->fast_exit_z
        && MotionAdaptiveRange_Abs(state->last_slope_mm_s) >= config->fast_speed_mm_s;
    quiet = state->last_motion_score <= config->slow_exit_z
        && MotionAdaptiveRange_Abs(state->last_slope_mm_s) <= config->slow_speed_mm_s;

    if (fast_evidence != 0U)
    {
        if (state->fast_confirm_count < 0xFFU)
            state->fast_confirm_count++;
    }
    else
    {
        state->fast_confirm_count = 0U;
    }
    if (slow_evidence != 0U)
    {
        if (state->slow_confirm_count < 0xFFU)
            state->slow_confirm_count++;
    }
    else if (state->mode != (uint8_t)MOTION_RANGE_FAST)
    {
        state->slow_confirm_count = 0U;
    }

    if (quiet != 0U)
    {
        if (state->quiet_count < 0xFFU)
            state->quiet_count++;
    }
    else
    {
        state->quiet_count = 0U;
    }

    if (state->mode == (uint8_t)MOTION_RANGE_STATIC)
    {
        if (state->fast_confirm_count >= config->fast_confirm_samples)
            MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_FAST);
        else if (state->slow_confirm_count >= config->slow_confirm_samples)
            MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_SLOW);
    }
    else if (state->mode == (uint8_t)MOTION_RANGE_SLOW)
    {
        if (state->fast_confirm_count >= config->fast_confirm_samples)
            MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_FAST);
        else if (state->quiet_count >= config->settle_dwell_samples)
            MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_SETTLING);
    }
    else if (state->mode == (uint8_t)MOTION_RANGE_FAST)
    {
        if (fast_still == 0U && slow_evidence != 0U)
            MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_SLOW);
        else if (state->quiet_count >= config->settle_dwell_samples)
            MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_SETTLING);
    }
    else if (state->mode == (uint8_t)MOTION_RANGE_SETTLING)
    {
        if (state->fast_confirm_count >= config->fast_confirm_samples)
            MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_FAST);
        else if (state->slow_confirm_count >= config->slow_confirm_samples)
            MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_SLOW);
        else if (state->quiet_count >= config->static_dwell_samples)
            MotionAdaptiveRange_Enter(state, (uint8_t)MOTION_RANGE_STATIC);
    }

    state->reject_streak = 0U;
    decision.mode_changed = state->mode != previous_mode ? 1U : 0U;
    decision.slope_mm_s = state->last_slope_mm_s;
    decision.motion_score = state->last_motion_score;
    return decision;
}

#endif /* MOTION_ADAPTIVE_RANGE_H */
