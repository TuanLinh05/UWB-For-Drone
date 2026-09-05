/**
 ******************************************************************************
 * @file    legacy_adaptive_tracking.h
 * @brief   Pure, allocation-free state machine for the opt-in Adaptive Legacy
 *          range tracker.
 *
 * The deployed Legacy filter remains authoritative while the feature is OFF.
 * This header deliberately owns only confirmation / tracking state; the
 * existing median-3, dynamic gate and scalar Kalman implementation stay in
 * tag_ranging.c. Keeping this unit HAL-free makes its timing behaviour
 * deterministic in host tests and prevents a second, hidden range filter.
 ******************************************************************************
 */

#ifndef LEGACY_ADAPTIVE_TRACKING_H
#define LEGACY_ADAPTIVE_TRACKING_H

#include <stdint.h>

typedef enum {
    LEGACY_ADAPTIVE_STATE_STABLE = 0U,
    LEGACY_ADAPTIVE_STATE_CANDIDATE = 1U,
    LEGACY_ADAPTIVE_STATE_TRACKING = 2U,
    LEGACY_ADAPTIVE_STATE_STALE_REACQUIRE = 3U,
} LegacyAdaptiveTrackingMode_t;

typedef struct {
    /* All values are explicit seed values. They require replay + hardware
     * validation before an ACTIVE build is accepted for flight. */
    double motion_enter_mm;
    double settle_residual_mm;
    double candidate_cluster_mm;
    double max_radial_speed_mm_s;
    double gate_margin_up_mm;
    double gate_margin_down_mm;
    float reacquire_min_fpp_dbm;
    uint8_t motion_confirm_samples;
    uint8_t stale_reacquire_samples;
    uint8_t settle_samples;
    uint32_t stale_reset_ms;
    uint32_t candidate_max_gap_ms;
    uint32_t nominal_sample_ms;
    uint32_t min_tracking_dt_ms;
    uint32_t max_tracking_dt_ms;
} LegacyAdaptiveTrackingConfig_t;

typedef struct {
    uint8_t initialized;
    uint8_t state;

    uint8_t candidate_count;
    int8_t candidate_direction;
    double candidate_mean_mm;
    uint32_t candidate_last_tick;

    uint8_t settle_count;
    uint8_t tracking_has_last_measurement;
    uint8_t tracking_has_update;
    double tracking_last_measurement_mm;
    uint32_t tracking_last_measurement_tick;
    uint32_t tracking_start_tick;
    uint32_t tracking_last_update_tick;

    uint32_t last_sample_tick;

    uint32_t track_enter_count;
    uint32_t track_exit_count;
    uint32_t true_reject_count;
    uint32_t stale_reacquire_count;
} LegacyAdaptiveTrackingState_t;

static inline double LegacyAdaptive_Abs(double value)
{
    return value < 0.0 ? -value : value;
}

static inline int8_t LegacyAdaptive_Sign(double value)
{
    return value > 0.0 ? 1 : (value < 0.0 ? -1 : 0);
}

static inline uint32_t LegacyAdaptive_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
    /* Unsigned subtraction is intentional: it is correct across HAL tick wrap. */
    return now_ms - then_ms;
}

static inline uint32_t LegacyAdaptive_ClampU32(uint32_t value,
                                                uint32_t minimum,
                                                uint32_t maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static inline void LegacyAdaptiveTracking_ResetCandidate(
    LegacyAdaptiveTrackingState_t *state)
{
    state->candidate_count = 0U;
    state->candidate_direction = 0;
    state->candidate_mean_mm = 0.0;
    state->candidate_last_tick = 0U;
}

static inline void LegacyAdaptiveTracking_Init(LegacyAdaptiveTrackingState_t *state)
{
    uint8_t *bytes = (uint8_t *)state;
    uint32_t index;

    /* Avoid depending on libc memset in the hot-path/header-only host model. */
    for (index = 0U; index < (uint32_t)sizeof(*state); index++)
        bytes[index] = 0U;
    state->state = (uint8_t)LEGACY_ADAPTIVE_STATE_STABLE;
}

/**
 * @brief Record that a radio/ToF range sample has arrived.
 * @retval 1 when the previous valid sample is older than stale_reset_ms.
 *
 * A stale event deliberately clears candidate/tracking state. The caller is
 * responsible for clearing its median window before it begins the controlled
 * three-sample reacquire sequence.
 */
static inline uint8_t LegacyAdaptiveTracking_BeginSample(
    LegacyAdaptiveTrackingState_t *state,
    uint32_t now_ms,
    const LegacyAdaptiveTrackingConfig_t *config)
{
    uint8_t stale = 0U;

    if (state->initialized != 0U
        && LegacyAdaptive_ElapsedMs(now_ms, state->last_sample_tick) > config->stale_reset_ms)
    {
        state->state = (uint8_t)LEGACY_ADAPTIVE_STATE_STALE_REACQUIRE;
        LegacyAdaptiveTracking_ResetCandidate(state);
        state->settle_count = 0U;
        state->tracking_has_last_measurement = 0U;
        state->tracking_has_update = 0U;
        state->stale_reacquire_count++;
        stale = 1U;
    }

    state->initialized = 1U;
    state->last_sample_tick = now_ms;
    return stale;
}

static inline uint8_t LegacyAdaptiveTracking_CandidateIsCompatible(
    const LegacyAdaptiveTrackingState_t *state,
    double measurement_mm,
    int8_t direction,
    uint32_t now_ms,
    const LegacyAdaptiveTrackingConfig_t *config,
    uint8_t require_same_direction)
{
    uint32_t dt_ms;
    double allowed_mm;

    if (state->candidate_count == 0U)
        return 0U;
    if (require_same_direction != 0U && direction != state->candidate_direction)
        return 0U;

    dt_ms = LegacyAdaptive_ElapsedMs(now_ms, state->candidate_last_tick);
    if (dt_ms > config->candidate_max_gap_ms)
        return 0U;

    /* A real range can move while confirmation is collected. The allowed
     * cluster therefore expands by Vmax * actual elapsed time instead of
     * assuming a fixed 20 ms period. */
    allowed_mm = config->candidate_cluster_mm
               + config->max_radial_speed_mm_s * ((double)dt_ms / 1000.0);
    return LegacyAdaptive_Abs(measurement_mm - state->candidate_mean_mm) <= allowed_mm;
}

static inline void LegacyAdaptiveTracking_StartCandidate(
    LegacyAdaptiveTrackingState_t *state,
    double measurement_mm,
    int8_t direction,
    uint32_t now_ms)
{
    state->candidate_count = 1U;
    state->candidate_direction = direction;
    state->candidate_mean_mm = measurement_mm;
    state->candidate_last_tick = now_ms;
}

static inline void LegacyAdaptiveTracking_AddCandidate(
    LegacyAdaptiveTrackingState_t *state,
    double measurement_mm,
    uint32_t now_ms)
{
    state->candidate_count++;
    state->candidate_mean_mm += (measurement_mm - state->candidate_mean_mm)
                            / (double)state->candidate_count;
    state->candidate_last_tick = now_ms;
}

static inline void LegacyAdaptiveTracking_StartTracking(
    LegacyAdaptiveTrackingState_t *state,
    uint32_t now_ms)
{
    state->state = (uint8_t)LEGACY_ADAPTIVE_STATE_TRACKING;
    state->tracking_has_last_measurement = 0U;
    state->tracking_has_update = 0U;
    state->tracking_start_tick = now_ms;
    state->tracking_last_update_tick = now_ms;
    state->settle_count = 0U;
    state->track_enter_count++;
    LegacyAdaptiveTracking_ResetCandidate(state);
}

/**
 * @brief Confirm a fresh movement candidate before entering TRACK.
 * @retval 1 exactly on the observation that enters TRACK.
 */
static inline uint8_t LegacyAdaptiveTracking_ObserveMotionCandidate(
    LegacyAdaptiveTrackingState_t *state,
    double measurement_mm,
    double estimate_mm,
    uint32_t now_ms,
    const LegacyAdaptiveTrackingConfig_t *config)
{
    const double innovation_mm = measurement_mm - estimate_mm;
    const int8_t direction = LegacyAdaptive_Sign(innovation_mm);

    if (direction == 0
        || LegacyAdaptive_Abs(innovation_mm) < config->motion_enter_mm)
    {
        if (state->state == (uint8_t)LEGACY_ADAPTIVE_STATE_CANDIDATE)
            state->state = (uint8_t)LEGACY_ADAPTIVE_STATE_STABLE;
        LegacyAdaptiveTracking_ResetCandidate(state);
        return 0U;
    }

    if (!LegacyAdaptiveTracking_CandidateIsCompatible(
            state, measurement_mm, direction, now_ms, config, 1U))
    {
        LegacyAdaptiveTracking_StartCandidate(state, measurement_mm, direction, now_ms);
        state->state = (uint8_t)LEGACY_ADAPTIVE_STATE_CANDIDATE;
        return 0U;
    }

    LegacyAdaptiveTracking_AddCandidate(state, measurement_mm, now_ms);
    state->state = (uint8_t)LEGACY_ADAPTIVE_STATE_CANDIDATE;
    if (state->candidate_count < config->motion_confirm_samples)
        return 0U;

    LegacyAdaptiveTracking_StartTracking(state, now_ms);
    return 1U;
}

/**
 * @brief Collect a controlled reacquire cluster after a true stale gap.
 * @retval 1 once stale_reacquire_samples compatible median samples are seen.
 */
static inline uint8_t LegacyAdaptiveTracking_ObserveStaleReacquire(
    LegacyAdaptiveTrackingState_t *state,
    double measurement_mm,
    float fpp_dbm,
    uint32_t now_ms,
    const LegacyAdaptiveTrackingConfig_t *config,
    double *reacquired_mm)
{
    if (fpp_dbm < config->reacquire_min_fpp_dbm)
    {
        LegacyAdaptiveTracking_ResetCandidate(state);
        return 0U;
    }

    if (!LegacyAdaptiveTracking_CandidateIsCompatible(
            state, measurement_mm, 0, now_ms, config, 0U))
    {
        LegacyAdaptiveTracking_StartCandidate(state, measurement_mm, 0, now_ms);
    }
    else
    {
        LegacyAdaptiveTracking_AddCandidate(state, measurement_mm, now_ms);
    }

    state->state = (uint8_t)LEGACY_ADAPTIVE_STATE_STALE_REACQUIRE;
    if (state->candidate_count < config->stale_reacquire_samples)
        return 0U;

    if (reacquired_mm != 0)
        *reacquired_mm = state->candidate_mean_mm;
    state->state = (uint8_t)LEGACY_ADAPTIVE_STATE_STABLE;
    LegacyAdaptiveTracking_ResetCandidate(state);
    return 1U;
}

/**
 * @brief Hard kinematic gate during TRACK, relative to the previous median.
 *
 * This intentionally never compares against a delayed Kalman estimate. A
 * rejected sample does not move the reference sample, so the next real sample
 * receives an appropriately wider actual-dt allowance.
 */
static inline uint8_t LegacyAdaptiveTracking_AcceptTrackingMeasurement(
    LegacyAdaptiveTrackingState_t *state,
    double measurement_mm,
    uint32_t now_ms,
    const LegacyAdaptiveTrackingConfig_t *config)
{
    uint32_t dt_ms;
    double delta_mm;
    double up_limit_mm;
    double down_limit_mm;

    if (state->tracking_has_last_measurement == 0U)
    {
        state->tracking_has_last_measurement = 1U;
        state->tracking_last_measurement_mm = measurement_mm;
        state->tracking_last_measurement_tick = now_ms;
        return 1U;
    }

    dt_ms = LegacyAdaptive_ElapsedMs(now_ms, state->tracking_last_measurement_tick);
    dt_ms = LegacyAdaptive_ClampU32(dt_ms,
                                    config->min_tracking_dt_ms,
                                    config->max_tracking_dt_ms);
    delta_mm = measurement_mm - state->tracking_last_measurement_mm;
    up_limit_mm = config->max_radial_speed_mm_s * ((double)dt_ms / 1000.0)
                + config->gate_margin_up_mm;
    down_limit_mm = -(config->max_radial_speed_mm_s * ((double)dt_ms / 1000.0)
                    + config->gate_margin_down_mm);
    if (delta_mm > up_limit_mm || delta_mm < down_limit_mm)
    {
        state->true_reject_count++;
        return 0U;
    }

    state->tracking_last_measurement_mm = measurement_mm;
    state->tracking_last_measurement_tick = now_ms;
    return 1U;
}

static inline uint32_t LegacyAdaptiveTracking_TrackingDtMs(
    const LegacyAdaptiveTrackingState_t *state,
    uint32_t now_ms,
    const LegacyAdaptiveTrackingConfig_t *config)
{
    uint32_t dt_ms;

    if (state->tracking_has_update == 0U)
        return config->nominal_sample_ms;

    dt_ms = LegacyAdaptive_ElapsedMs(now_ms, state->tracking_last_update_tick);
    return LegacyAdaptive_ClampU32(dt_ms,
                                   config->min_tracking_dt_ms,
                                   config->max_tracking_dt_ms);
}

static inline void LegacyAdaptiveTracking_NoteTrackingUpdate(
    LegacyAdaptiveTrackingState_t *state,
    uint32_t now_ms)
{
    state->tracking_last_update_tick = now_ms;
    state->tracking_has_update = 1U;
}

/**
 * @brief Finish tracking only after several settled median samples.
 * @retval 1 exactly when state returns to STABLE.
 */
static inline uint8_t LegacyAdaptiveTracking_ObserveSettled(
    LegacyAdaptiveTrackingState_t *state,
    double measurement_mm,
    double estimate_mm,
    const LegacyAdaptiveTrackingConfig_t *config)
{
    if (LegacyAdaptive_Abs(measurement_mm - estimate_mm) > config->settle_residual_mm)
    {
        state->settle_count = 0U;
        return 0U;
    }

    if (state->settle_count < 0xFFU)
        state->settle_count++;
    if (state->settle_count < config->settle_samples)
        return 0U;

    state->state = (uint8_t)LEGACY_ADAPTIVE_STATE_STABLE;
    state->tracking_has_last_measurement = 0U;
    state->tracking_has_update = 0U;
    state->settle_count = 0U;
    state->track_exit_count++;
    return 1U;
}

#endif /* LEGACY_ADAPTIVE_TRACKING_H */
