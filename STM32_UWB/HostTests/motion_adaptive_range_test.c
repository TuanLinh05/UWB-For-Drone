#include "motion_adaptive_range.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static const MotionAdaptiveRangeConfig_t kConfig = {
    .slow_enter_z = 2.5f,
    .slow_exit_z = 1.5f,
    .fast_enter_z = 5.0f,
    .fast_exit_z = 3.0f,
    .cusum_drift_z = 0.5f,
    .slow_speed_mm_s = 250.0f,
    .fast_speed_mm_s = 1000.0f,
    .slow_gain = 0.12f,
    .fast_gain = 0.26f,
    .settling_gain = 0.10f,
    .minimum_sigma_mm = 50.0f,
    .reacquire_min_fpp_dbm = -105.0f,
    .reacquire_cluster_mm = 250.0f,
    .max_radial_speed_mm_s = 10000.0f,
    .gate_margin_up_mm = 100.0f,
    .gate_margin_down_mm = 250.0f,
    .slow_confirm_samples = 3U,
    .fast_confirm_samples = 2U,
    .settle_dwell_samples = 10U,
    .static_dwell_samples = 25U,
    .reacquire_samples = 3U,
    .degrade_after_rejects = 3U,
    .stale_reset_ms = 500U,
    .candidate_max_gap_ms = 100U,
    .min_dt_ms = 10U,
    .max_dt_ms = 500U,
};

static void test_reacquire_and_tick_wrap(void)
{
    MotionAdaptiveRangeState_t state;
    float mean_mm = 0.0f;

    MotionAdaptiveRange_Init(&state);
    assert(MotionAdaptiveRange_BeginSample(&state, 0xfffffff0U, &kConfig) == 0U);
    /* uint32 subtraction is intentional: 20 ms across HAL tick wrap is fresh. */
    assert(MotionAdaptiveRange_BeginSample(&state, 0x00000004U, &kConfig) == 0U);
    assert(state.mode == MOTION_RANGE_REACQUIRE);

    assert(MotionAdaptiveRange_ObserveReacquire(
        &state, 3000.0f, -106.0f, 4U, &kConfig, &mean_mm) == 0U);
    assert(MotionAdaptiveRange_ObserveReacquire(
        &state, 3000.0f, -97.0f, 24U, &kConfig, &mean_mm) == 0U);
    assert(MotionAdaptiveRange_ObserveReacquire(
        &state, 3010.0f, -97.0f, 44U, &kConfig, &mean_mm) == 0U);
    assert(MotionAdaptiveRange_ObserveReacquire(
        &state, 2990.0f, -97.0f, 64U, &kConfig, &mean_mm) == 1U);
    assert(fabsf(mean_mm - 3000.0f) < 1.0f);
    assert(state.mode == MOTION_RANGE_STATIC);
    assert(state.static_enter_count == 1U);
}

static void test_kinematic_gate_and_degrade(void)
{
    MotionAdaptiveRangeState_t state;

    MotionAdaptiveRange_Init(&state);
    state.mode = MOTION_RANGE_STATIC;
    state.last_median_mm = 1000.0f;
    state.last_median_tick = 20U;
    assert(MotionAdaptiveRange_AcceptKinematic(&state, 1200.0f, 40U, &kConfig) != 0U);
    assert(MotionAdaptiveRange_AcceptKinematic(&state, 1800.0f, 40U, &kConfig) == 0U);

    MotionAdaptiveRange_NoteReject(&state, &kConfig);
    MotionAdaptiveRange_NoteReject(&state, &kConfig);
    assert(state.mode == MOTION_RANGE_STATIC);
    MotionAdaptiveRange_NoteReject(&state, &kConfig);
    assert(state.mode == MOTION_RANGE_DEGRADED);
    assert(state.degraded_enter_count == 1U);
    assert(state.true_reject_count == 3U);
}

static void test_fast_settling_static_sequence(void)
{
    MotionAdaptiveRangeState_t state;
    MotionAdaptiveRangeDecision_t decision;
    uint32_t now_ms = 20U;
    uint32_t index;
    float median_mm = 1000.0f;

    MotionAdaptiveRange_Init(&state);
    state.mode = MOTION_RANGE_STATIC;
    state.last_median_mm = median_mm;
    state.last_median_tick = now_ms;

    /* Three coherent 1 m/s radial samples create CUSUM + slope evidence and
     * must enter FAST without a response to one isolated high measurement. */
    for (index = 0U; index < 3U; index++)
    {
        now_ms += 20U;
        median_mm += 20.0f;
        decision = MotionAdaptiveRange_ObserveAccepted(
            &state, median_mm, 300.0f, 100.0f, now_ms, &kConfig);
    }
    assert(state.mode == MOTION_RANGE_FAST);
    assert(decision.mode_changed != 0U);
    assert(state.fast_enter_count == 1U);
    assert(MotionAdaptiveRange_TargetGain(&state, &kConfig) == kConfig.fast_gain);

    /* A quiet 20 ms stream first has to drain evidence and slope history,
     * then dwell in SETTLING, then reaches STATIC. */
    for (index = 0U; index < 55U; index++)
    {
        now_ms += 20U;
        decision = MotionAdaptiveRange_ObserveAccepted(
            &state, median_mm, 0.0f, 100.0f, now_ms, &kConfig);
    }
    assert(decision.publish_valid != 0U);
    assert(state.mode == MOTION_RANGE_STATIC);
    assert(state.settling_enter_count == 1U);
    assert(state.static_enter_count == 1U);
}

static void test_stale_resets_motion_evidence(void)
{
    MotionAdaptiveRangeState_t state;

    MotionAdaptiveRange_Init(&state);
    state.mode = MOTION_RANGE_FAST;
    state.initialized = 1U;
    state.last_sample_tick = 100U;
    state.slope_count = 5U;
    state.positive_evidence = 9.0f;
    assert(MotionAdaptiveRange_BeginSample(&state, 601U, &kConfig) != 0U);
    assert(state.mode == MOTION_RANGE_REACQUIRE);
    assert(state.slope_count == 0U);
    assert(state.positive_evidence == 0.0f);
    assert(state.stale_reacquire_count == 1U);
}

static void test_weak_fpp_static_noise_never_enters_motion(void)
{
    static const int32_t noise[] = {-90, 60, -35, 95, -70, 40, 0, -60, 80, -20};
    MotionAdaptiveRangeState_t state;
    uint32_t index;

    MotionAdaptiveRange_Init(&state);
    state.mode = MOTION_RANGE_STATIC;
    state.last_median_mm = 2000.0f;
    state.last_median_tick = 20U;
    for (index = 1U; index < 3000U; index++)
    {
        float value = 2000.0f + (float)noise[index % (sizeof(noise) / sizeof(noise[0]))];
        (void)MotionAdaptiveRange_ObserveAccepted(
            &state, value, value - 2000.0f, 100.0f,
            20U + index * 20U, &kConfig);
        assert(state.mode != MOTION_RANGE_SLOW);
        assert(state.mode != MOTION_RANGE_FAST);
    }
    assert(state.slow_enter_count == 0U);
    assert(state.fast_enter_count == 0U);
}

int main(void)
{
    test_reacquire_and_tick_wrap();
    test_kinematic_gate_and_degrade();
    test_fast_settling_static_sequence();
    test_stale_resets_motion_evidence();
    test_weak_fpp_static_noise_never_enters_motion();
    puts("motion adaptive range: all host tests passed");
    return 0;
}
