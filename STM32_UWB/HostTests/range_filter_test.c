#include "range_filter.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static RangeFilterInput_t input_at(int32_t range_mm, uint32_t now_ms)
{
    RangeFilterInput_t input = {
        .corrected_raw_mm = range_mm,
        .fpp_dbm = -75.0f,
        .now_ms = now_ms,
        .radio_status = 0U,
    };
    return input;
}

static RangeFilterOutput_t update(
    RangeFilterState_t *state,
    int32_t range_mm,
    uint32_t now_ms,
    const RangeFilterConfig_t *config)
{
    RangeFilterInput_t input = input_at(range_mm, now_ms);
    return RangeFilter_Update(state, &input, config);
}

static void test_physical_and_cold_start(void)
{
    RangeFilterState_t state;
    RangeFilter_Init(&state);
    RangeFilterOutput_t negative = update(&state, -1, 10U, &g_range_filter_config);
    assert(negative.publish_valid == 0U);
    assert(negative.decision == RANGE_FILTER_REJECTED_PHYSICAL);
    assert(state.initialized == 0U);
    assert(state.physical_reject_count == 1U);

    RangeFilterOutput_t first = update(&state, 1000, 20U, &g_range_filter_config);
    assert(first.publish_valid == 1U);
    assert(first.filtered_mm == 1000);
    assert(first.decision == RANGE_FILTER_ACCEPTED);
    assert(state.initialized == 1U);

    RangeFilterInput_t radio_error = input_at(1000, 40U);
    radio_error.radio_status = 0x01U;
    assert(RangeFilter_Update(&state, &radio_error, &g_range_filter_config).publish_valid == 0U);
}

static void test_single_spike_and_ramp(void)
{
    RangeFilterState_t state;
    RangeFilter_Init(&state);
    update(&state, 1000, 20U, &g_range_filter_config);
    update(&state, 1000, 40U, &g_range_filter_config);
    update(&state, 1000, 60U, &g_range_filter_config);

    RangeFilterOutput_t spike = update(&state, 5000, 80U, &g_range_filter_config);
    assert(spike.filtered_mm < 1500);
    RangeFilterOutput_t normal = update(&state, 1000, 100U, &g_range_filter_config);
    assert(normal.publish_valid == 1U);
    assert(normal.filtered_mm < 1500);

    RangeFilter_Init(&state);
    update(&state, 1000, 20U, &g_range_filter_config);
    for (uint32_t step = 1U; step <= 20U; step++)
    {
        /* 8 m/s radial ramp at 20 ms = 160 mm/sample. */
        RangeFilterOutput_t ramp = update(
            &state,
            1000 + (int32_t)(160U * step),
            20U + 20U * step,
            &g_range_filter_config);
        assert(ramp.publish_valid == 1U);
    }
}

static void test_reacquire_and_stale_reset(void)
{
    RangeFilterState_t state;
    RangeFilter_Init(&state);
    update(&state, 1000, 20U, &g_range_filter_config);
    update(&state, 1000, 40U, &g_range_filter_config);
    update(&state, 1000, 60U, &g_range_filter_config);

    RangeFilterOutput_t latest = {0};
#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_MEDIAN_GATE
    uint32_t samples_to_reacquire = 0U;
#endif
    for (uint32_t index = 0U; index < 8U; index++)
    {
#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_MEDIAN_GATE
        samples_to_reacquire++;
#endif
        latest = update(
            &state,
            3000 + (int32_t)(index % 2U) * 20,
            80U + index * 20U,
            &g_range_filter_config);
        if (latest.decision == RANGE_FILTER_REACQUIRED)
            break;
    }
    assert(latest.decision == RANGE_FILTER_REACQUIRED);
    assert(latest.publish_valid == 1U);
    assert(state.reacquire_count == 1U);
    assert(abs(latest.filtered_mm - 3010) < 100);
#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_MEDIAN_GATE
    /* C9 must reacquire a consistent large step within 80 ms (four 20 ms
     * samples including median priming), not wait for stale-reset timeout. */
    assert(samples_to_reacquire <= 4U);
#endif

    RangeFilterOutput_t after_gap = update(&state, 7000, 1000U, &g_range_filter_config);
    assert(after_gap.decision == RANGE_FILTER_REACQUIRED);
    assert(after_gap.filtered_mm == 7000);
    assert(state.stale_reset_count == 1U);
}

static void test_wrap_and_variable_period(void)
{
    RangeFilterState_t state;
    RangeFilter_Init(&state);
    update(&state, 1000, 0xfffffff0U, &g_range_filter_config);
    RangeFilterOutput_t wrapped = update(&state, 1100, 0x00000018U, &g_range_filter_config);
    assert(wrapped.publish_valid == 1U);

    const uint32_t periods[] = {20U, 25U, 28U, 40U};
    uint32_t now = 0x18U;
    int32_t range = 1100;
    for (size_t index = 0U; index < sizeof(periods) / sizeof(periods[0]); index++)
    {
        now += periods[index];
        range += (int32_t)(periods[index] * 5U);
        assert(update(&state, range, now, &g_range_filter_config).publish_valid == 1U);
    }
}

#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_CV_KALMAN_V2
static void test_cv_nis_and_covariance(void)
{
    RangeFilterState_t state;
    RangeFilter_Init(&state);
    update(&state, 1000, 20U, &g_range_filter_config);
    update(&state, 1000, 40U, &g_range_filter_config);
    update(&state, 1000, 60U, &g_range_filter_config);

    RangeFilterOutput_t spike = update(&state, 4500, 80U, &g_range_filter_config);
    assert(spike.publish_valid == 0U || spike.filtered_mm < 2000);
    assert(isfinite(state.p00_mm2));
    assert(isfinite(state.p11_mm2_s2));
    assert(state.p00_mm2 > 0.0f);
    assert(state.p11_mm2_s2 > 0.0f);
}
#endif

int main(void)
{
    test_physical_and_cold_start();
    test_single_spike_and_ramp();
    test_reacquire_and_stale_reset();
    test_wrap_and_variable_period();
#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_CV_KALMAN_V2
    test_cv_nis_and_covariance();
#endif
    printf("range_filter mode %u: all host tests passed\n", (unsigned)UWB_RANGE_FILTER_MODE);
    return 0;
}
