#include "legacy_adaptive_tracking.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* This host model mirrors only the deployed scalar Legacy conditioner plus
 * legacy_adaptive_tracking.h. It deliberately takes now_ms as an argument so
 * timing, stale gaps and uint32 wrap are deterministic. */

#define BASE_Q 0.05
#define TRACK_GAIN 0.15

typedef struct {
    int32_t values[3];
    uint8_t head;
    uint8_t count;
} Median3_t;

typedef struct {
    double q;
    double r;
    double x;
    double p;
    uint8_t outlier_count;
    uint32_t last_meas_ms;
    uint8_t initialized;
} ScalarKalman_t;

typedef struct {
    Median3_t median;
    ScalarKalman_t kf;
    LegacyAdaptiveTrackingState_t adaptive;
} AdaptiveModel_t;

typedef struct {
    uint8_t valid;
    int32_t filtered_mm;
    uint8_t tracking;
} ModelResult_t;

static const LegacyAdaptiveTrackingConfig_t kConfig = {
    .motion_enter_mm = 250.0,
    .settle_residual_mm = 120.0,
    .candidate_cluster_mm = 250.0,
    .max_radial_speed_mm_s = 10000.0,
    .gate_margin_up_mm = 100.0,
    .gate_margin_down_mm = 250.0,
    .reacquire_min_fpp_dbm = -105.0f,
    .motion_confirm_samples = 4U,
    .stale_reacquire_samples = 3U,
    .settle_samples = 5U,
    .stale_reset_ms = 500U,
    .candidate_max_gap_ms = 100U,
    .nominal_sample_ms = 20U,
    .min_tracking_dt_ms = 10U,
    .max_tracking_dt_ms = 100U,
};

static int32_t median3_push(Median3_t *median, int32_t value)
{
    int32_t a;
    int32_t b;
    int32_t c;
    int32_t tmp;

    median->values[median->head] = value;
    median->head = (uint8_t)((median->head + 1U) % 3U);
    if (median->count < 3U)
        median->count++;
    if (median->count < 3U)
        return value;

    a = median->values[0];
    b = median->values[1];
    c = median->values[2];
    if (a > b) { tmp = a; a = b; b = tmp; }
    if (b > c) { tmp = b; b = c; c = tmp; }
    if (a > b) { tmp = a; a = b; b = tmp; }
    return b;
}

static double noise_for_fpp(float fpp)
{
    if (fpp <= -95.0f)
        return 10000.0;
    if (fpp > -75.0f)
        return 50.0;
    if (fpp > -82.0f)
        return 200.0;
    return 1000.0;
}

static void kf_initialize(ScalarKalman_t *kf, int32_t measurement, uint32_t now_ms)
{
    kf->q = BASE_Q;
    kf->r = 0.0;
    kf->x = (double)measurement;
    kf->p = 1.0;
    kf->outlier_count = 0U;
    kf->last_meas_ms = now_ms;
    kf->initialized = 1U;
}

static int32_t kf_update(ScalarKalman_t *kf, int32_t measurement, float fpp)
{
    const double r = noise_for_fpp(fpp);
    const double predicted_p = kf->p + kf->q;
    const double gain = predicted_p / (predicted_p + r);

    kf->r = r;
    kf->x += gain * ((double)measurement - kf->x);
    kf->p = (1.0 - gain) * predicted_p;
    return (int32_t)kf->x;
}

static uint8_t legacy_gate(ScalarKalman_t *kf, int32_t measurement,
                            uint32_t now_ms, uint8_t allow_snap)
{
    uint32_t dt_ms = now_ms - kf->last_meas_ms;
    double delta;
    double up;
    double down;

    if (dt_ms < 20U) dt_ms = 20U;
    if (dt_ms > 500U) dt_ms = 500U;
    up = 10000.0 * ((double)dt_ms / 1000.0) + 100.0;
    down = -(10000.0 * ((double)dt_ms / 1000.0) + 250.0);
    delta = (double)measurement - kf->x;
    if (delta > up || delta < down)
    {
        if (kf->outlier_count < 0xffU)
            kf->outlier_count++;
        if (allow_snap == 0U || kf->outlier_count < 15U)
            return 1U;
        kf->x = (double)measurement;
        kf->outlier_count = 0U;
        kf->last_meas_ms = now_ms;
        return 0U;
    }
    kf->outlier_count = 0U;
    kf->last_meas_ms = now_ms;
    return 0U;
}

static double steady_p(double r)
{
    return (sqrt(BASE_Q * BASE_Q + 4.0 * BASE_Q * r) - BASE_Q) * 0.5;
}

static double tracking_q(double r, uint32_t dt_ms)
{
    const double nominal_q = r * TRACK_GAIN * TRACK_GAIN / (1.0 - TRACK_GAIN);
    return nominal_q * ((double)dt_ms / 20.0);
}

static void model_init(AdaptiveModel_t *model)
{
    memset(model, 0, sizeof(*model));
    LegacyAdaptiveTracking_Init(&model->adaptive);
}

static ModelResult_t baseline_update(AdaptiveModel_t *model, int32_t raw_mm,
                                     float fpp, uint32_t now_ms)
{
    const int32_t median_mm = median3_push(&model->median, raw_mm);
    float update_fpp = fpp;

    if (model->kf.initialized == 0U)
        kf_initialize(&model->kf, median_mm, now_ms);
    if (legacy_gate(&model->kf, median_mm, now_ms, 1U) != 0U)
        update_fpp = -100.0f;
    return (ModelResult_t){ 1U, kf_update(&model->kf, median_mm, update_fpp), 0U };
}

static ModelResult_t adaptive_update(AdaptiveModel_t *model, int32_t raw_mm,
                                     float fpp, uint32_t now_ms)
{
    int32_t median_mm;
    uint8_t entered_tracking;
    uint8_t gate_rejected;

    if (LegacyAdaptiveTracking_BeginSample(&model->adaptive, now_ms, &kConfig) != 0U)
        memset(&model->median, 0, sizeof(model->median));
    median_mm = median3_push(&model->median, raw_mm);

    if (model->adaptive.state == LEGACY_ADAPTIVE_STATE_STALE_REACQUIRE)
    {
        double reacquired_mm = 0.0;
        if (LegacyAdaptiveTracking_ObserveStaleReacquire(
                &model->adaptive, (double)median_mm, fpp, now_ms,
                &kConfig, &reacquired_mm) == 0U)
            return (ModelResult_t){ 0U, 0, 0U };
        model->kf.x = reacquired_mm;
        model->kf.q = BASE_Q;
        model->kf.r = noise_for_fpp(fpp);
        model->kf.p = steady_p(model->kf.r);
        model->kf.outlier_count = 0U;
        model->kf.last_meas_ms = now_ms;
        model->kf.initialized = 1U;
        return (ModelResult_t){ 1U, (int32_t)reacquired_mm, 0U };
    }

    if (model->kf.initialized == 0U)
        kf_initialize(&model->kf, median_mm, now_ms);

    if (model->adaptive.state == LEGACY_ADAPTIVE_STATE_TRACKING)
    {
        const uint32_t dt_ms = LegacyAdaptiveTracking_TrackingDtMs(
            &model->adaptive, now_ms, &kConfig);
        if (LegacyAdaptiveTracking_AcceptTrackingMeasurement(
                &model->adaptive, (double)median_mm, now_ms, &kConfig) == 0U)
            return (ModelResult_t){ 0U, 0, 1U };
        model->kf.q = tracking_q(noise_for_fpp(fpp), dt_ms);
        {
            const int32_t filtered = kf_update(&model->kf, median_mm, fpp);
            LegacyAdaptiveTracking_NoteTrackingUpdate(&model->adaptive, now_ms);
            if (LegacyAdaptiveTracking_ObserveSettled(
                    &model->adaptive, (double)median_mm, model->kf.x, &kConfig) != 0U)
            {
                model->kf.q = BASE_Q;
                model->kf.p = steady_p(model->kf.r);
            }
            return (ModelResult_t){ 1U, filtered,
                model->adaptive.state == LEGACY_ADAPTIVE_STATE_TRACKING };
        }
    }

    gate_rejected = legacy_gate(&model->kf, median_mm, now_ms, 0U);
    entered_tracking = LegacyAdaptiveTracking_ObserveMotionCandidate(
        &model->adaptive, (double)median_mm, model->kf.x, now_ms, &kConfig);
    if (entered_tracking != 0U)
    {
        model->kf.r = noise_for_fpp(fpp);
        model->kf.q = tracking_q(model->kf.r, 20U);
        model->kf.p = model->kf.r * TRACK_GAIN;
        model->kf.outlier_count = 0U;
        model->kf.last_meas_ms = now_ms;
        assert(LegacyAdaptiveTracking_AcceptTrackingMeasurement(
            &model->adaptive, (double)median_mm, now_ms, &kConfig) != 0U);
        {
            const int32_t filtered = kf_update(&model->kf, median_mm, fpp);
            LegacyAdaptiveTracking_NoteTrackingUpdate(&model->adaptive, now_ms);
            return (ModelResult_t){ 1U, filtered, 1U };
        }
    }
    if (gate_rejected != 0U)
    {
        model->adaptive.true_reject_count++;
        return (ModelResult_t){ 0U, 0, 0U };
    }

    model->kf.q = BASE_Q;
    return (ModelResult_t){ 1U, kf_update(&model->kf, median_mm, fpp), 0U };
}

static void test_header_timing_and_candidates(void)
{
    LegacyAdaptiveTrackingState_t state;
    double reacquired_mm = 0.0;

    LegacyAdaptiveTracking_Init(&state);
    assert(LegacyAdaptiveTracking_BeginSample(&state, 0xfffffff0U, &kConfig) == 0U);
    /* 20 ms through uint32 wrap must not look stale. */
    assert(LegacyAdaptiveTracking_BeginSample(&state, 0x00000004U, &kConfig) == 0U);

    assert(LegacyAdaptiveTracking_ObserveMotionCandidate(
        &state, 1500.0, 1000.0, 24U, &kConfig) == 0U);
    assert(LegacyAdaptiveTracking_ObserveMotionCandidate(
        &state, 1510.0, 1000.0, 44U, &kConfig) == 0U);
    assert(LegacyAdaptiveTracking_ObserveMotionCandidate(
        &state, 1520.0, 1000.0, 64U, &kConfig) == 0U);
    assert(LegacyAdaptiveTracking_ObserveMotionCandidate(
        &state, 1530.0, 1000.0, 84U, &kConfig) == 1U);
    assert(state.state == LEGACY_ADAPTIVE_STATE_TRACKING);
    assert(state.track_enter_count == 1U);

    assert(LegacyAdaptiveTracking_AcceptTrackingMeasurement(
        &state, 1530.0, 84U, &kConfig) == 1U);
    assert(LegacyAdaptiveTracking_AcceptTrackingMeasurement(
        &state, 1690.0, 104U, &kConfig) == 1U); /* 8 m/s radial ramp */
    LegacyAdaptiveTracking_NoteTrackingUpdate(&state, 104U);
    assert(LegacyAdaptiveTracking_TrackingDtMs(&state, 129U, &kConfig) == 25U);
    assert(LegacyAdaptiveTracking_AcceptTrackingMeasurement(
        &state, 1890.0, 129U, &kConfig) == 1U); /* 8 m/s for 25 ms */
    assert(LegacyAdaptiveTracking_AcceptTrackingMeasurement(
        &state, 2210.0, 169U, &kConfig) == 1U); /* 8 m/s for 40 ms */
    assert(LegacyAdaptiveTracking_AcceptTrackingMeasurement(
        &state, 5000.0, 189U, &kConfig) == 0U); /* one implausible spike */
    assert(state.true_reject_count == 1U);

    LegacyAdaptiveTracking_Init(&state);
    assert(LegacyAdaptiveTracking_BeginSample(&state, 20U, &kConfig) == 0U);
    assert(LegacyAdaptiveTracking_BeginSample(&state, 621U, &kConfig) == 1U);
    assert(state.state == LEGACY_ADAPTIVE_STATE_STALE_REACQUIRE);
    /* Weak DS FPP is valid, but below the reacquire floor it must not seed a
     * new canonical location. */
    assert(LegacyAdaptiveTracking_ObserveStaleReacquire(
        &state, 3000.0, -106.0f, 621U, &kConfig, &reacquired_mm) == 0U);
    assert(LegacyAdaptiveTracking_ObserveStaleReacquire(
        &state, 3000.0, -97.0f, 641U, &kConfig, &reacquired_mm) == 0U);
    assert(LegacyAdaptiveTracking_ObserveStaleReacquire(
        &state, 3010.0, -97.0f, 661U, &kConfig, &reacquired_mm) == 0U);
    assert(LegacyAdaptiveTracking_ObserveStaleReacquire(
        &state, 2990.0, -97.0f, 681U, &kConfig, &reacquired_mm) == 1U);
    assert(fabs(reacquired_mm - 3000.0) < 1.0);
    assert(state.state == LEGACY_ADAPTIVE_STATE_STABLE);
}

static void test_legacy_golden_vector(void)
{
    AdaptiveModel_t model;
    const int32_t input[] = {1000, 1100, 1100, 1000, 1000, 5000, 1000, 1000};
    /* Generated from the original median-3 + gate + scalar Kalman equations
     * at FPP -70 dBm. This is a regression vector for the OFF baseline. */
    const int32_t expected[] = {1000, 1002, 1004, 1006, 1006, 1006, 1005, 1005};
    size_t index;

    model_init(&model);
    for (index = 0U; index < sizeof(input) / sizeof(input[0]); index++)
    {
        ModelResult_t output = baseline_update(
            &model, input[index], -70.0f, 20U + (uint32_t)index * 20U);
        assert(output.valid == 1U);
        assert(output.filtered_mm == expected[index]);
    }
}

static void test_static_and_spikes_never_enter_tracking(void)
{
    AdaptiveModel_t baseline;
    AdaptiveModel_t adaptive;
    const int32_t noise[] = {-90, 60, -35, 95, -70, 40, 0, -60, 80, -20};
    uint32_t index;

    model_init(&baseline);
    model_init(&adaptive);
    for (index = 0U; index < 3000U; index++) /* 60 seconds at 50 Hz */
    {
        const int32_t raw = 2000 + noise[index % (sizeof(noise) / sizeof(noise[0]))];
        const uint32_t now_ms = 20U + index * 20U;
        const ModelResult_t base = baseline_update(&baseline, raw, -97.0f, now_ms);
        const ModelResult_t candidate = adaptive_update(&adaptive, raw, -97.0f, now_ms);
        assert(candidate.valid == 1U);
        assert(candidate.filtered_mm == base.filtered_mm);
        assert(candidate.tracking == 0U);
    }
    assert(adaptive.adaptive.track_enter_count == 0U);

    (void)adaptive_update(&adaptive, 5000, -97.0f, 60020U);
    (void)adaptive_update(&adaptive, 5000, -97.0f, 60040U);
    (void)adaptive_update(&adaptive, 2000, -97.0f, 60060U);
    (void)adaptive_update(&adaptive, 2000, -97.0f, 60080U);
    assert(adaptive.adaptive.track_enter_count == 0U);
}

static void test_alternating_static_multipath_never_enters_tracking(void)
{
    AdaptiveModel_t model;
    uint32_t index;

    model_init(&model);
    for (index = 0U; index < 10U; index++)
        (void)adaptive_update(&model, 2000, -97.0f, 20U + index * 20U);
    for (index = 0U; index < 300U; index++)
    {
        /* 275 mm alternating excursions are large enough to exercise motion
         * candidate handling but remain inside the deployed 20 ms gate. */
        const int32_t raw = (index & 1U) == 0U ? 1725 : 2275;
        const ModelResult_t output = adaptive_update(
            &model, raw, -97.0f, 220U + index * 20U);
        assert(output.valid == 1U);
        assert(output.tracking == 0U);
    }
    assert(model.adaptive.track_enter_count == 0U);
}

static void test_step_converges_without_a_fresh_snap(void)
{
    AdaptiveModel_t model;
    uint32_t now_ms = 20U;
    uint32_t step_start_ms;
    uint32_t t90_ms = 0U;
    int32_t latest = 1000;
    uint32_t index;

    model_init(&model);
    for (index = 0U; index < 10U; index++, now_ms += 20U)
        latest = adaptive_update(&model, 1000, -97.0f, now_ms).filtered_mm;
    step_start_ms = now_ms;
    for (index = 0U; index < 50U; index++, now_ms += 20U)
    {
        const ModelResult_t output = adaptive_update(&model, 3000, -97.0f, now_ms);
        if (output.valid != 0U)
            latest = output.filtered_mm;
        if (t90_ms == 0U && latest >= 2800)
            t90_ms = now_ms - step_start_ms;
    }

    assert(model.adaptive.track_enter_count == 1U);
    assert(t90_ms > 0U && t90_ms <= 500U);
    assert(latest >= 2900 && latest <= 3050);
    assert(model.adaptive.stale_reacquire_count == 0U);
}

static void test_eight_meters_per_second_ramp_enters_and_stays_tracking(void)
{
    AdaptiveModel_t model;
    uint32_t index;
    int32_t latest = 1000;

    model_init(&model);
    for (index = 0U; index < 10U; index++)
        latest = adaptive_update(&model, 1000, -97.0f, 20U + index * 20U).filtered_mm;

    for (index = 1U; index <= 30U; index++)
    {
        const ModelResult_t output = adaptive_update(
            &model, 1000 + (int32_t)(160U * index), -97.0f,
            220U + index * 20U);
        if (output.valid != 0U)
            latest = output.filtered_mm;
    }

    assert(model.adaptive.track_enter_count == 1U);
    assert(model.adaptive.state == LEGACY_ADAPTIVE_STATE_TRACKING);
    assert(latest > 4000);
}

static void test_stale_reacquire_does_not_mix_old_median(void)
{
    AdaptiveModel_t model;
    ModelResult_t output;

    model_init(&model);
    (void)adaptive_update(&model, 1000, -97.0f, 20U);
    (void)adaptive_update(&model, 1000, -97.0f, 40U);
    (void)adaptive_update(&model, 1000, -97.0f, 60U);
    output = adaptive_update(&model, 3000, -97.0f, 700U);
    assert(output.valid == 0U);
    output = adaptive_update(&model, 3010, -97.0f, 720U);
    assert(output.valid == 0U);
    output = adaptive_update(&model, 2990, -97.0f, 740U);
    assert(output.valid == 1U);
    assert(output.filtered_mm >= 2980 && output.filtered_mm <= 3020);
}

int main(void)
{
    test_header_timing_and_candidates();
    test_legacy_golden_vector();
    test_static_and_spikes_never_enter_tracking();
    test_alternating_static_multipath_never_enters_tracking();
    test_step_converges_without_a_fresh_snap();
    test_eight_meters_per_second_ramp_enters_and_stays_tracking();
    test_stale_reacquire_does_not_mix_old_median();
    puts("legacy adaptive tracking: all host tests passed");
    return 0;
}
