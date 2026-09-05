#ifndef RANGE_FILTER_H
#define RANGE_FILTER_H

#include <stdint.h>
#include "range_filter_config.h"

typedef enum {
    RANGE_FILTER_ACCEPTED = 0,
    RANGE_FILTER_REJECTED_PHYSICAL,
    RANGE_FILTER_REJECTED_DYNAMIC,
    RANGE_FILTER_REJECTED_NIS,
    RANGE_FILTER_REACQUIRED
} RangeFilterDecision_t;

typedef struct {
    int32_t corrected_raw_mm;
    float fpp_dbm;
    uint32_t now_ms;
    uint8_t radio_status;
} RangeFilterInput_t;

typedef struct {
    int32_t median_mm;
    int32_t filtered_mm;
    float innovation_mm;
    float innovation_variance_mm2;
    float nis_1d;
    float measurement_variance_mm2;
    RangeFilterDecision_t decision;
    uint8_t publish_valid;
} RangeFilterOutput_t;

typedef struct {
    float distance_mm;
    float radial_velocity_mm_s;
    float p00_mm2;
    float p01_mm2_s;
    float p11_mm2_s2;

    int32_t median_window_mm[3];
    uint8_t median_count;
    uint8_t median_head;

    uint32_t last_predict_ms;
    uint32_t last_accepted_ms;
    uint16_t reject_streak;
    uint16_t candidate_count;
    float candidate_mean_mm;
    uint8_t initialized;

    uint32_t accepted_count;
    uint32_t physical_reject_count;
    uint32_t dynamic_reject_count;
    uint32_t nis_reject_count;
    uint32_t reacquire_count;
    uint32_t stale_reset_count;
} RangeFilterState_t;

void RangeFilter_Init(RangeFilterState_t *state);

RangeFilterOutput_t RangeFilter_Update(
    RangeFilterState_t *state,
    const RangeFilterInput_t *input,
    const RangeFilterConfig_t *config);

#endif /* RANGE_FILTER_H */
