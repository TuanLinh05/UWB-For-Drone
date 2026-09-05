#ifndef RANGE_FILTER_CONFIG_H
#define RANGE_FILTER_CONFIG_H

#include <stdint.h>
#include "uwb_calibration.h"

/**
 * C9 range-conditioner configuration.
 *
 * All values are validation seeds, not production-tuned constants. Legacy
 * remains the build default until replay plus multi-distance hardware gates.
 */
typedef struct {
    int32_t min_range_mm;                    /* inclusive physical envelope, mm */
    int32_t max_range_mm;                    /* inclusive physical envelope, mm */
    uint32_t max_radial_speed_mm_s;          /* motion envelope, mm/s */
    uint32_t gate_margin_up_mm;              /* positive jump margin, mm */
    uint32_t gate_margin_down_mm;            /* negative jump margin, mm */
    uint32_t stale_reset_ms;                 /* reset median/reacquire after gap */
    float process_accel_sigma_mm_s2;         /* CV process acceleration sigma */
    float initial_velocity_sigma_mm_s;       /* CV initialization velocity sigma */
    float nis_gate_1d;                       /* CV chi-square gate */
    float min_dt_s;                          /* CV numerical lower dt */
    float max_tracking_dt_s;                 /* no clamping; larger gaps reacquire */
    uint16_t reacquire_min_samples;           /* consecutive clustered candidates */
    int32_t reacquire_cluster_mm;             /* maximum cluster deviation, mm */
    float reacquire_min_fpp_dbm;              /* minimum quality for reacquisition */
    float fpp_strong_dbm;
    float fpp_medium_dbm;
    float variance_strong_mm2;
    float variance_medium_mm2;
    float variance_weak_mm2;
    float variance_floor_mm2;
    float variance_ceiling_mm2;
} RangeFilterConfig_t;

extern const RangeFilterConfig_t g_range_filter_config;

#endif /* RANGE_FILTER_CONFIG_H */
