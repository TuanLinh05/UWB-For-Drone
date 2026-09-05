/**
 ******************************************************************************
 * @file    telemetry.h
 * @brief   Telemetry output — binary packet hoặc ASCII CSV (Phase 1)
 *
 * Mọi output đi qua UART_TX ring buffer (non-blocking).
 *
 * Chọn định dạng bằng macro TELEM_ASCII:
 *   TELEM_ASCII = 1            : CSV dễ đọc trên serial monitor để bring-up.
 *   TELEM_ASCII = 0 (mặc định) : binary protocol cho ESP32/GUI.
 *
 * Binary packet:
 *   TYPE 0x00 info payload (schema 2):
 *     schema[1] flags[1] ranging_mode[1] num_rec[1]
 *     ds_calibrated_mask[1] filter_mode[1] phy_profile_id[1] spi_clock_mhz[1]
 *     c9_2_motion_mode[1] c9_2_global_motion_state[1]
 *     + N records: anchor_id[2] active_offset_um[4].
 *     flags bit0=HW antenna delay, bit1=legacy offset, bit2=DS build,
 *     bits3..4=Adaptive Legacy mode (0=OFF, 1=SHADOW, 2=ACTIVE).
 *   SOF[2]=0xAA,0x55 | VER[1]=1 | TYPE[1] | LEN[2] | SEQ[4] | TIME[4] |
 *   PAYLOAD[LEN] | CRC16[2]
 *   CRC16-CCITT (0x1021, init 0xFFFF) tính trên VER..hết PAYLOAD, little-endian.
 *
 *   TYPE 0x01 = range cycle: num_rec[1] + N×record(16B)
 *     record: anchor_id[2] valid[1] status[1] age_ms[2]
 *             raw_mm[4] filtered_mm[4] fpp_cdbm[2]   (đều little-endian)
 *   TYPE 0x02 = stats: poll[4] ok[4] rx_to[4] rx_err[4]
 *             overrun[4] uart_ovf[4] cyc_hz[2] ops_hz[2]
 *
 * ASCII range schema v2 (TELEM_ASCII=1):
 *   R2,seq,time,id,valid,status,age,raw,filt,fpp,...
 * Status is mandatory because raw is pre-offset when calibration is missing.
 ******************************************************************************
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tag_ranging.h"

/** 1 = ASCII CSV (dễ test qua Serial Monitor); 0 = binary cho ESP32 bridge.
 *  Mặc định = 0 (binary) — flash ESP32 bridge + GUI cần binary.
 *  Đổi thành 1 tạm thời nếu cần debug qua Serial Monitor không có ESP32. */
#ifndef TELEM_ASCII
#define TELEM_ASCII   0
#endif

#define TELEM_VER            1
#define TELEM_TYPE_INFO      0x00
#define TELEM_TYPE_RANGE     0x01
#define TELEM_TYPE_STATS     0x02
#define TELEM_INFO_SCHEMA    2U
#define TELEM_INFO_HEADER_LEN 10U

/* INFO flags stay within the existing one-byte payload. Adding the Adaptive
 * Legacy mode here makes replay profiles fail closed when an A/B build changes
 * it, without changing the frozen range/stats record contracts. */
#define TELEM_INFO_FLAG_HW_ANTENNA_DELAY       0x01U
#define TELEM_INFO_FLAG_LEGACY_OFFSET           0x02U
#define TELEM_INFO_FLAG_DS_BUILD                0x04U
#define TELEM_INFO_FLAG_LEGACY_ADAPTIVE_SHIFT   3U
#define TELEM_INFO_FLAG_LEGACY_ADAPTIVE_MASK    0x18U

/** Send the active ranging/calibration profile. This is intentionally sent
 * periodically (not only at boot), because USB/WebSocket clients attach late. */
void Telem_SendInfo(void);

/**
 * @brief  Gửi một gói range cycle (một dòng CSV hoặc một binary packet).
 * @param  snap  snapshot đã chụp từ Tag_GetSnapshot()
 */
void Telem_SendRangeCycle(const TagCycleSnapshot_t *snap);

/**
 * @brief  Gửi gói thống kê (~1Hz) — counters + tần số đo được.
 * @param  cyc_hz  số chu kỳ/giây đo được
 * @param  ops_hz  số phép đo thành công/giây đo được
 */
void Telem_SendStats(uint16_t cyc_hz, uint16_t ops_hz);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
