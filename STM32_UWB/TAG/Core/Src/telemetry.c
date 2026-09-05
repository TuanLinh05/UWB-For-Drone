/**
 ******************************************************************************
 * @file    telemetry.c
 * @brief   Telemetry output — binary packet / ASCII CSV (Phase 1)
 ******************************************************************************
 */

#include "telemetry.h"
#include "uart_tx.h"
#include "uwb_calibration.h"
#include <stdio.h>

#if !TELEM_ASCII   /* các helper dưới đây chỉ dùng cho binary protocol */

/* ========================================================================== */
/*                     LITTLE-ENDIAN PACKING HELPERS                           */
/* ========================================================================== */

static uint16_t put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    return 2;
}

static uint16_t put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
    return 4;
}

/* ========================================================================== */
/*                     CRC16-CCITT (0x1021, init 0xFFFF)                       */
/* ========================================================================== */

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ========================================================================== */
/*                     BINARY PACKET BUILDER                                   */
/* ========================================================================== */

/* Header: SOF[2] VER TYPE LEN[2] SEQ[4] TIME[4] = 14 byte. CRC tính từ VER. */
#define TELEM_HDR_LEN             14U
#define TELEM_RANGE_RECORD_LEN    16U
#define TELEM_RANGE_PAYLOAD_MAX   (1U + TAG_NUM_ANCHORS * TELEM_RANGE_RECORD_LEN)
#define TELEM_MAX_PKT             (TELEM_HDR_LEN + TELEM_RANGE_PAYLOAD_MAX + 2U)

_Static_assert(TELEM_MAX_PKT <= UART_TX_BUF_SIZE,
               "Telemetry packet exceeds UART TX ring capacity");

/* Đóng khung: điền header, payload đã nằm sẵn ở buf[TELEM_HDR_LEN..],
 * rồi thêm CRC16. Trả về tổng độ dài gói. */
static uint16_t frame_pack(uint8_t *buf, uint8_t type, uint32_t seq,
                           uint32_t time_ms, uint16_t payload_len)
{
    buf[0] = 0xAA;
    buf[1] = 0x55;
    buf[2] = TELEM_VER;
    buf[3] = type;
    put_u16(&buf[4], payload_len);
    put_u32(&buf[6], seq);
    put_u32(&buf[10], time_ms);

    /* CRC trên VER..hết payload (bỏ SOF, bỏ CRC) */
    uint16_t crc_len = (uint16_t)(TELEM_HDR_LEN - 2 + payload_len);
    uint16_t crc = crc16_ccitt(&buf[2], crc_len);

    uint16_t off = (uint16_t)(TELEM_HDR_LEN + payload_len);
    off += put_u16(&buf[off], crc);
    return off;
}

#endif /* !TELEM_ASCII */

/* ========================================================================== */
/*                     PUBLIC: FIRMWARE / CALIBRATION INFO                    */
/* ========================================================================== */

void Telem_SendInfo(void)
{
    uint8_t flags = 0U;
#if UWB_USE_HW_ANTENNA_DELAY
    flags |= TELEM_INFO_FLAG_HW_ANTENNA_DELAY;
#endif
#if UWB_USE_LEGACY_OFFSET
    flags |= TELEM_INFO_FLAG_LEGACY_OFFSET;
#endif
#if UWB_USE_DS_TWR
    flags |= TELEM_INFO_FLAG_DS_BUILD;
#endif
    flags |= (uint8_t)(((uint8_t)UWB_LEGACY_ADAPTIVE_MODE <<
                        TELEM_INFO_FLAG_LEGACY_ADAPTIVE_SHIFT) &
                       TELEM_INFO_FLAG_LEGACY_ADAPTIVE_MASK);

#if TELEM_ASCII
    char line[256];
    int n = snprintf(line, sizeof(line), "I,2,%u,%u,%u,%u,%u,%u,%u",
        (unsigned)flags,
        (unsigned)uwb_ds_mode_enabled_build,
        (unsigned)TAG_NUM_ANCHORS,
        (unsigned)uwb_ds_calibrated_mask_build,
        (unsigned)UWB_RANGE_FILTER_MODE,
        (unsigned)uwb_c9_2_motion_mode_build,
        (unsigned)c9_2_global_motion_state);

    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS && n > 0 && n < (int)sizeof(line); i++)
    {
        double active_offset_m;
#if UWB_USE_DS_TWR
        static const double ds_offset_m[TAG_NUM_ANCHORS] = {
            UWB_DS_OFFSET_A1_M, UWB_DS_OFFSET_A2_M,
            UWB_DS_OFFSET_A3_M, UWB_DS_OFFSET_A4_M
        };
        active_offset_m = ds_offset_m[i];
#elif UWB_USE_LEGACY_OFFSET
        active_offset_m = calibration_offset_m[i];
#else
        active_offset_m = residual_offset_m[i];
#endif
        int32_t active_offset_um = (int32_t)(active_offset_m * 1000000.0);
        n += snprintf(line + n, sizeof(line) - n, ",%u,%ld",
            (unsigned)(i + 1), (long)active_offset_um);
    }
    if (n > 0 && n < (int)sizeof(line) - 2)
    {
        line[n++] = '\r';
        line[n++] = '\n';
        UART_TX_Write((const uint8_t *)line, (uint16_t)n);
    }
#else
    uint8_t buf[TELEM_MAX_PKT];
    uint16_t off = TELEM_HDR_LEN;

    buf[off++] = TELEM_INFO_SCHEMA;
    buf[off++] = flags;
    buf[off++] = uwb_ds_mode_enabled_build;       /* configured ranging mode */
    buf[off++] = TAG_NUM_ANCHORS;
    buf[off++] = uwb_ds_calibrated_mask_build;
    buf[off++] = (uint8_t)UWB_RANGE_FILTER_MODE;
    buf[off++] = (uint8_t)DW_PHY_PROFILE_ID;
    buf[off++] = dw1000_spi_mhz;
    buf[off++] = uwb_c9_2_motion_mode_build;
    buf[off++] = c9_2_global_motion_state;

    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        double active_offset_m;
#if UWB_USE_DS_TWR
        static const double ds_offset_m[TAG_NUM_ANCHORS] = {
            UWB_DS_OFFSET_A1_M, UWB_DS_OFFSET_A2_M,
            UWB_DS_OFFSET_A3_M, UWB_DS_OFFSET_A4_M
        };
        active_offset_m = ds_offset_m[i];
#elif UWB_USE_LEGACY_OFFSET
        active_offset_m = calibration_offset_m[i];
#else
        active_offset_m = residual_offset_m[i];
#endif
        int32_t active_offset_um = (int32_t)(active_offset_m * 1000000.0);
        off += put_u16(&buf[off], (uint16_t)(i + 1));
        off += put_u32(&buf[off], (uint32_t)active_offset_um);
    }

    uint16_t payload_len = (uint16_t)(off - TELEM_HDR_LEN);
    uint16_t total = frame_pack(buf, TELEM_TYPE_INFO,
                                tag_sample_seq, HAL_GetTick(), payload_len);
    UART_TX_Write(buf, total);
#endif
}

/* ========================================================================== */
/*                     PUBLIC: RANGE CYCLE                                     */
/* ========================================================================== */

void Telem_SendRangeCycle(const TagCycleSnapshot_t *snap)
{
#if TELEM_ASCII
    char line[384];
    int n = snprintf(line, sizeof(line),
        "R2,%lu,%lu",
        (unsigned long)snap->seq, (unsigned long)snap->time_ms);

    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS && n > 0 && n < (int)sizeof(line); i++)
    {
        const TagAnchorSample_t *a = &snap->anchor[i];
        n += snprintf(line + n, sizeof(line) - n,
            ",%u,%u,%u,%u,%ld,%ld,%d",
            (unsigned)a->anchor_id, (unsigned)a->valid, (unsigned)a->status,
            (unsigned)a->age_ms,
            (long)a->raw_mm, (long)a->filtered_mm, (int)a->fpp_cdbm);
    }
    if (n > 0 && n < (int)sizeof(line) - 2)
    {
        line[n++] = '\r';
        line[n++] = '\n';
        UART_TX_Write((const uint8_t *)line, (uint16_t)n);
    }
#else
    uint8_t buf[TELEM_MAX_PKT];
    uint16_t off = TELEM_HDR_LEN;

    buf[off++] = TAG_NUM_ANCHORS;                 /* num_rec */
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        const TagAnchorSample_t *a = &snap->anchor[i];
        off += put_u16(&buf[off], a->anchor_id);
        buf[off++] = a->valid;
        buf[off++] = a->status;
        off += put_u16(&buf[off], a->age_ms);
        off += put_u32(&buf[off], (uint32_t)a->raw_mm);
        off += put_u32(&buf[off], (uint32_t)a->filtered_mm);
        off += put_u16(&buf[off], (uint16_t)a->fpp_cdbm);
    }

    uint16_t payload_len = (uint16_t)(off - TELEM_HDR_LEN);
    uint16_t total = frame_pack(buf, TELEM_TYPE_RANGE,
                                snap->seq, snap->time_ms, payload_len);
    UART_TX_Write(buf, total);
#endif
}

/* ========================================================================== */
/*                     PUBLIC: STATS                                           */
/* ========================================================================== */

void Telem_SendStats(uint16_t cyc_hz, uint16_t ops_hz)
{
#if TELEM_ASCII
    char line[160];
    int n = snprintf(line, sizeof(line),
        "S,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u\r\n",
        (unsigned long)poll_sent_count,
        (unsigned long)response_ok_count,
        (unsigned long)rx_timeout_count,
        (unsigned long)rx_error_count,
        (unsigned long)cycle_overrun_count,
        (unsigned long)uart_tx_overflow_count,
        (unsigned)cyc_hz, (unsigned)ops_hz);
    if (n > 0 && n < (int)sizeof(line))
        UART_TX_Write((const uint8_t *)line, (uint16_t)n);
#else
    uint8_t buf[TELEM_MAX_PKT];
    uint16_t off = TELEM_HDR_LEN;

    off += put_u32(&buf[off], poll_sent_count);
    off += put_u32(&buf[off], response_ok_count);
    off += put_u32(&buf[off], rx_timeout_count);
    off += put_u32(&buf[off], rx_error_count);
    off += put_u32(&buf[off], cycle_overrun_count);
    off += put_u32(&buf[off], uart_tx_overflow_count);
    off += put_u16(&buf[off], cyc_hz);
    off += put_u16(&buf[off], ops_hz);

    uint16_t payload_len = (uint16_t)(off - TELEM_HDR_LEN);
    uint16_t total = frame_pack(buf, TELEM_TYPE_STATS,
                                tag_sample_seq, HAL_GetTick(), payload_len);
    UART_TX_Write(buf, total);
#endif
}
