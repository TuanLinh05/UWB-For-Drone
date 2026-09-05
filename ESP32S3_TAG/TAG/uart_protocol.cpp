/**
 * uart_protocol.cpp
 * Binary frame parser — byte-by-byte state machine (no blocking reads).
 *
 * Design rules:
 *  - NEVER use readStringUntil / readBytes — UART is a stream; one loop()
 *    iteration may deliver partial data. Everything goes through feed().
 *  - Fixed stack buffers only, no String/heap alloc in hot path.
 *  - On any error: increment counter, jump back to WAIT_SOF1.
 *  - Unknown TYPE: skip exactly LEN bytes then continue — safe for future
 *    STM32 firmware updates that add TYPE before ESP32 is updated.
 */

#include "uart_protocol.h"
#include <Arduino.h>
#include <string.h>  // memcpy

// -------------------------------------------------------
// Diagnostic counters
// -------------------------------------------------------
uint32_t uart_crc_error_count    = 0;
uint32_t uart_gap_count          = 0;
uint32_t uart_unknown_type_count = 0;
uint32_t uart_overflow_count     = 0;
uint32_t uart_version_error_count = 0;

// -------------------------------------------------------
// CRC16/CCITT-FALSE
// poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000
// Check value: crc16_ccitt("123456789", 9) == 0x29B1
// -------------------------------------------------------
uint16_t crc16_ccitt(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// -------------------------------------------------------
// State machine
// -------------------------------------------------------
enum ParseState {
    WAIT_SOF1,   // Waiting for 0xAA
    WAIT_SOF2,   // Waiting for 0x55
    HDR,         // Collecting 12-byte header (VER..TIME)
    PAYLOAD,     // Collecting LEN payload bytes
    CRC1,        // CRC low byte
    CRC2,        // CRC high byte → validate → dispatch
};

static ParseState s_state  = WAIT_SOF1;
static uint8_t  s_buf[UART_RX_MAX_PKT]; // pkt_buf: from VER byte onward
static uint16_t s_buf_idx  = 0;
static uint16_t s_hdr_cnt  = 0;  // bytes received in HDR state
static uint16_t s_pay_cnt  = 0;  // bytes received in PAYLOAD state
static uint16_t s_pay_len  = 0;  // declared payload length (LEN field)
static uint8_t  s_type     = 0;
static uint32_t s_seq      = 0;
static uint32_t s_time_ms  = 0;
static uint8_t  s_crc_lo   = 0;

// Last sequence per TYPE for gap detection
static uint32_t s_last_seq[256];
static bool     s_last_seq_valid[256];

static void reset_state(void)
{
    s_state   = WAIT_SOF1;
    s_buf_idx = 0;
    s_hdr_cnt = 0;
    s_pay_cnt = 0;
}

// -------------------------------------------------------
// Decoders
// -------------------------------------------------------
static void decode_range(const uint8_t* payload, uint16_t len)
{
    if (len < 1) return;
    uint8_t num_rec = payload[0];
    if (num_rec > MAX_ANCHORS) return;

    uint16_t expected = 1 + (uint16_t)num_rec * 16;
    if (len != expected) return;

    RangePkt pkt;
    pkt.seq          = s_seq;
    pkt.time_ms      = s_time_ms;
    pkt.num_records  = num_rec;

    const uint8_t* p = payload + 1;
    for (uint8_t i = 0; i < num_rec; i++) {
        AnchorRecord& r = pkt.anchors[i];
        memcpy(&r.anchor_id,    p,      2); p += 2;
        r.valid    = *p++;
        r.status   = *p++;
        memcpy(&r.age_ms,       p,      2); p += 2;
        memcpy(&r.raw_mm,       p,      4); p += 4;
        memcpy(&r.filtered_mm,  p,      4); p += 4;
        memcpy(&r.fpp_cdbm,     p,      2); p += 2;
    }

    on_range_packet(&pkt);
}

static void decode_stats(const uint8_t* payload, uint16_t len)
{
    if (len != 28) return;

    StatsPkt pkt;
    pkt.seq       = s_seq;
    pkt.time_ms   = s_time_ms;

    const uint8_t* p = payload;
    memcpy(&pkt.poll_sent,     p, 4); p += 4;
    memcpy(&pkt.response_ok,   p, 4); p += 4;
    memcpy(&pkt.rx_timeout,    p, 4); p += 4;
    memcpy(&pkt.rx_error,      p, 4); p += 4;
    memcpy(&pkt.cycle_overrun, p, 4); p += 4;
    memcpy(&pkt.uart_overflow, p, 4); p += 4;
    memcpy(&pkt.cyc_hz,        p, 2); p += 2;
    memcpy(&pkt.ops_hz,        p, 2);

    on_stats_packet(&pkt);
}

static void decode_info(const uint8_t* payload, uint16_t len)
{
    if (len < 8) return;

    const uint8_t schema = payload[0];
    uint8_t header_len;
    if (schema == 1U) {
        header_len = 8U;
    } else if (schema == 2U) {
        header_len = 10U;
    } else {
        return; // fail closed for an INFO layout this bridge does not know
    }
    if (len < header_len) return;

    uint8_t declared_records = payload[3];
    uint16_t expected = (uint16_t)header_len + (uint16_t)declared_records * 6U;
    if (declared_records > MAX_ANCHORS || len != expected) return;

    InfoPkt pkt;
    pkt.seq                = s_seq;
    pkt.time_ms            = s_time_ms;
    pkt.schema             = schema;
    pkt.flags              = payload[1];
    pkt.ranging_mode       = payload[2];
    pkt.num_records        = declared_records;
    pkt.ds_calibrated_mask = payload[4];
    pkt.filter_mode        = payload[5];
    pkt.phy_profile_id     = payload[6];
    pkt.spi_clock_mhz      = payload[7];
    pkt.c9_2_motion_mode = schema >= 2U ? payload[8] : 0U;
    pkt.c9_2_global_motion_state = schema >= 2U ? payload[9] : 0U;

    const uint8_t* p = payload + header_len;
    for (uint8_t i = 0; i < pkt.num_records; i++) {
        memcpy(&pkt.anchors[i].anchor_id,        p,     2); p += 2;
        memcpy(&pkt.anchors[i].active_offset_um, p,     4); p += 4;
    }

    on_info_packet(&pkt);
}

static void dispatch(void)
{
    // The source sequence belongs to RANGE cycles. STATS/INFO reuse a snapshot
    // of that counter at a lower rate, so applying +1 to those packet types
    // produces false gaps by design.
    if (s_type == PKT_TYPE_RANGE && s_last_seq_valid[s_type]) {
        uint32_t expected = s_last_seq[s_type] + 1;
        if (s_seq != expected && s_seq != 0) {
            // seq wrapped around (0 is valid at boot) — count as gap
            uart_gap_count++;
        }
    }
    s_last_seq[s_type]       = s_seq;
    s_last_seq_valid[s_type] = true;

    // payload starts at byte 12 in s_buf (0=VER, 1=TYPE, 2-3=LEN, 4-7=SEQ, 8-11=TIME)
    const uint8_t* payload = s_buf + 12;

    switch (s_type) {
        case PKT_TYPE_INFO:
            decode_info(payload, s_pay_len);
            break;
        case PKT_TYPE_RANGE:
            decode_range(payload, s_pay_len);
            break;
        case PKT_TYPE_STATS:
            decode_stats(payload, s_pay_len);
            break;
        case PKT_TYPE_POS:
            // TODO Phase 5: decode EKF position
            Serial.printf("[UART] POSITION packet received, len=%u (stub)\n", s_pay_len);
            break;
        default:
            uart_unknown_type_count++;
            Serial.printf("[UART] Unknown type 0x%02X, len=%u — skipped\n", s_type, s_pay_len);
            break;
    }
}

// -------------------------------------------------------
// Public API
// -------------------------------------------------------
void uart_protocol_init(void)
{
    reset_state();
    memset(s_last_seq,       0, sizeof(s_last_seq));
    memset(s_last_seq_valid, 0, sizeof(s_last_seq_valid));
    uart_crc_error_count    = 0;
    uart_gap_count          = 0;
    uart_unknown_type_count = 0;
    uart_overflow_count     = 0;
    uart_version_error_count = 0;
}

void uart_protocol_feed(uint8_t byte)
{
    switch (s_state) {

    // ---- Sync on SOF ----
    case WAIT_SOF1:
        if (byte == 0xAA) s_state = WAIT_SOF2;
        break;

    case WAIT_SOF2:
        if (byte == 0x55) {
            // SOF found — start collecting header
            s_buf_idx = 0;
            s_hdr_cnt = 0;
            s_state   = HDR;
        } else if (byte == 0xAA) {
            // Overlapping 0xAA: stay in WAIT_SOF2
        } else {
            s_state = WAIT_SOF1;
        }
        break;

    // ---- Collect 12-byte header (VER..TIME) ----
    case HDR:
        if (s_buf_idx < UART_RX_MAX_PKT) s_buf[s_buf_idx++] = byte;
        s_hdr_cnt++;

        if (s_hdr_cnt == 12) {
            // Parse fields (all LE)
            // s_buf[0]=VER, [1]=TYPE, [2-3]=LEN, [4-7]=SEQ, [8-11]=TIME
            s_type    = s_buf[1];
            s_pay_len = (uint16_t)s_buf[2] | ((uint16_t)s_buf[3] << 8);
            s_seq     = (uint32_t)s_buf[4]
                      | ((uint32_t)s_buf[5] << 8)
                      | ((uint32_t)s_buf[6] << 16)
                      | ((uint32_t)s_buf[7] << 24);
            s_time_ms = (uint32_t)s_buf[8]
                      | ((uint32_t)s_buf[9] << 8)
                      | ((uint32_t)s_buf[10] << 16)
                      | ((uint32_t)s_buf[11] << 24);

            if (s_buf[0] != PKT_VERSION) {
                uart_version_error_count++;
                reset_state();
                break;
            }

            // Guard: payload must fit in buffer
            if (s_pay_len > UART_RX_MAX_PKT - 16) {
                uart_overflow_count++;
                reset_state();
                break;
            }

            if (s_pay_len == 0) {
                // No payload — go straight to CRC
                s_state = CRC1;
            } else {
                s_pay_cnt = 0;
                s_state   = PAYLOAD;
            }
        }
        break;

    // ---- Collect payload ----
    case PAYLOAD:
        if (s_buf_idx < UART_RX_MAX_PKT) s_buf[s_buf_idx++] = byte;
        s_pay_cnt++;
        if (s_pay_cnt == s_pay_len) s_state = CRC1;
        break;

    // ---- Collect CRC (2 bytes LE) ----
    case CRC1:
        s_crc_lo = byte;
        s_state  = CRC2;
        break;

    case CRC2: {
        uint16_t recv_crc = (uint16_t)s_crc_lo | ((uint16_t)byte << 8);
        // CRC covers s_buf[0..11+pay_len] = VER..end of PAYLOAD
        uint16_t calc_crc = crc16_ccitt(s_buf, 12 + s_pay_len);

        if (recv_crc != calc_crc) {
            uart_crc_error_count++;
            reset_state();
            break;
        }

        dispatch();
        reset_state();
        break;
    }

    default:
        reset_state();
        break;
    }
}
