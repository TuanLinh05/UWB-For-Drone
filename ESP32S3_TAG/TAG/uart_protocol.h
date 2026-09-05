/**
 * uart_protocol.h
 * Binary frame parser — byte-by-byte state machine.
 *
 * Frame format:
 *   SOF[2]=0xAA,0x55 | VER[1] | TYPE[1] | LEN[2] | SEQ[4] | TIME[4] | PAYLOAD[LEN] | CRC16[2]
 * All multi-byte integers: little-endian.
 * CRC16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over bytes VER..end of PAYLOAD.
 */

#pragma once
#include <stdint.h>

// -------------------------------------------------------
// Packet types
// -------------------------------------------------------
#define PKT_TYPE_RANGE  0x01   // TYPE 0x01: Range cycle  (~50 Hz)
#define PKT_TYPE_STATS  0x02   // TYPE 0x02: Stats        (~1 Hz)
#define PKT_TYPE_INFO   0x00   // TYPE 0x00: Info/profile (periodic + cached)
#define PKT_TYPE_POS    0x03   // TYPE 0x03: Position EKF (Phase 5, stub)
#define PKT_VERSION     0x01

// Max payload we accept (typ. TYPE 0x01 = 14+1+3*16+2 = 65 bytes payload)
#define UART_RX_MAX_PKT  96

// Header size: VER(1)+TYPE(1)+LEN(2)+SEQ(4)+TIME(4) = 12 bytes
// Plus SOF(2) = 14 bytes total before payload.
#define HDR_PAYLOAD_OFFSET 12   // bytes after SOF before payload starts (stored in pkt_buf[0..])
#define HDR_SOF_SIZE       2

// -------------------------------------------------------
// Decoded structures
// -------------------------------------------------------

#define MAX_ANCHORS 8  // guard against future expansion

struct AnchorRecord {
    uint16_t anchor_id;
    uint8_t  valid;
    uint8_t  status;
    uint16_t age_ms;
    int32_t  raw_mm;
    int32_t  filtered_mm;
    int16_t  fpp_cdbm;
};

struct RangePkt {
    uint32_t    seq;
    uint32_t    time_ms;
    uint8_t     num_records;
    AnchorRecord anchors[MAX_ANCHORS];
};

struct StatsPkt {
    uint32_t seq;
    uint32_t time_ms;
    uint32_t poll_sent;
    uint32_t response_ok;
    uint32_t rx_timeout;
    uint32_t rx_error;
    uint32_t cycle_overrun;
    uint32_t uart_overflow;
    uint16_t cyc_hz;
    uint16_t ops_hz;
};

struct InfoAnchorRecord {
    uint16_t anchor_id;
    int32_t  active_offset_um;
};

struct InfoPkt {
    uint32_t seq;
    uint32_t time_ms;
    uint8_t  schema;
    uint8_t  flags;               // bit0=HW delay, bit1=legacy, bit2=DS, bits3..4=Adaptive Legacy mode
    uint8_t  ranging_mode;        // 0=SS, 1=DS build
    uint8_t  num_records;
    uint8_t  ds_calibrated_mask;
    uint8_t  filter_mode;
    uint8_t  phy_profile_id;       // 0=unspecified, 1=legacy1024, 2=Fast-256
    uint8_t  spi_clock_mhz;        // selected runtime SPI after readback
    uint8_t  c9_2_motion_mode;     // schema2: 0=OFF, 1=SHADOW, 2=ACTIVE
    uint8_t  c9_2_global_motion_state; // schema2: range regime, 0..5
    InfoAnchorRecord anchors[MAX_ANCHORS];
};

// -------------------------------------------------------
// Diagnostic counters (readable from ws_bridge for JSON)
// -------------------------------------------------------
extern uint32_t uart_crc_error_count;
extern uint32_t uart_gap_count;
extern uint32_t uart_unknown_type_count;
extern uint32_t uart_overflow_count;   // pkt too large
extern uint32_t uart_version_error_count;

// -------------------------------------------------------
// Callbacks — implement in TAG.ino or ws_bridge.cpp
// -------------------------------------------------------
// Called when a valid TYPE 0x01 frame is parsed.
void on_range_packet(const RangePkt* pkt);
// Called when a valid TYPE 0x02 frame is parsed.
void on_stats_packet(const StatsPkt* pkt);
// Called for TYPE 0x00. The STM32 repeats this packet so late clients can sync.
void on_info_packet(const InfoPkt* pkt);

// -------------------------------------------------------
// API
// -------------------------------------------------------
void uart_protocol_init(void);

// Feed one byte at a time from Serial1.read().
// The state machine accumulates bytes and calls the callbacks above
// when a complete, valid packet is ready.
void uart_protocol_feed(uint8_t byte);

// CRC16/CCITT-FALSE — exposed for testing.
uint16_t crc16_ccitt(const uint8_t* data, uint16_t len);
