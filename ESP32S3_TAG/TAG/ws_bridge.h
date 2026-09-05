/**
 * ws_bridge.h
 * WebSocket JSON bridge:
 *   - Receives decoded packets from uart_protocol.
 *   - Serialises to JSON using ArduinoJson (StaticJsonDocument — no heap alloc).
 *   - Broadcasts to all connected clients.
 *   - Caches latest JSON per TYPE and sends to newly connected clients.
 *
 * Range JSON sequence contract:
 *   - ws_seq: bridge broadcast sequence; contiguous despite the 50 Hz limiter.
 *   - source_seq: original STM32 range-cycle sequence (may skip after limiting).
 *   - seq: legacy alias of source_seq for older GUI clients.
 */

#pragma once
#include <stdint.h>
#include "uart_protocol.h"

// WebSocket port
#define WS_PORT 81

// Max JSON string length for each TYPE
// TYPE 0x01 (range): up to ~560 bytes for 4 anchors with worst-case values
// TYPE 0x02 (stats): ~220 bytes
// TYPE 0x00 (info):  ~500 bytes for the parser's maximum 8 offset records
#define WS_JSON_MAX  640

void ws_bridge_begin(void);

// Called from loop()
void ws_bridge_poll(void);

// Called by uart_protocol callbacks (from uart_protocol_feed inside loop())
void ws_bridge_send_info(const InfoPkt* pkt);
void ws_bridge_send_range(const RangePkt* pkt,
                          uint32_t crc_errors,
                          uint32_t gap_count);
void ws_bridge_send_stats(const StatsPkt* pkt,
                          uint32_t crc_errors,
                          uint32_t gap_count);
