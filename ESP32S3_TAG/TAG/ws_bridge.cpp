/**
 * ws_bridge.cpp
 * WebSocket JSON bridge implementation.
 *
 * Library dependency: ArduinoJson by Benoit Blanchon (install from Library Manager).
 * Library dependency: WebSockets by Markus Sattler (install from Library Manager).
 *
 * StaticJsonDocument is used in all hot paths (no heap allocation per packet).
 * Cache is a plain char array per TYPE — updated in-place.
 */

#include "ws_bridge.h"
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <string.h>

// -------------------------------------------------------
// WebSocket server instance
// -------------------------------------------------------
static WebSocketsServer s_ws(WS_PORT);

// -------------------------------------------------------
// JSON cache per TYPE
// Updated every time a packet is decoded.
// Sent immediately to newly connected clients.
// -------------------------------------------------------
static char s_cache_range[WS_JSON_MAX] = "";
static char s_cache_stats[WS_JSON_MAX] = "";
static char s_cache_info[WS_JSON_MAX] = "";

// -------------------------------------------------------
// Static JSON documents — NOT on stack (avoid stack overflow at 50Hz)
// -------------------------------------------------------
static StaticJsonDocument<768> s_doc_range;
static StaticJsonDocument<448> s_doc_stats;
static StaticJsonDocument<768> s_doc_info;

// -------------------------------------------------------
// Range data is emitted at the TAG cycle rate. The GUI separately decimates
// chart history, while calibration, logging and the position pipeline retain
// every received frame.
// -------------------------------------------------------
static uint32_t s_last_range_broadcast_ms = 0;
// Sequence of range JSON frames broadcast by this bridge. It advances only
// when a rate-limited frame is actually selected for broadcast, so the GUI can
// distinguish WebSocket loss from intentional STM32 -> WebSocket downsampling.
static uint32_t s_range_ws_seq = 0;
#define WS_RANGE_MIN_INTERVAL_MS  20U  /* 1000/20 = 50 Hz max */

/* Measured bridge emission rate. These counters distinguish an intentional
 * rate limit from a low-heap skipped broadcast in the GUI diagnostics. */
static uint32_t s_range_rate_window_started_ms = 0;
static uint32_t s_range_rate_window_count = 0;
static uint32_t s_range_broadcast_hz = 0;
static uint32_t s_range_low_heap_drop_count = 0;

static void ws_bridge_refresh_range_rate(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (s_range_rate_window_started_ms == 0U)
    {
        s_range_rate_window_started_ms = now_ms;
        return;
    }

    elapsed_ms = now_ms - s_range_rate_window_started_ms;
    if (elapsed_ms < 500U)
        return;

    s_range_broadcast_hz =
        (s_range_rate_window_count * 1000U + elapsed_ms / 2U) / elapsed_ms;
    s_range_rate_window_count = 0U;
    s_range_rate_window_started_ms = now_ms;
}

static void ws_bridge_note_range_broadcast(uint32_t now_ms)
{
    ws_bridge_refresh_range_rate(now_ms);
    if (s_range_rate_window_count < UINT32_MAX)
        s_range_rate_window_count++;
}


// -------------------------------------------------------
// WebSocket event handler
// -------------------------------------------------------
static void ws_event(uint8_t num, WStype_t type, uint8_t* payload, size_t length)
{
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[WS] Client %u disconnected\n", num);
            break;

        case WStype_CONNECTED: {
            IPAddress ip = s_ws.remoteIP(num);
            Serial.printf("[WS] Client %u connected from %s\n",
                          num, ip.toString().c_str());

            // Send firmware profile first, so cached telemetry is interpreted
            // with the correct ranging/calibration mode from the first sample.
            if (s_cache_info[0] != '\0')
                s_ws.sendTXT(num, s_cache_info);
            if (s_cache_range[0] != '\0')
                s_ws.sendTXT(num, s_cache_range);
            if (s_cache_stats[0] != '\0')
                s_ws.sendTXT(num, s_cache_stats);
            break;
        }

        case WStype_TEXT:
            // GUI may send commands in future; log for now
            Serial.printf("[WS] Client %u sent: %.*s\n",
                          num, (int)length, (char*)payload);
            break;

        default:
            break;
    }
}

// -------------------------------------------------------
// Public API
// -------------------------------------------------------
void ws_bridge_begin(void)
{
    s_ws.begin();
    s_ws.onEvent(ws_event);
    Serial.printf("[WS] WebSocket server started on port %d\n", WS_PORT);
}

void ws_bridge_poll(void)
{
    s_ws.loop();
}

// TYPE 0x00 -> JSON "t":"i". This packet is repeated by STM32 and cached
// here so clients connecting later still learn the active calibration profile.
void ws_bridge_send_info(const InfoPkt* pkt)
{
    s_doc_info.clear();
    s_doc_info["t"]                  = "i";
    s_doc_info["seq"]                = pkt->seq;
    s_doc_info["time_ms"]            = pkt->time_ms;
    s_doc_info["schema"]             = pkt->schema;
    s_doc_info["flags"]              = pkt->flags;
    s_doc_info["ranging_mode"]       = pkt->ranging_mode;
    s_doc_info["ds_calibrated_mask"] = pkt->ds_calibrated_mask;
    s_doc_info["filter_mode"]         = pkt->filter_mode;
    s_doc_info["phy_profile_id"]      = pkt->phy_profile_id;
    s_doc_info["spi_clock_mhz"]       = pkt->spi_clock_mhz;
    s_doc_info["c9_2_motion_mode"]    = pkt->c9_2_motion_mode;
    s_doc_info["c9_2_global_motion_state"] = pkt->c9_2_global_motion_state;

    JsonArray offsets = s_doc_info.createNestedArray("offsets");
    for (uint8_t i = 0; i < pkt->num_records; i++) {
        const InfoAnchorRecord& a = pkt->anchors[i];
        JsonObject obj = offsets.createNestedObject();
        obj["id"]               = a.anchor_id;
        obj["active_offset_um"] = a.active_offset_um;
    }

    size_t n = serializeJson(s_doc_info, s_cache_info, sizeof(s_cache_info));
    if (n == 0 || n >= sizeof(s_cache_info)) {
        Serial.println("[WS] info JSON serialise failed");
        return;
    }

    s_ws.broadcastTXT(s_cache_info, n);
}

// -------------------------------------------------------
// TYPE 0x01 → JSON "t":"r"
// {
//   "t":"r",
//   "ws_seq":123, "source_seq":5976, "seq":5976, "time_ms":120416,
//   "anchors":[
//     {"id":1,"valid":true,"status":0,"age_ms":13,
//      "raw_mm":398,"filt_mm":406,"fpp_dbm":-83.33}, ...
//   ]
// }
// -------------------------------------------------------
void ws_bridge_send_range(const RangePkt* pkt,
                          uint32_t crc_errors,
                          uint32_t gap_count)
{
    // Rate limit at the 50 Hz TAG cycle. The free-heap guard is retained as a
    // fail-safe and every skipped frame is exposed through TYPE 0x02 stats.
    uint32_t now_ms = millis();
    bool do_broadcast = (now_ms - s_last_range_broadcast_ms >= WS_RANGE_MIN_INTERVAL_MS);

    // Decide before serialising: cache-only updates retain the last broadcast
    // sequence, while the next broadcast advances it exactly once. A newly
    // connected client receives that cached value as its sequence baseline.
    if (do_broadcast && ESP.getFreeHeap() < 8192) {
        Serial.printf("[WS] LOW HEAP (%u bytes) — skipping range broadcast\n",
                      ESP.getFreeHeap());
        if (s_range_low_heap_drop_count < UINT32_MAX)
            s_range_low_heap_drop_count++;
        do_broadcast = false;
    }
    const uint32_t frame_ws_seq = do_broadcast
                                ? (uint32_t)(s_range_ws_seq + 1U)
                                : s_range_ws_seq;

    // Always update cache (so new clients get latest state on connect)
    // but only broadcast at limited rate
    s_doc_range.clear();
    s_doc_range["t"]       = "r";
    s_doc_range["ws_seq"]  = frame_ws_seq;
    s_doc_range["source_seq"] = pkt->seq;
    s_doc_range["seq"]     = pkt->seq; // Legacy alias for source_seq.
    s_doc_range["time_ms"] = pkt->time_ms;

    JsonArray anchors = s_doc_range.createNestedArray("anchors");
    for (uint8_t i = 0; i < pkt->num_records; i++) {
        const AnchorRecord& a = pkt->anchors[i];
        JsonObject obj = anchors.createNestedObject();
        obj["id"]      = a.anchor_id;
        obj["valid"]   = (bool)(a.valid != 0);
        obj["status"]  = a.status;
        obj["age_ms"]  = a.age_ms;
        obj["raw_mm"]  = a.raw_mm;
        obj["filt_mm"] = a.filtered_mm;
        obj["fpp_dbm"] = (float)a.fpp_cdbm / 100.0f;
    }

    size_t n = serializeJson(s_doc_range, s_cache_range, sizeof(s_cache_range));
    if (n == 0 || n >= sizeof(s_cache_range)) {
        Serial.println("[WS] range JSON serialise failed");
        return;
    }

    if (do_broadcast) {
        // Commit only after JSON serialisation succeeded. This prevents a
        // local encoding failure from looking like WebSocket packet loss.
        s_range_ws_seq = frame_ws_seq;
        s_last_range_broadcast_ms = now_ms;
        s_ws.broadcastTXT(s_cache_range, n);
        ws_bridge_note_range_broadcast(now_ms);
    }
}


// -------------------------------------------------------
// TYPE 0x02 → JSON "t":"s"
// {
//   "t":"s",
//   "seq":300, "time_ms":121000,
//   "poll_sent":1000, "response_ok":950,
//   "rx_timeout":30, "rx_error":20,
//   "cycle_overrun":0, "uart_overflow":0,
//   "cyc_hz":50, "ops_hz":145,
//   "uart_gap_count":0, "crc_error_count":0
// }
// -------------------------------------------------------
void ws_bridge_send_stats(const StatsPkt* pkt,
                          uint32_t crc_errors,
                          uint32_t gap_count)
{
    ws_bridge_refresh_range_rate(millis());
    s_doc_stats.clear();

    s_doc_stats["t"]               = "s";
    s_doc_stats["seq"]             = pkt->seq;
    s_doc_stats["time_ms"]         = pkt->time_ms;
    s_doc_stats["poll_sent"]       = pkt->poll_sent;
    s_doc_stats["response_ok"]     = pkt->response_ok;
    s_doc_stats["rx_timeout"]      = pkt->rx_timeout;
    s_doc_stats["rx_error"]        = pkt->rx_error;
    s_doc_stats["cycle_overrun"]   = pkt->cycle_overrun;
    s_doc_stats["uart_overflow"]   = pkt->uart_overflow;
    s_doc_stats["cyc_hz"]          = pkt->cyc_hz;
    s_doc_stats["ops_hz"]          = pkt->ops_hz;
    s_doc_stats["uart_gap_count"]  = gap_count;
    s_doc_stats["crc_error_count"] = crc_errors;
    // Heap status in stats packet — useful for monitoring
    s_doc_stats["heap_free"]       = (uint32_t)ESP.getFreeHeap();
    s_doc_stats["ws_range_hz"]     = s_range_broadcast_hz;
    s_doc_stats["ws_range_drop_count"] = s_range_low_heap_drop_count;
    s_doc_stats["ws_range_interval_ms"] = WS_RANGE_MIN_INTERVAL_MS;

    size_t n = serializeJson(s_doc_stats, s_cache_stats, sizeof(s_cache_stats));
    if (n == 0 || n >= sizeof(s_cache_stats)) {
        Serial.println("[WS] stats JSON serialise failed");
        return;
    }

    s_ws.broadcastTXT(s_cache_stats, n);
}
