/**
 * wifi_manager.h
 * Non-blocking WiFi connect + auto-reconnect.
 *
 * Design:
 *  - wifi_manager_begin(): start connection attempt (non-blocking, uses WiFi.begin).
 *  - wifi_manager_poll(): call every loop(). Checks status, reconnects when needed.
 *    Never calls delay() — safe to call 50k+ times per second.
 *  - Exponential-ish back-off: wait RETRY_INTERVAL_MS between attempts.
 */

#pragma once

// Seconds between reconnect attempts after drop
#define WIFI_RETRY_INTERVAL_MS  5000

// Timeout before giving up the initial connect and proceeding anyway
// (WebSocket and UART parsing still run; WiFi will reconnect later)
#define WIFI_INIT_TIMEOUT_MS    10000

void wifi_manager_begin(const char* ssid, const char* password);
void wifi_manager_poll(void);
bool wifi_manager_connected(void);
