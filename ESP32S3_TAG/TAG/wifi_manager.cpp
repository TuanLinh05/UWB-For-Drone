/**
 * wifi_manager.cpp
 * Non-blocking WiFi connect + auto-reconnect.
 */

#include "wifi_manager.h"
#include <WiFi.h>
#include <Arduino.h>

static const char* s_ssid     = nullptr;
static const char* s_password = nullptr;

static uint32_t s_last_attempt_ms = 0;
static bool     s_connecting      = false;

void wifi_manager_begin(const char* ssid, const char* password)
{
    s_ssid     = ssid;
    s_password = password;

    Serial.printf("[WiFi] Connecting to \"%s\"...\n", ssid);

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);  // We handle reconnect manually (non-blocking)
    WiFi.begin(ssid, password);

    s_connecting      = true;
    s_last_attempt_ms = millis();

    // Wait up to WIFI_INIT_TIMEOUT_MS in setup() so the IP address is printed
    // before entering loop(). Still non-blocking relative to UART/WS.
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_INIT_TIMEOUT_MS) {
        delay(200);
        Serial.print('.');
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.printf("[WiFi] Connected! IP: %s\n",
                      WiFi.localIP().toString().c_str());
        s_connecting = false;
    } else {
        Serial.println();
        Serial.println("[WiFi] Initial connect failed — will retry in background.");
    }
}

bool wifi_manager_connected(void)
{
    return WiFi.status() == WL_CONNECTED;
}

void wifi_manager_poll(void)
{
    if (WiFi.status() == WL_CONNECTED) {
        s_connecting = false;
        return;
    }

    // Not connected — attempt reconnect after back-off interval
    uint32_t now = millis();
    if (now - s_last_attempt_ms >= WIFI_RETRY_INTERVAL_MS) {
        s_last_attempt_ms = now;
        Serial.printf("[WiFi] Not connected (status=%d), reconnecting...\n",
                      WiFi.status());
        WiFi.disconnect(false);
        WiFi.begin(s_ssid, s_password);
        s_connecting = true;
    }
}
