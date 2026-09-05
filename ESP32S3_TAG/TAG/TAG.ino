/**
 * TAG.ino — ESP32-S3 UWB Bridge Firmware
 *
 * Architecture:
 *   STM32 TAG ──UART 115200──▶ ESP32-S3 ──WiFi/WebSocket:81──▶ GUI (browser)
 *
 * This file is intentionally thin — just orchestration.
 * All heavy logic lives in the companion .cpp files:
 *   uart_protocol.cpp  — binary frame parser (state machine, CRC16)
 *   ws_bridge.cpp      — JSON serialisation + WebSocket broadcast + cache
 *   wifi_manager.cpp   — non-blocking WiFi connect/reconnect
 *
 * ⚠ BEFORE FLASHING — verify these in your hardware setup:
 *   1. UART pins: RXD1/TXD1 below must match the physical wires connecting
 *      this board to the STM32 TAG's USART1 TX/RX pins.
 *      (Board comment says GPIO1/2 to avoid camera pins on ESP32-S3-CAM)
 *   2. LED_PIN: adjust if your board's LED is on a different GPIO.
 *   3. WiFi credentials: replace SSID/PASSWORD with your network's values.
 *
 * ⚠ BINARY MODE: Set TELEM_ASCII=0 in STM32_UWB/TAG/Core/Inc/telemetry.h
 *   and reflash the STM32 AFTER verifying this ESP32 firmware works
 *   (keep ASCII while debugging with Serial Monitor).
 *
 * Libraries required (Arduino Library Manager):
 *   - ArduinoJson   by Benoit Blanchon    (>=7.x or 6.x)
 *   - WebSockets    by Markus Sattler
 *   - Board:        esp32 by Espressif Systems
 */

#include <Arduino.h>
#include <WiFi.h>          // WiFi.localIP() — dùng trong loop() diagnostic print
#include "uart_protocol.h"
#include "ws_bridge.h"
#include "wifi_manager.h"

// ---------------------------------------------------------
// CONFIGURATION — edit these for your setup
// ---------------------------------------------------------
static const char* WIFI_SSID     = "YOUR_WIFI_SSID";       // ← replace with your WiFi SSID
static const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";  // ← replace with your WiFi password

// UART1 pins connecting to STM32 TAG USART1
// ⚠ Verify against your board schematic before flashing!
#define RXD1  1   // ESP32 RX ← STM32 TX (PA9)
#define TXD1  2   // ESP32 TX → STM32 RX (not used for ranging, only future CMD)

// LED indicator (active HIGH on most devboards; invert if needed)
#define LED_PIN  33

// Stats log interval (ms) — heap + counters printed to USB serial
#define STATS_LOG_INTERVAL_MS  5000UL   /* 5s — đủ để phát hiện heap leak sớm */

// Heap warning threshold — print warning if free heap drops below this
#define HEAP_WARN_BYTES  20000UL

// ---------------------------------------------------------
// UART protocol callbacks (called from uart_protocol_feed)
// These are the "glue" between the parser and the WS bridge.
// ---------------------------------------------------------
void on_range_packet(const RangePkt* pkt)
{
    // Pass to WS bridge which serialises to JSON and broadcasts
    ws_bridge_send_range(pkt, uart_crc_error_count, uart_gap_count);
}

void on_stats_packet(const StatsPkt* pkt)
{
    ws_bridge_send_stats(pkt, uart_crc_error_count, uart_gap_count);
}

void on_info_packet(const InfoPkt* pkt)
{
    ws_bridge_send_info(pkt);
}

// ---------------------------------------------------------
// setup()
// ---------------------------------------------------------
void setup()
{
    // LED init
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // USB debug serial
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}  // wait up to 2s for USB enumeration

    Serial.println("\n==================================");
    Serial.println("  UWB ESP32-S3 Bridge Firmware");
    Serial.println("==================================");

    // UART1 ← STM32 TAG
    // ⚠ If STM32 moves to HSE + higher baud, update this value too.
    // The STM32 emits one 4-anchor range frame every 20 ms (~4.2 kB/s at
    // 115200 baud).  A larger RX FIFO absorbs Wi-Fi/WebSocket scheduling
    // bursts without dropping bytes and producing a cascading CRC failure.
    Serial1.setRxBufferSize(2048);
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
    Serial.printf("[UART] Serial1 started: RX=GPIO%d TX=GPIO%d @ 115200\n",
                  RXD1, TXD1);

    // WiFi (blocking up to WIFI_INIT_TIMEOUT_MS, then continues in background)
    wifi_manager_begin(WIFI_SSID, WIFI_PASSWORD);

    // WebSocket server
    ws_bridge_begin();

    // UART parser state machine
    uart_protocol_init();

    Serial.println("[BOOT] Ready. Waiting for STM32 binary packets...");
    Serial.println("[BOOT] (Set TELEM_ASCII=0 in STM32 telemetry.h to enable binary mode)");

    digitalWrite(LED_PIN, HIGH);   // LED on = firmware running
}

// ---------------------------------------------------------
// loop()
// ---------------------------------------------------------
static uint32_t s_last_stats_ms  = 0;
static uint32_t s_last_blink_ms  = 0;
static bool     s_blink_state    = false;
static uint32_t s_bytes_received = 0;

void loop()
{
    // 1. WiFi reconnect (non-blocking)
    wifi_manager_poll();

    // 2. WebSocket housekeeping (heartbeat, ping/pong, accept new clients)
    ws_bridge_poll();

    // 3. Drain UART — feed every available byte into the state machine.
    //    Limit bytes per loop() call to avoid starving WiFi/WS stack.
    //    At 115200 baud, 50 bytes = ~4.3ms of data → safe budget.
    // 512 bytes is a burst guard (~44ms at 115200) while the enlarged RX
    // buffer absorbs shorter Wi-Fi/WebSocket scheduling stalls.
    uint16_t uart_budget = 512;
    while (uart_budget-- && Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();
        uart_protocol_feed(b);
        s_bytes_received++;
    }
    // Yield to WiFi/WebSocket task after UART drain
    yield();

    // 4. Periodic diagnostics on USB serial (~every 5s)
    uint32_t now = millis();
    if (now - s_last_stats_ms >= STATS_LOG_INTERVAL_MS) {
        s_last_stats_ms = now;
        uint32_t heap = ESP.getFreeHeap();
        Serial.printf("[DIAG] heap_free=%u  bytes_rx=%lu"
                      "  crc_err=%lu  gap=%lu  unknown_type=%lu  overflow=%lu"
                      "  wifi=%s\n",
                      heap,
                      s_bytes_received,
                      (unsigned long)uart_crc_error_count,
                      (unsigned long)uart_gap_count,
                      (unsigned long)uart_unknown_type_count,
                      (unsigned long)uart_overflow_count,
                      wifi_manager_connected() ? WiFi.localIP().toString().c_str()
                                               : "disconnected");
        if (heap < HEAP_WARN_BYTES) {
            Serial.printf("[WARN] LOW HEAP: %u bytes free! Consider reboot.\n", heap);
        }
        s_bytes_received = 0;  // reset per-interval counter
    }

    // 5. Blink LED to indicate data flow (fast = data, slow = idle)
    //    LED blinks at ~2Hz when parsing bytes, ~0.5Hz when idle.
    uint32_t blink_period = (s_bytes_received > 0) ? 250 : 1000;
    if (now - s_last_blink_ms >= blink_period) {
        s_last_blink_ms = now;
        s_blink_state   = !s_blink_state;
        digitalWrite(LED_PIN, s_blink_state ? HIGH : LOW);
    }
}
