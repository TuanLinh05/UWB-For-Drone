#pragma once

// Minimal host-only declaration used by syntax checks for uart_protocol.cpp.
// The real ESP32 build uses the Arduino core header instead.
struct HostSerialStub {
    template <typename... Args>
    int printf(const char*, Args...) { return 0; }
};

extern HostSerialStub Serial;
