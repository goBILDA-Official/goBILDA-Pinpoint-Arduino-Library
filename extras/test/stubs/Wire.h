// Host-side stand-in for Wire.h: every read returns the bytes in rx.
#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
class TwoWire {
public:
    std::vector<uint8_t> rx;
    size_t pos = 0;
    void begin() {}
    void beginTransmission(uint8_t) {}
    size_t write(uint8_t) { return 1; }
    size_t write(const uint8_t*, size_t n) { return n; }
    uint8_t endTransmission(bool) { pos = 0; return 0; }
    uint8_t requestFrom(uint8_t, uint8_t n, uint8_t) { return n; }
    int available() { return (int)(rx.size() - pos); }
    int read() { return pos < rx.size() ? rx[pos++] : 0; }
};
extern TwoWire Wire;
