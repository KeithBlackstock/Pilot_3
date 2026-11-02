#include "qmc5883.hpp"
#include "i2c1.hpp"

#include <cmath>

namespace {

constexpr uint8_t kAddr = 0x0D;  // QMC5883L fixed 7-bit address

constexpr uint8_t kRegDataStart = 0x00;  // X,Y,Z as little-endian pairs, 0x00..0x05
constexpr uint8_t kRegStatus    = 0x06;  // bit0 = DRDY
constexpr uint8_t kRegCtrl1     = 0x09;
constexpr uint8_t kRegSetReset  = 0x0B;
constexpr uint8_t kRegChipId    = 0x0D;

constexpr uint8_t kChipIdExpect = 0xFF;  // QMC5883L ID register reads all-ones

// OSR=512, range=8 G, ODR=100 Hz, continuous mode.
constexpr uint8_t kCtrl1Continuous = 0x19;
constexpr uint8_t kStatusDrdy = 0x01;

volatile uint32_t g_reads_ok = 0;
volatile uint32_t g_errors = 0;

bool read_u8(uint8_t reg, uint8_t& out) {
    if (i2c1::read_reg(kAddr, reg, out)) return true;
    g_errors++;
    return false;
}

}  // namespace

namespace qmc5883 {

bool init() {
    uint8_t id = 0;
    if (!i2c1::read_reg(kAddr, kRegChipId, id) || id != kChipIdExpect) return false;
    // Datasheet-recommended set/reset period value, then continuous mode.
    if (!i2c1::write_reg(kAddr, kRegSetReset, 0x01)) return false;
    return i2c1::write_reg(kAddr, kRegCtrl1, kCtrl1Continuous);
}

bool step(Reading& out) {
    // Continuous mode: nothing to trigger, just wait for DRDY and read the
    // sample out one byte per call (the driver has no multi-byte burst --
    // same constraint and pattern as bmp280::step). Any I2C failure abandons
    // the sample in progress.
    enum State : uint8_t { STATUS, READING };
    static State state = STATUS;
    static uint8_t idx = 0;
    static uint8_t raw[6];

    switch (state) {
    case STATUS: {
        uint8_t status = 0;
        if (!read_u8(kRegStatus, status)) return false;
        if (status & kStatusDrdy) {
            idx = 0;
            state = READING;
        }
        return false;
    }
    case READING:
        if (!read_u8(static_cast<uint8_t>(kRegDataStart + idx), raw[idx])) {
            state = STATUS;
            return false;
        }
        if (++idx < 6) return false;
        state = STATUS;
        break;
    }

    out.x = static_cast<int16_t>(raw[0] | (raw[1] << 8));
    out.y = static_cast<int16_t>(raw[2] | (raw[3] << 8));
    out.z = static_cast<int16_t>(raw[4] | (raw[5] << 8));
    float hdg = std::atan2(static_cast<float>(out.y), static_cast<float>(out.x)) *
                57.29577951308232f;
    if (hdg < 0.0f) hdg += 360.0f;
    out.heading_deg = hdg;
    g_reads_ok++;
    return true;
}

Stats snapshot() { return Stats{g_reads_ok, g_errors}; }

}  // namespace qmc5883
