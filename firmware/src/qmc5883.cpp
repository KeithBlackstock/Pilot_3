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
// Read data on DRDY *or* DOR: if the chip ever skips a sample unread (boot
// timing, bus contention), DOR sets and DRDY stops asserting until someone
// reads the data registers -- gating reads on DRDY alone deadlocks
// (observed on the bench: status stuck at 0x04 until a manual data read).
constexpr uint8_t kStatusDataReady = 0x05;  // DRDY | DOR

volatile uint32_t g_reads_ok = 0;
volatile uint32_t g_errors = 0;

bool read_u8(uint8_t reg, uint8_t& out) {
    if (i2c1::read_reg(kAddr, reg, out)) return true;
    g_errors++;
    return false;
}

float clampf(float v, float lo, float hi) {
    if (!(v >= lo)) return lo;  // also catches NaN
    if (v > hi) return hi;
    return v;
}

// Scale may be negative: a mirrored (flipped) module makes the heading run
// backwards, and the ritual corrects it with a negative scale. Sanitize to
// magnitude [0.1, 10] preserving sign; NaN -> 1.
float sanitize_scale(float v) {
    if (!(v == v)) return 1.0f;  // NaN
    if (v < 0.0f) return clampf(v, -10.0f, -0.1f);
    return clampf(v, 0.1f, 10.0f);
}

}  // namespace

// Live magnetometer calibration; the dashboard finds it by symbol name and
// rewrites it over SWD. Re-read (and sanitized) on every sample, so a bad
// live write degrades the heading instead of NaN-ing it.
volatile qmc5883::Cal g_mag_cal = {
    qmc5883::kCalDefaults.off_x, qmc5883::kCalDefaults.off_y, qmc5883::kCalDefaults.off_z,
    qmc5883::kCalDefaults.scale_x, qmc5883::kCalDefaults.scale_y};

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
        if (status & kStatusDataReady) {
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
    // Heading uses the calibrated field; telemetry keeps the raw counts.
    float cx = (static_cast<float>(out.x) - clampf(g_mag_cal.off_x, -8000.0f, 8000.0f)) *
               sanitize_scale(g_mag_cal.scale_x);
    float cy = (static_cast<float>(out.y) - clampf(g_mag_cal.off_y, -8000.0f, 8000.0f)) *
               sanitize_scale(g_mag_cal.scale_y);
    float hdg = std::atan2(cy, cx) * 57.29577951308232f;
    if (hdg < 0.0f) hdg += 360.0f;
    out.heading_deg = hdg;
    g_reads_ok++;
    return true;
}

Stats snapshot() { return Stats{g_reads_ok, g_errors}; }

}  // namespace qmc5883
