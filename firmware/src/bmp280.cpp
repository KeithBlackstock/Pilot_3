#include "bmp280.hpp"
#include "i2c1.hpp"

#include <cmath>

namespace {

constexpr uint8_t kAddr = 0x77;  // SDO tied high on this board

constexpr uint8_t kRegCalibStart = 0x88;  // 24 bytes: dig_T1..T3, dig_P1..P9
constexpr uint8_t kRegChipId     = 0xD0;
constexpr uint8_t kRegCtrlMeas   = 0xF4;
constexpr uint8_t kRegStatus     = 0xF3;
constexpr uint8_t kRegPressMsb   = 0xF7;  // press(3) + temp(3) contiguous

constexpr uint8_t kChipIdExpect = 0x58;  // BMP280 (0x60 would be BME280)

// osrs_t=1, osrs_p=1, mode=forced -- one conversion then back to sleep.
constexpr uint8_t kCtrlMeasForced = (1u << 5) | (1u << 2) | 0x1;
constexpr uint8_t kStatusMeasuring = 1u << 3;
constexpr int kStatusPollLimit = 50;  // ~0.9 ms/poll -> up to ~45 ms, well over the ~6.4 ms datasheet max

constexpr float kSeaLevelPa = 101325.0f;

uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

bool read_u8(uint8_t reg, uint8_t& out) { return i2c1::read_reg(kAddr, reg, out); }

bool read_calibration() {
    uint8_t b[24];
    for (int i = 0; i < 24; ++i) {
        if (!read_u8(static_cast<uint8_t>(kRegCalibStart + i), b[i])) return false;
    }
    dig_T1 = static_cast<uint16_t>(b[0] | (b[1] << 8));
    dig_T2 = static_cast<int16_t>(b[2] | (b[3] << 8));
    dig_T3 = static_cast<int16_t>(b[4] | (b[5] << 8));
    dig_P1 = static_cast<uint16_t>(b[6] | (b[7] << 8));
    dig_P2 = static_cast<int16_t>(b[8] | (b[9] << 8));
    dig_P3 = static_cast<int16_t>(b[10] | (b[11] << 8));
    dig_P4 = static_cast<int16_t>(b[12] | (b[13] << 8));
    dig_P5 = static_cast<int16_t>(b[14] | (b[15] << 8));
    dig_P6 = static_cast<int16_t>(b[16] | (b[17] << 8));
    dig_P7 = static_cast<int16_t>(b[18] | (b[19] << 8));
    dig_P8 = static_cast<int16_t>(b[20] | (b[21] << 8));
    dig_P9 = static_cast<int16_t>(b[22] | (b[23] << 8));
    return true;
}

// Bosch BMP280 datasheet 3.11.3, integer compensation -- ported verbatim
// (not reinvented), variable names kept close to the datasheet for anyone
// cross-checking against it.
int32_t compensate_temperature(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - (static_cast<int32_t>(dig_T1) << 1))) * static_cast<int32_t>(dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - static_cast<int32_t>(dig_T1)) *
                      ((adc_T >> 4) - static_cast<int32_t>(dig_T1))) >> 12) *
                    static_cast<int32_t>(dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;  // 0.01 degC
}

uint32_t compensate_pressure(int32_t adc_P) {
    int64_t var1 = static_cast<int64_t>(t_fine) - 128000;
    int64_t var2 = var1 * var1 * static_cast<int64_t>(dig_P6);
    var2 += (var1 * static_cast<int64_t>(dig_P5)) << 17;
    var2 += static_cast<int64_t>(dig_P4) << 35;
    var1 = ((var1 * var1 * static_cast<int64_t>(dig_P3)) >> 8) + ((var1 * static_cast<int64_t>(dig_P2)) << 12);
    var1 = ((static_cast<int64_t>(1) << 47) + var1) * static_cast<int64_t>(dig_P1) >> 33;
    if (var1 == 0) return 0;  // avoid divide-by-zero (datasheet-specified guard)
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (static_cast<int64_t>(dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (static_cast<int64_t>(dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (static_cast<int64_t>(dig_P7) << 4);
    return static_cast<uint32_t>(p);  // Q24.8: Pa = value / 256
}

}  // namespace

namespace bmp280 {

bool init() {
    i2c1::init();
    uint8_t id = 0;
    if (!read_u8(kRegChipId, id) || id != kChipIdExpect) return false;
    return read_calibration();
}

Reading read() {
    Reading r{0.0f, 0.0f, 0.0f};

    if (!i2c1::write_reg(kAddr, kRegCtrlMeas, kCtrlMeasForced)) return r;

    uint8_t status = kStatusMeasuring;
    for (int i = 0; i < kStatusPollLimit && (status & kStatusMeasuring); ++i) {
        if (!read_u8(kRegStatus, status)) return r;
    }

    uint8_t raw[6];
    for (int i = 0; i < 6; ++i) {
        if (!read_u8(static_cast<uint8_t>(kRegPressMsb + i), raw[i])) return r;
    }
    int32_t adc_P = (static_cast<int32_t>(raw[0]) << 12) | (static_cast<int32_t>(raw[1]) << 4) | (raw[2] >> 4);
    int32_t adc_T = (static_cast<int32_t>(raw[3]) << 12) | (static_cast<int32_t>(raw[4]) << 4) | (raw[5] >> 4);

    int32_t temp_centi_c = compensate_temperature(adc_T);
    uint32_t press_q24_8 = compensate_pressure(adc_P);

    r.temperature_c = static_cast<float>(temp_centi_c) / 100.0f;
    r.pressure_pa = static_cast<float>(press_q24_8) / 256.0f;
    r.altitude_m = 44330.0f * (1.0f - std::pow(r.pressure_pa / kSeaLevelPa, 0.1903f));
    return r;
}

}  // namespace bmp280
