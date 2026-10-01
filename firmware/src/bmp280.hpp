// BMP280 driver over I2C1, addr 0x77 (SDO strapped high on this board --
// same as pilot3_swd_probe.py's probe_baro()). Forced mode: trigger one
// conversion, poll the status register, read raw, apply Bosch's integer
// compensation formulas (from the datasheet, not reinvented).
#pragma once
#include <cstdint>

namespace bmp280 {

struct Reading {
    float temperature_c;
    float pressure_pa;
    float altitude_m;
};

// WHO_AM_I(chip id) check + one-time calibration coefficient read.
bool init();

// Triggers a forced-mode conversion and blocks (bounded) until it's ready,
// then reads + compensates it. Call at a reduced rate (e.g. ~10 Hz) from
// the main loop -- a single call takes a few ms, unlike the IMU read.
Reading read();

}  // namespace bmp280
