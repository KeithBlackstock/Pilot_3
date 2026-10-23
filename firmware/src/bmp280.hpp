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

// Non-blocking forced-mode read, advanced one small I2C transaction per call.
// Call every main-loop tick. When idle, `start` begins a new cycle (trigger
// conversion, poll status, read the 6 raw bytes one per call, compensate);
// calls with start=false while idle do nothing. Returns true, filling `out`,
// only on the call that completes a reading. A cycle takes ~10 ticks, so
// request a new one at a reduced rate (e.g. ~10 Hz).
bool step(bool start, Reading& out);

}  // namespace bmp280
