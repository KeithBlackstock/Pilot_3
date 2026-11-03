// QMC5883L 3-axis magnetometer (on the GPS module), on I2C1 alongside the
// BMP280. Used for now as a direct magnetic heading reference for the
// dashboard's map cursor and PATH steering -- no fusion, no tilt
// compensation, no declination (PATH adds its own). Hard-iron calibration
// below; mounting orientation is still bench-TBD.
#pragma once
#include <cstdint>

namespace qmc5883 {

struct Reading {
    int16_t x, y, z;    // raw counts, datasheet axis frame (module orientation TBD)
    float heading_deg;  // atan2(y, x) wrapped to [0, 360) -- magnetic north reference
};

struct Stats {
    uint32_t reads_ok;  // full 6-byte samples delivered
    uint32_t errors;    // failed I2C transactions
};

// Hard-iron calibration, applied as (raw - offset) * scale before the
// heading is computed. Telemetry still carries raw counts -- the
// dashboard's calibration ritual needs them. Field order is mirrored in
// tools/pilot3_fc_control.py -- keep in sync. off_z is stored but unused:
// the planar ritual captures x/y only, and the heading is planar.
struct Cal {
    float off_x, off_y, off_z;  // raw counts
    float scale_x, scale_y;     // dimensionless, ~1.0; negative = mirrored module
};

// What the firmware boots with -- the dashboard's calibration save rewrites
// this one line; g_mag_cal (qmc5883.cpp) is the live RAM copy.
constexpr Cal kCalDefaults = {-807.25f, 193.15f, -241.167f, 0.890311f, -1.14052f};  // MAGCAL-DEFAULTS-LINE

// False = chip absent or wrong ID; the caller then leaves mag_* telemetry
// untouched (present flag stays 0).
bool init();

// One I2C transaction per call, like bmp280::step -- call every loop tick;
// returns true when a full sample has been assembled into `out`.
bool step(Reading& out);

Stats snapshot();

}  // namespace qmc5883
