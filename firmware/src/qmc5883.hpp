// QMC5883L 3-axis magnetometer (on the GPS module), on I2C1 alongside the
// BMP280. Used for now as a direct magnetic heading reference for the
// dashboard's map cursor -- no fusion, no calibration (hard/soft iron and
// module orientation are bench-TBD), no declination.
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

// False = chip absent or wrong ID; the caller then leaves mag_* telemetry
// untouched (present flag stays 0).
bool init();

// One I2C transaction per call, like bmp280::step -- call every loop tick;
// returns true when a full sample has been assembled into `out`.
bool step(Reading& out);

Stats snapshot();

}  // namespace qmc5883
