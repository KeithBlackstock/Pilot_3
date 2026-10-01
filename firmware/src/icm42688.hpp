// ICM-42688-P driver: WHO_AM_I check + one-shot accel/gyro burst read.
//
// Full-scale ranges are left at their power-on-reset value (FS_SEL=0 for
// both accel and gyro per the datasheet) -- PWR_MGMT0/ACCEL_CONFIG0/
// GYRO_CONFIG0 are written explicitly rather than assumed, but with the
// same reset values pilot3_swd_probe.py's INT1 test already exercised
// (which measured a real 1 kHz DRDY with these registers untouched), so
// the ODR this produces is known-good even though its exact bit encoding
// isn't re-derived here. FS_SEL=0 => accel +-16g (2048 LSB/g), gyro
// +-2000 dps (16.4 LSB/dps) -- both from the ICM-42688-P datasheet.
#pragma once
#include <cstdint>

namespace icm {

struct Sample {
    float accel_g[3];
    float gyro_dps[3];
};

// Brings up SPI2 and checks WHO_AM_I. Returns false if the sensor doesn't
// answer or answers with the wrong ID -- caller should not proceed to
// read() in that case.
bool init();

// One burst SPI read of accel + gyro data registers, converted to engineering
// units. Only valid to call after a successful init().
Sample read();

}  // namespace icm
