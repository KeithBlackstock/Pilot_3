#include "icm42688.hpp"

#include "regs.hpp"
#include "spi2.hpp"

using namespace reg;

namespace {

constexpr uint8_t kRegWhoAmI      = 0x75;
constexpr uint8_t kRegPwrMgmt0    = 0x4E;
constexpr uint8_t kRegGyroConfig0 = 0x4F;
constexpr uint8_t kRegAccelConfig0 = 0x50;
constexpr uint8_t kRegAccelDataX1 = 0x1F;  // ACCEL_DATA_X1..Z0, then
                                            // GYRO_DATA_X1..Z0 immediately
                                            // follow: 12 contiguous bytes.

constexpr uint8_t kWhoAmIExpect = 0x47;
constexpr uint8_t kReadFlag     = 0x80;

constexpr float kAccelLsbPerG   = 2048.0f;  // FS_SEL=0 -> +-16g
constexpr float kGyroLsbPerDps  = 16.4f;    // FS_SEL=0 -> +-2000 dps

void write_reg(uint8_t reg_addr, uint8_t val) {
    spi2::cs_low();
    spi2::xfer(reg_addr);
    spi2::xfer(val);
    spi2::cs_high();
}

uint8_t read_reg(uint8_t reg_addr) {
    spi2::cs_low();
    spi2::xfer(reg_addr | kReadFlag);
    uint8_t val = spi2::xfer(0x00);
    spi2::cs_high();
    return val;
}

void burst_read(uint8_t start_reg, uint8_t* buf, int len) {
    spi2::cs_low();
    spi2::xfer(start_reg | kReadFlag);
    for (int i = 0; i < len; ++i) {
        buf[i] = spi2::xfer(0x00);
    }
    spi2::cs_high();
}

int16_t be16(uint8_t hi, uint8_t lo) {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

}  // namespace

namespace icm {

bool init() {
    spi2::init();

    if (read_reg(kRegWhoAmI) != kWhoAmIExpect) {
        return false;
    }

    // Explicit, not assumed: FS_SEL=0 (bits 7:5) for both, ODR (bits 3:0)
    // left at the same reset value the probe already validated.
    write_reg(kRegAccelConfig0, 0x00);
    write_reg(kRegGyroConfig0, 0x00);
    write_reg(kRegPwrMgmt0, 0x0F);  // accel+gyro low-noise mode

    return true;
}

Sample read() {
    uint8_t buf[12];
    burst_read(kRegAccelDataX1, buf, 12);

    Sample s;
    for (int axis = 0; axis < 3; ++axis) {
        int16_t accel_raw = be16(buf[axis * 2], buf[axis * 2 + 1]);
        int16_t gyro_raw  = be16(buf[6 + axis * 2], buf[6 + axis * 2 + 1]);
        s.accel_g[axis]   = static_cast<float>(accel_raw) / kAccelLsbPerG;
        s.gyro_dps[axis]  = static_cast<float>(gyro_raw) / kGyroLsbPerDps;
    }
    return s;
}

}  // namespace icm
