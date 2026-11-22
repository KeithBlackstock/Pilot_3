// I2C1 bring-up for the BMP280: SCL=PB6, SDA=PB7. Direct port of
// i2c1_setup()/i2c1_read_reg() from pilot3_swd_probe.py -- same standard-mode
// 100 kHz timing, same NACK/ADDR/BTF sequencing, just without the telnet
// round-trips. Single-register access only (no true multi-byte burst) --
// simpler than implementing the STM32 F1 multi-byte-read ACK/NACK edge
// cases correctly, and the BMP280 read volume here (24 calib bytes once,
// 6 data bytes at ~10 Hz) doesn't need the throughput.
#pragma once
#include <cstdint>

namespace i2c1 {

void init();
bool read_reg(uint8_t addr7, uint8_t reg, uint8_t& out);
bool write_reg(uint8_t addr7, uint8_t reg, uint8_t val);

}  // namespace i2c1
