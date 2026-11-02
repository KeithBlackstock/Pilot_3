// SPI2 bring-up for the ICM-42688-P: SCK=PB13, MISO=PB14, MOSI=PB15, CS=PB5.
// Bit-for-bit the same pin config and CR1 bring-up as spi2_setup()/spi2_xfer()
// in pilot3_swd_probe.py, minus the telnet round-trips.
#pragma once
#include <cstdint>

namespace spi2 {

void init();
uint8_t xfer(uint8_t byte);
void cs_low();
void cs_high();

}  // namespace spi2
