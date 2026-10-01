#include "spi2.hpp"
#include "regs.hpp"

using namespace reg;

namespace {
constexpr uint32_t kNibble = 0xF;
}  // namespace

namespace spi2 {

void init() {
    // Clocks: GPIOB + SPI2. No AFIO clock needed (no remapping used).
    // The dummy read-back after each enable is required on real silicon --
    // the peripheral clock enable write can still be in flight on the bus
    // when the next instruction tries to touch that peripheral's registers,
    // which silently drops the access. The probe script never hit this
    // because each telnet round-trip was slow enough to mask it.
    mmio(RCC_APB2ENR) |= RCC_APB2ENR_IOPBEN;
    { volatile uint32_t sync = mmio(RCC_APB2ENR); (void)sync; }
    mmio(RCC_APB1ENR) |= RCC_APB1ENR_SPI2EN;
    { volatile uint32_t sync = mmio(RCC_APB1ENR); (void)sync; }

    // PB5 = CS. Latch the output high BEFORE switching the pin to output,
    // same reasoning as the probe: after reset ODR is 0, so configuring the
    // pin first would briefly drive CS low.
    mmio(GPIOB_BSRR) = 1u << 5;
    mmio(GPIOB_CRL) = (mmio(GPIOB_CRL) & ~(kNibble << 20)) | (PIN_OUT_PP_50 << 20);

    // PB13 SCK = AF PP, PB14 MISO = floating input, PB15 MOSI = AF PP.
    uint32_t crh = mmio(GPIOB_CRH);
    crh &= ~((kNibble << 20) | (kNibble << 24) | (kNibble << 28));
    crh |= (PIN_AF_PP_50 << 20) | (PIN_IN_FLOAT << 24) | (PIN_AF_PP_50 << 28);
    mmio(GPIOB_CRH) = crh;

    // CR1: master, SPE, SSM+SSI (CS handled as GPIO), BR = fPCLK/256,
    // mode 0 (CPOL=0 CPHA=0), MSB first, 8-bit -- same as the probe.
    mmio(SPI2_CR1) = (1u << 2) | (1u << 6) | (7u << 3) | (1u << 8) | (1u << 9);
}

uint8_t xfer(uint8_t byte) {
    while (!(mmio(SPI2_SR) & SPI2_SR_TXE)) {
    }
    mmio(SPI2_DR) = byte;
    while (!(mmio(SPI2_SR) & SPI2_SR_RXNE)) {
    }
    return static_cast<uint8_t>(mmio(SPI2_DR) & 0xFF);
}

void cs_low() { mmio(GPIOB_BSRR) = 1u << (5 + 16); }
void cs_high() { mmio(GPIOB_BSRR) = 1u << 5; }

}  // namespace spi2
