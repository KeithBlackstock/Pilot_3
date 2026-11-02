// Register map for FC_bluepill (GD32F303CCT6), ported from the addresses
// already validated over SWD by pilot3_swd_probe.py.
//
// PA15 CONSTRAINT -- READ BEFORE EDITING:
// PA15 is shorted to GND on this board (layout mistake). It must stay in its
// reset state (JTDI, floating input w/ pull-up -- safe against the short);
// configuring it as an output and driving it high would be a direct short.
// PA15's config nibble is bits 28-31 of GPIOA_CRH. The rule is narrow: never
// change those bits. Other GPIOA_CRH pins (PA8-PA12) and AFIO are fair game,
// but GPIOA_CRH must only be modified through gpioa_crh_modify() below, which
// refuses at compile time any mask that touches the PA15 nibble.
#pragma once
#include <cstdint>

namespace reg {

inline volatile uint32_t& mmio(uint32_t addr) {
    return *reinterpret_cast<volatile uint32_t*>(addr);
}

// ---- RCC ----
constexpr uint32_t RCC_BASE    = 0x40021000;
constexpr uint32_t RCC_CR      = RCC_BASE + 0x00;
constexpr uint32_t RCC_APB2ENR = RCC_BASE + 0x18;
constexpr uint32_t RCC_APB1ENR = RCC_BASE + 0x1C;

constexpr uint32_t RCC_APB2ENR_IOPAEN = 1u << 2;
constexpr uint32_t RCC_APB2ENR_IOPBEN = 1u << 3;
constexpr uint32_t RCC_APB2ENR_USART1EN = 1u << 14;
constexpr uint32_t RCC_APB1ENR_TIM2EN  = 1u << 0;
constexpr uint32_t RCC_APB1ENR_TIM3EN  = 1u << 1;
constexpr uint32_t RCC_APB1ENR_SPI2EN  = 1u << 14;
constexpr uint32_t RCC_APB1ENR_USART3EN = 1u << 18;
constexpr uint32_t RCC_APB1ENR_I2C1EN  = 1u << 21;

// ---- GPIOA ---- (CRH: use gpioa_crh_modify() only -- see PA15 CONSTRAINT)
constexpr uint32_t GPIOA_BASE = 0x40010800;
constexpr uint32_t GPIOA_CRL  = GPIOA_BASE + 0x00;   // pins 0-7 config
constexpr uint32_t GPIOA_CRH  = GPIOA_BASE + 0x04;   // pins 8-15 config

constexpr uint32_t GPIOA_CRH_PA15_MASK = 0xFu << 28;

// Read-modify-write of GPIOA_CRH that cannot touch PA15's nibble.
template <uint32_t ClearMask, uint32_t SetBits>
inline void gpioa_crh_modify() {
    static_assert(((ClearMask | SetBits) & GPIOA_CRH_PA15_MASK) == 0,
                  "PA15 is grounded on this board: do not modify GPIOA_CRH bits 28-31");
    uint32_t v = mmio(GPIOA_CRH);
    v = (v & ~ClearMask) | SetBits;
    mmio(GPIOA_CRH) = v;
}

// ---- GPIOB ----
constexpr uint32_t GPIOB_BASE = 0x40010C00;
constexpr uint32_t GPIOB_CRL  = GPIOB_BASE + 0x00;   // pins 0-7 config
constexpr uint32_t GPIOB_CRH  = GPIOB_BASE + 0x04;   // pins 8-15 config
constexpr uint32_t GPIOB_BSRR = GPIOB_BASE + 0x10;

// GPIO CRL/CRH config nibbles (F1-style MODE+CNF), same encoding the
// probe script uses.
constexpr uint32_t PIN_OUT_PP_50 = 0x3;   // general purpose output push-pull, 50 MHz
constexpr uint32_t PIN_AF_PP_50  = 0xB;   // alternate function push-pull, 50 MHz
constexpr uint32_t PIN_IN_FLOAT  = 0x4;   // floating input
constexpr uint32_t PIN_AF_OD_50  = 0xF;   // alternate function open-drain, 50 MHz

// ---- SPI2 ----
constexpr uint32_t SPI2_BASE = 0x40003800;
constexpr uint32_t SPI2_CR1  = SPI2_BASE + 0x00;
constexpr uint32_t SPI2_SR   = SPI2_BASE + 0x08;
constexpr uint32_t SPI2_DR   = SPI2_BASE + 0x0C;

constexpr uint32_t SPI2_SR_RXNE = 1u << 0;
constexpr uint32_t SPI2_SR_TXE  = 1u << 1;
constexpr uint32_t SPI2_SR_BSY  = 1u << 7;

// ---- I2C1 ----
constexpr uint32_t I2C1_BASE   = 0x40005400;
constexpr uint32_t I2C1_CR1    = I2C1_BASE + 0x00;
constexpr uint32_t I2C1_CR2    = I2C1_BASE + 0x04;
constexpr uint32_t I2C1_DR     = I2C1_BASE + 0x10;
constexpr uint32_t I2C1_SR1    = I2C1_BASE + 0x14;
constexpr uint32_t I2C1_SR2    = I2C1_BASE + 0x18;
constexpr uint32_t I2C1_CCR    = I2C1_BASE + 0x1C;
constexpr uint32_t I2C1_TRISE  = I2C1_BASE + 0x20;

constexpr uint32_t I2C1_CR1_PE    = 1u << 0;
constexpr uint32_t I2C1_CR1_ACK   = 1u << 10;
constexpr uint32_t I2C1_CR1_STOP  = 1u << 9;
constexpr uint32_t I2C1_CR1_START = 1u << 8;
constexpr uint32_t I2C1_CR1_SWRST = 1u << 15;

constexpr uint32_t I2C1_SR1_SB   = 1u << 0;
constexpr uint32_t I2C1_SR1_ADDR = 1u << 1;
constexpr uint32_t I2C1_SR1_BTF  = 1u << 2;
constexpr uint32_t I2C1_SR1_RXNE = 1u << 6;
constexpr uint32_t I2C1_SR1_TXE  = 1u << 7;
constexpr uint32_t I2C1_SR1_AF   = 1u << 10;   // acknowledge failure (address NACK)

constexpr uint32_t I2C1_SR2_BUSY = 1u << 1;

// ---- USART3 ---- (PB10=TX, unused/unconfigured here; PB11=RX, IBUS input)
constexpr uint32_t USART3_BASE = 0x40004800;
constexpr uint32_t USART3_SR   = USART3_BASE + 0x00;
constexpr uint32_t USART3_DR   = USART3_BASE + 0x04;
constexpr uint32_t USART3_BRR  = USART3_BASE + 0x08;
constexpr uint32_t USART3_CR1  = USART3_BASE + 0x0C;

constexpr uint32_t USART3_SR_RXNE = 1u << 5;
constexpr uint32_t USART3_CR1_RE     = 1u << 2;
constexpr uint32_t USART3_CR1_RXNEIE = 1u << 5;
constexpr uint32_t USART3_CR1_UE     = 1u << 13;

// ---- USART1 ---- (PA10=RX, GPS UBX input; PA9=TX unused so far)
// USART1 is an APB2 peripheral (unlike USART2/3 on APB1). PA10 needs no
// GPIO config: its reset state is floating input, which is what an RX pin
// wants. PA9 (TX) would need an AF push-pull nibble in GPIOA_CRH, set via
// gpioa_crh_modify<>() (PA15 CONSTRAINT at the top of this file).
constexpr uint32_t USART1_BASE = 0x40013800;
constexpr uint32_t USART1_SR   = USART1_BASE + 0x00;
constexpr uint32_t USART1_DR   = USART1_BASE + 0x04;
constexpr uint32_t USART1_BRR  = USART1_BASE + 0x08;
constexpr uint32_t USART1_CR1  = USART1_BASE + 0x0C;

constexpr uint32_t USART1_SR_RXNE = 1u << 5;
constexpr uint32_t USART1_CR1_RE     = 1u << 2;
constexpr uint32_t USART1_CR1_RXNEIE = 1u << 5;
constexpr uint32_t USART1_CR1_UE     = 1u << 13;

// ---- TIM2 (PWM_S4 / PA1, channel 2) and TIM3 (PWM_S1 / PA6, channel 1) ----
// Standard STM32F1-family general-purpose timer layout.
constexpr uint32_t TIM2_BASE = 0x40000000;
constexpr uint32_t TIM3_BASE = 0x40000400;

constexpr uint32_t TIM_CR1_OFFSET   = 0x00;
constexpr uint32_t TIM_CCMR1_OFFSET = 0x18;
constexpr uint32_t TIM_CCER_OFFSET  = 0x20;
constexpr uint32_t TIM_PSC_OFFSET   = 0x28;
constexpr uint32_t TIM_ARR_OFFSET   = 0x2C;
constexpr uint32_t TIM_CCR1_OFFSET  = 0x34;
constexpr uint32_t TIM_CCR2_OFFSET  = 0x38;

constexpr uint32_t TIM_CR1_CEN  = 1u << 0;
constexpr uint32_t TIM_CR1_ARPE = 1u << 7;

// PWM mode 1, channel 1: OC1M[2:0] = bits 6:4 = 110, OC1PE = bit 3.
constexpr uint32_t TIM_CCMR1_OC1PE     = 1u << 3;
constexpr uint32_t TIM_CCMR1_OC1M_PWM1 = 0x6u << 4;
constexpr uint32_t TIM_CCMR1_CC1S_MASK = 0x3u << 0;  // must be 00 (output) for OC1M to apply

// PWM mode 1, channel 2: OC2M[2:0] = bits 14:12 = 110, OC2PE = bit 11.
constexpr uint32_t TIM_CCMR1_OC2PE     = 1u << 11;
constexpr uint32_t TIM_CCMR1_OC2M_PWM1 = 0x6u << 12;
constexpr uint32_t TIM_CCMR1_CC2S_MASK = 0x3u << 8;  // must be 00 (output) for OC2M to apply

constexpr uint32_t TIM_CCER_CC1E = 1u << 0;
constexpr uint32_t TIM_CCER_CC2E = 1u << 4;

// ---- NVIC (ARMv7-M core peripheral) ----
// USART3 is IRQ39 on this family (standard STM32F1/GD32F30x vector layout);
// IRQ39 falls in ISER1 (covers IRQ32-63), bit (39-32)=7.
// USART1 is IRQ37 in the same layout -> ISER1 bit (37-32)=5.
constexpr uint32_t NVIC_ISER1 = 0xE000E104;
constexpr uint32_t NVIC_ISER1_USART3 = 1u << 7;
constexpr uint32_t NVIC_ISER1_USART1 = 1u << 5;

// ---- SysTick (ARMv7-M core peripheral, not vendor-specific) ----
constexpr uint32_t SYST_CSR = 0xE000E010;
constexpr uint32_t SYST_RVR = 0xE000E014;
constexpr uint32_t SYST_CVR = 0xE000E018;

constexpr uint32_t SYST_CSR_ENABLE    = 1u << 0;
constexpr uint32_t SYST_CSR_CLKSOURCE = 1u << 2;  // processor clock, no /8
constexpr uint32_t SYST_CSR_COUNTFLAG = 1u << 16;

}  // namespace reg
