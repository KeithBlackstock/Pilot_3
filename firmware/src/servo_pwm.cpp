#include "servo_pwm.hpp"
#include "regs.hpp"

using namespace reg;

namespace {

constexpr uint32_t kNibble = 0xF;

constexpr uint16_t kNeutralUs = 1500;
constexpr uint16_t kMinValidUs = 900;
constexpr uint16_t kMaxValidUs = 2100;

// PSC=7 -> 8 MHz / 8 = 1 MHz counter (1 tick = 1 us).
// ARR=19999 -> 20000 ticks = 20 ms period (50 Hz).
constexpr uint32_t kPrescaler = 7;
constexpr uint32_t kAutoReload = 19999;

uint16_t clamp_or_neutral(uint16_t pulse_us) {
    if (pulse_us < kMinValidUs || pulse_us > kMaxValidUs) return kNeutralUs;
    return pulse_us;
}

}  // namespace

namespace servo_pwm {

void init() {
    mmio(RCC_APB2ENR) |= RCC_APB2ENR_IOPAEN;
    { volatile uint32_t sync = mmio(RCC_APB2ENR); (void)sync; }
    mmio(RCC_APB1ENR) |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
    { volatile uint32_t sync = mmio(RCC_APB1ENR); (void)sync; }

    // PA1 (TIM2_CH2) and PA6 (TIM3_CH1), both AF push-pull, 50 MHz. Both
    // live in GPIOA_CRL -- PA15's config (GPIOA_CRH) is never touched.
    uint32_t crl = mmio(GPIOA_CRL);
    crl &= ~((kNibble << 4) | (kNibble << 24));   // PA1, PA6 nibbles
    crl |= (PIN_AF_PP_50 << 4) | (PIN_AF_PP_50 << 24);
    mmio(GPIOA_CRL) = crl;

    // TIM2 (PWM_S4, channel 2): PWM mode 1, preload enabled, start at
    // neutral so the servo never sees an all-zero/no-pulse signal.
    mmio(TIM2_BASE + TIM_PSC_OFFSET) = kPrescaler;
    mmio(TIM2_BASE + TIM_ARR_OFFSET) = kAutoReload;
    mmio(TIM2_BASE + TIM_CCMR1_OFFSET) =
        (mmio(TIM2_BASE + TIM_CCMR1_OFFSET) & ~TIM_CCMR1_CC2S_MASK) |
        TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
    mmio(TIM2_BASE + TIM_CCR2_OFFSET) = kNeutralUs;
    mmio(TIM2_BASE + TIM_CCER_OFFSET) |= TIM_CCER_CC2E;
    mmio(TIM2_BASE + TIM_CR1_OFFSET) = TIM_CR1_ARPE | TIM_CR1_CEN;

    // TIM3 (PWM_S1, channel 1): same setup.
    mmio(TIM3_BASE + TIM_PSC_OFFSET) = kPrescaler;
    mmio(TIM3_BASE + TIM_ARR_OFFSET) = kAutoReload;
    mmio(TIM3_BASE + TIM_CCMR1_OFFSET) =
        (mmio(TIM3_BASE + TIM_CCMR1_OFFSET) & ~TIM_CCMR1_CC1S_MASK) |
        TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
    mmio(TIM3_BASE + TIM_CCR1_OFFSET) = kNeutralUs;
    mmio(TIM3_BASE + TIM_CCER_OFFSET) |= TIM_CCER_CC1E;
    mmio(TIM3_BASE + TIM_CR1_OFFSET) = TIM_CR1_ARPE | TIM_CR1_CEN;
}

uint16_t set_s1_us(uint16_t pulse_us) {
    uint16_t applied = clamp_or_neutral(pulse_us);
    mmio(TIM3_BASE + TIM_CCR1_OFFSET) = applied;
    return applied;
}

uint16_t set_s4_us(uint16_t pulse_us) {
    uint16_t applied = clamp_or_neutral(pulse_us);
    mmio(TIM2_BASE + TIM_CCR2_OFFSET) = applied;
    return applied;
}

}  // namespace servo_pwm
