// Standard analog-servo PWM output on PWM_S1 (PA6, TIM3_CH1) and PWM_S4
// (PA1, TIM2_CH2) -- pin mapping confirmed against the actual KiCad project,
// not guessed. Both are used in their default, non-remapped timer position,
// so AFIO is never touched. Both pins live in GPIOA_CRL, a register
// entirely separate from GPIOA_CRH where PA15 (grounded on this board)
// lives -- see the PA15 CONSTRAINT comment in regs.hpp.
//
// 50 Hz frame (20 ms period), 1000-2000 us pulse width -- the same
// convention raw iBus channel values already use, so a channel value in
// microseconds maps directly onto the compare register with no rescaling.
#pragma once
#include <cstdint>

namespace servo_pwm {

void init();

// Pulse width in microseconds. A value of 0 (channel never updated) or
// anything outside roughly [900, 2100] is treated as invalid input and
// held at neutral (1500 us) instead of passed through -- a physical servo
// should never see an out-of-spec or all-zero signal. Returns the value
// actually applied (post-clamp), for telemetry.
uint16_t set_s1_us(uint16_t pulse_us);
uint16_t set_s4_us(uint16_t pulse_us);

}  // namespace servo_pwm
