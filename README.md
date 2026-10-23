# Pilot_3

Bare-metal C++ firmware (no HAL) for a "blue pill"-style flight-controller board built around a GD32F303CCT6.

**Status: bench-verified.** Valid RX, IMU, baro, and GPS readouts have been confirmed on hardware.

## Hardware

- IMU: ICM-42688-P on SPI2
- Baro: BMP280 on I2C1
- RC input: FlySky iBus on USART3
- GPS: u-blox M10, UBX-NAV-PVT on USART1
- Outputs: TIM2/TIM3 PWM, 50 Hz, 1000-2000 us
  - PWM_S1 (PA6, TIM3_CH1) and PWM_S4 (PA1, TIM2_CH2): ruddervator servos
  - PWM_S2 (PA7, TIM3_CH2): brushless ESC

## Control architecture

A 3-channel airplane with a V-tail (ruddervators) and a single motor. Control is currently direct RC-to-output; there is no stabilization loop.

| iBus channel | Input | Output |
|---|---|---|
| CH1 | Elevator | Ruddervator mix (S1, S4) |
| CH2 | Rudder | Ruddervator mix (S1, S4) |
| CH3 | Throttle | ESC (S2) |

Ruddervator mix (offsets in us around 1500, clamped to 1000-2000):

- S1 = 1500 - elevator + rudder
- S4 = 1500 - elevator - rudder

The servos are mounted mirrored, so the signs are not interchangeable between S1 and S4. They were set empirically on the bench.

Failsafe behavior:

- Servo inputs that are invalid (0 or outside 900-2100 us) count as centered (1500 us).
- ESC input that is invalid (0 or outside 1000-2000 us) fails to 1000 us, motor off, including at boot before the first iBus frame.
- If no valid iBus frame arrives for ~200 ms, the motor is cut to 1000 us; it resumes when frames return.

### Attitude loop (CH6 stabilize mode)

CH6 (Mode) > 1500 enables an accelerometer-only wing-leveler (`firmware/src/attitude_level.hpp`):

- **Input:** the raw accel-derived pitch and roll angles. No gyro rate, no fused filter output, no integrator.
- **Control law:** proportional only. Each axis outputs `-sign * Kp * angle` (4 us per degree), limited to +-250 us. Pitch acts through the elevator channel and roll through the rudder channel, since in this 3CH scheme yaw induces roll.
- **Pilot override:** if either stick (CH1 or CH2) is outside a +-30 us deadband, the loop lets go of both axes and the pilot's commands pass straight through to the ruddervator mix. With both sticks centered, it drives both axes toward level.
- **Off / failsafe:** below 1500, or with CH6 missing or invalid, the sticks always pass straight through. If the RX goes stale while the mode is on, both axes level (the motor is cut separately).

The design prioritizes testability over ideal function. Being stateless, proportional-only and driven by one raw sensor, it is trivial to reason about and to verify on the bench: tilt the airframe and watch the surfaces oppose it. The cost is real. Raw accel angles are corrupted by sustained acceleration (turns, climbs) and vibration, and with no rate damping a higher gain could oscillate. A fused-angle, rate-damped (P-D) loop would fly better and is the natural next step once the signs and basic behavior are confirmed.

The gain (`kPitchKp`, `kRollKp`) is a conservative first guess. The feedback signs (`kPitchSign`, `kRollSign`) are bench-verified on the airframe: surfaces oppose a pitch or roll disturbance.

### Yaw induces roll

In this 3CH scheme there are no ailerons, so rudder (yaw) input is what induces roll. For control-input purposes, treat the CH2 "rudder" stick as the roll control: yaw and roll are coupled, and any future stabilization or mixing should regard it that way.

## Verified on the bench

- iBus RX: valid frames, zero bad frames, all channels decoding
- IMU: roll/pitch from the complementary filter respond correctly to tilt
- Baro: pressure/temperature sane, tracks real altitude change
- GPS: 3D fix, 0 bad UBX frames
- Servos: ruddervator response confirmed correct for both axes
- ESC/motor: throttle on CH3 drives the motor correctly

## Known limitations

- Not flight-ready: the leveler is untuned, there is no gyro bias calibration, altitude uses a fixed sea-level constant.

## Layout

- `firmware/` - firmware source (`src/`), linker script, startup, `build.bat` (Windows) and `Makefile`
- `tools/` - OpenOCD-based telemetry monitor and web dashboard
- `circuit_reference/` - board schematic (KiCanvas) and netlist
- `pilot3_swd_probe.py` - SWD register probe used for initial bring-up

## Board errata

PA15 is shorted to GND on this board. Never configure it as an output; see the PA15 CONSTRAINT in `firmware/src/regs.hpp`.
