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

- Not flight-ready: no attitude-driven control loop, no gyro bias calibration, altitude uses a fixed sea-level constant.
- No iBus staleness failsafe: if the receiver link drops, the last channel values (including throttle) are held.
- The loop does not measure its real period, so the filter's `dt` is nominal.

## Layout

- `firmware/` - firmware source (`src/`), linker script, startup, `build.bat` (Windows) and `Makefile`
- `tools/` - OpenOCD-based telemetry monitor and web dashboard
- `circuit_reference/` - board schematic (KiCanvas) and netlist
- `pilot3_swd_probe.py` - SWD register probe used for initial bring-up

## Board errata

PA15 is shorted to GND on this board. Never configure it as an output; see the PA15 CONSTRAINT in `firmware/src/regs.hpp`.
