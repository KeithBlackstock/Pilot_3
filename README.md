# Pilot_3

Bare-metal C++ firmware (no HAL) for a "blue pill"-style flight-controller board built around a GD32F303CCT6.

**Status: bench-verified.** Valid RX, IMU, baro, and GPS readouts have been confirmed on hardware.

## Hardware

- IMU: ICM-42688-P on SPI2
- Baro: BMP280 on I2C1
- RC input: FlySky iBus on USART3
- GPS: u-blox M10, UBX-NAV-PVT on USART1
- Bluetooth: CC41-A (BLE-only HM-10 clone) on USART2, 9600 8N1 -- wireless dashboard (below)
- Outputs: TIM2/TIM3 PWM, 50 Hz, 1000-2000 us
  - PWM_S1 (PA6, TIM3_CH1) and PWM_S4 (PA1, TIM2_CH2): ruddervator servos
  - PWM_S2 (PA7, TIM3_CH2): brushless ESC

## Control architecture

A 3-channel airplane with a V-tail (ruddervators) and a single motor.
| iBus channel | Input | Output |
|---|---|---|
| CH1 | Elevator | Ruddervator mix (S1, S4) |
| CH2 | Rudder | Ruddervator mix (S1, S4) |
| CH3 | Throttle | ESC (S2) |
| CH5 | Cruise arm | ESC (S2) speed hold (below) |

Ruddervator mix (offsets in us around 1500, clamped to 1000-2000):

- S1 = 1500 - elevator + rudder
- S4 = 1500 - elevator - rudder

The servos are mounted mirrored, so the signs are not interchangeable between S1 and S4. They were set empirically on the bench.

Failsafe behavior:

- Servo inputs that are invalid (0 or outside 900-2100 us) count as centered (1500 us).
- ESC input that is invalid (0 or outside 1000-2000 us) fails to 1000 us, motor off, including at boot before the first iBus frame.
- If no valid iBus frame arrives for ~200 ms, the motor is cut to 1000 us; it resumes when frames return.

### Attitude loop (CH6 stabilize mode)

CH6 is a 3-way switch: low (<= 1300) direct, mid (> 1300) stabilize, high (> 1700) PATH (see below; falls back to plain stabilize unless waypoints are loaded and GPS fix, mag and RX are live). Stabilize enables a stateless P-D angle loop (`firmware/src/attitude_level.hpp`). The elevator (CH1) and rudder (CH2) sticks become pitch and roll angle requests, up to `max_pitch_deg` / `max_roll_deg` at full stick, and the output replaces the stick offset ahead of the ruddervator mix. In direct mode the sticks pass straight through; on a stale RX both axes request level.

- **Control law:** `-sign * (Kp * (angle - request) + Kd * gyro_rate)`, limited to +-`max_output_us`. P uses the complementary-filter angle, D uses the gyro rate and simply opposes angular velocity. A fixed one-pole low-pass (`kRateLpfHz` in `attitude_level.hpp`, 40 Hz, not dashboard-tunable) smooths the gyro rate for the D term only; the angle estimate keeps the raw gyro.
- **Tuning:** gains, directions, angle limits, deadband and authority are live-editable in the dashboard (below). Defaults in `kDefaults`: Kp 4 us/deg, Kd 0.1 us per deg/s, 25 degrees, +-30 us deadband, +-250 us authority.
- **Gyro bias:** measured on every activation of stabilize mode (rising edge of CH6); there is no calibration at power-up, so the bias is zero until the first activation. The cal is non-blocking -- stabilize engages immediately on the old bias, and a window that isn't quiet (motion/vibration, or an implausible offset) is rejected so the previous bias keeps running.
- **Limits, by design:** simplicity is chosen over ideal function so it is easy to test on the bench. The accel angle is corrupted by sustained acceleration and vibration.

### PATH (waypoint steering)

A roll-only navigation overlay on stabilize (`firmware/src/path_mode.hpp`), modelled on the PATH mode in the author's INAV fork (`path_mode.c`). CH6 high engages it, but only while waypoints are loaded and a GPS 3D fix, the mag and the RX are all live -- otherwise the high position is silently plain stabilize. PATH flies the route in order by banking toward each waypoint, while the pilot keeps pitch and throttle.

- **Steering:** `bank_deg = k_path * heading_error_deg` (default `k_path` 0.5). The result is a roll angle request that replaces the CH1 roll stick offset ahead of `attitude_level::roll_angle`, so the existing angle loop does the flying and no separate inner loop exists; the clamp to `max_roll_deg` happens there.
- **Route:** switching PATH on always restarts at waypoint 1. A waypoint counts as reached inside `capture_radius_m` (default 10 m). At the end of the route, PATH holds wings level.
- **Pilot abort:** the roll stick (CH1) past `abort_deadband_us` (default +-100 us) instantly reverts to plain stabilize -- the natural "pull it back" reflex, no switch needed. The abort latches while CH6 stays high (so an availability glitch cannot re-arm it); moving CH6 off high re-arms, and the next rise restarts the route at waypoint 1.
- **Heading:** QMC5883L compass in the HGLRC M100 Pro GPS module, on I2C1 via the GPS connector (SCL/SDA, pins 5 and 6), shared with the BMP280. Address 0x0D. The driver, six-pose hard-iron/scale calibration (dashboard "Compass Calibration" button, with mirrored-module detection) and the heading offset to true north (`declination_deg`, live-writable and saveable from the dashboard) are implemented. Still missing: tilt compensation from the existing roll/pitch. The heading sign and module mounting orientation are not bench-verified yet -- verify before flight.
- **Waypoints:** up to 16 as `lat_e7`, `lon_e7`. The dashboard's Compile & Flash bakes the map's route into a generated `firmware/src/waypoints.hpp`, so the flashed firmware boots with the route loaded and it survives power cycles. There is no live over-SWD upload; changing the route means reflashing.

### Yaw induces roll

In this 3CH scheme there are no ailerons, so rudder (yaw) input is what induces roll. For control-input purposes, treat the CH2 "rudder" stick as the roll control: yaw and roll are coupled, and any future stabilization or mixing should regard it that way.

### Cruise mode (CH5 speed hold)

A proportional speed controller on the motor (`firmware/src/cruise_mode.hpp`). CH5 high hands the throttle to it, replacing the CH3 stick at the ESC output; CH5 low (or an invalid/absent CH5) passes the stick straight through, and the RX-stale motor cut applies either way. Everything else -- sticks, mix, stabilize, PATH -- is untouched; cruise only owns the motor.

- **Engagement is immediate:** there is no arm/launch sequencing. The moment CH5 goes high the control law is in charge, so on the ground at standstill (speed 0, far below target) the output rails to `max_throttle_us`. That is deliberate -- the mode is meant to be switched on at the throw -- but it means CH5-high on the ground is FULL POWER. Flip it only at launch.
- **Feedback:** the derived 3-D speed (`speed3d_mps` in telemetry: GPS groundspeed + baro-derived climb in quadrature). Ground-relative, not airspeed: in wind the loop holds constant ground speed, so true airspeed rises downwind and sags upwind.
- **Control law:** `throttle_us = min_throttle_us + kp * (target_mps - speed3d_mps)`, clamped to [`min_throttle_us`, `max_throttle_us`]. P-only with a hard clamp: no integrator, so no anti-windup. The price is droop -- at zero error the output sits at min throttle, so the settled speed is always somewhat below target; raise kp to close the gap.
- **GPS degradation:** with no fix or a stale NAV-PVT stream the speed feedback is frozen and the P output is meaningless, so cruise drops to `min_throttle_us` (a glide) until it returns or the pilot takes over.
- **Tuning:** target speed, kp and min/max throttle are live-editable on the dashboard's cruise card (range-checked there and again in the firmware's `snapshot()`). "Save as firmware defaults" rewrites `CRUISE-DEFAULTS-LINE` in `cruise_mode.hpp`. Defaults: target 12 m/s, kp 50 us per m/s, min 1100 us, max 1600 us.

## Dashboard and configurator

`pilot3.bat` (prompts for the link: 1 = SWD, 2 = BLE) starts OpenOCD and a browser dashboard (`tools/`) with live telemetry plus:

- **Attitude leveler card:** shows whether the angle loop is on, and edits the gains (Kp, Kd), directions, max pitch/roll angles, deadband and authority limit. "Apply live" writes the values into the running FC's RAM over SWD (range-checked in the dashboard and again in the firmware; lost on reset). "Save as firmware defaults" rewrites `kDefaults` in `attitude_level.hpp` so the next build boots with them.
- **Cruise mode card:** shows whether CH5 has engaged the speed hold, and edits the target speed, Kp and min/max throttle, with the same apply-live / save-as-defaults flow (defaults in `cruise_mode.hpp`).
- **Compile & Flash:** runs `firmware/build.bat`, flashes the ELF through the running OpenOCD (verify + reset), then re-applies any live values and reads the config back. Telemetry pauses while it runs. It resets the board, so remove the prop first.
- **Waypoint planner:** an OpenStreetMap map (Leaflet, needs internet) for placing, dragging and deleting up to 16 waypoints, with leg distances and the live aircraft position and course. Compile & Flash bakes the route into a generated `firmware/src/waypoints.hpp`, so the flashed firmware boots with the route loaded (see PATH above).

The dashboard's WebSocket only accepts connections from the dashboard page itself (Origin check), since it can flash the board.

### Bluetooth (wireless) dashboard

A CC41-A module (a BLE-only HM-10 clone, sold as "ZS-040") on the UART2 connector (module RX <- PA2, module TX -> PA3, 3.3 V logic) carries the same dashboard over the air -- no ST-Link, no pairing, no COM port. Choose option 2 in `pilot3.bat`. Everything works wirelessly except Compile & Flash, which spawns OpenOCD on demand and still needs the ST-Link attached at that moment.

- **Module identity:** the board-marked ZS-040 turned out to be a CC41-A (CC2541, Bluetooth 4.0 LE only -- `AT+VERSION?` reports "Firmware V3.0.6, Bluetooth V4.0 LE"). There is no classic SPP/RFCOMM in it, so the original COM-port plan was impossible; the dashboard talks BLE GATT instead via `tools/pilot3_ble_link.py` (needs `pip install --user bleak`). Serial service is `0xDFB0` with one characteristic `0xDFB1` (read/notify/write). The module advertises as "Pilot3" (address `00:15:87:10:9E:53`); Windows discovery often doesn't surface the name for this clone, so the launcher connects by address.
- **9600 baud, 5 Hz:** the clone's UART bridge is hard-locked to 9600 baud -- it accepts `AT+BAUD4` and replies `+BAUD=4`, but the bridge never actually leaves 9600, and its AT interface is flaky (garbage replies at every baud; verified extensively over a SWD register bridge). So the firmware runs USART2 at 9600 8N1 and pushes telemetry at ~5 Hz: a frame is ~130 bytes (135 ms on the wire), and 5 Hz uses ~68% of the line. This is a module limitation, not a design choice -- a genuine HM-10 (or a classic HC-05, using the dashboard's `--serial` mode) would allow 115200 and 10 Hz again with a one-line BRR change in `bt.cpp`.
- **How it works:** the firmware (`firmware/src/bt.cpp`) pushes the same 132-byte `Telemetry` struct the SWD dashboard polls, as framed packets, and answers peek/poke requests for 32-bit SRAM words -- the exact primitive every dashboard control is built on, so live config, baro zero, compass calibration and waypoint readback all work unchanged. Frames are `A5 5A | type | len | payload | crc16` (CRC16-CCITT); a CRC failure is dropped, never executed, and reads/writes are bounded to SRAM. Notifications arrive in ~20-byte BLE chunks; the host-side `FrameParser` (shared with `tools/pilot3_serial_link.py`, which has a codec self-test: run it directly) reassembles them. Link check: `python tools/pilot3_ble_link.py`.
- **AT notes (if you ever re-probe it):** AT is answered at 9600 while the module is unconnected; set-args take no `=` (`AT+NAMEPilot3`); `AT+RESET` reboots it. Holding the button at power-up drops this clone into a 38400-baud command mode. While unconnected it interprets the FC's telemetry stream as AT garbage and floods `ERROR` back -- harmless (the firmware just counts them in `g_bt_stats.crc_bad`), and it stops the moment a BLE central connects.
- **Independence:** the SWD dashboard (option 1 in `pilot3.bat`) is untouched by all of this and works exactly as before.

## Verified on the bench

- iBus RX: valid frames, zero bad frames, all channels decoding
- IMU: roll/pitch from the complementary filter respond correctly to tilt
- Baro: pressure/temperature sane, tracks real altitude change
- GPS: 3D fix, 0 bad UBX frames
- Servos: ruddervator response confirmed correct for both axes
- ESC/motor: throttle on CH3 drives the motor correctly
- Cruise: CH5 engages the speed hold immediately (full power at standstill, so flip it at the throw)

## Layout

- `firmware/` - firmware source (`src/`), linker script, startup, `build.bat` (Windows) and `Makefile`
- `tools/` - OpenOCD-based telemetry monitor, web dashboard, and the Bluetooth links (`pilot3_ble_link.py` for the CC41-A BLE module, `pilot3_serial_link.py` for a classic SPP module)
- `pilot3.bat` - unified dashboard launcher (prompts: 1 = SWD, 2 = BLE)
- `circuit_reference/` - board schematic (KiCanvas) and netlist
- `pilot3_swd_probe.py` - SWD register probe used for initial bring-up

## Board errata

PA15 is shorted to GND on this board. Never configure it as an output; see the PA15 CONSTRAINT in `firmware/src/regs.hpp`.
