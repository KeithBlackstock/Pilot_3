# Pilot_3

Bare-metal C++ firmware (no HAL) for a "blue pill"-style flight-controller board built around a GD32F303CCT6.

**Status: bench-verified.** Valid RX, IMU, baro, and GPS readouts have been confirmed on hardware.

## Hardware

- IMU: ICM-42688-P on SPI2
- Baro: BMP280 on I2C1
- RC input: FlySky iBus on USART3
- GPS: u-blox M10, UBX-NAV-PVT on USART1
- WiFi: ESP-01S on USART2, 115200 8N1 -- wireless dashboard (below)
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

CH6 is a 3-way switch: low (<= 1300) direct, mid (> 1300) stabilize, high (> 1700) PATH (see below; falls back to plain stabilize unless waypoints are loaded and GPS fix, the selected heading source and RX are live). Stabilize enables a stateless P-D angle loop (`firmware/src/attitude_level.hpp`). The elevator (CH1) and rudder (CH2) sticks become pitch and roll angle requests, up to `max_pitch_deg` / `max_roll_deg` at full stick, and the output replaces the stick offset ahead of the ruddervator mix. In direct mode the sticks pass straight through; on a stale RX both axes request level.

- **Control law:** `-sign * (Kp * (angle - request) + Kd * angle_rate) + FF * normalized_stick * 500 us`, with the combined P-D + FF output limited to +-`max_output_us`. P uses the complementary-filter angle, and D damps the matching Euler angle rate rather than raw body `p`/`q`: roll uses `p + (q*sin(roll) + r*cos(roll))*tan(pitch)`, while pitch uses `q*cos(roll) - r*sin(roll)`, so D does not fight P when body rates do not change the regulated Euler angle. A fixed one-pole low-pass (`kRateLpfHz` in `attitude_level.hpp`, 40 Hz, not dashboard-tunable) smooths the rate for the D term only; the angle estimate keeps the raw gyro. FF is direct stick pass-through after the existing deadband shaping. PATH's roll-angle override deliberately omits FF; ordinary stabilize uses it after a PATH abort.
- **Tuning:** gains (including pitch/roll FF), directions, angle limits, deadband and authority are live-editable in the dashboard (below). FF defaults to 0.2 on each axis, adding 100 us at full stick; 0 disables it and 1 passes through the full normalized stick. Other defaults in `kDefaults`: Kp 4 us/deg, Kd 0.1 us per deg/s, 25 degrees, +-30 us deadband, +-250 us authority.
- **Gyro bias:** measured on every activation of stabilize mode (rising edge of CH6); there is no calibration at power-up, so the bias is zero until the first activation. The cal is non-blocking -- stabilize engages immediately on the old bias, and a window that isn't quiet (motion/vibration, or an implausible offset) is rejected so the previous bias keeps running.
- **Limits, by design:** simplicity is chosen over ideal function so it is easy to test on the bench. The accel angle is corrupted by sustained acceleration and vibration.

### PATH (waypoint steering)

A roll-only navigation overlay on stabilize (`firmware/src/path_mode.hpp`), modelled on the PATH mode in the author's INAV fork (`path_mode.c`). CH6 high engages it, but only while waypoints are loaded and a GPS 3D fix, the selected heading source and the RX are all live -- otherwise the high position is silently plain stabilize. PATH flies the route in order by banking toward each waypoint, while the pilot keeps pitch and throttle.

- **Steering:** `bank_deg = k_path * heading_error_deg` (default `k_path` 0.5). The result is a roll angle request that replaces the CH1 roll stick offset ahead of `attitude_level::roll_angle`, so the existing angle loop does the flying and no separate inner loop exists; the clamp to `max_roll_deg` happens there.
- **Route:** switching PATH on always restarts at waypoint 1. A waypoint counts as reached inside `capture_radius_m` (default 10 m). At the end of the route, PATH holds wings level.
- **Pilot abort:** the roll stick (CH1) past `abort_deadband_us` (default +-100 us) instantly reverts to plain stabilize -- the natural "pull it back" reflex, no switch needed. The abort latches while CH6 stays high (so an availability glitch cannot re-arm it); moving CH6 off high re-arms, and the next rise restarts the route at waypoint 1.
- **TIP (temporary interrupt of path):** if the bank angle itself exceeds `tip_bank_deg` (default 40 deg) while engaged -- a gust, wake, or a botched turn -- PATH suspends and plain stabilize (plus the airframe's natural stability) levels the wings. Not sticky: once roll is back under the threshold, PATH re-engages on the same leg and carries on the route -- unlike a fresh engage, it does not restart at waypoint 1.
- **Heading:** the FC's shared `heading_deg` uses the QMC5883L magnetic heading plus `declination_deg` below 5 m/s GPS ground speed, and GPS course over ground (COG), already referenced to true north, at and above 5 m/s. PATH, the dashboard, map cursor and blackbox all use this selected heading; raw magnetic heading and COG remain separately visible. The compass is in the HGLRC M100 Pro GPS module, on I2C1 via the GPS connector (SCL/SDA, pins 5 and 6), shared with the BMP280. Address 0x0D. The driver, six-pose hard-iron/scale calibration (dashboard "Compass Calibration" button, with mirrored-module detection) and the heading offset to true north (live-writable and saveable from the dashboard) are implemented. Still missing: tilt compensation from the existing roll/pitch. The heading sign and module mounting orientation are not bench-verified yet -- verify before flight.
- **Waypoints:** up to 16 as `lat_e7`, `lon_e7`. Two ways to load a route: Compile & Flash bakes the map's route into a generated `firmware/src/waypoints.hpp`, so the flashed firmware boots with it loaded and it survives power cycles; or the planner's "Push route to FC" writes it straight into `g_path_config` (RAM) over the live link -- PATH flies it immediately with no reflash, but it is lost on reboot.

### Yaw induces roll

In this 3CH scheme there are no ailerons, so rudder (yaw) input is what induces roll. For control-input purposes, treat the CH2 "rudder" stick as the roll control: yaw and roll are coupled, and any future stabilization or mixing should regard it that way.

### Cruise mode (CH5 speed hold)

A proportional speed controller on the motor (`firmware/src/cruise_mode.hpp`). CH5 high hands the throttle to it, replacing the CH3 stick at the ESC output; CH5 low (or an invalid/absent CH5) passes the stick straight through, and the RX-stale motor cut applies either way. Everything else -- sticks, mix, stabilize, PATH -- is untouched; cruise only owns the motor.

- **Engagement is immediate:** there is no arm/launch sequencing. The moment CH5 goes high the control law is in charge, so on the ground at standstill (speed 0, far below target) the output rails to `max_throttle_us`. That is deliberate -- the mode is meant to be switched on at the throw -- but it means CH5-high on the ground is FULL POWER. Flip it only at launch.
- **Feedback:** GPS horizontal ground speed (`ground_speed_mps` in telemetry). Barometric altitude and climb are still tracked but do not contribute to the speed estimate. Ground-relative, not airspeed: in wind the loop holds constant ground speed, so true airspeed rises downwind and sags upwind.
- **Control law:** `throttle_us = min_throttle_us + kp * (target_mps - ground_speed_mps)`, clamped to [`min_throttle_us`, `max_throttle_us`]. P-only with a hard clamp: no integrator, so no anti-windup. The price is droop -- at zero error the output sits at min throttle, so the settled speed is always somewhat below target; raise kp to close the gap.
- **GPS degradation:** with no fix or a stale NAV-PVT stream the speed feedback is frozen and the P output is meaningless, so cruise drops to `min_throttle_us` (a glide) until it returns or the pilot takes over.
- **Tuning:** target speed, kp and min/max throttle are live-editable on the dashboard's cruise card (range-checked there and again in the firmware's `snapshot()`). "Save as firmware defaults" rewrites `CRUISE-DEFAULTS-LINE` in `cruise_mode.hpp`. Defaults: target 12 m/s, kp 50 us per m/s, min 1100 us, max 1600 us.

## Dashboard and configurator

`pilot3.bat` (prompts for the link: 1 = SWD, 2 = WiFi) starts a browser dashboard (`tools/`) with live telemetry plus:

- **Attitude leveler card:** shows whether the angle loop is on, and edits the gains (Kp, Kd), directions, max pitch/roll angles, deadband and authority limit. "Apply live" writes the values into the running FC's RAM over SWD (range-checked in the dashboard and again in the firmware; lost on reset). "Save as firmware defaults" rewrites `kDefaults` in `attitude_level.hpp` so the next build boots with them.
- **Cruise mode card:** shows whether CH5 has engaged the speed hold, and edits the target speed, Kp and min/max throttle, with the same apply-live / save-as-defaults flow (defaults in `cruise_mode.hpp`).
- **Blackbox:** records 60 seconds of fresh telemetry (about 600 samples at 10 Hz) and overwrites `logs/pilot3_blackbox.csv` with elapsed time, sequence number, measured and target pitch/roll, magnetic heading, GPS ground speed, and barometric altitude. Targets are the final angle requests used by the attitude loop, including PATH's clamped roll override; their cells are blank in direct mode. Each capture replaces the previous log. Link drops or sample gaps are reported as a partial capture; heading is blank if the magnetometer is absent.
- **Compile & Flash:** runs `firmware/build.bat`, flashes the ELF through the running OpenOCD (verify + reset), then re-applies any live values and reads the config back. Telemetry pauses while it runs. It resets the board, so remove the prop first.
- **Waypoint planner:** an OpenStreetMap map (Leaflet, needs internet) for placing, dragging and deleting up to 16 waypoints, with leg distances and the live aircraft position and course. "Push route to FC" loads the route into RAM over the live link for immediate use (volatile -- field route-planning); Compile & Flash bakes it into a generated `firmware/src/waypoints.hpp` so the flashed firmware boots with it (see PATH above).

The dashboard's WebSocket only accepts connections from the dashboard page itself (Origin check), since it can flash the board.

### ESP-01S WiFi dashboard

An ESP-01S on USART2 (module RX <- PA2, module TX -> PA3, 3.3 V logic) carries the dashboard as a transparent TCP client. The configured module advertises the open `ESP-704796` network (`192.168.4.1`); join it from the PC, verify the PC has `192.168.4.2`, then choose option 2 in `pilot3.bat`. The PC listens on `192.168.4.2:8082`, and the ESP connects to it. Allow Python through the private-network firewall if Windows prompts. Live config, waypoints and blackbox work wirelessly; Compile & Flash still needs ST-Link. Joining this device-only WiFi network normally interrupts internet access.

The firmware (`firmware/src/bt.cpp`) pushes the same 144-byte `Telemetry` struct the SWD dashboard polls, as framed packets, and answers peek/poke requests for 32-bit SRAM words. Frames are `A5 5A | type | len | payload | crc16` (CRC16-CCITT); a CRC failure is dropped, never executed, and reads/writes are bounded to SRAM. The host-side `FrameParser` in `tools/pilot3_serial_link.py` is shared with `tools/pilot3_tcp_link.py`; both have offline self-tests. USART2 runs at 115200 8N1 with telemetry at 10 Hz; the ESP's saved UART setting must match (see `Misc Connections/README.md`).

The side-firmware and AT setup used to configure the ESP without a USB-TTL adapter are documented in `Misc Connections/README.md`.

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
- `tools/` - OpenOCD-based telemetry monitor, web dashboard, and wireless links (`pilot3_serial_link.py` for classic SPP, `pilot3_tcp_link.py` for ESP-01S WiFi)
- `pilot3.bat` - unified dashboard launcher (prompts: 1 = SWD, 2 = WiFi)
- `Misc Connections/` - temporary ESP-01S AT setup bridge and configuration notes
- `circuit_reference/` - board schematic (KiCanvas) and netlist
- `pilot3_swd_probe.py` - SWD register probe used for initial bring-up

## Board errata

PA15 is shorted to GND on this board. Never configure it as an output; see the PA15 CONSTRAINT in `firmware/src/regs.hpp`.
