// CRUISE mode: proportional speed hold on the motor (header-only, no I/O).
//
// CH5 high hands the throttle to this module, replacing the CH3 stick at the
// servo_pwm::set_s2_motor_us call; CH5 low (or an invalid/absent CH5) passes
// the stick straight through, and the caller's RX-stale motor cut applies
// either way. Everything else -- sticks, mix, stabilize, PATH -- is
// untouched; cruise only owns the motor.
//
// Engagement is immediate: there is no arm/launch sequencing. The moment CH5
// goes high the P law below is in charge, so on the ground at standstill
// (speed 0, far below target) the output rails straight to max_throttle_us.
// That is deliberate -- the mode is built to be switched on at the throw --
// but it means CH5-high on the ground is FULL POWER, not an idle. Flip it
// only at launch, prop first.
//
// Feedback is the derived 3-D speed (speed3d_mps in telemetry.hpp: GPS
// groundspeed + baro climb in quadrature). It is ground-relative, not
// airspeed: in wind the loop holds constant ground speed and true airspeed
// varies with it. No pitot or wind estimate on board, so that is inherent.
//
// Control law: throttle_us = min_throttle_us + kp * (target_mps - speed),
// clamped to [min_throttle_us, max_throttle_us]. P-only with a hard clamp:
// no integrator, so no anti-windup to get wrong. The price is droop -- at
// zero error the output sits at min throttle, so the settled speed is always
// somewhat below target; raise kp to close the gap.
//
// GPS degradation: with no fix or a stale stream the speed feedback is
// frozen and the P output is meaningless, so gps_ok=false drops to
// min_throttle_us (a glide) until it returns or the pilot takes over.
//
// Tunables live in g_cruise_config (RAM, defined in main.cpp) so the
// dashboard can change them live over SWD/BLE; they are re-read every loop
// tick and sanitized here, so a bad write can't drive the motor outside
// [min, max]. kDefaults is what the firmware boots with -- the dashboard's
// "save as defaults" rewrites that one line.
#pragma once
#include <cstdint>

namespace cruise_mode {

// Field order is mirrored in tools/pilot3_fc_control.py -- keep in sync.
struct Config {
    float kp_us_per_mps;   // throttle us per m/s of speed error
    float min_throttle_us; // lower clamp while engaged
    float max_throttle_us; // upper clamp while engaged
    float target_mps;      // 3-D speed setpoint (ground-relative)
};

constexpr Config kDefaults = {50.0f, 1100.0f, 1600.0f, 12.0f};  // CRUISE-DEFAULTS-LINE

constexpr float kMaxKp = 500.0f;
constexpr float kMaxTargetMps = 40.0f;
constexpr float kMinThrottleUs = 1000.0f;
constexpr float kMaxThrottleUs = 2000.0f;

}  // namespace cruise_mode

extern volatile cruise_mode::Config g_cruise_config;

namespace cruise_mode {

inline float clampf(float v, float lo, float hi) {
    if (!(v >= lo)) return lo;  // also catches NaN
    if (v > hi) return hi;
    return v;
}

// Plain, sanitized copy of the live config, taken once per loop tick.
inline Config snapshot() {
    Config c;
    c.kp_us_per_mps   = clampf(g_cruise_config.kp_us_per_mps, 0.0f, kMaxKp);
    c.min_throttle_us = clampf(g_cruise_config.min_throttle_us, kMinThrottleUs, kMaxThrottleUs);
    c.max_throttle_us = clampf(g_cruise_config.max_throttle_us, kMinThrottleUs, kMaxThrottleUs);
    if (c.max_throttle_us < c.min_throttle_us) c.max_throttle_us = c.min_throttle_us;
    c.target_mps      = clampf(g_cruise_config.target_mps, 0.0f, kMaxTargetMps);
    return c;
}

// Call every loop tick. `gps_ok` = fix present and stream fresh. Returns the
// motor pulse to drive: the stick when CH5 is low, min throttle while the
// speed feedback is unusable, and the P law the moment CH5 is high.
inline uint16_t step(const Config& c, bool switch_high, bool gps_ok,
                     float speed_mps, uint16_t stick_us) {
    if (!switch_high) return stick_us;
    if (!gps_ok) return static_cast<uint16_t>(c.min_throttle_us);
    float u = c.min_throttle_us + c.kp_us_per_mps * (c.target_mps - speed_mps);
    return static_cast<uint16_t>(clampf(u, c.min_throttle_us, c.max_throttle_us));
}

}  // namespace cruise_mode
