// Angle-command loop for pitch and roll: P on the angle error plus D on the
// gyro rate. Stateless (no integrator, no filter state). The P term uses the
// complementary-filter angle (gyro-led short term, so brief linear accelerations
// from launch/throttle/vibration are mostly rejected); the D term uses the gyro rate directly and simply
// opposes angular velocity (it wants 0 deg/s), so there are no rate limits
// and no cascade -- the two terms just act together on the surface offset.
//
// The pilot's sticks are angle requests: full CH2 deflection (the pitch stick,
// differential slot) asks for +-max_pitch_deg, full CH1 (common slot) for
// +-max_roll_deg, centered sticks ask for level.
// The loop outputs a surface offset that drives the measured angle toward the
// requested one. (Outside stabilize mode the sticks bypass this entirely.)
// Kd defaults to a small value (0.1 us per deg/s); tune it from the dashboard.
//
// V-tail: pitch is the differential S1/S4 term (ch2), roll is the common term
// (ch1) -- see the mix comment in main.cpp (3CH: yaw induces roll).
//
// Signs are bench-verified on the airframe (roll is right-wing-down positive
// after the axis fix in main.cpp): a surface offset u moves the angle in the direction of
// `sign * u`, so the request for a stick offset s is `sign * s_norm * max`
// and the output is -sign * (kp * (angle - request) + kd * rate) -- a stick
// pushed the way it would push the surface asks for the angle that surface
// would produce. The gyro rate shares the angle's sign convention (the
// complementary filter integrates it into the angle).
//
// Tunables live in g_attitude_config (RAM, defined in main.cpp) so the
// dashboard can change them live over SWD; they are re-read every loop tick
// and sanitized here, so a bad write can't command more than max_output_us.
// kDefaults is what the firmware boots with -- the dashboard's "save as
// defaults" rewrites that one line.
#pragma once

namespace attitude_level {

// Field order is mirrored in tools/pilot3_fc_control.py -- keep in sync.
struct Config {
    float pitch_kp;       // us of surface per degree of angle error
    float roll_kp;
    float pitch_kd;       // us of surface per deg/s of gyro rate (damping)
    float roll_kd;
    float pitch_sign;     // only the sign matters (<0 inverts)
    float roll_sign;
    float max_pitch_deg;  // angle requested at full pitch stick (ch2)
    float max_roll_deg;   // angle requested at full roll stick (ch1)
    float deadband_us;    // stick offset (us) around center that requests level
    float max_output_us;  // authority limit (gentle)
    float trim_pitch_deg; // pitch angle held with centered sticks (+ = nose up)
    float trim_pitch_us;  // pitch subtrim added after the loop, all modes (+ = nose up)
};

constexpr Config kDefaults = {3.0f, 3.0f, 0.4f, 0.4f, -1.0f, 1.0f, 20.0f, 20.0f, 30.0f, 250.0f, 5.0f, 0.0f};  // DEFAULTS-LINE

// Cutoff of the one-pole low-pass on the gyro rate fed to the D term (see
// RateLpf). Deliberately not a live tunable: a mild fixed value (~4 ms of lag)
// that just keeps vibration out of the servos. 0 disables it. Must stay below
// the 100 Hz Nyquist of the 200 Hz loop.
constexpr float kRateLpfHz = 40.0f;

constexpr float kMaxKp = 20.0f;
constexpr float kMaxKd = 5.0f;
constexpr float kMaxAngleDeg = 60.0f;
constexpr float kMaxDeadbandUs = 200.0f;
constexpr float kStickFullUs = 500.0f;  // full stick deflection from center
constexpr float kMaxOutputLimitUs = 500.0f;
constexpr float kMaxTrimUs = 200.0f;

}  // namespace attitude_level

extern volatile attitude_level::Config g_attitude_config;

namespace attitude_level {

inline float clamp_range(float v, float lo, float hi) {
    if (!(v >= lo)) return lo;  // also catches NaN
    if (v > hi) return hi;
    return v;
}

// Plain, sanitized copy of the live config, taken once per loop tick.
inline Config snapshot() {
    Config c;
    c.pitch_kp      = clamp_range(g_attitude_config.pitch_kp, 0.0f, kMaxKp);
    c.roll_kp       = clamp_range(g_attitude_config.roll_kp, 0.0f, kMaxKp);
    c.pitch_kd      = clamp_range(g_attitude_config.pitch_kd, 0.0f, kMaxKd);
    c.roll_kd       = clamp_range(g_attitude_config.roll_kd, 0.0f, kMaxKd);
    c.pitch_sign    = g_attitude_config.pitch_sign < 0.0f ? -1.0f : 1.0f;
    c.roll_sign     = g_attitude_config.roll_sign < 0.0f ? -1.0f : 1.0f;
    c.max_pitch_deg = clamp_range(g_attitude_config.max_pitch_deg, 0.0f, kMaxAngleDeg);
    c.max_roll_deg  = clamp_range(g_attitude_config.max_roll_deg, 0.0f, kMaxAngleDeg);
    c.deadband_us   = clamp_range(g_attitude_config.deadband_us, 0.0f, kMaxDeadbandUs);
    c.max_output_us = clamp_range(g_attitude_config.max_output_us, 0.0f, kMaxOutputLimitUs);
    c.trim_pitch_deg = clamp_range(g_attitude_config.trim_pitch_deg, -kMaxAngleDeg, kMaxAngleDeg);
    c.trim_pitch_us  = clamp_range(g_attitude_config.trim_pitch_us, -kMaxTrimUs, kMaxTrimUs);
    return c;
}

// One-pole low-pass for the gyro rate used by the D term (the angle estimate
// keeps the raw gyro). Discretised with the measured dt so loop overruns
// don't shift the cutoff. cutoff_hz <= 0 passes the input straight through and
// tracks it, so enabling the filter live doesn't start from a stale state.
// The first call seeds from the input to avoid a startup transient.
struct RateLpf {
    float y = 0.0f;
    bool seeded = false;

    float update(float x, float cutoff_hz, float dt_s) {
        if (!seeded || !(cutoff_hz > 0.0f) || !(dt_s > 0.0f)) {
            y = x;
            seeded = true;
            return y;
        }
        constexpr float kTwoPi = 6.283185307179586f;
        float rc = 1.0f / (kTwoPi * cutoff_hz);
        float a = dt_s / (rc + dt_s);
        y += a * (x - y);
        return y;
    }
};

// Stick offset -> [-1, 1], zero inside the deadband and ramping from there
// (no jump at the deadband edge).
inline float stick_norm(const Config& c, float stick_offset_us) {
    float mag = stick_offset_us < 0.0f ? -stick_offset_us : stick_offset_us;
    if (mag <= c.deadband_us) return 0.0f;
    float n = (mag - c.deadband_us) / (kStickFullUs - c.deadband_us);
    if (n > 1.0f) n = 1.0f;
    return stick_offset_us < 0.0f ? -n : n;
}

inline float track(const Config& c, float stick_offset_us, float angle_deg, float rate_dps,
                   float max_angle_deg, float sign, float kp, float kd, float trim_deg) {
    // Trim is a real angle (not scaled by sign) added to the stick's request.
    float request_deg = sign * stick_norm(c, stick_offset_us) * max_angle_deg + trim_deg;
    float out = -sign * (kp * (angle_deg - request_deg) + kd * rate_dps);
    if (out > c.max_output_us) return c.max_output_us;
    if (out < -c.max_output_us) return -c.max_output_us;
    return out;
}

// Roll loop driven by an angle request (deg, filter frame: right wing down
// positive) instead of a stick offset -- used by PATH. Same P-D law and
// output limit as track(); the request is clamped to max_roll_deg.
inline float roll_angle(const Config& c, float request_deg, float roll_deg, float roll_rate_dps) {
    request_deg = clamp_range(request_deg, -c.max_roll_deg, c.max_roll_deg);
    float out = -c.roll_sign * (c.roll_kp * (roll_deg - request_deg) + c.roll_kd * roll_rate_dps);
    if (out > c.max_output_us) return c.max_output_us;
    if (out < -c.max_output_us) return -c.max_output_us;
    return out;
}

inline float pitch(const Config& c, float stick_offset_us, float pitch_deg, float pitch_rate_dps) {
    return track(c, stick_offset_us, pitch_deg, pitch_rate_dps,
                 c.max_pitch_deg, c.pitch_sign, c.pitch_kp, c.pitch_kd, c.trim_pitch_deg);
}
inline float roll(const Config& c, float stick_offset_us, float roll_deg, float roll_rate_dps) {
    return track(c, stick_offset_us, roll_deg, roll_rate_dps,
                 c.max_roll_deg, c.roll_sign, c.roll_kp, c.roll_kd, 0.0f);
}

}  // namespace attitude_level
