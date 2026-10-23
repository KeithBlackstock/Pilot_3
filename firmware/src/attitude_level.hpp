// Accelerometer-only wing-leveler for pitch and roll. Proportional only: no
// gyro rate, no integrator, no filter state -- the input is the raw
// accel-derived angle.
//
// Behavior (decided in main.cpp): while BOTH pilot sticks are inside
// kDeadbandUs, the loop drives pitch and roll toward level; as soon as either
// stick leaves the deadband the loop lets go of both axes and the sticks pass
// straight through to the ruddervator mix.
//
// Pitch acts through elevator, roll through rudder (3CH: yaw induces roll).
//
// Signs are bench-verified on the airframe (roll is inverted relative to the
// naive assumption). The correction is -sign * kp * angle.
#pragma once

namespace attitude_level {

constexpr float kPitchSign = 1.0f;
constexpr float kRollSign  = -1.0f;

constexpr int   kDeadbandUs  = 30;     // stick offset (us) treated as "hands off"
constexpr float kMaxOutputUs = 250.0f; // authority limit (gentle)

// us of surface per degree of angle error.
constexpr float kPitchKp = 4.0f;
constexpr float kRollKp  = 4.0f;

inline bool hands_off(int stick_offset_us) {
    return stick_offset_us > -kDeadbandUs && stick_offset_us < kDeadbandUs;
}

inline float level(float angle_deg, float sign, float kp) {
    float out = -sign * kp * angle_deg;
    if (out > kMaxOutputUs) return kMaxOutputUs;
    if (out < -kMaxOutputUs) return -kMaxOutputUs;
    return out;
}

inline float pitch(float accel_pitch_deg) { return level(accel_pitch_deg, kPitchSign, kPitchKp); }
inline float roll(float accel_roll_deg)   { return level(accel_roll_deg, kRollSign, kRollKp); }

}  // namespace attitude_level
