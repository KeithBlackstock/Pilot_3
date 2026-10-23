// Minimal complementary filter: fuses accel-derived roll/pitch with
// integrated gyro rate. Written from scratch -- no borrowed AHRS code.
//
// Axis convention (verify against the physical board before trusting signs):
// roll is rotation about the X axis (right-wing-down positive), pitch is
// rotation about the Y axis (nose-up positive), using the ICM-42688-P's own
// accel/gyro axis labels as mounted on this board.
#pragma once
#include <cmath>

class ComplementaryFilter {
public:
    void update(float ax_g, float ay_g, float az_g, float gx_dps, float gy_dps, float dt_s) {
        constexpr float kRadToDeg = 57.29577951308232f;
        float accel_roll_deg  = std::atan2(ay_g, az_g) * kRadToDeg;
        float accel_pitch_deg = std::atan2(-ax_g, std::sqrt(ay_g * ay_g + az_g * az_g)) * kRadToDeg;

        accel_roll_deg_  = accel_roll_deg;
        accel_pitch_deg_ = accel_pitch_deg;

        roll_deg_  = kAlpha * (roll_deg_ + gx_dps * dt_s) + (1.0f - kAlpha) * accel_roll_deg;
        pitch_deg_ = kAlpha * (pitch_deg_ + gy_dps * dt_s) + (1.0f - kAlpha) * accel_pitch_deg;
    }

    float roll_deg() const { return roll_deg_; }
    float pitch_deg() const { return pitch_deg_; }

    // Raw accel-derived angles from the most recent update() (no gyro).
    float accel_roll_deg() const { return accel_roll_deg_; }
    float accel_pitch_deg() const { return accel_pitch_deg_; }

private:
    static constexpr float kAlpha = 0.98f;
    float roll_deg_  = 0.0f;
    float pitch_deg_ = 0.0f;
    float accel_roll_deg_  = 0.0f;
    float accel_pitch_deg_ = 0.0f;
};
