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

        // Dynamic accel weight: the accelerometer only measures gravity alone
        // when |a| is ~1 g. The further |a| is from 1 g (launch, throttle punch,
        // impact, vibration, banked turn), the less the accel angle is trusted,
        // down to gyro-only at kAccelBandG. Note |a| barely moves for a modest
        // acceleration at right angles to gravity (0.3 g -> 1.04 g), so this
        // rejects big events, not small sustained ones.
        float mag_g = std::sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
        float w = 1.0f - std::fabs(mag_g - 1.0f) / kAccelBandG;
        if (!(w > 0.0f)) w = 0.0f;  // also catches NaN
        float k = (1.0f - kAlpha) * w;

        roll_deg_  = (1.0f - k) * (roll_deg_ + gx_dps * dt_s) + k * accel_roll_deg;
        pitch_deg_ = (1.0f - k) * (pitch_deg_ + gy_dps * dt_s) + k * accel_pitch_deg;
    }

    float roll_deg() const { return roll_deg_; }
    float pitch_deg() const { return pitch_deg_; }

    // Raw accel-derived angles from the most recent update() (no gyro).
    float accel_roll_deg() const { return accel_roll_deg_; }
    float accel_pitch_deg() const { return accel_pitch_deg_; }

private:
    static constexpr float kAlpha = 0.98f;       // gyro share when the accel is fully trusted
    static constexpr float kAccelBandG = 0.25f;  // | |a| - 1 g | at which the accel weight reaches 0
    float roll_deg_  = 0.0f;
    float pitch_deg_ = 0.0f;
    float accel_roll_deg_  = 0.0f;
    float accel_pitch_deg_ = 0.0f;
};
