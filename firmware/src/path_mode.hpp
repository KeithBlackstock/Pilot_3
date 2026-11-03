// PATH mode: roll-only waypoint steering (header-only, no I/O).
//
// Proportional heading steering: bank_deg = k_path * heading_error_deg. The
// result is a roll *angle* request (clamped to attitude_level's max_roll_deg
// by roll_angle(); there is no separate PATH bank limit) that
// replaces the CH1 stick request ahead of attitude_level::roll_angle(), so the
// existing angle loop does the flying; pitch and throttle stay with the pilot.
//
// Waypoints live in RAM (g_path_config) and are empty at boot: with
// count == 0 PATH never engages and the CH6 high position is silently treated
// as plain stabilize. Nothing uploads waypoints yet (dashboard planner is
// planning-only); until then the array can only be written over SWD.
//
// Heading is the QMC5883L magnetic heading (hard-iron/scale calibrated, with
// mirrored-module correction -- see qmc5883.hpp) + declination_deg, which the
// dashboard's compass calibration sets as the heading offset to true north.
// The mag has no tilt compensation yet and its mounting orientation is not
// bench-verified -- see telemetry.hpp. Heading is assumed clockwise-positive
// from north, same as the bearing; verify on the bench before flight.
//
// Roll sign: roll is right-wing-down positive (main.cpp), so a positive bank
// request = right turn = heading increasing.
#pragma once
#include <cmath>
#include <cstdint>

namespace path_mode {

constexpr int kMaxWaypoints = 16;

struct Waypoint {
    int32_t lat_e7;
    int32_t lon_e7;
};

struct Config {
    uint32_t count;          // waypoints loaded (0 = PATH disabled)
    float k_path;            // bank deg per deg of heading error
    float capture_radius_m;  // waypoint reached inside this distance
    float declination_deg;   // magnetic -> true, + = east
    Waypoint wp[kMaxWaypoints];
};

constexpr float kMaxCaptureM = 500.0f;
constexpr float kMetersPerDegLat = 111320.0f;
constexpr float kRadToDeg = 57.29578f;
constexpr float kDegToRad = 0.01745329f;

inline float clampf(float v, float lo, float hi) {
    if (!(v >= lo)) return lo;  // also catches NaN
    if (v > hi) return hi;
    return v;
}

// Wrap an angle difference to [-180, 180].
inline float wrap180(float d) {
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// Flat-earth bearing (deg clockwise from north, [0, 360)) and distance (m)
// from (lat1, lon1) to (lat2, lon2), 1e-7 degree units. Fine at route scale.
inline void bearing_distance(int32_t lat1_e7, int32_t lon1_e7, int32_t lat2_e7, int32_t lon2_e7,
                             float& bearing_deg, float& dist_m) {
    float d_n = static_cast<float>(lat2_e7 - lat1_e7) * 1e-7f;
    float mid_lat = (static_cast<float>(lat1_e7) + static_cast<float>(lat2_e7)) * 0.5e-7f;
    float d_e = static_cast<float>(lon2_e7 - lon1_e7) * 1e-7f * std::cos(mid_lat * kDegToRad);
    float b = std::atan2(d_e, d_n) * kRadToDeg;
    if (b < 0.0f) b += 360.0f;
    bearing_deg = b;
    dist_m = std::sqrt(d_n * d_n + d_e * d_e) * kMetersPerDegLat;
}

struct State {
    bool engaged = false;
    uint32_t leg = 0;  // index of the waypoint being flown; == count when finished
};

struct Output {
    float bank_deg;  // roll angle request, right wing down positive
    bool finished;   // route complete: hold wings level
};

// Can PATH run right now? Needs waypoints, a 3D fix, a live mag and a live RX.
inline bool available(const Config& c, bool gps_3d, bool mag_ok, bool rx_ok) {
    return c.count > 0 && gps_3d && mag_ok && rx_ok;
}

// Call every loop tick. `want` = switch high AND available(). A rising edge
// restarts the route at waypoint 1. Returns the bank request.
inline Output step(State& st, const Config& c, bool want, int32_t lat_e7, int32_t lon_e7,
                   float mag_heading_deg) {
    if (!want) {
        st.engaged = false;
        return {0.0f, false};
    }
    uint32_t count = c.count > static_cast<uint32_t>(kMaxWaypoints)
                         ? static_cast<uint32_t>(kMaxWaypoints) : c.count;
    if (!st.engaged) {
        st.engaged = true;
        st.leg = 0;
    }
    float capture = clampf(c.capture_radius_m, 1.0f, kMaxCaptureM);
    float bearing = 0.0f, dist = 0.0f;
    while (st.leg < count) {
        const Waypoint& w = c.wp[st.leg];
        bearing_distance(lat_e7, lon_e7, w.lat_e7, w.lon_e7, bearing, dist);
        if (dist > capture) break;
        st.leg++;  // reached: advance (may chain if waypoints sit inside the radius)
    }
    if (st.leg >= count) return {0.0f, true};  // end of route: wings level

    float heading = mag_heading_deg + clampf(c.declination_deg, -90.0f, 90.0f);
    float err = wrap180(bearing - heading);
    float k = clampf(c.k_path, 0.0f, 5.0f);
    return {k * err, false};  // unclamped here: roll_angle() applies max_roll_deg
}

}  // namespace path_mode
