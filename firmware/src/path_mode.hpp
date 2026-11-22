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
// The caller supplies true heading, already wrapped to [0, 360) by main.cpp
// (PATH does not normalize it): QMC5883L magnetic heading + declination_deg
// below 5 m/s ground speed, and GPS course over ground at and above 5 m/s. The
// dashboard's compass calibration sets declination_deg as the magnetic-to-true
// heading offset. The mag has no tilt compensation yet and its mounting
// orientation is not bench-verified -- see telemetry.hpp. Heading is assumed
// clockwise-positive from north, same as the bearing; verify before flight.
//
// Roll sign: roll is right-wing-down positive (main.cpp), so a positive bank
// request = right turn = heading increasing.
//
// Pilot abort: roll stick (CH1) past abort_deadband_us latches PATH off and
// hands roll back to the stick -- the natural "pull it back" reflex when the
// plane departs or tracks wrong. The latch holds while CH6 stays high (PATH
// behaves as plain stabilize, so an availability glitch cannot silently
// re-arm it); the caller clears it when CH6 leaves the high position, and
// the next rise restarts the route at waypoint 1 as usual.
//
// TIP (temporary interrupt of path): if the bank angle itself exceeds
// tip_bank_deg while engaged -- a gust, wake, or a botched turn -- PATH
// suspends and the caller's plain-stabilize path levels the wings. Unlike
// the pilot abort this is not sticky: once the roll is back under the
// threshold PATH re-engages on the same leg and carries on the route (st.leg
// is preserved -- a fresh engage edge, which restarts at waypoint 1, only
// happens from a non-TIP disengagement).
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
    // Appended last: the dashboard reaches declination_deg by byte offset.
    float abort_deadband_us; // pilot roll stick past this aborts to stabilize
    float tip_bank_deg;      // |roll| past this suspends PATH until level again
};

constexpr float kMaxCaptureM = 500.0f;
constexpr float kMaxTipBankDeg = 90.0f;  // clamp on the TIP threshold
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
    bool aborted = false;  // pilot took the roll stick; latched until CH6 leaves high
    bool tip = false;      // suspended on excessive bank; resumes (same leg) when level
    uint32_t leg = 0;      // index of the waypoint being flown; == count when finished
};

struct Output {
    float bank_deg;  // roll angle request, right wing down positive
    bool finished;   // route complete: hold wings level
};

// Can PATH run right now? Needs waypoints, a 3D fix, a live heading and RX.
// heading_ok must include mag freshness only when the caller selected mag.
inline bool available(const Config& c, bool gps_3d, bool heading_ok, bool rx_ok) {
    return c.count > 0 && gps_3d && heading_ok && rx_ok;
}

// Call every loop tick. `want` = switch high AND available(). A rising edge
// restarts the route at waypoint 1. `pilot_roll_us` is the roll stick offset
// (us about center): past abort_deadband_us it latches the abort (see the
// header comment). `roll_deg` is the current bank (filter frame) for the TIP
// check. Returns the bank request.
inline Output step(State& st, const Config& c, bool want, int32_t lat_e7, int32_t lon_e7,
                   float heading_deg, float pilot_roll_us, float roll_deg) {
    if (!want) {
        st.engaged = false;
        st.tip = false;
        return {0.0f, false};
    }
    uint32_t count = c.count > static_cast<uint32_t>(kMaxWaypoints)
                         ? static_cast<uint32_t>(kMaxWaypoints) : c.count;
    if (!st.engaged && !st.tip) {
        // Fresh engage (not a TIP resume): restart the route at waypoint 1.
        st.engaged = true;
        st.leg = 0;
    }
    // The abort is checked before steering so the pilot's input wins on the
    // very tick it appears; disengaging here makes the caller run the plain
    // stabilize path, which already has the stick's roll request in hand.
    if (st.aborted ||
        std::fabs(pilot_roll_us) > clampf(c.abort_deadband_us, 0.0f, 500.0f)) {
        st.engaged = false;
        st.aborted = true;
        st.tip = false;
        return {0.0f, false};
    }
    // TIP (see the header comment): over-banked while steering -> suspend to
    // plain stabilize until the wings are back under the threshold, then
    // resume on the same leg. 0 disables the check.
    float tip_deg = clampf(c.tip_bank_deg, 0.0f, kMaxTipBankDeg);
    if (tip_deg > 0.0f) {
        if (st.engaged && std::fabs(roll_deg) > tip_deg) {
            st.engaged = false;
            st.tip = true;
            return {0.0f, false};
        }
        if (st.tip) {
            if (std::fabs(roll_deg) > tip_deg) return {0.0f, false};  // still hot
            st.tip = false;
            st.engaged = true;  // resume; st.leg untouched
        }
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

    // Both inputs are in [0, 360); wrapping the *difference* picks the short
    // way round (bearing 350, heading 10 -> -20, not +340). This is turn
    // direction, not heading normalization -- that happens in main.cpp.
    float err = wrap180(bearing - heading_deg);
    float k = clampf(c.k_path, 0.0f, 5.0f);
    return {k * err, false};  // unclamped here: roll_angle() applies max_roll_deg
}

}  // namespace path_mode
