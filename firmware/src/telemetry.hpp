// Shared telemetry struct written by main.cpp's loop and polled by
// tools/pilot3_attitude_monitor.py over the same OpenOCD telnet link
// pilot3_swd_probe.py uses (mdw at a fixed, known address) -- no
// semihosting, so nothing here blocks the sensor loop on a print.
#pragma once
#include <cstdint>

#include "ibus.hpp"

struct Telemetry {
    uint32_t seq;
    float roll_deg;
    float pitch_deg;
    float baro_temp_c;
    float baro_pressure_pa;
    float baro_altitude_m;
    uint32_t ibus_rx_bytes;
    uint32_t ibus_frames_ok;
    uint32_t ibus_frames_bad;
    uint16_t ibus_ch[ibus::kNumChannels];
    uint16_t pwm_s1_us;  // actual (clamped/failsafe'd) value being driven, not raw channel
    uint16_t pwm_s4_us;
    // GPS (u-blox M10 on USART1, see gps.cpp). lat/lon/alt/speed/course are
    // the last decoded UBX NAV-PVT values, zeroed until the first fix.
    int32_t gps_lat_e7;  // degrees * 1e-7
    int32_t gps_lon_e7;
    float gps_alt_m;
    float gps_speed_mps;
    float gps_course_deg;
    uint32_t gps_rx_bytes;
    uint32_t gps_sentences_ok;
    uint32_t gps_sentences_bad;
    uint16_t gps_sats;
    uint16_t gps_fix;  // UBX fixType: 0 none, 2 2D, 3 3D, 4 GNSS+DR
};

// Layout is hand-mirrored in tools/pilot3_attitude_monitor.py's struct
// format string -- this catches an accidental alignment/padding change on
// the C++ side (won't catch a Python-side mismatch, but that's the half
// this file can enforce).
static_assert(sizeof(Telemetry) == 4 + 4 * 5 + 4 * 3 + 2 * ibus::kNumChannels + 2 * 2
                                   + 4 * 5 + 4 * 3 + 2 * 2,
              "Telemetry layout changed -- update pilot3_attitude_monitor.py's struct format too");

// Fixed SRAM address: must match kTelemetryAddr in
// tools/pilot3_attitude_monitor.py and the .telemetry placement in
// linker/gd32f303cc.ld.
constexpr uint32_t kTelemetryAddr = 0x20000000;

extern volatile Telemetry g_telemetry;
