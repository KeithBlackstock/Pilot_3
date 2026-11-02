// u-blox GPS (HGLRC M100-Pro / MAX-M10) binary UBX receiver on USART1
// (PA10=RX; PA9=TX unused so far -- PA10's reset state is floating input, so
// no GPIO config is needed; see the PA15 CONSTRAINT in regs.hpp before
// touching GPIOA_CRH for TX). The module streams UBX-NAV-PVT (no NMEA). 115200 8N1, same BRR
// setting as the proven USART3/IBUS link.
//
// Same shape as ibus.cpp: an RXNE interrupt per byte, because polling from
// the 5ms main loop would overrun the single-byte holding register at this
// baud. The ISR only frames the UBX stream, validates the Fletcher checksum,
// and hands complete NAV-PVT payloads to the main loop through a single
// slot -- decoding (float conversion) happens in gps::poll(), never in
// interrupt context. Other UBX messages are skipped by length.
#pragma once
#include <cstdint>

namespace gps {

struct Stats {
    uint32_t rx_bytes;
    uint32_t sentences_ok;   // checksum-valid NAV-PVT frames
    uint32_t sentences_bad;  // frames that failed the checksum
};

struct Fix {
    int32_t lat_e7;    // +N, degrees * 1e-7
    int32_t lon_e7;    // +E, degrees * 1e-7
    float alt_m;       // above mean sea level
    float speed_mps;   // ground speed
    float course_deg;  // track over ground
    uint8_t sats;      // satellites used
    uint8_t fix;       // UBX fixType: 0 none, 2 2D, 3 3D, 4 GNSS+dead-reckoning
};

void init();

// Parses whatever sentence the ISR completed since the last call (at most
// one per call). Cheap; meant to run every main-loop iteration.
void poll();

// Same synchronization story as ibus: reads of these uint32_t counters
// don't need synchronization with the ISR on a 32-bit core.
Stats snapshot();

// ...while Fix is one logical unit -- copied with interrupts briefly off,
// like ibus::read_channels().
Fix read_fix();

}  // namespace gps
