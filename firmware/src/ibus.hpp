// FlySky iBus RX activity monitor on USART3 (PB11=RX; PB10=TX is unused --
// iBus receivers only transmit). 115200 8N1.
//
// This is the firmware's first interrupt: at 115200 baud a byte arrives
// every ~87us, and USART3 has only a single-byte RX holding register --
// polling it from the 5ms main loop would overrun and drop nearly every
// byte. The RXNE interrupt fires per byte instead, and the frame state
// machine runs directly in the ISR (cheap enough per byte that it doesn't
// need a ring buffer): looks for the 0x20 0x40 header, collects the 32-byte
// frame, and checks the trailing checksum. Only activity counters are kept
// here -- not full 14-channel decode.
#pragma once
#include <cstdint>

namespace ibus {

constexpr int kNumChannels = 14;

struct Stats {
    uint32_t rx_bytes;
    uint32_t frames_ok;
    uint32_t frames_bad;
};

void init();

// Safe to call from the main loop: reads of these uint32_t counters don't
// need synchronization with the ISR on a 32-bit core (worst case, one
// stale-by-one-loop-iteration read -- fine for activity monitoring).
Stats snapshot();

// Last known-good channel values (raw iBus units, ~1000-2000), only
// updated on a checksum-valid frame. Unlike the Stats counters, this reads
// 14 uint16_t as one logical unit -- copying it without synchronization
// could interleave with the ISR mid-update and hand back a torn mix of old
// and new channels, so this briefly disables interrupts around the copy.
void read_channels(uint16_t out[kNumChannels]);

}  // namespace ibus
