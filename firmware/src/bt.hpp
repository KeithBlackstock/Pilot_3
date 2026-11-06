// Bluetooth dashboard link (CC41-A BLE clone) on USART2, PA2=TX / PA3=RX,
// 9600 8N1. See bt.cpp for the wire protocol.
#pragma once
#include <cstdint>

namespace bt {

// Link statistics, exposed as a named global so the host can read them over
// the link itself (RD32 of g_bt_stats, found by symbol name in the ELF --
// same mechanism the dashboard uses for g_attitude_config).
struct Stats {
    uint32_t rx_bytes;    // bytes received on USART2
    uint32_t crc_bad;     // frames dropped on CRC mismatch (never executed)
    uint32_t tx_dropped;  // telemetry/response frames the TX ring couldn't fit
    uint32_t cmds_ok;     // RD32/WR32 commands executed
};

void init();

// Service at most one pending host command (RD32/WR32), main-loop context.
// Cheap when idle: one flag check.
void poll();

// Enqueue a framed snapshot of g_telemetry. Never blocks: if the TX ring
// can't fit the frame it is dropped and counted (tx_dropped).
void send_telemetry();

}  // namespace bt

// Named global, alongside g_telemetry / g_attitude_config.
extern volatile bt::Stats g_bt_stats;
