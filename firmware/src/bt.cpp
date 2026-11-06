// Bluetooth dashboard link (CC41-A BLE clone) on USART2 (PA2=TX, PA3=RX),
// 9600 8N1. Carries the dashboard over the air: the Telemetry struct
// (telemetry.hpp) is streamed as framed packets, and the host can peek/poke
// 32-bit SRAM words -- the same primitive the SWD dashboard uses over
// OpenOCD (tools/pilot3_fc_control.py), so every dashboard control (leveler
// config, baro ref, mag cal, waypoint readback) works unchanged. Flashing
// is NOT possible over this link -- that stays on the ST-Link.
//
// Wire protocol (little-endian), both directions:
//   0xA5 0x5A | type u8 | len u8 | payload[len] | crc16 lo | crc16 hi
// CRC16-CCITT (poly 0x1021, init 0xFFFF) over type..payload.
//   host->FC  0x10 RD32 {addr u32, count u8}   -> 0x90 {addr u32, words...}
//   host->FC  0x11 WR32 {addr u32, value u32}  -> 0x91 {addr, value} (echo)
//   FC->host  0x81 TELEMETRY {packed Telemetry}  (pushed, unsolicited)
//   FC->host  0x9F ERROR {code u8}   1=bad addr  2=bad count  3=bad type
// Reads and writes are restricted to SRAM [0x20000000, 0x2000C000): every
// config symbol (g_attitude_config, g_baro_ref_pa, g_mag_cal, g_path_config)
// lives there, and the bound makes a corrupt frame incapable of touching
// peripheral registers. Mirrored in tools/pilot3_serial_link.py.
//
// Concurrency: the RX state machine runs in the USART2 ISR but only parses
// and CRC-checks frames; RD32/WR32 execute in bt::poll() (main-loop
// context, the same handoff pattern as gps.cpp), so the ISR stays short
// next to the GPS and iBus ISRs. TX is a ring buffer drained by the TXE
// interrupt; only the main loop enqueues (single producer), so the only
// guard needed is cpsid/cpsie around head/TXEIE manipulation.

#include "bt.hpp"

#include "regs.hpp"
#include "telemetry.hpp"

using namespace reg;

volatile bt::Stats g_bt_stats = {};

namespace {

constexpr uint8_t kSync0 = 0xA5;
constexpr uint8_t kSync1 = 0x5A;

constexpr uint8_t kCmdRead32    = 0x10;
constexpr uint8_t kCmdWrite32   = 0x11;
constexpr uint8_t kRspTelemetry = 0x81;
constexpr uint8_t kRspRead      = 0x90;
constexpr uint8_t kRspWrite     = 0x91;
constexpr uint8_t kRspError     = 0x9F;

constexpr uint8_t kErrAddr  = 1;
constexpr uint8_t kErrCount = 2;
constexpr uint8_t kErrType  = 3;

// RD32 word cap: the response frame must fit the TX ring alongside the
// telemetry stream. 40 words covers the biggest dashboard read (the
// waypoint list, 36 words).
constexpr uint8_t kMaxReadWords = 40;

constexpr uint32_t kSramLo = 0x20000000;
constexpr uint32_t kSramHi = 0x2000C000;  // == RAM_HI in pilot3_fc_control.py

// Largest host command payload: WR32 = addr(4) + value(4).
constexpr uint8_t kMaxCmdPayload = 8;

constexpr uint32_t kNibble = 0xF;

inline void disable_irq() { asm volatile("cpsid i" ::: "memory"); }
inline void enable_irq()  { asm volatile("cpsie i" ::: "memory"); }

// ---- CRC16-CCITT (poly 0x1021, init 0xFFFF) ----

inline void crc_byte(uint16_t& crc, uint8_t b) {
    crc ^= static_cast<uint16_t>(b) << 8;
    for (int i = 0; i < 8; ++i) {
        crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                             : static_cast<uint16_t>(crc << 1);
    }
}

// ---- TX ring (producer: main loop; consumer: TXE ISR) ----

constexpr uint32_t kTxRingSize = 512;  // power of two
uint8_t s_tx_buf[kTxRingSize];
volatile uint32_t s_tx_head = 0;  // next write, producer only
volatile uint32_t s_tx_tail = 0;  // next read, consumer only

// Free space. A stale tail (ISR is mid-drain) only under-reports, which can
// drop a frame that would have fit -- safe, never corrupt.
uint32_t tx_free() {
    return (s_tx_tail - s_tx_head - 1) & (kTxRingSize - 1);
}

// Enqueue a whole run of bytes, all or nothing: a frame is never
// half-queued. Main-loop context only. Returns false if it didn't fit.
bool tx_enqueue(const uint8_t* data, uint32_t n) {
    if (n > tx_free()) return false;
    disable_irq();
    for (uint32_t i = 0; i < n; ++i) {
        s_tx_buf[s_tx_head] = data[i];
        s_tx_head = (s_tx_head + 1) & (kTxRingSize - 1);
    }
    mmio(USART2_CR1) |= USART2_CR1_TXEIE;  // TXE is level-high when DR is empty
    enable_irq();
    return true;
}

// Build a framed packet into out (must hold len + 6). Returns the size.
uint32_t build_frame(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* out) {
    out[0] = kSync0;
    out[1] = kSync1;
    out[2] = type;
    out[3] = len;
    uint16_t crc = 0xFFFF;
    crc_byte(crc, type);
    crc_byte(crc, len);
    for (uint8_t i = 0; i < len; ++i) {
        out[4 + i] = payload[i];
        crc_byte(crc, payload[i]);
    }
    out[4 + len] = static_cast<uint8_t>(crc & 0xFF);
    out[5 + len] = static_cast<uint8_t>(crc >> 8);
    return static_cast<uint32_t>(len) + 6;
}

void send_error(uint8_t code) {
    uint8_t frame[7];
    uint32_t n = build_frame(kRspError, &code, 1, frame);
    if (!tx_enqueue(frame, n)) g_bt_stats.tx_dropped++;
}

// ---- RX: byte state machine (ISR context only) ----

enum RxState : uint8_t {
    R_SYNC0, R_SYNC1, R_TYPE, R_LEN, R_PAYLOAD, R_CRC1, R_CRC2, R_SKIP
};
RxState s_state = R_SYNC0;
uint8_t s_type = 0;
uint8_t s_len = 0;
uint16_t s_idx = 0;      // payload index, or bytes left to skip (len+2)
uint16_t s_crc = 0;
uint8_t s_crc_lo = 0;
uint8_t s_buf[kMaxCmdPayload];

// ISR -> main loop handoff: one slot, written only when clear (same
// pattern as gps.cpp). A command arriving while the slot is busy is
// dropped; the host times out and retries.
volatile bool s_cmd_pending = false;
uint8_t s_cmd_type = 0;
uint8_t s_cmd_buf[kMaxCmdPayload];

void on_frame() {
    if (s_cmd_pending) return;
    s_cmd_type = s_type;
    for (uint8_t i = 0; i < s_len; ++i) s_cmd_buf[i] = s_buf[i];
    s_cmd_pending = true;
}

void on_byte(uint8_t b) {
    g_bt_stats.rx_bytes++;
    switch (s_state) {
    case R_SYNC0:
        if (b == kSync0) s_state = R_SYNC1;
        break;
    case R_SYNC1:
        if (b == kSync1) { s_crc = 0xFFFF; s_state = R_TYPE; }
        else if (b != kSync0) { s_state = R_SYNC0; }  // repeated A5 stays in SYNC1
        break;
    case R_TYPE:
        s_type = b;
        crc_byte(s_crc, b);
        s_state = R_LEN;
        break;
    case R_LEN:
        s_len = b;
        crc_byte(s_crc, b);
        if (b <= kMaxCmdPayload) {
            s_idx = 0;
            s_state = b ? R_PAYLOAD : R_CRC1;
        } else {
            s_idx = static_cast<uint16_t>(b + 2);  // payload + 2 CRC bytes
            s_state = R_SKIP;
        }
        break;
    case R_PAYLOAD:
        s_buf[s_idx++] = b;
        crc_byte(s_crc, b);
        if (s_idx == s_len) s_state = R_CRC1;
        break;
    case R_CRC1:
        s_crc_lo = b;
        s_state = R_CRC2;
        break;
    case R_CRC2:
        if (s_crc == static_cast<uint16_t>(s_crc_lo | (static_cast<uint16_t>(b) << 8))) {
            on_frame();
        } else {
            g_bt_stats.crc_bad++;
        }
        s_state = R_SYNC0;
        break;
    case R_SKIP:
        if (--s_idx == 0) s_state = R_SYNC0;
        break;
    default:
        s_state = R_SYNC0;
        break;
    }
}

// ---- command execution (bt::poll, main-loop context) ----

uint32_t le_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

void put_le_u32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

bool addr_ok(uint32_t addr, uint32_t n_words) {
    return (addr & 3u) == 0 && addr >= kSramLo && addr <= kSramHi - 4u * n_words;
}

void exec_read(const uint8_t* payload) {
    uint32_t addr = le_u32(payload);
    uint8_t count = payload[4];
    if (count == 0 || count > kMaxReadWords) { send_error(kErrCount); return; }
    if (!addr_ok(addr, count)) { send_error(kErrAddr); return; }
    // Response: {addr u32, words...} -- the addr echo lets the host match
    // the response to its request even if a retry races a late reply.
    uint8_t rsp[4 + 4 * kMaxReadWords];
    put_le_u32(rsp, addr);
    const volatile uint32_t* src = reinterpret_cast<const volatile uint32_t*>(addr);
    for (uint8_t i = 0; i < count; ++i) put_le_u32(rsp + 4 + 4 * i, src[i]);
    uint8_t frame[6 + sizeof(rsp)];
    uint32_t n = build_frame(kRspRead, rsp, static_cast<uint8_t>(4 + 4 * count), frame);
    if (!tx_enqueue(frame, n)) g_bt_stats.tx_dropped++;
    else g_bt_stats.cmds_ok++;
}

void exec_write(const uint8_t* payload) {
    uint32_t addr = le_u32(payload);
    uint32_t value = le_u32(payload + 4);
    if (!addr_ok(addr, 1)) { send_error(kErrAddr); return; }
    *reinterpret_cast<volatile uint32_t*>(addr) = value;
    uint8_t frame[6 + 8];
    uint32_t n = build_frame(kRspWrite, payload, 8, frame);  // echo = ack
    if (!tx_enqueue(frame, n)) g_bt_stats.tx_dropped++;
    else g_bt_stats.cmds_ok++;
}

}  // namespace

extern "C" void USART2_IRQHandler() {
    // RXNE clears automatically on the DR read that follows the SR read --
    // same ack-by-read pattern as USART1/3. TXE clears on the DR write.
    uint32_t sr = mmio(USART2_SR);
    if (sr & USART2_SR_RXNE) {
        uint8_t b = static_cast<uint8_t>(mmio(USART2_DR) & 0xFF);
        on_byte(b);
    }
    if (sr & USART2_SR_TXE) {
        if (s_tx_tail != s_tx_head) {
            mmio(USART2_DR) = s_tx_buf[s_tx_tail];
            s_tx_tail = (s_tx_tail + 1) & (kTxRingSize - 1);
        }
        if (s_tx_tail == s_tx_head) {
            mmio(USART2_CR1) &= ~USART2_CR1_TXEIE;  // ring empty: stop interrupting
        }
    }
}

namespace bt {

void init() {
    mmio(RCC_APB2ENR) |= RCC_APB2ENR_IOPAEN;
    { volatile uint32_t sync = mmio(RCC_APB2ENR); (void)sync; }
    mmio(RCC_APB1ENR) |= RCC_APB1ENR_USART2EN;
    { volatile uint32_t sync = mmio(RCC_APB1ENR); (void)sync; }

    // PA2 = TX: alternate-function push-pull. PA2's config nibble is bits
    // 8-11 of GPIOA_CRL (pins 0-7 -- nowhere near the PA15 constraint,
    // which is a CRH pin). PA3 = RX needs no config: floating input is its
    // reset state, same rationale as the GPS's PA10.
    uint32_t crl = mmio(GPIOA_CRL);
    crl &= ~(kNibble << 8);
    crl |= (PIN_AF_PP_50 << 8);
    mmio(GPIOA_CRL) = crl;

    // 9600 baud @ PCLK1 = 8 MHz (USART2 is on APB1, like USART3): BRR =
    // 8 MHz / 9600 = 833.33 -> 0x341 (833, ~0.04% slow). Pinned to 9600
    // because the CC41-A BLE clone's UART bridge is hard-locked to 9600 --
    // it accepts AT+BAUD4 but the bridge never leaves 9600 (verified over
    // the air: clean frames only when both ends run 9600).
    mmio(USART2_BRR) = (52u << 4) | 1u;
    mmio(USART2_CR1) = USART2_CR1_UE | USART2_CR1_RE | USART2_CR1_TE | USART2_CR1_RXNEIE;

    mmio(NVIC_ISER1) = NVIC_ISER1_USART2;
}

void poll() {
    if (!s_cmd_pending) return;
    uint8_t type;
    uint8_t payload[kMaxCmdPayload];
    disable_irq();
    type = s_cmd_type;
    for (uint8_t i = 0; i < kMaxCmdPayload; ++i) payload[i] = s_cmd_buf[i];
    s_cmd_pending = false;
    enable_irq();
    switch (type) {
    case kCmdRead32:  exec_read(payload);  break;
    case kCmdWrite32: exec_write(payload); break;
    default:          send_error(kErrType); break;
    }
}

void send_telemetry() {
    static_assert(sizeof(Telemetry) <= 255 - 6, "Telemetry no longer fits one frame");
    // Member-wise copy off the volatile global: the implicit copy-assignment
    // operator isn't volatile-qualified (same pattern as gps.cpp's
    // fix_snapshot).
    Telemetry t;
    t.seq = g_telemetry.seq;
    t.roll_deg = g_telemetry.roll_deg;
    t.pitch_deg = g_telemetry.pitch_deg;
    t.baro_temp_c = g_telemetry.baro_temp_c;
    t.baro_pressure_pa = g_telemetry.baro_pressure_pa;
    t.baro_altitude_m = g_telemetry.baro_altitude_m;
    t.ibus_rx_bytes = g_telemetry.ibus_rx_bytes;
    t.ibus_frames_ok = g_telemetry.ibus_frames_ok;
    t.ibus_frames_bad = g_telemetry.ibus_frames_bad;
    for (int i = 0; i < ibus::kNumChannels; ++i) t.ibus_ch[i] = g_telemetry.ibus_ch[i];
    t.pwm_s1_us = g_telemetry.pwm_s1_us;
    t.pwm_s4_us = g_telemetry.pwm_s4_us;
    t.gps_lat_e7 = g_telemetry.gps_lat_e7;
    t.gps_lon_e7 = g_telemetry.gps_lon_e7;
    t.gps_alt_m = g_telemetry.gps_alt_m;
    t.gps_speed_mps = g_telemetry.gps_speed_mps;
    t.gps_course_deg = g_telemetry.gps_course_deg;
    t.gps_rx_bytes = g_telemetry.gps_rx_bytes;
    t.gps_sentences_ok = g_telemetry.gps_sentences_ok;
    t.gps_sentences_bad = g_telemetry.gps_sentences_bad;
    t.gps_sats = g_telemetry.gps_sats;
    t.gps_fix = g_telemetry.gps_fix;
    t.mag_x = g_telemetry.mag_x;
    t.mag_y = g_telemetry.mag_y;
    t.mag_z = g_telemetry.mag_z;
    t.mag_present = g_telemetry.mag_present;
    t.mag_heading_deg = g_telemetry.mag_heading_deg;
    t.mag_reads_ok = g_telemetry.mag_reads_ok;
    t.mag_errors = g_telemetry.mag_errors;
    uint8_t frame[6 + sizeof(Telemetry)];
    uint32_t n = build_frame(kRspTelemetry, reinterpret_cast<const uint8_t*>(&t),
                             sizeof(Telemetry), frame);
    if (!tx_enqueue(frame, n)) g_bt_stats.tx_dropped++;
}

}  // namespace bt
