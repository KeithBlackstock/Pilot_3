#include "gps.hpp"
#include "regs.hpp"

using namespace reg;

namespace {

// UBX-NAV-PVT: class 0x01, id 0x07, fixed 92-byte payload (M8/M10).
constexpr uint8_t kUbxSync1 = 0xB5;
constexpr uint8_t kUbxSync2 = 0x62;
constexpr uint8_t kPvtClass = 0x01;
constexpr uint8_t kPvtId    = 0x07;
constexpr uint16_t kPvtLen  = 92;

// "sentences" in Stats are UBX frames: ok = checksum-valid NAV-PVT, bad =
// checksum failure.
volatile uint32_t g_rx_bytes      = 0;
volatile uint32_t g_sentences_ok  = 0;
volatile uint32_t g_sentences_bad = 0;
volatile gps::Fix g_fix = {};

// ISR -> main loop handoff: one slot, written only when s_pending is clear,
// so a slow main loop drops newer frames instead of tearing reads.
uint8_t s_slot[kPvtLen];
volatile bool s_pending = false;

// ISR-only state -- never touched from the main loop.
// UBX framing: B5 62 | class | id | len(LE16) | payload | ck_a ck_b, where
// the Fletcher checksum covers class..payload.
enum UbxState : uint8_t {
    U_IDLE, U_SYNC2, U_CLASS, U_ID, U_LEN1, U_LEN2, U_PAYLOAD, U_CKA, U_CKB, U_SKIP
};
UbxState s_state = U_IDLE;
uint8_t s_class = 0;
uint8_t s_id = 0;
uint16_t s_len = 0;
uint16_t s_idx = 0;   // payload index, or bytes left to skip
uint8_t s_ck_a = 0;
uint8_t s_ck_b = 0;
uint8_t s_buf[kPvtLen];

inline void disable_irq() { asm volatile("cpsid i" ::: "memory"); }
inline void enable_irq()  { asm volatile("cpsie i" ::: "memory"); }

inline void ck_add(uint8_t b) {
    s_ck_a = static_cast<uint8_t>(s_ck_a + b);
    s_ck_b = static_cast<uint8_t>(s_ck_b + s_ck_a);
}

void on_byte(uint8_t b) {
    g_rx_bytes++;

    switch (s_state) {
    case U_IDLE:
        if (b == kUbxSync1) s_state = U_SYNC2;
        break;
    case U_SYNC2:
        if (b == kUbxSync2) {
            s_ck_a = 0; s_ck_b = 0;
            s_state = U_CLASS;
        } else if (b != kUbxSync1) {
            s_state = U_IDLE;  // (a repeated B5 just stays in SYNC2)
        }
        break;
    case U_CLASS: s_class = b; ck_add(b); s_state = U_ID; break;
    case U_ID:    s_id = b;    ck_add(b); s_state = U_LEN1; break;
    case U_LEN1:  s_len = b;   ck_add(b); s_state = U_LEN2; break;
    case U_LEN2:
        s_len = static_cast<uint16_t>(s_len | (b << 8));
        ck_add(b);
        if (s_class == kPvtClass && s_id == kPvtId && s_len == kPvtLen) {
            s_idx = 0;
            s_state = U_PAYLOAD;
        } else {
            // Some other message (NAV-STATUS, etc.): skip payload + 2
            // checksum bytes without verifying it.
            s_idx = static_cast<uint16_t>(s_len + 2);
            s_state = U_SKIP;
        }
        break;
    case U_PAYLOAD:
        s_buf[s_idx++] = b;
        ck_add(b);
        if (s_idx == kPvtLen) s_state = U_CKA;
        break;
    case U_CKA:
        if (b == s_ck_a) s_state = U_CKB;
        else { g_sentences_bad++; s_state = U_IDLE; }
        break;
    case U_CKB:
        if (b == s_ck_b) {
            g_sentences_ok++;
            if (!s_pending) {
                for (uint16_t i = 0; i < kPvtLen; ++i) s_slot[i] = s_buf[i];
                s_pending = true;
            }
        } else {
            g_sentences_bad++;
        }
        s_state = U_IDLE;
        break;
    case U_SKIP:
        if (--s_idx == 0) s_state = U_IDLE;
        break;
    default:
        s_state = U_IDLE;
        break;
    }
}

// ---- parsing (main-loop context only) ----

// Member-wise copies to/from volatile g_fix: the implicit copy-assignment
// operator isn't volatile-qualified, so whole-struct assignment won't
// compile against a volatile object (ibus avoids this by copying arrays
// element-wise; same idea here).
gps::Fix fix_snapshot() {
    gps::Fix f;
    disable_irq();
    f.lat_e7 = g_fix.lat_e7;
    f.lon_e7 = g_fix.lon_e7;
    f.alt_m = g_fix.alt_m;
    f.speed_mps = g_fix.speed_mps;
    f.course_deg = g_fix.course_deg;
    f.sats = g_fix.sats;
    f.fix = g_fix.fix;
    enable_irq();
    return f;
}

void fix_store(const gps::Fix& f) {
    disable_irq();
    g_fix.lat_e7 = f.lat_e7;
    g_fix.lon_e7 = f.lon_e7;
    g_fix.alt_m = f.alt_m;
    g_fix.speed_mps = f.speed_mps;
    g_fix.course_deg = f.course_deg;
    g_fix.sats = f.sats;
    g_fix.fix = f.fix;
    enable_irq();
}

int32_t le_i32(const uint8_t* p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24));
}

// NAV-PVT payload offsets: fixType@20 flags@21(bit0 gnssFixOK) numSV@23
// lon@24 lat@28 (1e-7 deg) hMSL@36 (mm) gSpeed@60 (mm/s) headMot@64 (1e-5 deg).
void parse_pvt(const uint8_t* p) {
    gps::Fix f = fix_snapshot();
    bool fix_ok = (p[21] & 0x01) != 0;
    f.fix = fix_ok ? p[20] : 0;
    f.sats = p[23];
    if (f.fix >= 2) {
        f.lon_e7 = le_i32(p + 24);
        f.lat_e7 = le_i32(p + 28);
        f.alt_m = static_cast<float>(le_i32(p + 36)) * 0.001f;
        f.speed_mps = static_cast<float>(le_i32(p + 60)) * 0.001f;
        f.course_deg = static_cast<float>(le_i32(p + 64)) * 1e-5f;
    }
    fix_store(f);
}

}  // namespace

extern "C" void USART1_IRQHandler() {
    // RXNE clears automatically on the DR read that follows the SR read --
    // same ack-by-read pattern as USART3.
    if (mmio(USART1_SR) & USART1_SR_RXNE) {
        uint8_t b = static_cast<uint8_t>(mmio(USART1_DR) & 0xFF);
        on_byte(b);
    }
}

namespace gps {

void init() {
    mmio(RCC_APB2ENR) |= RCC_APB2ENR_IOPAEN;
    { volatile uint32_t sync = mmio(RCC_APB2ENR); (void)sync; }
    mmio(RCC_APB2ENR) |= RCC_APB2ENR_USART1EN;
    { volatile uint32_t sync = mmio(RCC_APB2ENR); (void)sync; }

    // No GPIO config: PA10 (RX) is a floating input out of reset, which is
    // exactly what USART RX wants, so GPIOA_CRH needn't be touched.

    // 115200 baud @ PCLK2 = 8 MHz: USARTDIV = 8,000,000 / (16*115200) =
    // 4.34 -> mantissa=4, fraction=round(0.34*16)=5 -> BRR = 0x45, the
    // same value USART3 uses (~0.6% fast, well within tolerance).
    mmio(USART1_BRR) = (4u << 4) | 5u;
    mmio(USART1_CR1) = USART1_CR1_UE | USART1_CR1_RE | USART1_CR1_RXNEIE;

    mmio(NVIC_ISER1) = NVIC_ISER1_USART1;
}

void poll() {
    if (!s_pending) return;
    uint8_t buf[kPvtLen];
    disable_irq();
    for (uint16_t i = 0; i < kPvtLen; ++i) buf[i] = s_slot[i];
    s_pending = false;
    enable_irq();
    parse_pvt(buf);
}

Stats snapshot() {
    return Stats{g_rx_bytes, g_sentences_ok, g_sentences_bad};
}

Fix read_fix() {
    return fix_snapshot();
}

}  // namespace gps
