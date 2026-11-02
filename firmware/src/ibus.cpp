#include "ibus.hpp"
#include "regs.hpp"

using namespace reg;

namespace {

constexpr uint32_t kNibble = 0xF;

constexpr uint8_t kFrameLen  = 32;
constexpr uint8_t kHdrLen    = 0x20;
constexpr uint8_t kHdrCmd    = 0x40;
constexpr uint8_t kPayloadLen = kFrameLen - 2;  // bytes summed for checksum

volatile uint32_t g_rx_bytes  = 0;
volatile uint32_t g_frames_ok = 0;
volatile uint32_t g_frames_bad = 0;
volatile uint16_t g_channels[ibus::kNumChannels] = {};

// ISR-only state -- never touched from the main loop.
uint8_t s_buf[kFrameLen];
uint8_t s_pos = 0;

inline void disable_irq() { asm volatile("cpsid i" ::: "memory"); }
inline void enable_irq()  { asm volatile("cpsie i" ::: "memory"); }

void on_byte(uint8_t b) {
    g_rx_bytes++;

    if (s_pos == 0) {
        if (b == kHdrLen) { s_buf[0] = b; s_pos = 1; }
        return;
    }
    if (s_pos == 1) {
        if (b == kHdrCmd) { s_buf[1] = b; s_pos = 2; }
        else { s_pos = (b == kHdrLen) ? 1 : 0; if (s_pos == 1) s_buf[0] = b; }
        return;
    }

    s_buf[s_pos++] = b;
    if (s_pos < kFrameLen) return;

    uint16_t sum = 0;
    for (uint8_t i = 0; i < kPayloadLen; ++i) sum += s_buf[i];
    uint16_t checksum = static_cast<uint16_t>(s_buf[kFrameLen - 2] | (s_buf[kFrameLen - 1] << 8));
    if (checksum == static_cast<uint16_t>(0xFFFFu - sum)) {
        g_frames_ok++;
        // Channels start at byte 2, little-endian, 2 bytes each. Only
        // written on a valid checksum -- a bad frame leaves the last
        // known-good values in place rather than corrupting them.
        for (int ch = 0; ch < ibus::kNumChannels; ++ch) {
            g_channels[ch] = static_cast<uint16_t>(s_buf[2 + ch * 2] | (s_buf[3 + ch * 2] << 8));
        }
    } else {
        g_frames_bad++;
    }
    s_pos = 0;
}

}  // namespace

extern "C" void USART3_IRQHandler() {
    // RXNE clears automatically on the DR read that follows the SR read --
    // standard USART ack-by-read pattern, same idea as I2C1's ADDR clear.
    if (mmio(USART3_SR) & USART3_SR_RXNE) {
        uint8_t b = static_cast<uint8_t>(mmio(USART3_DR) & 0xFF);
        on_byte(b);
    }
}

namespace ibus {

void init() {
    mmio(RCC_APB2ENR) |= RCC_APB2ENR_IOPBEN;
    { volatile uint32_t sync = mmio(RCC_APB2ENR); (void)sync; }
    mmio(RCC_APB1ENR) |= RCC_APB1ENR_USART3EN;
    { volatile uint32_t sync = mmio(RCC_APB1ENR); (void)sync; }

    // PB11 = RX, floating input (reset default, set explicitly for
    // clarity). PB10 = TX is left untouched -- iBus is receive-only here.
    uint32_t crh = mmio(GPIOB_CRH);
    crh &= ~(kNibble << 12);
    crh |= (PIN_IN_FLOAT << 12);
    mmio(GPIOB_CRH) = crh;

    // 115200 baud @ PCLK1 = 8 MHz: USARTDIV = 8,000,000 / (16*115200) =
    // 4.34 -> mantissa=4, fraction=round(0.34*16)=5 -> BRR = 0x45.
    mmio(USART3_BRR) = (4u << 4) | 5u;
    mmio(USART3_CR1) = USART3_CR1_UE | USART3_CR1_RE | USART3_CR1_RXNEIE;

    mmio(NVIC_ISER1) = NVIC_ISER1_USART3;
}

Stats snapshot() {
    return Stats{g_rx_bytes, g_frames_ok, g_frames_bad};
}

void read_channels(uint16_t out[kNumChannels]) {
    disable_irq();
    for (int i = 0; i < kNumChannels; ++i) out[i] = g_channels[i];
    enable_irq();
}

}  // namespace ibus
