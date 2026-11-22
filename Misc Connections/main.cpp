// Temporary ESP-01S setup firmware: OpenOCD SRAM mailbox <-> USART2.
// This is not flight firmware. No IMU, motor, servo, GPS, or other peripherals
// are initialized. PA2 = ESP RX, PA3 = ESP TX; PA15 remains untouched.
#include <cstddef>
#include <cstdint>

#include "regs.hpp"

using namespace reg;

namespace {

constexpr uint32_t kMagic = 0x45535031u;  // "ESP1"
constexpr uint32_t kDefaultBaud = 115200;
constexpr uint32_t kPclk1Hz = 8000000;
constexpr uint32_t kCapacity = 64;

// All entries are 32-bit words so OpenOCD mdw/mww accesses are simple and
// atomic. Buffers store one byte per word. Counters increase monotonically;
// their unsigned difference is the number of bytes in flight.
struct Bridge {
    uint32_t magic;
    uint32_t baud_request;
    uint32_t baud_active;
    uint32_t host_write;   // host -> ESP producer
    uint32_t fc_read;      // host -> ESP consumer
    uint32_t fc_write;     // ESP -> host producer
    uint32_t host_read;    // ESP -> host consumer
    uint32_t rx_dropped;
    uint32_t to_esp[kCapacity];
    uint32_t from_esp[kCapacity];
};

static_assert(offsetof(Bridge, to_esp) == 32, "host-to-ESP mailbox offset changed");
static_assert(offsetof(Bridge, from_esp) == 288, "ESP-to-host mailbox offset changed");

volatile Bridge g_bridge __attribute__((section(".telemetry")));

bool valid_baud(uint32_t baud) {
    return baud >= 1200 && baud <= 1000000;
}

void set_baud(uint32_t baud) {
    mmio(USART2_CR1) = 0;
    // GD32F303 USART BRR is PCLK/baud in the usual F1-style fixed-point form.
    mmio(USART2_BRR) = (kPclk1Hz + baud / 2) / baud;
    mmio(USART2_CR1) = USART2_CR1_UE | USART2_CR1_RE | USART2_CR1_TE;
    g_bridge.baud_active = baud;
}

void usart2_init(uint32_t baud) {
    mmio(RCC_APB2ENR) |= RCC_APB2ENR_IOPAEN;
    { volatile uint32_t sync = mmio(RCC_APB2ENR); (void)sync; }
    mmio(RCC_APB1ENR) |= RCC_APB1ENR_USART2EN;
    { volatile uint32_t sync = mmio(RCC_APB1ENR); (void)sync; }

    // PA2 TX = alternate-function push-pull. PA3 RX stays a floating input.
    uint32_t crl = mmio(GPIOA_CRL);
    crl &= ~(0xFu << 8);
    crl |= PIN_AF_PP_50 << 8;
    mmio(GPIOA_CRL) = crl;
    set_baud(baud);
}

}  // namespace

// The shared startup vector table references these handlers directly. This
// polling-only utility never enables their interrupts.
extern "C" void USART1_IRQHandler() {}
extern "C" void USART2_IRQHandler() {}
extern "C" void USART3_IRQHandler() {}

int main() {
    g_bridge.magic = 0;
    g_bridge.baud_request = 0;
    g_bridge.baud_active = 0;
    g_bridge.host_write = 0;
    g_bridge.fc_read = 0;
    g_bridge.fc_write = 0;
    g_bridge.host_read = 0;
    g_bridge.rx_dropped = 0;

    usart2_init(kDefaultBaud);
    g_bridge.magic = kMagic;  // publish last: mailbox and UART are ready

    while (true) {
        uint32_t requested = g_bridge.baud_request;
        if (valid_baud(requested) && requested != g_bridge.baud_active) {
            set_baud(requested);
        }

        if (mmio(USART2_SR) & USART2_SR_RXNE) {
            uint32_t b = mmio(USART2_DR) & 0xFF;
            uint32_t write = g_bridge.fc_write;
            if (write - g_bridge.host_read < kCapacity) {
                g_bridge.from_esp[write & (kCapacity - 1)] = b;
                g_bridge.fc_write = write + 1;
            } else {
                g_bridge.rx_dropped = g_bridge.rx_dropped + 1;
            }
        }

        uint32_t read = g_bridge.fc_read;
        if (read != g_bridge.host_write && (mmio(USART2_SR) & USART2_SR_TXE)) {
            uint32_t b = g_bridge.to_esp[read & (kCapacity - 1)];
            mmio(USART2_DR) = b & 0xFF;
            g_bridge.fc_read = read + 1;
        }
    }
}
