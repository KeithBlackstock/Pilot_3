#include "i2c1.hpp"
#include "regs.hpp"

using namespace reg;

namespace {

constexpr uint32_t kNibble = 0xF;

// Bounded spin instead of a real timeout: this driver has no SysTick
// dependency of its own, and a few thousand cycles is far more than one
// I2C bit period at 100 kHz / 8 MHz core clock needs.
constexpr int kSpinLimit = 100000;

bool wait_set(uint32_t addr, uint32_t mask) {
    for (int i = 0; i < kSpinLimit; ++i) {
        if (mmio(addr) & mask) return true;
    }
    return false;
}

bool wait_clear(uint32_t addr, uint32_t mask) {
    for (int i = 0; i < kSpinLimit; ++i) {
        if (!(mmio(addr) & mask)) return true;
    }
    return false;
}

// Clearing ADDR requires actually reading SR1 then SR2 (a bare (void)-cast
// on a volatile mmio() reference doesn't guarantee GCC emits the load) --
// this is a real hardware side effect, not just a memory-ordering barrier.
void clear_addr_flag() {
    volatile uint32_t sr2 = mmio(I2C1_SR2);
    (void)sr2;
}

// Shared prologue for both read_reg and write_reg: START, address+write,
// then write the register pointer. Returns false on NACK/timeout (STOP is
// already issued in that case).
bool start_and_point_reg(uint8_t addr7, uint8_t reg) {
    if (!wait_clear(I2C1_SR2, I2C1_SR2_BUSY)) return false;
    mmio(I2C1_CR1) |= I2C1_CR1_START;
    if (!wait_set(I2C1_SR1, I2C1_SR1_SB)) return false;
    mmio(I2C1_DR) = addr7 << 1;  // address, write
    if (!wait_set(I2C1_SR1, I2C1_SR1_ADDR | I2C1_SR1_AF)) return false;
    if (mmio(I2C1_SR1) & I2C1_SR1_AF) {
        mmio(I2C1_SR1) &= ~I2C1_SR1_AF;
        mmio(I2C1_CR1) |= I2C1_CR1_STOP;
        return false;
    }
    clear_addr_flag();
    if (!wait_set(I2C1_SR1, I2C1_SR1_TXE)) return false;
    mmio(I2C1_DR) = reg;
    return true;
}

}  // namespace

namespace i2c1 {

void init() {
    mmio(RCC_APB2ENR) |= RCC_APB2ENR_IOPBEN;
    { volatile uint32_t sync = mmio(RCC_APB2ENR); (void)sync; }
    mmio(RCC_APB1ENR) |= RCC_APB1ENR_I2C1EN;
    { volatile uint32_t sync = mmio(RCC_APB1ENR); (void)sync; }

    // PB6 SCL / PB7 SDA = AF open-drain (module's open-drain drivers; the
    // board's pull-ups do the rest) -- same as the probe's i2c1_setup().
    uint32_t crl = mmio(GPIOB_CRL);
    crl &= ~((kNibble << 24) | (kNibble << 28));
    crl |= (PIN_AF_OD_50 << 24) | (PIN_AF_OD_50 << 28);
    mmio(GPIOB_CRL) = crl;

    mmio(I2C1_CR1) = I2C1_CR1_SWRST;
    mmio(I2C1_CR1) = 0;
    mmio(I2C1_CR2) = 8;       // PCLK1 = 8 MHz
    mmio(I2C1_CCR) = 40;      // 100 kHz standard mode
    mmio(I2C1_TRISE) = 9;
    mmio(I2C1_CR1) = I2C1_CR1_PE;
}

bool read_reg(uint8_t addr7, uint8_t reg, uint8_t& out) {
    if (!start_and_point_reg(addr7, reg)) return false;
    if (!wait_set(I2C1_SR1, I2C1_SR1_BTF)) return false;

    mmio(I2C1_CR1) |= I2C1_CR1_START;  // repeated START
    if (!wait_set(I2C1_SR1, I2C1_SR1_SB)) return false;
    mmio(I2C1_DR) = (addr7 << 1) | 1;  // address, read
    if (!wait_set(I2C1_SR1, I2C1_SR1_ADDR | I2C1_SR1_AF)) return false;
    if (mmio(I2C1_SR1) & I2C1_SR1_AF) {
        mmio(I2C1_SR1) &= ~I2C1_SR1_AF;
        mmio(I2C1_CR1) |= I2C1_CR1_STOP;
        return false;
    }
    mmio(I2C1_CR1) &= ~I2C1_CR1_ACK;   // NACK the single byte we're about to get
    clear_addr_flag();
    mmio(I2C1_CR1) |= I2C1_CR1_STOP;
    if (!wait_set(I2C1_SR1, I2C1_SR1_RXNE)) return false;
    out = static_cast<uint8_t>(mmio(I2C1_DR) & 0xFF);
    return true;
}

bool write_reg(uint8_t addr7, uint8_t reg, uint8_t val) {
    if (!start_and_point_reg(addr7, reg)) return false;
    if (!wait_set(I2C1_SR1, I2C1_SR1_TXE)) return false;
    mmio(I2C1_DR) = val;
    if (!wait_set(I2C1_SR1, I2C1_SR1_BTF)) return false;
    mmio(I2C1_CR1) |= I2C1_CR1_STOP;
    return true;
}

}  // namespace i2c1
