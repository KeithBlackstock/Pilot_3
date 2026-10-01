#!/usr/bin/env python3
"""FC_bluepill board probe -- finds each active device on the board via SWD.

Talks to an already-running OpenOCD telnet session (default localhost:4444)
and pokes the GD32F303's peripheral registers directly (mdw/mww) while the
target keeps running. No firmware, no halt, no UART -- just the ST-Link on
the SWD1 header. Start OpenOCD separately first (bluepill_swd_probe.bat does
this), e.g.:

    openocd -f interface/stlink.cfg -f target/stm32f3x.cfg

then run this script alongside it. Checks, in order:

  1. MCU  -- DBGMCU_IDCODE @ 0xE0042000 reads back sane (SWD link alive).
  2. XTAL -- Y1 8 MHz HSE: set HSEON, HSERDY must follow (SYSCLK stays on
             HSI throughout, so this is safe even with a dead crystal).
  3. IMU  -- ICM-42688-P on SPI2 (SCK=PB13, MISO=PB14, MOSI=PB15, CS=PB5).
             WHO_AM_I (reg 0x75) must read 0x47.
  4. INT1 -- latch gyro data-ready onto the ICM's INT1 pin (push-pull,
             active-high, latched); PA0 must go high in GPIOA_IDR, then low
             again after INT_STATUS is read. GPIOA is read-only here.
  5. Baro -- BMP280 on I2C1 (SCL=PB6, SDA=PB7), addr 0x77 (SDO strapped
             high). Chip ID (reg 0xD0) must read 0x58.

PA15 CONSTRAINT -- READ BEFORE EDITING:
  PA15 is shorted to GND on this board (layout mistake). It must stay in its
  reset state (JTAG TDI input -- an input is safe against the short). Driving
  it as an output would be a direct short to ground. This script therefore
  NEVER writes GPIOA or AFIO registers; every bus pin it uses is on GPIOB.
  write_word() hard-refuses addresses in the GPIOA/AFIO ranges -- do not
  remove that guard.

Assumes the chip is in its post-reset clock state (HSI 8 MHz, PCLK1 = 8 MHz):
SPI2 runs at fPCLK/256 (~31 kHz) and I2C1 at 100 kHz, both valid at 8 MHz.
"""
import argparse
import re
import socket
import sys
import time

# ---------------------------------------------------------------- registers
RCC_BASE   = 0x40021000
RCC_CR      = RCC_BASE + 0x00   # bit 16 = HSEON, bit 17 = HSERDY
RCC_APB2ENR = RCC_BASE + 0x18   # bit 3 = IOPBEN (GPIOB clock)
RCC_APB1ENR = RCC_BASE + 0x1C   # bit 14 = SPI2EN, bit 21 = I2C1EN

GPIOB_BASE = 0x40010C00
GPIOB_CRL  = GPIOB_BASE + 0x00  # pins 0-7 config
GPIOB_CRH  = GPIOB_BASE + 0x04  # pins 8-15 config
GPIOB_BSRR = GPIOB_BASE + 0x10

SPI2_BASE = 0x40003800
SPI2_CR1  = SPI2_BASE + 0x00
SPI2_SR   = SPI2_BASE + 0x08
SPI2_DR   = SPI2_BASE + 0x0C

I2C1_BASE = 0x40005400
I2C1_CR1  = I2C1_BASE + 0x00
I2C1_CR2  = I2C1_BASE + 0x04
I2C1_CCR  = I2C1_BASE + 0x1C
I2C1_TRISE = I2C1_BASE + 0x20
I2C1_SR1  = I2C1_BASE + 0x14
I2C1_SR2  = I2C1_BASE + 0x18
I2C1_DR   = I2C1_BASE + 0x10

DBGMCU_IDCODE = 0xE0042000

# GPIO config nibbles (F1-style CRL/CRH): MODE+CNF
PIN_OUT_PP_50  = 0x3   # general purpose output push-pull, 50 MHz
PIN_AF_PP_50   = 0xB   # alternate function push-pull, 50 MHz
PIN_AF_OD_50   = 0xF   # alternate function open-drain, 50 MHz
PIN_IN_FLOAT   = 0x4   # floating input

ICM_WHO_AM_I_EXPECT = 0x47
BARO_ID_EXPECT      = 0x58   # BMP280 (0x60 would be BME280)
BARO_ADDR           = 0x77   # SDO tied high on this board

# PA15 guard: writes into these ranges are a hard error. GPIOA owns PA15
# (which is grounded on this board) and AFIO_MAPR could re-assign it.
FORBIDDEN_WRITES = [
    (0x40010800, 0x40010C00, "GPIOA"),
    (0x40010000, 0x40010400, "AFIO"),
]
ALLOWED_WRITES = [
    (RCC_BASE,   RCC_BASE + 0x400),   # clock enables only
    (GPIOB_BASE, GPIOB_BASE + 0x400), # every pin we drive is on port B
    (SPI2_BASE,  SPI2_BASE + 0x400),
    (I2C1_BASE,  I2C1_BASE + 0x400),
]

# ------------------------------------------------------------ telnet client
class Ocd:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.send("")  # drain banner/initial prompt

    def drain(self, timeout=0.05):
        self.sock.settimeout(timeout)
        try:
            while self.sock.recv(4096):
                pass
        except socket.timeout:
            pass

    def send(self, cmd):
        self.drain()
        self.sock.sendall((cmd + "\n").encode())
        data = b""
        self.sock.settimeout(1.0)
        try:
            while True:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                data += chunk
                if data.endswith(b"> ") or data.endswith(b"\xfb\x01"):
                    break
        except socket.timeout:
            pass
        return data.decode(errors="replace")

    def read_word(self, addr):
        out = self.send(f"mdw 0x{addr:08x}")
        m = re.search(r"0x[0-9a-fA-F]+:\s+([0-9a-fA-F]{8})", out)
        if not m:
            raise RuntimeError(f"mdw 0x{addr:08x} failed: {out.strip()!r}")
        return int(m.group(1), 16)

    def write_word(self, addr, value):
        for lo, hi, name in FORBIDDEN_WRITES:
            if lo <= addr < hi:
                raise AssertionError(
                    f"write to {name} (0x{addr:08x}) refused -- PA15 is "
                    f"grounded on this board, GPIOA/AFIO are off-limits")
        if not any(lo <= addr < hi for lo, hi in ALLOWED_WRITES):
            raise AssertionError(f"write to 0x{addr:08x} outside allowed regions")
        self.send(f"mww 0x{addr:08x} 0x{value:08x}")

    def rmw(self, addr, clear, set_):
        self.write_word(addr, (self.read_word(addr) & ~clear) | set_)

    def wait_bits(self, addr, mask, timeout=0.5, want=None):
        """Poll (reg & mask) until it matches: by default, any bit set (so a
        multi-bit mask means "whichever flag comes first"); with `want`,
        exact equality (want=0 waits for all mask bits clear)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            v = self.read_word(addr) & mask
            if (v != 0) if want is None else (v == want):
                return True
            time.sleep(0.001)
        return False

    def close(self):
        self.sock.close()

# ------------------------------------------------------------------- probes
def probe_mcu(ocd):
    """SWD link + DBGMCU_IDCODE. GD32F303 IDCODE isn't a documented STM32
    value, so just require a non-blank, non-stuck read."""
    idcode = ocd.read_word(DBGMCU_IDCODE)
    ok = idcode not in (0x00000000, 0xFFFFFFFF)
    return ok, f"DBGMCU_IDCODE = 0x{idcode:08x}"


def probe_crystal(ocd):
    """Y1 8 MHz HSE: turn on the HSE oscillator and wait for HSERDY. SYSCLK
    stays on HSI the whole time, so a dead crystal can't wedge the chip --
    this only proves the oscillator starts, not that it's exactly 8 MHz."""
    HSEON, HSERDY = 1 << 16, 1 << 17
    ocd.rmw(RCC_CR, 0, HSEON)
    t0 = time.monotonic()
    started = ocd.wait_bits(RCC_CR, HSERDY, timeout=0.5)
    start_ms = (time.monotonic() - t0) * 1000
    ocd.rmw(RCC_CR, HSEON, 0)  # leave the clock tree as we found it
    if not started:
        return False, "HSERDY never set -- crystal not oscillating (check Y1, C10/C11)"
    return True, f"HSERDY set, oscillator started in ~{start_ms:.0f} ms"


def spi2_setup(ocd):
    # Clocks: GPIOB + SPI2. No AFIO clock needed (no remapping used).
    ocd.rmw(RCC_APB2ENR, 0, 1 << 3)
    ocd.rmw(RCC_APB1ENR, 0, 1 << 14)
    # PB5 = CS. Set the output latch high BEFORE switching the pin to
    # output -- after reset ODR is 0, so configuring the pin first would
    # actively drive CS low for the milliseconds between the two telnet
    # round-trips. Latch-first means the pin comes up high.
    ocd.write_word(GPIOB_BSRR, 1 << 5)
    ocd.rmw(GPIOB_CRL, PIN_NIBBLE_MASK << 20, PIN_OUT_PP_50 << 20)
    # PB13 SCK = AF PP, PB14 MISO = floating input, PB15 MOSI = AF PP.
    ocd.rmw(GPIOB_CRH, PIN_NIBBLE_MASK << 20 | PIN_NIBBLE_MASK << 24 | PIN_NIBBLE_MASK << 28,
            PIN_AF_PP_50 << 20 | PIN_IN_FLOAT << 24 | PIN_AF_PP_50 << 28)
    # CR1: master, SPE, SSM+SSI (CS handled as GPIO), BR = fPCLK/256,
    # mode 0 (CPOL=0 CPHA=0), MSB first, 8-bit.
    ocd.write_word(SPI2_CR1, (1 << 2) | (1 << 6) | (7 << 3) | (1 << 8) | (1 << 9))


PIN_NIBBLE_MASK = 0xF


def spi2_xfer(ocd, byte):
    if not ocd.wait_bits(SPI2_SR, 1 << 1):  # TXE
        raise RuntimeError("SPI2 TXE timeout")
    ocd.write_word(SPI2_DR, byte)
    if not ocd.wait_bits(SPI2_SR, 1 << 0):  # RXNE
        raise RuntimeError("SPI2 RXNE timeout")
    return ocd.read_word(SPI2_DR) & 0xFF


def probe_imu(ocd):
    """ICM-42688-P WHO_AM_I (reg 0x75) over SPI2."""
    spi2_setup(ocd)
    ocd.write_word(GPIOB_BSRR, 1 << (5 + 16))   # CS low
    try:
        spi2_xfer(ocd, 0x75 | 0x80)             # read WHO_AM_I
        val = spi2_xfer(ocd, 0x00)
        ocd.wait_bits(SPI2_SR, 1 << 7, want=0)  # BSY clear
    finally:
        # Always terminate the transaction: a timeout mid-frame must not
        # leave CS low, or the ICM never sees a clean falling edge next scan.
        ocd.write_word(GPIOB_BSRR, 1 << 5)      # CS high
    if val == ICM_WHO_AM_I_EXPECT:
        return True, f"WHO_AM_I = 0x{val:02x} (ICM-42688-P found)"
    if val == 0x00:
        return False, "WHO_AM_I = 0x00 -- MISO stuck low (IMU absent/unpowered?)"
    if val == 0xFF:
        return False, "WHO_AM_I = 0xFF -- MISO stuck high (IMU absent/unpowered?)"
    return False, f"WHO_AM_I = 0x{val:02x} -- bus is alive but ID is wrong"


GPIOA_BASE = 0x40010800   # READ-ONLY here: IDR reads need no config writes,
GPIOA_IDR  = GPIOA_BASE + 0x08  # so the PA15 write guard is fully intact.


def icm_read(ocd, reg):
    ocd.write_word(GPIOB_BSRR, 1 << (5 + 16))   # CS low
    try:
        spi2_xfer(ocd, reg | 0x80)
        return spi2_xfer(ocd, 0x00)
    finally:
        ocd.write_word(GPIOB_BSRR, 1 << 5)      # CS high


def icm_write(ocd, reg, val):
    ocd.write_word(GPIOB_BSRR, 1 << (5 + 16))   # CS low
    try:
        spi2_xfer(ocd, reg)
        spi2_xfer(ocd, val)
    finally:
        ocd.write_word(GPIOB_BSRR, 1 << 5)      # CS high


def probe_imu_int(ocd):
    """INT1 (U3.4) -> PA0 (U1.10): latch gyro data-ready onto INT1 as
    push-pull/active-high, then watch PA0 in GPIOA_IDR. Latched mode holds
    the level until INT_STATUS is read, so telnet-speed polling can't miss
    it. GPIOA is only READ -- PA0 is a floating input by reset default.
    NOTE: GPIOA's clock (IOPAEN) must be on or IDR never samples the pins
    and reads as zeros -- an RCC write only, no PA15 config is touched."""
    spi2_setup(ocd)
    ocd.rmw(RCC_APB2ENR, 0, 1 << 2)  # IOPAEN: without it IDR reads are all 0
    icm_write(ocd, 0x64, 0x00)   # INT_CONFIG1: clear INT_ASYNC_RESET -- it
                                 # defaults to 1, and with it set the INT1/INT2
                                 # pins idle per polarity but NEVER assert
    icm_write(ocd, 0x14, 0x07)   # INT_CONFIG:  INT1 latched, push-pull, active high
    icm_write(ocd, 0x65, 0x08)   # INT_SOURCE0: route UI data-ready to INT1
    icm_write(ocd, 0x4E, 0x0F)   # PWR_MGMT0:   gyro+accel low-noise (DRDY @ 1kHz)
    try:
        if not ocd.wait_bits(GPIOA_IDR, 1 << 0, timeout=0.2):
            return False, "PA0 never went high with DRDY latched to INT1 (check U3.4 -> U1.10)"
        # Stop the sensors BEFORE clearing the latch: at 1kHz DRDY the next
        # sample would re-latch INT1 within ~1ms and look like a stuck-high
        # fault. Sensors off -> once cleared, the latch stays clear.
        icm_write(ocd, 0x4E, 0x00)
        time.sleep(0.005)        # let any in-flight sample land
        icm_read(ocd, 0x2D)      # INT_STATUS read clears the latch
        time.sleep(0.002)
        if ocd.read_word(GPIOA_IDR) & 1:
            return False, "PA0 stuck high after INT_STATUS read (short to a driven net?)"
        return True, "PA0 asserts on data-ready and clears on INT_STATUS read"
    finally:
        icm_write(ocd, 0x4E, 0x00)   # PWR_MGMT0: back to sleep, IMU quiet


def i2c1_setup(ocd):
    ocd.rmw(RCC_APB2ENR, 0, 1 << 3)
    ocd.rmw(RCC_APB1ENR, 0, 1 << 21)
    # PB6 SCL / PB7 SDA = AF open-drain (module's open-drain drivers; the
    # board's R3/R4 1k pull-ups do the rest).
    ocd.rmw(GPIOB_CRL, PIN_NIBBLE_MASK << 24 | PIN_NIBBLE_MASK << 28,
            PIN_AF_OD_50 << 24 | PIN_AF_OD_50 << 28)
    ocd.write_word(I2C1_CR1, 1 << 15)   # SWRST: reset the peripheral
    ocd.write_word(I2C1_CR1, 0)
    ocd.write_word(I2C1_CR2, 8)         # PCLK1 = 8 MHz
    ocd.write_word(I2C1_CCR, 40)        # 100 kHz standard mode
    ocd.write_word(I2C1_TRISE, 9)
    ocd.write_word(I2C1_CR1, 1 << 0)    # PE


def i2c1_read_reg(ocd, addr7, reg):
    """Single-register read; returns the byte, or None on address NACK
    (device not present)."""
    if not ocd.wait_bits(I2C1_SR2, 1 << 1, want=0):  # BUSY clear
        return None
    ocd.rmw(I2C1_CR1, 0, 1 << 8)                      # START
    if not ocd.wait_bits(I2C1_SR1, 1 << 0):           # SB
        return None
    ocd.write_word(I2C1_DR, addr7 << 1)               # address, write
    if not ocd.wait_bits(I2C1_SR1, (1 << 1) | (1 << 10)):
        return None
    if ocd.read_word(I2C1_SR1) & (1 << 10):           # AF = address NACK
        ocd.rmw(I2C1_SR1, 1 << 10, 0)                 # clear AF
        ocd.rmw(I2C1_CR1, 0, 1 << 9)                  # STOP
        return None
    ocd.read_word(I2C1_SR2)                           # clear ADDR
    if not ocd.wait_bits(I2C1_SR1, 1 << 7):           # TxE
        return None
    ocd.write_word(I2C1_DR, reg)
    if not ocd.wait_bits(I2C1_SR1, 1 << 2):           # BTF
        return None
    ocd.rmw(I2C1_CR1, 0, 1 << 8)                      # repeated START
    if not ocd.wait_bits(I2C1_SR1, 1 << 0):
        return None
    ocd.write_word(I2C1_DR, (addr7 << 1) | 1)         # address, read
    if not ocd.wait_bits(I2C1_SR1, (1 << 1) | (1 << 10)):
        return None
    if ocd.read_word(I2C1_SR1) & (1 << 10):
        ocd.rmw(I2C1_SR1, 1 << 10, 0)
        ocd.rmw(I2C1_CR1, 0, 1 << 9)
        return None
    ocd.rmw(I2C1_CR1, 1 << 10, 0)                     # ACK off (1-byte read)
    ocd.read_word(I2C1_SR2)                           # clear ADDR
    ocd.rmw(I2C1_CR1, 0, 1 << 9)                      # STOP
    if not ocd.wait_bits(I2C1_SR1, 1 << 6):           # RxNE
        return None
    return ocd.read_word(I2C1_DR) & 0xFF


def probe_baro(ocd):
    """BMP280 chip ID (reg 0xD0) on I2C1 at 0x77."""
    i2c1_setup(ocd)
    val = i2c1_read_reg(ocd, BARO_ADDR, 0xD0)
    if val is None:
        return False, f"no ACK at 0x{BARO_ADDR:02x} -- baro absent or bus fault"
    if val == BARO_ID_EXPECT:
        return True, f"chip ID = 0x{val:02x} (BMP280 found)"
    if val == 0x60:
        return False, "chip ID = 0x60 -- that's a BME280, not a BMP280"
    return False, f"chip ID = 0x{val:02x} -- device answers, but ID is wrong"


def blink_led(ocd, times=3):
    """PB1 = board LED (U1.19, active high via R2). GPIOB only, PA15 unaffected."""
    ocd.rmw(GPIOB_CRL, PIN_NIBBLE_MASK << 4, PIN_OUT_PP_50 << 4)
    for _ in range(times):
        ocd.write_word(GPIOB_BSRR, 1 << 1)
        time.sleep(0.15)
        ocd.write_word(GPIOB_BSRR, 1 << (1 + 16))
        time.sleep(0.15)


# --------------------------------------------------------------------- main
def scan(ocd):
    results = []
    for name, fn in (("MCU  (GD32F303, SWD)", probe_mcu),
                     ("XTAL (Y1 8MHz HSE)", probe_crystal),
                     ("IMU  (ICM-42688-P, SPI2)", probe_imu),
                     ("INT1 (ICM DRDY -> PA0)", probe_imu_int),
                     ("Baro (BMP280 @ 0x77, I2C1)", probe_baro)):
        try:
            ok, detail = fn(ocd)
        except (RuntimeError, AssertionError) as e:
            ok, detail = False, f"probe error: {e}"
        results.append(ok)
        print(f"  [{' OK ' if ok else 'FAIL'}] {name:28s} {detail}")
    return all(results)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=4444)
    ap.add_argument("--watch", type=float, default=0,
                    help="seconds between scans; 0 = single scan")
    ap.add_argument("--no-blink", action="store_true", help="skip the end-of-scan LED blink")
    args = ap.parse_args()

    try:
        ocd = Ocd(args.host, args.port)
    except OSError as e:
        print(f"Cannot reach OpenOCD telnet at {args.host}:{args.port} -- {e}")
        print("Start it first, e.g.: openocd -f interface/stlink.cfg -f target/stm32f3x.cfg")
        return 2

    print("FC_bluepill probe. PA15 (grounded) is never touched; all bus pins are GPIOB.")
    try:
        while True:
            try:
                all_ok = scan(ocd)
            except (OSError, RuntimeError) as e:
                print(f"<scan failed -- OpenOCD link lost? {e}>")
                return 1
            if not args.watch:
                if all_ok and not args.no_blink:
                    blink_led(ocd)
                print(f"\nResult: {'ALL DEVICES FOUND' if all_ok else 'MISSING DEVICE(S)'}")
                return 0 if all_ok else 1
            time.sleep(args.watch)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        ocd.close()


if __name__ == "__main__":
    sys.exit(main())
