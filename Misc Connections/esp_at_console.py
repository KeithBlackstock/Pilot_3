#!/usr/bin/env python3
"""Line-oriented ESP-AT console through the Pilot_3 side-firmware mailbox.

Run flash.bat first, start/reuse OpenOCD (console.bat does this), then enter
AT commands. Enter appends CR LF. Local commands: :baud N, :esc (send a
bare +++ to leave saved passthrough) and :quit.
"""
import argparse
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from pilot3_swd_probe import Ocd  # noqa: E402

BASE = 0x20000000
MAGIC = 0x45535031
CAPACITY = 64
BAUD_REQUEST = BASE + 4
BAUD_ACTIVE = BASE + 8
HOST_WRITE = BASE + 12
FC_READ = BASE + 16
FC_WRITE = BASE + 20
HOST_READ = BASE + 24
RX_DROPPED = BASE + 28
TO_ESP = BASE + 32
FROM_ESP = BASE + 288


class AtConsole:
    def __init__(self, ocd):
        self.ocd = ocd
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.tx_seq = 0
        self.rx_seq = 0
        self.dropped = 0

    def write_word(self, addr, value):
        # pilot3_swd_probe.write_word deliberately allows only peripheral
        # setup addresses. This side utility's mailbox is in SRAM instead.
        self.ocd.send(f"mww 0x{addr:08x} 0x{value:08x}")

    def wait_ready(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                magic = self.ocd.read_word(BASE)
            if magic == MAGIC:
                with self.lock:
                    self.tx_seq = self.ocd.read_word(HOST_WRITE)
                    self.rx_seq = self.ocd.read_word(FC_WRITE)
                    self.dropped = self.ocd.read_word(RX_DROPPED)
                return
            time.sleep(0.1)
        raise RuntimeError(
            "ESP bridge mailbox not found -- flash Misc Connections side-firmware "
            "and reset the FC first")

    def set_baud(self, baud):
        if not 1200 <= baud <= 1000000:
            raise ValueError("baud must be between 1200 and 1000000")
        with self.lock:
            self.write_word(BAUD_REQUEST, baud)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with self.lock:
                active = self.ocd.read_word(BAUD_ACTIVE)
            if active == baud:
                print(f"\n[FC USART2 baud: {baud}]")
                return
            time.sleep(0.05)
        raise RuntimeError("FC did not acknowledge the baud change")

    def send_bytes(self, data):
        for b in data:
            while True:
                with self.lock:
                    consumed = self.ocd.read_word(FC_READ)
                    if self.tx_seq - consumed >= CAPACITY:
                        free = False
                    else:
                        index = self.tx_seq & (CAPACITY - 1)
                        self.write_word(TO_ESP + 4 * index, b)
                        self.tx_seq += 1
                        # Publish the byte only after its data word is written.
                        self.write_word(HOST_WRITE, self.tx_seq)
                        free = True
                if free:
                    break
                time.sleep(0.005)

    def send_burst(self, data):
        """Queue all of `data`, then publish it with one HOST_WRITE so the FC
        sends it back to back (send_bytes publishes per byte, which OpenOCD
        latency can spread across the ESP's packet boundary)."""
        assert len(data) <= CAPACITY
        while True:
            with self.lock:
                consumed = self.ocd.read_word(FC_READ)
                if CAPACITY - (self.tx_seq - consumed) >= len(data):
                    for i, b in enumerate(data):
                        self.write_word(TO_ESP + 4 * ((self.tx_seq + i) & (CAPACITY - 1)), b)
                    self.tx_seq += len(data)
                    self.write_word(HOST_WRITE, self.tx_seq)
                    return
            time.sleep(0.005)

    def escape_passthrough(self):
        """Leave ESP-AT transparent mode: a bare '+++' packet (no CR LF) with
        quiet time on both sides; AT commands work again afterwards."""
        time.sleep(1.1)
        self.send_burst(b"+++")
        time.sleep(1.1)
        print("\n[sent +++ -- try AT]")

    def pump_rx(self):
        try:
            while not self.stop.is_set():
                out = bytearray()
                with self.lock:
                    produced = self.ocd.read_word(FC_WRITE)
                    while self.rx_seq != produced:
                        index = self.rx_seq & (CAPACITY - 1)
                        out.append(self.ocd.read_word(FROM_ESP + 4 * index) & 0xFF)
                        self.rx_seq += 1
                    # Release consumed slots back to the FC producer.
                    self.write_word(HOST_READ, self.rx_seq)
                    dropped = self.ocd.read_word(RX_DROPPED)
                if out:
                    sys.stdout.write(out.decode(errors="replace"))
                    sys.stdout.flush()
                if dropped != self.dropped:
                    print(f"\n[ESP RX bytes dropped by mailbox: {dropped - self.dropped}]")
                    self.dropped = dropped
                time.sleep(0.02)
        except Exception as e:
            if not self.stop.is_set():
                print(f"\n[OpenOCD link lost: {e}]")
                self.stop.set()

    def close(self):
        self.stop.set()
        self.ocd.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="localhost", help="OpenOCD telnet host")
    ap.add_argument("--port", type=int, default=4444, help="OpenOCD telnet port")
    ap.add_argument("--baud", type=int, default=115200, help="ESP UART baud rate")
    args = ap.parse_args()

    ocd = Ocd(args.host, args.port)
    con = AtConsole(ocd)
    try:
        con.wait_ready()
        con.set_baud(args.baud)
        reader = threading.Thread(target=con.pump_rx, daemon=True)
        reader.start()
        print("Enter AT commands (CR LF is automatic). :baud N changes speed; "
              ":esc leaves passthrough; :quit exits.")
        for line in sys.stdin:
            cmd = line.strip()
            if cmd == ":quit":
                break
            if cmd.startswith(":baud"):
                try:
                    con.set_baud(int(cmd.split()[1]))
                except (ValueError, IndexError) as e:
                    print(f"Usage: :baud N ({e})")
                continue
            if cmd == ":esc":
                con.escape_passthrough()
                continue
            con.send_bytes(line.rstrip("\r\n").encode() + b"\r\n")
            if con.stop.is_set():
                break
    except KeyboardInterrupt:
        pass
    finally:
        con.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
