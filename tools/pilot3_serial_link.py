"""Frame codec for the Pilot_3 Bluetooth link, plus the legacy classic-SPP link.

Client side of the protocol implemented by firmware/src/bt.cpp (the
constants and framing below mirror it exactly). The codec (encode_frame,
FrameParser, constants) is shared with pilot3_ble_link.py, the supported
CC41-A BLE transport. SerialLink is the legacy path for a classic SPP module
(HC-05 etc.) presenting a plain COM port; it needs pyserial and a firmware
build with USART2 at 115200 (the CC41-A bridge is locked to 9600).

    0xA5 0x5A | type u8 | len u8 | payload[len] | crc16 lo | crc16 hi
    CRC16-CCITT (poly 0x1021, init 0xFFFF) over type..payload

    host->FC  0x10 RD32 {addr u32, count u8}   -> 0x90 {addr u32, words...}
    host->FC  0x11 WR32 {addr u32, value u32}  -> 0x91 {addr, value} (echo)
    FC->host  0x81 TELEMETRY {packed Telemetry}  (pushed at ~5 Hz)
    FC->host  0x9F ERROR {code u8}   1=bad addr  2=bad count  3=bad type

SerialLink exposes read_words()/write_word() with the same semantics as the
word-level primitives on pilot3_fc_control.OcdLink, so the whole control
layer (config, baro ref, mag cal, waypoints) works over Bluetooth unchanged.
Telemetry frames are decoded with pilot3_attitude_monitor.STRUCT_FMT, the
exact layout tools/pilot3_dashboard.py already consumes.

Run directly for a codec self-test:  python tools/pilot3_serial_link.py
"""
import struct
import threading
import time

try:
    import serial  # pyserial -- only SerialLink needs it, not the shared codec
except ImportError:
    serial = None

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pilot3_attitude_monitor import STRUCT_FMT

SYNC0, SYNC1 = 0xA5, 0x5A
CMD_READ32 = 0x10
CMD_WRITE32 = 0x11
RSP_TELEMETRY = 0x81
RSP_READ = 0x90
RSP_WRITE = 0x91
RSP_ERROR = 0x9F

# Keep in sync with bt.cpp: reads/writes are restricted to this range on the
# firmware side, and RD32 is capped at 40 words per request.
SRAM_LO, SRAM_HI = 0x20000000, 0x2000C000
MAX_READ_WORDS = 40

TELEMETRY_LEN = struct.calcsize(STRUCT_FMT)

ERROR_CODES = {1: "bad address", 2: "bad count", 3: "bad type"}


class LinkError(RuntimeError):
    pass


def crc16(data, crc=0xFFFF):
    """CRC16-CCITT (poly 0x1021, init 0xFFFF) -- bt.cpp's crc_byte."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(ftype, payload=b""):
    body = bytes((ftype, len(payload))) + payload
    return bytes((SYNC0, SYNC1)) + body + struct.pack("<H", crc16(body))


class FrameParser:
    """Incremental byte-stream parser; feed() returns (type, payload) pairs.
    Buffer-based resync: on a CRC mismatch only the first sync byte is
    dropped, so a 0xA5 0x5A hiding inside a corrupt frame can still start
    the next valid one. Host-side stats mirror bt::Stats where applicable."""

    def __init__(self):
        self._buf = bytearray()
        self.rx_bytes = 0
        self.crc_bad = 0

    def feed(self, data):
        self.rx_bytes += len(data)
        self._buf += data
        out = []
        while True:
            # Find the next sync pair.
            start = self._buf.find(bytes((SYNC0, SYNC1)))
            if start < 0:
                # Keep a trailing lone SYNC0 -- it may be half a sync pair.
                keep = 1 if self._buf.endswith(bytes((SYNC0,))) else 0
                del self._buf[:len(self._buf) - keep]
                break
            if start:
                del self._buf[:start]
            if len(self._buf) < 6:  # sync(2) + type + len + crc(2)
                break
            flen = self._buf[3]
            total = 4 + flen + 2
            if len(self._buf) < total:
                break
            frame = bytes(self._buf[:total])
            want, = struct.unpack("<H", frame[4 + flen:])
            if crc16(frame[2:4 + flen]) == want:
                out.append((frame[2], frame[4:4 + flen]))
                del self._buf[:total]
            else:
                self.crc_bad += 1
                del self._buf[0]  # drop first sync byte, rescan
        return out


class SerialLink:
    """Owns the COM port. One daemon reader thread demuxes frames:
    telemetry updates the latest slot, RD/WR responses complete pending
    requests. Request/response ops are serialized with a lock, so callers
    from different threads (dashboard poll loop vs command handler) cannot
    interleave a request. A link that hits a serial error is dead -- close
    it and make a new one (the dashboard's reconnect loop does this)."""

    def __init__(self, port, baud=115200, timeout=1.0):
        if serial is None:
            raise LinkError("pyserial is required:  pip install pyserial")
        self.port, self.timeout = port, timeout
        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._parser = FrameParser()
        self._lock = threading.Lock()          # one request in flight
        self._resp_event = threading.Event()
        self._resp = None                      # (type, payload) of last response
        self._latest = None                    # (decoded tuple, monotonic ts)
        self._latest_lock = threading.Lock()
        self._error = None
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    # ---- lifecycle ----

    @property
    def error(self):
        return self._error

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

    # ---- reader thread ----

    def _read_loop(self):
        while True:
            try:
                data = self._ser.read(4096)
            except (serial.SerialException, OSError) as e:
                self._error = f"serial port lost: {e}"
                self._resp_event.set()  # wake any waiter so it raises
                return
            if not data:
                continue
            for ftype, payload in self._parser.feed(data):
                if ftype == RSP_TELEMETRY:
                    if len(payload) == TELEMETRY_LEN:
                        with self._latest_lock:
                            self._latest = (struct.unpack(STRUCT_FMT, payload),
                                            time.monotonic())
                else:
                    self._resp = (ftype, payload)
                    self._resp_event.set()

    # ---- telemetry ----

    def latest_telemetry(self):
        """(decoded tuple, age_seconds) or (None, None). The tuple matches
        pilot3_attitude_monitor.read_telemetry()'s field order exactly."""
        with self._latest_lock:
            if self._latest is None:
                return None, None
            t, ts = self._latest
        return t, time.monotonic() - ts

    # ---- request/response (serialized) ----

    def _exchange(self, ftype, payload, expect, match, timeout=None):
        """Send a request, wait for a response of type `expect` that match()
        accepts (late replies from an earlier timed-out request are skipped).
        Raises LinkError on timeout, FC error frame, or dead link."""
        if self._error:
            raise LinkError(self._error)
        deadline = time.monotonic() + (timeout or self.timeout)
        with self._lock:
            self._resp = None
            self._resp_event.clear()
            try:
                self._ser.write(encode_frame(ftype, payload))
            except (serial.SerialException, OSError) as e:
                raise LinkError(f"serial write failed: {e}")
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise LinkError(f"no response from FC within {self.timeout:.1f} s "
                                    "(link up but firmware not answering?)")
                self._resp_event.wait(min(remaining, 0.2))
                self._resp_event.clear()
                if self._error:
                    raise LinkError(self._error)
                resp, self._resp = self._resp, None
                if resp is None:
                    continue
                rtype, rpayload = resp
                if rtype == RSP_ERROR and rpayload:
                    raise LinkError(f"FC rejected the request: "
                                    f"{ERROR_CODES.get(rpayload[0], f'code {rpayload[0]}')}")
                if rtype == expect and match(rpayload):
                    return rpayload

    @staticmethod
    def _check_addr(addr, n_words):
        if addr & 3 or not (SRAM_LO <= addr <= SRAM_HI - 4 * n_words):
            raise LinkError(f"address 0x{addr:08x} outside SRAM "
                            f"[0x{SRAM_LO:08x}, 0x{SRAM_HI:08x})")

    def read_words(self, addr, n):
        """Read n 32-bit words at addr. Same contract as OcdLink.read_words."""
        if not (1 <= n <= MAX_READ_WORDS):
            raise LinkError(f"read of {n} words outside [1, {MAX_READ_WORDS}]")
        self._check_addr(addr, n)
        payload = self._exchange(
            CMD_READ32, struct.pack("<IB", addr, n), RSP_READ,
            lambda p: len(p) == 4 + 4 * n and struct.unpack_from("<I", p)[0] == addr)
        return list(struct.unpack_from(f"<{n}I", payload, 4))

    def write_word(self, addr, word):
        """Write one 32-bit word; the FC echoes addr+value as the ack.
        Same contract as OcdLink.write_word."""
        self._check_addr(addr, 1)
        want = struct.pack("<II", addr, word & 0xFFFFFFFF)
        self._exchange(CMD_WRITE32, want, RSP_WRITE, lambda p: p == want)


# ---- self-test ----

def _selftest():
    # CRC16-CCITT (0xFFFF init) of "123456789" is the canonical 0x29B1.
    assert crc16(b"123456789") == 0x29B1, hex(crc16(b"123456789"))

    # Round-trip: encode -> parse, including a frame split mid-payload.
    p = FrameParser()
    frames = encode_frame(RSP_READ, struct.pack("<III", 0x20000040, 111, 222)) \
        + encode_frame(CMD_WRITE32, struct.pack("<II", 0x20000044, 0xDEADBEEF))
    assert p.feed(frames[:7]) == []
    got = p.feed(frames[7:])
    assert got[0][0] == RSP_READ and struct.unpack("<III", got[0][1]) == (0x20000040, 111, 222)
    assert got[1] == (CMD_WRITE32, struct.pack("<II", 0x20000044, 0xDEADBEEF))

    # Garbage resync: noise + a corrupt frame + a good frame -> one result,
    # one CRC failure counted, and the good frame still parses when a fake
    # sync pair sits inside the corrupt one.
    p = FrameParser()
    corrupt = bytearray(encode_frame(RSP_READ, b"\x01\x02\x03\x04"))
    corrupt[5] ^= 0xFF
    good = encode_frame(RSP_ERROR, b"\x02")
    got = p.feed(b"\x00\xff\xa5" + bytes(corrupt) + good)
    assert got == [(RSP_ERROR, b"\x02")], got
    assert p.crc_bad >= 1

    # Telemetry decode: golden payload through the real STRUCT_FMT.
    p = FrameParser()
    values = (42, 1.5, -2.5, 20.0, 101325.0, 15.5, 100, 50, 0) \
        + tuple(1000 + i for i in range(14)) + (1500, 1520) \
        + (479912345, -1223456789, 100.0, 12.5, 90.0, 999, 888, 0, 9, 3) \
        + (10, -20, 30, 1, 123.5, 7000, 2) + (0.5, 12.75)  # float32-exact values only
    payload = struct.pack(STRUCT_FMT, *values)
    got = p.feed(encode_frame(RSP_TELEMETRY, payload))
    assert got and got[0][0] == RSP_TELEMETRY
    assert struct.unpack(STRUCT_FMT, got[0][1]) == values

    # encode/decode the request shapes the control layer uses.
    assert encode_frame(CMD_READ32, struct.pack("<IB", 0x20000000, 40))
    assert len(encode_frame(RSP_READ, bytes(4 + 4 * MAX_READ_WORDS))) == 4 + 164 + 2
    print("pilot3_serial_link self-test OK "
          f"(telemetry payload = {TELEMETRY_LEN} bytes)")


if __name__ == "__main__":
    _selftest()
