"""Frame codec for the Pilot_3 wireless link (USART2 framing).

Client side of the protocol implemented by firmware/src/bt.cpp (the
constants and framing below mirror it exactly). The codec (encode_frame,
FrameParser, constants) is used by pilot3_tcp_link.py, the ESP-01S WiFi
transport.

    0xA5 0x5A | type u8 | len u8 | payload[len] | crc16 lo | crc16 hi
    CRC16-CCITT (poly 0x1021, init 0xFFFF) over type..payload

    host->FC  0x10 RD32 {addr u32, count u8}   -> 0x90 {addr u32, words...}
    host->FC  0x11 WR32 {addr u32, value u32}  -> 0x91 {addr, value} (echo)
    FC->host  0x81 TELEMETRY {packed Telemetry}  (pushed at ~10 Hz)
    FC->host  0x9F ERROR {code u8}   1=bad addr  2=bad count  3=bad type

Telemetry frames are decoded with pilot3_attitude_monitor.STRUCT_FMT, the
exact layout tools/pilot3_dashboard.py already consumes.

Run directly for a codec self-test:  python tools/pilot3_serial_link.py
"""
import struct
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
        + (10, -20, 30, 1, 123.5, 124.5, 7000, 2) + (0.5, 12.75, 5.0, -10.0)  # float32-exact values only
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
