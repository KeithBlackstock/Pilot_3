"""BLE link to the Pilot_3 flight controller via a CC41-A (HM-10 clone)
module on USART2.

The module is BLE-only (no classic SPP): it exposes serial service 0xDFB0
with a single characteristic 0xDFB1 (read / notify / write /
write-without-response). Its UART bridge is hard-locked to 9600 baud, which
is why the firmware runs USART2 at 9600 and streams telemetry at ~5 Hz.

BleLink mirrors pilot3_serial_link.SerialLink's interface exactly --
read_words(addr, n) / write_word(addr, word) / latest_telemetry() / error /
close() -- so the whole control layer (pilot3_fc_control) and the dashboard
work unchanged; only the transport differs. The frame codec is the serial
link's, imported from pilot3_serial_link.

Threading model: one daemon thread runs an asyncio loop owning the bleak
client. Notifications are demuxed like SerialLink's reader thread
(telemetry -> latest slot, RD/WR responses -> pending request). Requests
are serialized with a lock; responses are awaited with events, so callers
never touch asyncio directly. A disconnect or bleak error marks the link
dead -- close it and make a new one (the dashboard's reconnect loop does
this).

Request sizes: RD32 and WR32 frames are 11/14 bytes, under the default
ATT_MTU of 23 (20-byte payload), but writes are chunked at 20 bytes
regardless.

Run directly for a live link check:
    python tools/pilot3_ble_link.py [name-or-address]
"""
import asyncio
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pilot3_serial_link import (  # noqa: E402
    CMD_READ32, CMD_WRITE32, RSP_TELEMETRY, RSP_READ, RSP_WRITE, RSP_ERROR,
    ERROR_CODES, MAX_READ_WORDS, SRAM_LO, SRAM_HI, TELEMETRY_LEN,
    FrameParser, LinkError, encode_frame,
)
from pilot3_attitude_monitor import STRUCT_FMT  # noqa: E402

try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakError
except ImportError:  # bleak missing -- only constructing a BleLink needs it
    BleakClient = None
    BleakScanner = None

    class BleakError(Exception):
        pass

# CC41-A serial service/characteristic (NOT the usual HM-10 0xFFE0/0xFFE1).
CHAR_UUID = "0000dfb1-0000-1000-8000-00805f9b34fb"

# GATT writes are chunked to the default ATT_MTU payload.
WRITE_CHUNK = 20

# A request round-trip at 9600 baud: WR32 ack is 14 bytes (~15 ms on the
# wire), the biggest RD32 (40 words) is 170 bytes (~180 ms), plus BLE
# connection-interval latency per 20-byte notification chunk. 3 s is
# comfortable margin.
DEFAULT_TIMEOUT = 3.0

DEFAULT_NAME = "Pilot3"


def _looks_like_address(target):
    return ":" in target and len(target) >= 17


class BleLink:
    """Owns the BLE connection. See module docstring for the contract."""

    def __init__(self, target=DEFAULT_NAME, timeout=DEFAULT_TIMEOUT,
                 scan_timeout=8.0, connect_timeout=15.0):
        if BleakClient is None:
            raise LinkError("bleak is required:  pip install --user bleak")
        self.target, self.timeout = target, timeout
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True,
                                        name="pilot3-ble")
        self._thread.start()
        self._parser = FrameParser()
        self._lock = threading.Lock()          # one request in flight
        self._resp_event = threading.Event()
        self._resp = None                      # (type, payload) of last response
        self._latest = None                    # (decoded tuple, monotonic ts)
        self._latest_lock = threading.Lock()
        self._error = None
        self._client = None
        try:
            fut = asyncio.run_coroutine_threadsafe(
                self._connect(target, scan_timeout, connect_timeout), self._loop)
            fut.result(scan_timeout + connect_timeout + 10.0)
        except Exception:
            self._shutdown()
            raise

    # ---- asyncio loop thread ----

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    async def _connect(self, target, scan_timeout, connect_timeout):
        device = target
        if not _looks_like_address(target):
            found = await BleakScanner.discover(timeout=scan_timeout)
            by_name = {d.name: d for d in found if d.name}
            device = by_name.get(target)
            if device is None:
                seen = ", ".join(sorted(by_name)) or "nothing"
                raise LinkError(f"BLE device {target!r} not found "
                                f"(saw: {seen})")
        client = BleakClient(device, timeout=connect_timeout,
                             disconnected_callback=self._on_disconnect)
        try:
            await client.connect()
            await client.start_notify(CHAR_UUID, self._on_notify)
        except (BleakError, OSError, asyncio.TimeoutError) as e:
            try:
                await client.disconnect()
            except Exception:
                pass
            raise LinkError(f"BLE connect to {target!r} failed: {e}")
        self._client = client

    def _on_disconnect(self, _client):
        # Called by bleak from its own thread on link loss.
        self._error = "BLE link lost (device disconnected)"
        self._resp_event.set()  # wake any waiter so it raises

    def _on_notify(self, _char, data):
        # Demux identical to SerialLink._read_loop.
        for ftype, payload in self._parser.feed(bytes(data)):
            if ftype == RSP_TELEMETRY:
                if len(payload) == TELEMETRY_LEN:
                    with self._latest_lock:
                        self._latest = (struct.unpack(STRUCT_FMT, payload),
                                        time.monotonic())
            else:
                self._resp = (ftype, payload)
                self._resp_event.set()

    # ---- lifecycle ----

    @property
    def error(self):
        return self._error

    def _shutdown(self):
        try:
            if self._client is not None:
                fut = asyncio.run_coroutine_threadsafe(
                    self._client.disconnect(), self._loop)
                fut.result(5.0)
        except Exception:
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5.0)

    def close(self):
        self._shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()

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

    async def _write_frame(self, frame):
        for off in range(0, len(frame), WRITE_CHUNK):
            await self._client.write_gatt_char(
                CHAR_UUID, frame[off:off + WRITE_CHUNK], response=False)

    def _exchange(self, ftype, payload, expect, match, timeout=None):
        """Same semantics as SerialLink._exchange: send a request, wait for
        a response of type `expect` that match() accepts. Raises LinkError
        on timeout, FC error frame, or dead link."""
        if self._error:
            raise LinkError(self._error)
        deadline = time.monotonic() + (timeout or self.timeout)
        with self._lock:
            self._resp = None
            self._resp_event.clear()
            try:
                fut = asyncio.run_coroutine_threadsafe(
                    self._write_frame(encode_frame(ftype, payload)), self._loop)
                fut.result(5.0)
            except (BleakError, OSError, asyncio.TimeoutError) as e:
                raise LinkError(f"BLE write failed: {e}")
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


# ---- live link check ----

def _live_check(target):
    print(f"connecting to {target!r} ...")
    with BleLink(target) as link:
        print("connected; waiting for telemetry ...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            tele, age = link.latest_telemetry()
            if tele is not None and age < 1.0:
                break
            time.sleep(0.1)
        else:
            raise LinkError("no telemetry within 5 s (firmware streaming? "
                            "USART2 must run 9600 for this module)")
        tele, age = link.latest_telemetry()
        print(f"telemetry: seq={tele[0]} roll={tele[1]:.2f} "
              f"pitch={tele[2]:.2f} (age {age:.2f} s)")

        # RD32 round-trip: read the link's own stats block twice. The
        # payload is a snapshot taken before cmds_ok is incremented for the
        # request itself, so the second read must show +1 command and +11
        # rx_bytes (one 11-byte RD32 request frame) over the first.
        from pilot3_fc_control import symbol_address
        stats_addr = symbol_address("g_bt_stats")
        before = link.read_words(stats_addr, 4)
        after = link.read_words(stats_addr, 4)
        print(f"g_bt_stats over BLE: rx_bytes={after[0]} crc_bad={after[1]} "
              f"tx_dropped={after[2]} cmds_ok={after[3]}")
        if after[3] != before[3] + 1:
            raise LinkError(f"cmds_ok did not increment: {before[3]} -> {after[3]}")
        if after[0] - before[0] != 11:
            raise LinkError(f"rx_bytes delta {after[0] - before[0]} != 11 "
                            "(unexpected extra RX traffic)")
    print("ble link check OK")


if __name__ == "__main__":
    _live_check(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_NAME)
