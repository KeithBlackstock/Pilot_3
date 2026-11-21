#!/usr/bin/env python3
"""TCP listener transport for an ESP-01S running UART-WiFi passthrough.

The PC listens; the ESP connects as a TCP client. Once connected, the byte
stream and control contract are identical to pilot3_serial_link.SerialLink.
No third-party packages are required. Run directly for a loopback self-test.
"""
import socket
import struct
import threading
import time

from pilot3_serial_link import (
    CMD_READ32, CMD_WRITE32, ERROR_CODES, MAX_READ_WORDS, RSP_ERROR, RSP_READ,
    RSP_TELEMETRY, RSP_WRITE, SRAM_HI, SRAM_LO, TELEMETRY_LEN, FrameParser,
    LinkError, encode_frame,
)
from pilot3_attitude_monitor import STRUCT_FMT


class TcpLink:
    """One accepted TCP connection with SerialLink's dashboard interface."""

    def __init__(self, listen_host="0.0.0.0", port=8082, timeout=1.0,
                 accept_timeout=2.0):
        self.timeout = timeout
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((listen_host, port))
            listener.listen(1)
            listener.settimeout(accept_timeout)
            self._sock, self.peer = listener.accept()
        except socket.timeout:
            listener.close()
            raise LinkError(f"no ESP TCP connection on {listen_host}:{port}")
        except Exception:
            listener.close()
            raise
        listener.close()
        self._sock.settimeout(0.1)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._parser = FrameParser()
        self._lock = threading.Lock()          # one request in flight
        self._resp_event = threading.Event()
        self._resp = None
        self._latest = None                    # (decoded tuple, monotonic ts)
        self._latest_lock = threading.Lock()
        self._error = None
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    @property
    def error(self):
        return self._error

    def close(self):
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self._sock.close()
        except OSError:
            pass

    def _read_loop(self):
        while True:
            try:
                data = self._sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as e:
                self._error = f"TCP connection lost: {e}"
                self._resp_event.set()
                return
            if not data:
                self._error = "ESP closed the TCP connection"
                self._resp_event.set()
                return
            for ftype, payload in self._parser.feed(data):
                if ftype == RSP_TELEMETRY:
                    if len(payload) == TELEMETRY_LEN:
                        with self._latest_lock:
                            self._latest = (struct.unpack(STRUCT_FMT, payload),
                                            time.monotonic())
                else:
                    self._resp = (ftype, payload)
                    self._resp_event.set()

    def latest_telemetry(self):
        with self._latest_lock:
            if self._latest is None:
                return None, None
            t, ts = self._latest
        return t, time.monotonic() - ts

    def _exchange(self, ftype, payload, expect, match, timeout=None):
        if self._error:
            raise LinkError(self._error)
        deadline = time.monotonic() + (timeout or self.timeout)
        with self._lock:
            self._resp = None
            self._resp_event.clear()
            try:
                self._sock.sendall(encode_frame(ftype, payload))
            except OSError as e:
                raise LinkError(f"TCP write failed: {e}")
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise LinkError(f"no response from FC within {self.timeout:.1f} s")
                self._resp_event.wait(min(remaining, 0.2))
                self._resp_event.clear()
                if self._error:
                    raise LinkError(self._error)
                resp, self._resp = self._resp, None
                if resp is None:
                    continue
                rtype, rpayload = resp
                if rtype == RSP_ERROR and rpayload:
                    raise LinkError(
                        f"FC rejected the request: "
                        f"{ERROR_CODES.get(rpayload[0], f'code {rpayload[0]}')}")
                if rtype == expect and match(rpayload):
                    return rpayload

    @staticmethod
    def _check_addr(addr, n_words):
        if addr & 3 or not (SRAM_LO <= addr <= SRAM_HI - 4 * n_words):
            raise LinkError(f"address 0x{addr:08x} outside SRAM")

    def read_words(self, addr, n):
        if not (1 <= n <= MAX_READ_WORDS):
            raise LinkError(f"read of {n} words outside [1, {MAX_READ_WORDS}]")
        self._check_addr(addr, n)
        payload = self._exchange(
            CMD_READ32, struct.pack("<IB", addr, n), RSP_READ,
            lambda p: len(p) == 4 + 4 * n and struct.unpack_from("<I", p)[0] == addr)
        return list(struct.unpack_from(f"<{n}I", payload, 4))

    def write_word(self, addr, word):
        self._check_addr(addr, 1)
        want = struct.pack("<II", addr, word & 0xFFFFFFFF)
        self._exchange(CMD_WRITE32, want, RSP_WRITE, lambda p: p == want)


def _selftest():
    # Reserve an ephemeral loopback port, then have a fake FC connect to the
    # listener just as the ESP does in the field.
    tmp = socket.socket()
    tmp.bind(("127.0.0.1", 0))
    port = tmp.getsockname()[1]
    tmp.close()
    fake_error = []
    fake_stop = threading.Event()

    def fake_fc():
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=2)
            sock.settimeout(0.1)
            parser = FrameParser()
            sock.sendall(encode_frame(RSP_TELEMETRY, bytes(TELEMETRY_LEN)))
            while not fake_stop.is_set():
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                for ftype, payload in parser.feed(data):
                    if ftype == CMD_READ32:
                        addr, n = struct.unpack("<IB", payload)
                        sock.sendall(encode_frame(
                            RSP_READ, struct.pack("<I", addr) +
                            struct.pack(f"<{n}I", *range(1, n + 1))))
                    elif ftype == CMD_WRITE32:
                        sock.sendall(encode_frame(RSP_WRITE, payload))
            sock.close()
        except Exception as e:
            fake_error.append(e)

    thread = threading.Thread(target=fake_fc, daemon=True)
    thread.start()
    link = TcpLink("127.0.0.1", port, accept_timeout=2)
    try:
        deadline = time.monotonic() + 1
        while link.latest_telemetry()[0] is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert link.latest_telemetry()[0] is not None
        assert link.read_words(0x20000000, 3) == [1, 2, 3]
        link.write_word(0x20000004, 0xDEADBEEF)
    finally:
        fake_stop.set()
        link.close()
        thread.join(timeout=2)
    assert not fake_error, fake_error
    print("pilot3_tcp_link loopback self-test OK")


if __name__ == "__main__":
    _selftest()
