#!/usr/bin/env python3
"""Pilot_3 web dashboard -- a browser presentation of the same Telemetry
struct tools/pilot3_attitude_monitor.py prints to the terminal.

Data path is unchanged: poll the fixed SRAM struct over OpenOCD telnet
(`mdw`, see pilot3_attitude_monitor.py). This script only changes
presentation -- it serves tools/dashboard.html over HTTP and pushes JSON
snapshots to the page over a WebSocket:

    HTTP  http://127.0.0.1:8080/   -> dashboard.html
    WS    ws://127.0.0.1:8081/     -> telemetry JSON, ~--rate times/second

Requires the firmware already flashed and running, and OpenOCD already
attached (pilot3_fw_build_flash.bat does both). If either is missing the
page still loads and shows a "waiting for device" state until polling
succeeds.
"""
import argparse
import json
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from websockets.exceptions import ConnectionClosed
from websockets.sync.server import serve

from pilot3_attitude_monitor import NUM_CHANNELS, read_telemetry
from pilot3_swd_probe import Ocd

PAGE_PATH = Path(__file__).resolve().parent / "dashboard.html"


class Hub:
    """Latest telemetry snapshot + the set of connected browsers. The poll
    thread publishes; WebSocket handler threads only subscribe/unsubscribe."""

    def __init__(self):
        self._lock = threading.Lock()
        self._clients = set()
        self.latest = None  # last JSON string, replayed to new clients

    def publish(self, msg):
        data = json.dumps(msg)
        with self._lock:
            self.latest = data
            clients = list(self._clients)
        for conn in clients:
            try:
                conn.send(data)
            except ConnectionClosed:
                self.remove(conn)

    def add(self, conn):
        with self._lock:
            self._clients.add(conn)
            return self.latest

    def remove(self, conn):
        with self._lock:
            self._clients.discard(conn)


def poll_loop(hub, host, port, hz):
    """Forever: read the Telemetry struct and broadcast it. Reconnects to
    OpenOCD on failure so the page just shows 'disconnected' meanwhile."""
    ocd = None
    last_seq = None
    while True:
        if ocd is None:
            try:
                ocd = Ocd(host, port)
            except OSError:
                hub.publish({"connected": False,
                             "error": f"OpenOCD telnet unreachable at {host}:{port}"})
                time.sleep(2.0)
                continue
        try:
            f = read_telemetry(ocd)
        except Exception:
            # Telnet dropped or mdw parse failed -- reconnect next pass.
            try:
                ocd.close()
            except OSError:
                pass
            ocd = None
            continue
        (seq, roll_deg, pitch_deg, baro_temp_c, baro_pressure_pa, baro_altitude_m,
         ibus_rx_bytes, ibus_frames_ok, ibus_frames_bad) = f[:9]
        ch = list(f[9:9 + NUM_CHANNELS])
        pwm_s1_us, pwm_s4_us = f[9 + NUM_CHANNELS:9 + NUM_CHANNELS + 2]
        hub.publish({
            "connected": True,
            "stale": seq == last_seq,
            "seq": seq,
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "baro_temp_c": baro_temp_c,
            "baro_pressure_pa": baro_pressure_pa,
            "baro_altitude_m": baro_altitude_m,
            "ibus_rx_bytes": ibus_rx_bytes,
            "ibus_frames_ok": ibus_frames_ok,
            "ibus_frames_bad": ibus_frames_bad,
            "ibus_ch": ch,
            "pwm_s1_us": pwm_s1_us,
            "pwm_s4_us": pwm_s4_us,
            "gps": {**dict(zip(("lat_e7", "lon_e7", "alt_m", "speed_mps", "course_deg",
                                "rx_bytes", "sentences_ok", "sentences_bad", "sats", "fix"),
                               f[9 + NUM_CHANNELS + 2:])),
                    "lat_deg": f[9 + NUM_CHANNELS + 2] / 1e7,
                    "lon_deg": f[9 + NUM_CHANNELS + 3] / 1e7},
        })
        last_seq = seq
        time.sleep(1.0 / hz)


def ws_handler(hub, conn):
    """One browser client. We never expect inbound messages; the recv loop
    just holds the socket open until the browser goes away."""
    latest = hub.add(conn)
    if latest is not None:
        try:
            conn.send(latest)
        except ConnectionClosed:
            hub.remove(conn)
            return
    try:
        for _ in conn:
            pass
    except ConnectionClosed:
        pass
    finally:
        hub.remove(conn)


def make_page_handler(ws_port):
    class PageHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            body = PAGE_PATH.read_text(encoding="utf-8").replace("__WS_PORT__", str(ws_port)).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    return PageHandler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost", help="OpenOCD telnet host")
    ap.add_argument("--port", type=int, default=4444, help="OpenOCD telnet port")
    ap.add_argument("--rate", type=float, default=10.0, help="telemetry polls per second")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--ws-port", type=int, default=8081)
    ap.add_argument("--no-open", action="store_true", help="don't auto-open the browser")
    args = ap.parse_args()

    hub = Hub()

    threading.Thread(target=poll_loop, args=(hub, args.host, args.port, args.rate), daemon=True).start()

    ws_server = serve(lambda conn: ws_handler(hub, conn), "127.0.0.1", args.ws_port)
    threading.Thread(target=ws_server.serve_forever, daemon=True).start()

    url = f"http://127.0.0.1:{args.http_port}/"
    print(f"Dashboard: {url}  (WebSocket on :{args.ws_port}, polling OpenOCD {args.host}:{args.port})")
    if not args.no_open:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    try:
        ThreadingHTTPServer(("127.0.0.1", args.http_port), make_page_handler(args.ws_port)).serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ws_server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
