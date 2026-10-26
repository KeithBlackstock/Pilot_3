#!/usr/bin/env python3
"""Pilot_3 web dashboard -- a browser presentation of the same Telemetry
struct tools/pilot3_attitude_monitor.py prints to the terminal.

Data path is unchanged: poll the fixed SRAM struct over OpenOCD telnet
(`mdw`, see pilot3_attitude_monitor.py). This script only changes
presentation -- it serves tools/dashboard.html over HTTP and pushes JSON
snapshots to the page over a WebSocket:

    HTTP  http://127.0.0.1:8080/   -> dashboard.html
    WS    ws://127.0.0.1:8081/     -> telemetry JSON, ~--rate times/second

Requires the firmware already flashed and running. OpenOCD is adopted if an
instance is already answering on the telnet port (pilot3_dashboard.bat starts
one); otherwise the dashboard spawns its own. If the telnet link stays dead
for RESTART_AFTER_S, the wedged OpenOCD session is killed and respawned
automatically (--no-ocd-restart disables that). On exit, the dashboard kills
only an instance it spawned -- someone else's OpenOCD is left alone.
"""
import argparse
import json
import socket
import subprocess
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from websockets.exceptions import ConnectionClosed
from websockets.sync.server import serve

import pilot3_fc_control as fc
from pilot3_attitude_monitor import NUM_CHANNELS, read_telemetry
from pilot3_swd_probe import Ocd

PAGE_PATH = Path(__file__).resolve().parent / "dashboard.html"

# OpenOCD supervision (see module docstring). Same binary and args as
# pilot3_dashboard.bat.
OPENOCD_EXE = Path(r"C:\Users\goats\AppData\Local\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe\xpack-openocd-0.12.0-7\bin\openocd.exe")
OPENOCD_ARGS = ["-f", "interface/stlink.cfg", "-f", "target/stm32f3x.cfg", "-c", "adapter speed 950"]
RESTART_AFTER_S = 8.0  # link dead this long -> kill & respawn OpenOCD

_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


class OcdServer:
    """Owns the OpenOCD process behind the telnet link.

    Adopts a running instance if the port answers; otherwise spawns its own
    child. restart() kills whatever is there (a dead port means a wedged
    process) and spawns a fresh owned instance. shutdown() kills only an
    instance we spawned, matching the .bat's rule of leaving someone else's
    OpenOCD alone."""

    def __init__(self, host, port, auto=True):
        self.host, self.port, self.auto = host, port, auto
        self.child = None  # Popen, set only for instances we spawned

    def _port_up(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            try:
                socket.create_connection((self.host, self.port), timeout=1).close()
                return True
            except OSError:
                if time.monotonic() >= deadline:
                    return False
                time.sleep(0.2)

    def _spawn(self):
        self.child = subprocess.Popen(
            [str(OPENOCD_EXE), *OPENOCD_ARGS],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            creationflags=_NO_WINDOW)

    def ensure_up(self):
        """Adopt or spawn at startup. Returns 'adopted'/'started'/None."""
        if self._port_up(0.5):
            return "adopted"
        if not self.auto or not OPENOCD_EXE.exists():
            return None
        self._spawn()
        return "started" if self._port_up(6.0) else None

    def restart(self):
        """Link has been dead too long: kill the wedged session, spawn fresh.
        Returns False (no-op) when supervision is disabled or the binary is
        missing."""
        if not self.auto or not OPENOCD_EXE.exists():
            return False
        if self.child is not None:
            self.child.terminate()
            try:
                self.child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.child.kill()
            self.child = None
        else:
            subprocess.run(["taskkill", "/IM", "openocd.exe", "/F"],
                           capture_output=True, creationflags=_NO_WINDOW)
        self._spawn()
        self._port_up(6.0)
        return True

    def shutdown(self):
        if self.child is None:
            return
        self.child.terminate()
        try:
            self.child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.child.kill()
        self.child = None


class Hub:
    """Latest telemetry snapshot + the set of connected browsers. The poll
    thread publishes; WebSocket handler threads only subscribe/unsubscribe."""

    def __init__(self):
        self._lock = threading.Lock()
        self._clients = set()
        self.latest = None  # last JSON string, replayed to new clients
        self.sticky = {}    # last message of each non-telemetry type, also replayed

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

    def broadcast(self, msg, sticky=True):
        """Non-telemetry message (config / job status); replayed to new
        clients but never overwrites the telemetry snapshot."""
        data = json.dumps(msg)
        with self._lock:
            if sticky:
                self.sticky[msg["type"]] = data
            clients = list(self._clients)
        for conn in clients:
            try:
                conn.send(data)
            except ConnectionClosed:
                self.remove(conn)

    def add(self, conn):
        with self._lock:
            self._clients.add(conn)
            first = [self.latest] if self.latest else []
            return first + list(self.sticky.values())

    def remove(self, conn):
        with self._lock:
            self._clients.discard(conn)


def poll_loop(hub, host, port, hz, ctl, ocd_server):
    """Forever: read the Telemetry struct and broadcast it. Reconnects to
    OpenOCD on failure; if the link stays dead for RESTART_AFTER_S, the
    wedged OpenOCD session is killed and respawned so the dashboard
    recovers on its own."""
    ocd = None
    last_seq = None
    dead_since = None  # monotonic time the link first went down

    def note_dead(error):
        nonlocal dead_since
        now = time.monotonic()
        if dead_since is None:
            dead_since = now
        if now - dead_since >= RESTART_AFTER_S and ocd_server.restart():
            error += " -- OpenOCD was killed and restarted"
            dead_since = time.monotonic()  # the fresh instance gets a full window
        hub.publish({"connected": False, "error": error})

    while True:
        if ctl.busy.is_set():
            # Build/flash in progress: the target is being reset, don't poll.
            if ocd is not None:
                try:
                    ocd.close()
                except OSError:
                    pass
                ocd = None
            dead_since = None
            time.sleep(0.2)
            continue
        if ocd is None:
            try:
                ocd = Ocd(host, port)
            except OSError:
                note_dead(f"OpenOCD telnet unreachable at {host}:{port}")
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
            note_dead("OpenOCD link dropped")
            continue
        dead_since = None
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


class Controller:
    """Handles browser commands: leveler config and compile + flash. One
    background job at a time; telemetry polling pauses while it runs."""

    def __init__(self, hub, host, port):
        self.hub = hub
        self.link = fc.OcdLink(host, port)
        self.busy = threading.Event()
        self._job_lock = threading.Lock()
        self.live = None  # last values applied from the dashboard this session

    def _job(self, state, log=""):
        self.hub.broadcast({"type": "job", "state": state, "log": log})

    def _config(self, values, note=""):
        self.hub.broadcast({"type": "config", "values": values, "note": note})

    def handle(self, msg):
        cmd = msg.get("cmd")
        try:
            if cmd == "get_config":
                self._config(self.link.read_config(), "read from FC")
            elif cmd == "set_config":
                values = self.link.write_config(msg.get("values", {}))
                self.live = values
                self._config(values, "applied live (lost on reset unless saved)")
            elif cmd == "save_defaults":
                values = fc.validate(msg.get("values", {}))
                fc.save_defaults(values)
                # Keep the post-flash re-apply in sync with the new defaults;
                # otherwise _build_flash would clobber them with stale values.
                self.live = values
                self._config(values,
                             "saved to attitude_level.hpp -- takes effect after Compile & Flash")
            elif cmd == "build_flash":
                self._start_job()
            else:
                return
        except fc.ControlError as e:
            self.hub.broadcast({"type": "config_error", "error": str(e)}, sticky=False)
        except (OSError, RuntimeError) as e:
            self.hub.broadcast({"type": "config_error", "error": f"OpenOCD: {e}"}, sticky=False)

    def _start_job(self):
        if not self._job_lock.acquire(blocking=False):
            return
        self.busy.set()
        threading.Thread(target=self._build_flash, daemon=True).start()

    def _build_flash(self):
        try:
            self._job("building")
            ok, log = fc.build()
            if not ok:
                self._job("error", "BUILD FAILED\n" + log[-3000:])
                return
            self._job("flashing", log[-600:])
            try:
                self.link.flash()
            except (fc.ControlError, OSError) as e:
                self._job("error", f"FLASH FAILED: {e}")
                return
            time.sleep(1.0)  # let the firmware boot before touching its RAM
            note = ""
            try:
                if self.live is not None:
                    # Flash restored the compiled-in defaults; put back whatever
                    # the dashboard had applied so a re-flash doesn't silently
                    # revert tuning in progress.
                    self.link.write_config(self.live)
                    note = " Live-applied values were re-applied."
                self._config(self.link.read_config(), "read from FC after flash")
            except (fc.ControlError, OSError) as e:
                note = f" (could not read config back: {e})"
            self._job("done", "Build OK, flashed and verified." + note)
        finally:
            self.busy.clear()
            self._job_lock.release()


def origin_allowed(conn, http_port):
    """Reject WebSocket connections from foreign web pages: any site open in
    the browser could otherwise reach ws://127.0.0.1 and trigger flashes."""
    origin = conn.request.headers.get("Origin")
    if origin is None:  # non-browser local client
        return True
    return origin in {f"http://127.0.0.1:{http_port}", f"http://localhost:{http_port}"}


def ws_handler(hub, ctl, http_port, conn):
    """One browser client: replay current state, then handle its commands."""
    if not origin_allowed(conn, http_port):
        conn.close(1008, "bad origin")
        return
    try:
        for data in hub.add(conn):
            conn.send(data)
    except ConnectionClosed:
        hub.remove(conn)
        return
    try:
        for raw in conn:
            try:
                msg = json.loads(raw)
            except (TypeError, ValueError):
                continue
            if isinstance(msg, dict):
                # Off the socket thread: commands can take seconds (flash).
                threading.Thread(target=ctl.handle, args=(msg,), daemon=True).start()
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
    ap.add_argument("--no-ocd-restart", action="store_true",
                    help="never kill/respawn OpenOCD; just report link loss")
    args = ap.parse_args()

    hub = Hub()
    ctl = Controller(hub, args.host, args.port)

    ocd_server = OcdServer(args.host, args.port, auto=not args.no_ocd_restart)
    how = ocd_server.ensure_up()
    if how == "adopted":
        print(f"OpenOCD already running on {args.host}:{args.port} -- adopted.")
    elif how == "started":
        print("OpenOCD was not running -- started a supervised instance.")
    else:
        print(f"OpenOCD unreachable on {args.host}:{args.port} -- will keep retrying in the background.")

    threading.Thread(target=poll_loop, args=(hub, args.host, args.port, args.rate, ctl, ocd_server), daemon=True).start()

    ws_server = serve(lambda conn: ws_handler(hub, ctl, args.http_port, conn), "127.0.0.1", args.ws_port)
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
        ocd_server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
