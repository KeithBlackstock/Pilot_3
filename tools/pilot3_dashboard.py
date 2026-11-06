#!/usr/bin/env python3
"""Pilot_3 web dashboard -- a browser presentation of the Telemetry struct
the firmware publishes.

Three transports, selected by --ble / --serial:

    default      poll the fixed SRAM struct over OpenOCD telnet (`mdw`, see
                 pilot3_attitude_monitor.py). Requires the ST-Link attached.
    --ble        wireless (the supported module): consume the framed
                 telemetry stream the firmware pushes on USART2
                 (firmware/src/bt.cpp) through a CC41-A BLE clone's GATT
                 serial characteristic (pilot3_ble_link.py, bleak).
                 Dashboard commands ride the same link.
    --serial     wireless (legacy SPP): same framed stream through a
                 classic ZS-040 / HC-05 module's COM port
                 (pilot3_serial_link.py). The module actually in use is a
                 BLE-only clone, so this mode is for a swapped-in classic
                 module.
    In both wireless modes only Compile & Flash still spawns OpenOCD on
    demand (no wireless flashing).

Either way the presentation is identical -- this script serves
tools/dashboard.html over HTTP and pushes JSON snapshots to the page over a
WebSocket:

    HTTP  http://127.0.0.1:8080/   -> dashboard.html
    WS    ws://127.0.0.1:8081/     -> telemetry JSON, ~--rate times/second

Requires the firmware already flashed and running. In OpenOCD mode an
instance is adopted if one is already answering on the telnet port
(pilot3.bat starts one); otherwise the dashboard spawns its own.
If the telnet link stays dead for RESTART_AFTER_S, the wedged OpenOCD
session is killed and respawned automatically (--no-ocd-restart disables
that). On exit, the dashboard kills only an instance it spawned -- someone
else's OpenOCD is left alone.
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

try:
    import pilot3_serial_link as sl
except ImportError:  # pyserial missing -- only --serial mode needs it
    sl = None

try:
    import pilot3_ble_link as bl
except ImportError:  # bleak is guarded inside the module; guard anyway
    bl = None

PAGE_PATH = Path(__file__).resolve().parent / "dashboard.html"

# OpenOCD supervision (see module docstring). Same binary and args as
# pilot3.bat.
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
        hub.publish(telemetry_msg(f, last_seq))
        last_seq = f[0]
        time.sleep(1.0 / hz)


def telemetry_msg(f, last_seq):
    """Build the dashboard JSON from a decoded Telemetry tuple (the struct
    layout from pilot3_attitude_monitor.STRUCT_FMT). Shared by the OpenOCD
    poll loop and the Bluetooth serial consumer so the page sees an
    identical shape on either transport."""
    (seq, roll_deg, pitch_deg, baro_temp_c, baro_pressure_pa, baro_altitude_m,
     ibus_rx_bytes, ibus_frames_ok, ibus_frames_bad) = f[:9]
    ch = list(f[9:9 + NUM_CHANNELS])
    pwm_s1_us, pwm_s4_us = f[9 + NUM_CHANNELS:9 + NUM_CHANNELS + 2]
    return {
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
                           f[9 + NUM_CHANNELS + 2:9 + NUM_CHANNELS + 2 + 10])),
                "lat_deg": f[9 + NUM_CHANNELS + 2] / 1e7,
                "lon_deg": f[9 + NUM_CHANNELS + 3] / 1e7},
        # Mag group is 7 fields (4h f 2I): +17, not +16 -- the old poll_loop
        # sliced one short and the page's mag "err" readout was undefined.
        "mag": dict(zip(("x", "y", "z", "present", "heading_deg", "reads_ok", "errors"),
                        f[9 + NUM_CHANNELS + 2 + 10:9 + NUM_CHANNELS + 2 + 17])),
    }


def wireless_poll_loop(hub, connect, describe, hz, ctl):
    """Wireless variant of poll_loop: consumes the telemetry stream the
    firmware pushes on USART2 (see firmware/src/bt.cpp) and hands the live
    link to the Controller so dashboard commands ride the same transport.
    connect() builds the link -- a SerialLink (SPP COM port) or a BleLink
    (CC41-A BLE clone); describe names the transport for error messages.
    Reconnects forever on failure, mirroring the OpenOCD loop."""
    last_seq = None
    while True:
        if ctl.busy.is_set():
            time.sleep(0.2)
            continue
        try:
            link = connect()
        except Exception as e:
            hub.publish({"connected": False, "error": f"cannot connect to {describe}: {e}"})
            time.sleep(2.0)
            continue
        ctl.link = link  # config commands ride the live link (None when down)
        try:
            while True:
                if ctl.busy.is_set():
                    time.sleep(0.2)
                    continue
                if link.error:
                    raise RuntimeError(link.error)
                t, age = link.latest_telemetry()
                if t is None or age > 2.0:
                    hub.publish({"connected": False,
                                 "error": f"no telemetry from the FC on {describe} -- "
                                          "is the FC powered with the BT firmware, and "
                                          "the module on USART2?"})
                    time.sleep(0.5)
                    continue
                msg = telemetry_msg(t, last_seq)
                last_seq = msg["seq"]
                hub.publish(msg)
                time.sleep(1.0 / hz)
        except Exception as e:
            hub.publish({"connected": False, "error": str(e)})
        finally:
            ctl.link = None
            link.close()
            time.sleep(1.0)


class Controller:
    """Handles browser commands: leveler config and compile + flash. One
    background job at a time; telemetry polling pauses while it runs.

    Config commands run on self.link, which is an OcdLink (SWD mode) or the
    live wireless link (a pilot3_serial_link.SerialLink or
    pilot3_ble_link.BleLink, set/cleared by wireless_poll_loop, None while
    the link is down) -- all implement read_words/write_word, so the fc.*
    control functions work on any of them. Flashing always goes through
    OpenOCD (no wireless flashing)."""

    def __init__(self, hub, host, port, ocd_server=None, wireless_mode=False):
        self.hub = hub
        self.wireless_mode = wireless_mode
        self._host, self._port = host, port
        self._ocd_server = ocd_server
        self._flash_link = None
        self.link = None if wireless_mode else fc.OcdLink(host, port)
        self.busy = threading.Event()
        self._job_lock = threading.Lock()
        self.live = None  # last values applied from the dashboard this session
        self.cal_poses = []  # compass ritual: averaged raw poses captured so far

    def _require_link(self):
        if self.link is None:
            raise fc.ControlError(
                "no live link to the FC yet -- waiting for Bluetooth telemetry")
        return self.link

    def _get_flash_link(self):
        """OpenOCD link for flashing. In wireless mode OpenOCD is not
        supervised, so it is spawned on demand -- the ST-Link must be
        attached for Compile & Flash to work."""
        if not self.wireless_mode:
            return self.link
        srv = self._ocd_server
        if srv is not None and not srv._port_up(0.3):
            if not srv.restart() or not srv._port_up(6.0):
                raise fc.ControlError(
                    "flashing needs the ST-Link attached: OpenOCD did not come up")
        if self._flash_link is None:
            self._flash_link = fc.OcdLink(self._host, self._port)
        return self._flash_link

    def _job(self, state, log=""):
        self.hub.broadcast({"type": "job", "state": state, "log": log})

    def _config(self, values, note=""):
        self.hub.broadcast({"type": "config", "values": values, "note": note})

    def _send_waypoints(self):
        """Push the FC's loaded route to the page, so a reload/reconnect
        repopulates the map with what PATH would actually fly. A read
        failure only notes itself on the map card -- the config reload that
        triggered this has already succeeded."""
        try:
            points = fc.read_waypoints(self._require_link())
        except (fc.ControlError, OSError) as e:
            self.hub.broadcast({"type": "waypoints", "points": None,
                                "note": f"Could not read the route: {e}"}, sticky=False)
            return
        note = (f"Loaded {len(points)} waypoint(s) from the FC." if points
                else "The FC has no route loaded.")
        self.hub.broadcast({"type": "waypoints", "points": points, "note": note},
                           sticky=False)

    def handle(self, msg):
        cmd = msg.get("cmd")
        try:
            if cmd == "get_config":
                self._config(fc.read_config(self._require_link()), "read from FC")
                self._send_waypoints()
            elif cmd == "set_config":
                values = fc.write_config(self._require_link(), msg.get("values", {}))
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
            elif cmd == "baro_zero":
                self._baro_zero()
            elif cmd == "mag_cal_capture":
                self._mag_cal_capture()
            elif cmd == "mag_cal_abort":
                self.cal_poses.clear()
                self._mag_cal(True, "Calibration aborted.", poses=0)
            elif cmd == "mag_cal_apply":
                values = fc.write_mag_cal(self._require_link(), msg.get("values", {}))
                fc.write_declination(self._require_link(), msg.get("heading_offset", 0.0))
                self._mag_cal(True, "applied live (lost on reset unless saved)", values,
                              msg.get("heading_offset"))
            elif cmd == "mag_cal_save":
                values = fc.validate_mag_cal(msg.get("values", {}))
                fc.save_mag_cal_defaults(values)
                fc.save_declination_default(msg.get("heading_offset", 0.0))
                self._mag_cal(True, "saved to qmc5883.hpp + main.cpp -- takes effect after Compile & Flash",
                              values, msg.get("heading_offset"))
            elif cmd == "build_flash":
                self._start_job(msg.get("waypoints"))
            else:
                return
        except fc.ControlError as e:
            self.hub.broadcast({"type": "config_error", "error": str(e)}, sticky=False)
        except (OSError, RuntimeError) as e:
            self.hub.broadcast({"type": "config_error", "error": f"OpenOCD: {e}"}, sticky=False)

    def _baro_cal(self, ok, note):
        self.hub.broadcast({"type": "baro_cal", "ok": ok, "note": note}, sticky=False)

    def _mag_cal(self, ok, note, values=None, heading_offset=None, poses=None):
        self.hub.broadcast({"type": "mag_cal", "ok": ok, "note": note, "values": values,
                            "heading_offset": heading_offset,
                            "poses": len(self.cal_poses) if poses is None else poses},
                           sticky=False)

    def _mag_cal_capture(self):
        """Average ~1.5 s of raw mag from the live telemetry as one ritual
        pose; refused if the pose isn't still or the mag is absent. The 6th
        pose completes the set and computes the calibration."""
        samples = []
        for _ in range(15):
            with self.hub._lock:
                latest = self.hub.latest
            m = json.loads(latest) if latest else {}
            mag = m.get("mag") or {}
            if m.get("connected") and not m.get("stale") and mag.get("present"):
                samples.append((mag["x"], mag["y"], mag["z"]))
            time.sleep(0.1)
        if len(samples) < 10:
            self._mag_cal(False, "Capture failed: no live magnetometer data.")
            return
        xs, ys, zs = zip(*samples)
        spread = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        if spread > 150:
            self._mag_cal(False, f"Capture refused: field moved {spread:.0f} counts "
                                 "during sampling -- hold the pose still and retry.")
            return
        self.cal_poses.append((sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs)))
        n = len(self.cal_poses)
        if n < 6:
            self._mag_cal(True, f"Pose {n} of 6 captured.")
            return
        try:
            values, heading_offset, note = fc.compute_six_pose_cal(self.cal_poses)
        except fc.ControlError as e:
            self.cal_poses.clear()
            self._mag_cal(False, f"Calibration failed: {e}", poses=0)
            return
        self.cal_poses.clear()
        self._mag_cal(True, note, values, heading_offset, poses=6)

    def _baro_zero(self):
        """Average ~2 s of baro pressure from the live telemetry and make it
        the firmware's 0 m reference. Refuses if the readings are unsteady."""
        samples = []
        for _ in range(20):
            with self.hub._lock:
                latest = self.hub.latest
            m = json.loads(latest) if latest else {}
            p = m.get("baro_pressure_pa", 0.0)
            if m.get("connected") and not m.get("stale") and p > 0:
                samples.append(p)
            time.sleep(0.1)
        if len(samples) < 15:
            self._baro_cal(False, "Altitude calibration failed: no steady barometer data.")
            return
        spread = max(samples) - min(samples)
        if spread > 30.0:  # ~2.5 m of altitude
            self._baro_cal(False, f"Altitude calibration refused: pressure moved {spread:.0f} Pa "
                                  "during sampling (~{:.1f} m). Hold still and retry.".format(spread / 12.0))
            return
        ref = sum(samples) / len(samples)
        try:
            fc.write_baro_ref(self._require_link(), ref)
        except (fc.ControlError, OSError) as e:
            self._baro_cal(False, f"Altitude calibration failed: {e}")
            return
        self._baro_cal(True, f"Altitude zeroed at {ref / 100:.2f} hPa "
                             f"({len(samples)} samples). Lost on reset or reflash.")

    def _start_job(self, waypoints=None):
        if not self._job_lock.acquire(blocking=False):
            return
        self.busy.set()
        threading.Thread(target=self._build_flash, args=(waypoints,), daemon=True).start()

    def _build_flash(self, waypoints=None):
        try:
            self._job("building")
            if waypoints is not None:
                # Bake the map's route into waypoints.hpp before compiling;
                # a rejected route aborts the job before anything is built.
                try:
                    fc.write_waypoints(waypoints)
                except fc.ControlError as e:
                    self._job("error", f"WAYPOINTS REJECTED: {e}")
                    return
            ok, log = fc.build()
            if not ok:
                self._job("error", "BUILD FAILED\n" + log[-3000:])
                return
            self._job("flashing", log[-600:])
            try:
                self._get_flash_link().flash()
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
                    fc.write_config(self._require_link(), self.live)
                    note = " Live-applied values were re-applied."
                self._config(fc.read_config(self._require_link()), "read from FC after flash")
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
    ap.add_argument("--ble", metavar="NAME|ADDR", nargs="?", const="Pilot3",
                    help="BLE dashboard via the CC41-A clone on USART2 -- connect by "
                         "device name (default 'Pilot3') or MAC address; OpenOCD is "
                         "then only used on demand by Compile & Flash")
    ap.add_argument("--serial", metavar="COMx",
                    help="legacy classic-SPP COM port (ZS-040 / HC-05 on USART2) -- "
                         "same wireless dashboard over a swapped-in classic module")
    ap.add_argument("--baud", type=int, default=115200, help="--serial baud rate")
    ap.add_argument("--rate", type=float, default=10.0, help="telemetry polls per second")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--ws-port", type=int, default=8081)
    ap.add_argument("--no-open", action="store_true", help="don't auto-open the browser")
    ap.add_argument("--no-ocd-restart", action="store_true",
                    help="never kill/respawn OpenOCD; just report link loss")
    args = ap.parse_args()

    ble_mode = args.ble is not None
    serial_mode = args.serial is not None
    if ble_mode and serial_mode:
        print("--ble and --serial are alternatives; pick one")
        return 2
    if ble_mode and (bl is None or bl.BleakClient is None):
        print("--ble needs bleak (pip install --user bleak)")
        return 2
    if serial_mode and (sl is None or sl.serial is None):
        print("--serial needs pyserial (pip install pyserial)")
        return 2
    wireless_mode = ble_mode or serial_mode

    hub = Hub()
    ocd_server = OcdServer(args.host, args.port, auto=not args.no_ocd_restart)
    ctl = Controller(hub, args.host, args.port, ocd_server=ocd_server,
                     wireless_mode=wireless_mode)

    if wireless_mode:
        # Wireless: no OpenOCD supervision. The Bluetooth link is the
        # transport; OpenOCD is spawned on demand only if Compile & Flash
        # is clicked (which needs the ST-Link attached).
        if ble_mode:
            target = args.ble
            connect = lambda: bl.BleLink(target)  # noqa: E731
            describe = f"BLE '{target}'"
            transport = f"BLE {target} (CC41-A @ 9600)"
        else:
            port, baud = args.serial, args.baud
            connect = lambda: sl.SerialLink(port, baud)  # noqa: E731
            describe = f"COM port {port}"
            transport = f"Bluetooth SPP {port} @ {baud}"
        threading.Thread(target=wireless_poll_loop,
                         args=(hub, connect, describe, args.rate, ctl),
                         daemon=True).start()
    else:
        how = ocd_server.ensure_up()
        if how == "adopted":
            print(f"OpenOCD already running on {args.host}:{args.port} -- adopted.")
        elif how == "started":
            print("OpenOCD was not running -- started a supervised instance.")
        else:
            print(f"OpenOCD unreachable on {args.host}:{args.port} -- will keep retrying in the background.")
        threading.Thread(target=poll_loop, args=(hub, args.host, args.port, args.rate, ctl, ocd_server), daemon=True).start()
        transport = f"OpenOCD {args.host}:{args.port}"

    ws_server = serve(lambda conn: ws_handler(hub, ctl, args.http_port, conn), "127.0.0.1", args.ws_port)
    threading.Thread(target=ws_server.serve_forever, daemon=True).start()

    url = f"http://127.0.0.1:{args.http_port}/"
    print(f"Dashboard: {url}  (WebSocket on :{args.ws_port}, transport: {transport})")
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
