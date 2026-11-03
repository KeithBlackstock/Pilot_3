"""Flight-controller control for the dashboard: live leveler config over
OpenOCD, "save as firmware defaults", and compile + flash.

Config lives in the firmware's `g_attitude_config` (RAM, see
firmware/src/attitude_level.hpp). Its address is looked up by symbol name in
the built ELF, so it stays correct when the layout changes. Live writes are
volatile: a reset restores the firmware's compiled-in defaults.
"""
import re
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FIRMWARE = ROOT / "firmware"
ELF = FIRMWARE / "build" / "pilot3_fw.elf"
HEADER = FIRMWARE / "src" / "attitude_level.hpp"
NM = Path(r"C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin\arm-none-eabi-nm.exe")

# Field order/limits mirror attitude_level::Config and its sanitizer.
FIELDS = ("pitch_kp", "roll_kp", "pitch_kd", "roll_kd", "pitch_sign", "roll_sign",
          "max_pitch_deg", "max_roll_deg", "deadband_us", "max_output_us",
          "trim_pitch_deg", "trim_pitch_us")
LIMITS = {
    "pitch_kp": (0.0, 20.0), "roll_kp": (0.0, 20.0),
    "pitch_kd": (0.0, 5.0), "roll_kd": (0.0, 5.0),
    "pitch_sign": (-1.0, 1.0), "roll_sign": (-1.0, 1.0),
    "max_pitch_deg": (0.0, 60.0), "max_roll_deg": (0.0, 60.0),
    "deadband_us": (0.0, 200.0), "max_output_us": (0.0, 500.0),
    "trim_pitch_deg": (-60.0, 60.0), "trim_pitch_us": (-200.0, 200.0),
}
RAM_LO, RAM_HI = 0x20000000, 0x2000C000

LINE_RE = re.compile(r"0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{8}\s*)+)")
DEFAULTS_RE = re.compile(
    r"(constexpr Config kDefaults = \{)[^}]*(\};  // DEFAULTS-LINE)")


class ControlError(Exception):
    pass


def validate(values):
    """Return a clean {field: float} dict or raise ControlError."""
    out = {}
    for name in FIELDS:
        if name not in values:
            raise ControlError(f"missing field {name}")
        try:
            v = float(values[name])
        except (TypeError, ValueError):
            raise ControlError(f"{name}: not a number")
        lo, hi = LIMITS[name]
        if not (v == v) or v < lo or v > hi:  # also rejects NaN
            raise ControlError(f"{name}: {v} outside [{lo}, {hi}]")
        if name.endswith("_sign"):
            v = -1.0 if v < 0 else 1.0
        out[name] = v
    return out


def symbol_address(name):
    """Address of a RAM symbol in the current ELF."""
    if not ELF.exists():
        raise ControlError("no firmware build yet -- compile first")
    res = subprocess.run([str(NM), str(ELF)], capture_output=True, text=True, timeout=20)
    for line in res.stdout.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == name:
            addr = int(parts[0], 16)
            if not RAM_LO <= addr < RAM_HI:
                raise ControlError(f"{name} at unexpected address 0x{addr:08x}")
            return addr
    raise ControlError(f"{name} not found in ELF (firmware too old?)")


def config_address():
    """Address of g_attitude_config in the current ELF."""
    return symbol_address("g_attitude_config")


class OcdLink:
    """Own telnet connection to the running OpenOCD, separate from the
    telemetry poller's. All calls hold a lock."""

    PROMPT = b"> "

    def __init__(self, host, port):
        self.host, self.port = host, port
        self._lock = threading.Lock()
        self._sock = None

    def _connect(self):
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=5)
        except OSError:
            self._sock = None
            raise ControlError(
                f"OpenOCD telnet unreachable at {self.host}:{self.port} -- "
                "start it with pilot3_dashboard.bat")
        self._read_until_prompt(2.0)

    def _read_until_prompt(self, timeout):
        sock = self._sock
        data = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            sock.settimeout(max(0.05, min(0.5, deadline - time.monotonic())))
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                if data.endswith(self.PROMPT):
                    break
                continue
            if not chunk:
                raise ControlError("OpenOCD closed the telnet connection")
            data += chunk
            if data.endswith(self.PROMPT) or data.endswith(b"\xfb\x01"):
                break
        return data.decode(errors="replace")

    def command(self, cmd, timeout=2.0):
        with self._lock:
            if self._sock is None:
                self._connect()
            try:
                self._sock.settimeout(0.05)
                try:
                    while self._sock.recv(4096):
                        pass
                except socket.timeout:
                    pass
                self._sock.sendall((cmd + "\n").encode())
                return self._read_until_prompt(timeout)
            except (OSError, ControlError):
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
                raise

    # ---- config ----

    def read_config(self):
        addr = config_address()
        out = self.command(f"mdw 0x{addr:08x} {len(FIELDS)}")
        words = []
        for line in LINE_RE.findall(out):
            words += [int(w, 16) for w in line.split()]
        if len(words) < len(FIELDS):
            raise ControlError(f"mdw returned {len(words)} words: {out.strip()!r}")
        vals = struct.unpack(f"<{len(FIELDS)}f", struct.pack(f"<{len(FIELDS)}I", *words[:len(FIELDS)]))
        return {n: round(v, 6) for n, v in zip(FIELDS, vals)}

    def write_config(self, values):
        values = validate(values)
        addr = config_address()
        for i, name in enumerate(FIELDS):
            word = struct.unpack("<I", struct.pack("<f", values[name]))[0]
            self.command(f"mww 0x{addr + 4 * i:08x} 0x{word:08x}")
        back = self.read_config()
        for name in FIELDS:
            if abs(back[name] - values[name]) > 1e-4:
                raise ControlError(f"{name} read back {back[name]} (wrote {values[name]})")
        return back

    def write_baro_ref(self, pressure_pa):
        """Set the firmware's ground-zero pressure (Pa); altitude reads 0 m at it."""
        if not (30000.0 <= pressure_pa <= 110000.0):
            raise ControlError(f"baro reference {pressure_pa:.0f} Pa outside [30000, 110000]")
        addr = symbol_address("g_baro_ref_pa")
        word = struct.unpack("<I", struct.pack("<f", pressure_pa))[0]
        self.command(f"mww 0x{addr:08x} 0x{word:08x}")
        out = self.command(f"mdw 0x{addr:08x} 1")
        words = [int(w, 16) for line in LINE_RE.findall(out) for w in line.split()]
        if not words or words[0] != word:
            raise ControlError(f"baro reference read back {out.strip()!r}")

    def flash(self):
        elf = str(ELF).replace("\\", "/")  # OpenOCD runs Tcl: backslashes corrupt
        out = self.command(f'program "{elf}" verify reset', timeout=90.0)
        if "Verified OK" not in out:
            raise ControlError("flash did not verify:\n" + out.strip()[-600:])
        return out


def save_defaults(values):
    """Rewrite the kDefaults line in attitude_level.hpp so the next build
    boots with these values."""
    values = validate(values)
    with open(HEADER, encoding="utf-8", newline="") as f:  # newline="": keep the file's line endings
        text = f.read()
    items = ", ".join(_cfloat(values[n]) for n in FIELDS)
    new, n = DEFAULTS_RE.subn(lambda m: f"{m.group(1)}{items}{m.group(2)}", text)
    if n != 1:
        raise ControlError("could not find the DEFAULTS-LINE in attitude_level.hpp")
    with open(HEADER, "w", encoding="utf-8", newline="") as f:
        f.write(new)


def write_waypoints(points):
    """Generate firmware/src/waypoints.hpp from [[lat, lon], ...] (degrees) so
    the next build boots with the route loaded. An empty list means no route
    (PATH stays disabled)."""
    if not isinstance(points, list) or len(points) > 16:
        raise ControlError("waypoints: expected a list of at most 16 points")
    rows = []
    for p in points:
        try:
            lat, lon = float(p[0]), float(p[1])
        except (TypeError, ValueError, IndexError):
            raise ControlError(f"bad waypoint {p!r}")
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):  # also rejects NaN
            raise ControlError(f"waypoint out of range: {lat}, {lon}")
        rows.append(f"    {{{round(lat * 1e7)}, {round(lon * 1e7)}}},")
    body = "\n".join(rows) if rows else "    {0, 0},  // placeholder (empty arrays are illegal)"
    text = (
        "// GENERATED by the dashboard's Compile & Flash from the Waypoints map -- do not edit.\n"
        "#pragma once\n"
        "#include <cstdint>\n"
        '#include "path_mode.hpp"\n'
        "\n"
        f"constexpr uint32_t kRouteCount = {len(rows)};\n"
        "constexpr path_mode::Waypoint kRoute[] = {\n"
        f"{body}\n"
        "};\n"
    )
    with open(FIRMWARE / "src" / "waypoints.hpp", "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def _cfloat(v):
    s = f"{v:.6g}"
    if not any(c in s for c in ".eE"):
        s += ".0"
    return s + "f"


def build():
    """Run firmware/build.bat. Returns (ok, log)."""
    try:
        res = subprocess.run(["cmd", "/c", str(FIRMWARE / "build.bat")], cwd=str(FIRMWARE),
                             capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return False, "build timed out"
    log = (res.stdout + res.stderr).strip()
    return res.returncode == 0, log


# ---- magnetometer calibration + PATH declination ----
# Field order/limits mirror qmc5883::Cal (qmc5883.hpp) and the firmware's
# per-sample sanitizer (qmc5883.cpp). These take the OcdLink explicitly;
# the methods above predate the compass.
import math as _math

MAG_CAL_FIELDS = ("off_x", "off_y", "off_z", "scale_x", "scale_y")
MAG_CAL_LIMITS = {"off_x": (-8000.0, 8000.0), "off_y": (-8000.0, 8000.0),
                  "off_z": (-8000.0, 8000.0), "scale_x": (0.1, 10.0), "scale_y": (0.1, 10.0)}
MAGCAL_HEADER = FIRMWARE / "src" / "qmc5883.hpp"
MAGCAL_RE = re.compile(
    r"(constexpr Cal kCalDefaults = \{)[^}]*(\};  // MAGCAL-DEFAULTS-LINE)")
# PATH's heading-offset default lives in main.cpp's g_path_config initializer
# (marked line, same rewrite pattern as the other defaults).
MAIN_CPP = FIRMWARE / "src" / "main.cpp"
PATH_DECL_RE = re.compile(
    r"(volatile path_mode::Config g_path_config = \{[^,]+,[^,]+,[^,]+, )[-+0-9.eEf]+"
    r"(, \{\}\};  // PATH-DEFAULTS-LINE)")
# Byte offset of declination_deg in path_mode::Config: uint32 count, float
# k_path, float capture_radius_m come first.
PATH_DECL_BYTE_OFFSET = 12


def validate_mag_cal(values):
    """Return a clean {field: float} dict or raise ControlError."""
    out = {}
    for name in MAG_CAL_FIELDS:
        if name not in values:
            raise ControlError(f"missing field {name}")
        try:
            v = float(values[name])
        except (TypeError, ValueError):
            raise ControlError(f"{name}: not a number")
        lo, hi = MAG_CAL_LIMITS[name]
        if name.startswith("scale"):
            # Scales may be negative (mirrored-module correction); the
            # limits bound the magnitude.
            if not (v == v) or not (lo <= abs(v) <= hi):  # also rejects NaN
                raise ControlError(f"{name}: |{v}| outside [{lo}, {hi}]")
        elif not (v == v) or v < lo or v > hi:  # also rejects NaN
            raise ControlError(f"{name}: {v} outside [{lo}, {hi}]")
        out[name] = v
    return out


def read_mag_cal(link):
    addr = symbol_address("g_mag_cal")
    out = link.command(f"mdw 0x{addr:08x} {len(MAG_CAL_FIELDS)}")
    words = []
    for line in LINE_RE.findall(out):
        words += [int(w, 16) for w in line.split()]
    if len(words) < len(MAG_CAL_FIELDS):
        raise ControlError(f"mdw returned {len(words)} words: {out.strip()!r}")
    vals = struct.unpack(f"<{len(MAG_CAL_FIELDS)}f",
                         struct.pack(f"<{len(MAG_CAL_FIELDS)}I", *words[:len(MAG_CAL_FIELDS)]))
    return {n: round(v, 6) for n, v in zip(MAG_CAL_FIELDS, vals)}


def write_mag_cal(link, values):
    """Write the 5 calibration floats to g_mag_cal and verify the readback."""
    values = validate_mag_cal(values)
    addr = symbol_address("g_mag_cal")
    for i, name in enumerate(MAG_CAL_FIELDS):
        word = struct.unpack("<I", struct.pack("<f", values[name]))[0]
        link.command(f"mww 0x{addr + 4 * i:08x} 0x{word:08x}")
    back = read_mag_cal(link)
    for name in MAG_CAL_FIELDS:
        if abs(back[name] - values[name]) > 1e-4:
            raise ControlError(f"{name} read back {back[name]} (wrote {values[name]})")
    return back


def write_declination(link, deg):
    """Set PATH's heading offset (deg) in g_path_config (RAM; lost on
    reset/reflash). Measured by the compass ritual's north pose -- absorbs
    declination and mounting yaw in one number."""
    deg = float(deg)
    if not (-90.0 <= deg <= 90.0):  # also rejects NaN
        raise ControlError(f"declination {deg} outside [-90, 90]")
    addr = symbol_address("g_path_config") + PATH_DECL_BYTE_OFFSET
    word = struct.unpack("<I", struct.pack("<f", deg))[0]
    link.command(f"mww 0x{addr:08x} 0x{word:08x}")
    out = link.command(f"mdw 0x{addr:08x} 1")
    words = [int(w, 16) for line in LINE_RE.findall(out) for w in line.split()]
    if not words or words[0] != word:
        raise ControlError(f"declination read back {out.strip()!r}")


def compute_six_pose_cal(poses):
    """Six-pose compass calibration. poses = [(x, y, z), ...] of averaged raw
    counts in the fixed order N, E, S, W, UP, DOWN. N is a known-true-north
    facing (pointed carefully); E/S/W only need to be ~90 deg apart; UP/DOWN
    are nose vertical, 180 deg apart about the pitch axis, any azimuth.

    The x/y offsets are the mean of the four planar poses (the horizontal
    field cancels over a full turn); the z offset is the UP/DOWN midpoint --
    the planar poses all see the same vertical field, so only a flip about a
    horizontal axis isolates off_z. Scales equalize the planar N-S and E-W
    spans. The heading offset (what to add to a mag heading to get a true
    bearing) is measured at the N pose -- it absorbs declination, mounting
    yaw and residual cal bias in one number, and is stored in path_mode's
    declination_deg.

    Returns (cal_values, heading_offset_deg, note)."""
    if len(poses) != 6:
        raise ControlError("need exactly 6 poses")
    n, e, s, w, up, dn = poses
    planar = (n, e, s, w)
    off_x = sum(p[0] for p in planar) / 4.0
    off_y = sum(p[1] for p in planar) / 4.0
    off_z = (up[2] + dn[2]) / 2.0
    span_x = abs(n[0] - s[0])
    span_y = abs(e[1] - w[1])
    span_z = abs(up[2] - dn[2])
    if min(span_x, span_y) < 100.0:
        raise ControlError(f"planar poses too similar (spans {span_x:.0f}/{span_y:.0f} counts)"
                           " -- rotate ~90 deg between facings")
    if span_z < 100.0:
        raise ControlError(f"vertical poses too similar (z span {span_z:.0f} counts)"
                           " -- flip a full 180 deg between nose-up and nose-down")
    mean_span = (span_x + span_y) / 2.0
    scale_x = mean_span / span_x
    scale_y = mean_span / span_y

    def wrap180(d):
        return (d + 180.0) % 360.0 - 180.0

    def heading(px, py, sx, sy):
        return _math.degrees(_math.atan2((py - off_y) * sy, (px - off_x) * sx)) % 360.0

    # Chirality check from the E pose: the poses were captured rotating
    # clockwise, so the corrected heading must *increase* by ~90 deg from N
    # to E. A mirrored (flipped) module runs backwards -- corrected with a
    # negative y scale, which turns the reflection back into a rotation
    # (the north-pose offset then rides on top as usual).
    h_n = heading(n[0], n[1], scale_x, scale_y)
    h_e = heading(e[0], e[1], scale_x, scale_y)
    mirrored = abs(wrap180((h_e - h_n) + 90.0)) < abs(wrap180((h_e - h_n) - 90.0))
    if mirrored:
        scale_y = -scale_y
        h_n = heading(n[0], n[1], scale_x, scale_y)
    # Corrected heading at the north pose -> offset to true north.
    heading_offset = wrap180(-h_n)
    values = {"off_x": off_x, "off_y": off_y, "off_z": off_z,
              "scale_x": scale_x, "scale_y": scale_y}
    ecc = abs(span_x - span_y) / mean_span
    note = (f"offset ({off_x:.0f}, {off_y:.0f}, {off_z:.0f}) counts, "
            f"scale ({scale_x:.3f}, {scale_y:.3f}), eccentricity {ecc:.1%}, "
            f"heading offset {heading_offset:+.1f} deg")
    if mirrored:
        note += " -- mirrored module detected, corrected with negative y scale"
    return values, heading_offset, note


def save_mag_cal_defaults(values):
    """Rewrite the kCalDefaults line in qmc5883.hpp so the next build boots
    with this calibration."""
    values = validate_mag_cal(values)
    with open(MAGCAL_HEADER, encoding="utf-8", newline="") as f:
        text = f.read()
    items = ", ".join(_cfloat(values[n]) for n in MAG_CAL_FIELDS)
    new, n = MAGCAL_RE.subn(lambda m: f"{m.group(1)}{items}{m.group(2)}", text)
    if n != 1:
        raise ControlError("could not find the MAGCAL-DEFAULTS-LINE in qmc5883.hpp")
    with open(MAGCAL_HEADER, "w", encoding="utf-8", newline="") as f:
        f.write(new)


def save_declination_default(deg):
    """Rewrite the declination value in main.cpp's g_path_config initializer
    (PATH-DEFAULTS-LINE) so the next build boots with this heading offset."""
    deg = float(deg)
    if not (-90.0 <= deg <= 90.0):
        raise ControlError(f"declination {deg} outside [-90, 90]")
    with open(MAIN_CPP, encoding="utf-8", newline="") as f:
        text = f.read()
    new, n = PATH_DECL_RE.subn(lambda m: f"{m.group(1)}{_cfloat(deg)}{m.group(2)}", text)
    if n != 1:
        raise ControlError("could not find the PATH-DEFAULTS-LINE in main.cpp")
    with open(MAIN_CPP, "w", encoding="utf-8", newline="") as f:
        f.write(new)
