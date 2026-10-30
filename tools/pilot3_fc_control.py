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


def config_address():
    """Address of g_attitude_config in the current ELF."""
    if not ELF.exists():
        raise ControlError("no firmware build yet -- compile first")
    res = subprocess.run([str(NM), str(ELF)], capture_output=True, text=True, timeout=20)
    for line in res.stdout.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == "g_attitude_config":
            addr = int(parts[0], 16)
            if not RAM_LO <= addr < RAM_HI:
                raise ControlError(f"g_attitude_config at unexpected address 0x{addr:08x}")
            return addr
    raise ControlError("g_attitude_config not found in ELF (firmware too old?)")


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
