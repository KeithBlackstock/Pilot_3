#!/usr/bin/env python3
"""Pilot_3 attitude monitor -- polls the Telemetry struct the firmware
(firmware/src/main.cpp) writes to a fixed SRAM address, over the same
OpenOCD telnet link pilot3_swd_probe.py already uses. No semihosting: the
firmware just writes to RAM every loop iteration, this script reads it
with `mdw`, and neither side blocks on the other.

Requires the firmware already flashed and running, and OpenOCD already
attached (pilot3_fw_build_flash.bat does both).

Telemetry struct (see firmware/src/telemetry.hpp), 68 bytes at a fixed
SRAM address:
    uint32_t seq;
    float    roll_deg;
    float    pitch_deg;
    float    baro_temp_c;
    float    baro_pressure_pa;
    float    baro_altitude_m;
    uint32_t ibus_rx_bytes;
    uint32_t ibus_frames_ok;
    uint32_t ibus_frames_bad;
    uint16_t ibus_ch[14];
    uint16_t pwm_s1_us;
    uint16_t pwm_s4_us;
    int32_t  gps_lat_e7, gps_lon_e7;   (degrees * 1e-7)
    float    gps_alt_m, gps_speed_mps, gps_course_deg;
    uint32_t gps_rx_bytes, gps_sentences_ok, gps_sentences_bad;
    uint16_t gps_sats, gps_fix;      (104 bytes total)
"""
import argparse
import re
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from pilot3_swd_probe import Ocd  # reuse the probe's telnet/mdw client

TELEMETRY_ADDR = 0x20000000
NUM_CHANNELS = 14
# seq, roll, pitch, baro x3, ibus x3, then channels + 2 pwm outputs (2B each)
TELEMETRY_WORDS = 9 + (NUM_CHANNELS * 2 + 2 * 2) // 4 + 5 + 3 + 1
STRUCT_FMT = f"<IfffffIII{NUM_CHANNELS}H2H" "2i3f3I2H"
# Anchor on the "0xADDR: " prefix OpenOCD's mdw response uses, same as the
# probe script's read_word() -- a bare 8-hex-digit pattern would also match
# the echoed command text ("mdw 0x20000000 16") and the address label itself.
# OpenOCD wraps mdw output across multiple "0xADDR: ..." lines once the
# word count exceeds 8 per line, so this must findall across all of them,
# not just the first.
LINE_RE = re.compile(r"0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{8}\s*)+)")


def read_telemetry(ocd):
    out = ocd.send(f"mdw 0x{TELEMETRY_ADDR:08x} {TELEMETRY_WORDS}")
    lines = LINE_RE.findall(out)
    if not lines:
        raise RuntimeError(f"mdw returned no data line: {out.strip()!r}")
    words = []
    for line in lines:
        words.extend(line.split())
    if len(words) < TELEMETRY_WORDS:
        raise RuntimeError(f"mdw returned too few words: {out.strip()!r}")
    raw = b"".join(int(w, 16).to_bytes(4, "little") for w in words[:TELEMETRY_WORDS])
    return struct.unpack(STRUCT_FMT, raw)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=4444)
    ap.add_argument("--rate", type=float, default=10.0, help="polls per second")
    args = ap.parse_args()

    try:
        ocd = Ocd(args.host, args.port)
    except OSError as e:
        print(f"Cannot reach OpenOCD telnet at {args.host}:{args.port} -- {e}")
        print("Start it first, e.g.: openocd -f interface/stlink.cfg -f target/stm32f3x.cfg")
        return 2

    print("Pilot_3 attitude monitor. Ctrl+C to stop.")
    last_seq = None
    try:
        while True:
            fields = read_telemetry(ocd)
            (seq, roll_deg, pitch_deg, baro_temp_c, baro_pressure_pa, baro_altitude_m,
             ibus_rx_bytes, ibus_frames_ok, ibus_frames_bad) = fields[:9]
            ch = fields[9:9 + NUM_CHANNELS]
            pwm_s1_us, pwm_s4_us = fields[9 + NUM_CHANNELS:9 + NUM_CHANNELS + 2]
            g = fields[9 + NUM_CHANNELS + 2:]
            (g_lat, g_lon, g_alt, g_spd, g_crs, g_bytes, g_ok, g_bad, g_sats, g_fix) = g
            g_lat, g_lon = g_lat / 1e7, g_lon / 1e7
            stale = " (STALE -- seq unchanged, firmware not running?)" if seq == last_seq else ""
            ch_str = " ".join(f"{v:4d}" for v in ch)
            print(f"\rseq={seq:8d}  roll={roll_deg:+7.2f} deg  pitch={pitch_deg:+7.2f} deg  "
                  f"baro={baro_temp_c:+6.2f} C  {baro_pressure_pa:8.1f} Pa  alt={baro_altitude_m:+7.2f} m  "
                  f"ibus: {ibus_rx_bytes:7d}B  ok={ibus_frames_ok:5d}  bad={ibus_frames_bad:5d}  ch=[{ch_str}]  "
                  f"pwm: S1={pwm_s1_us:4d}us S4={pwm_s4_us:4d}us  "
                  f"gps: fix={g_fix} sats={g_sats:2d} {g_lat:+10.6f},{g_lon:+11.6f} alt={g_alt:7.1f}m "
                  f"{g_spd:5.1f}m/s crs={g_crs:5.1f} {g_bytes}B ok={g_ok} bad={g_bad}"
                  f"{stale}   ",
                  end="", flush=True)
            last_seq = seq
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ocd.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
