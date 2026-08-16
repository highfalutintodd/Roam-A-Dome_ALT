#!/usr/bin/env python3
"""Capture (and later replay) Roam-A-Dome configuration over USB serial.

Capture from the LEGACY firmware before flashing v2:
    python3 tools/capture_config.py --port /dev/cu.usbmodem* --capture ./capture

Replay captured settings into the NEW firmware (Phase 1+):
    python3 tools/capture_config.py --port /dev/cu.usbmodem* --replay ./capture/config.txt

Requires: pip install pyserial
"""
import argparse
import glob
import pathlib
import re
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: pip3 install pyserial")

# ESP32 core debug spam ("[  5035][D][esp32-hal-rmt.c:202] ..."), the legacy
# firmware's per-byte RX echo ("READ: X [88]"), and its "Invalid" replies to
# mesh chatter — none of it is capture data.
DEBUG_LINE = re.compile(r"^(\[\s*\d+\]\[[VDIWE]\]|READ: |Invalid$)")


def strip_debug(text: str) -> str:
    return "\n".join(l for l in text.splitlines() if not DEBUG_LINE.match(l)).strip()


def open_port(pattern: str, baud: int) -> "serial.Serial":
    matches = sorted(glob.glob(pattern)) if any(c in pattern for c in "*?[") else [pattern]
    if not matches:
        sys.exit(f"no serial port matches {pattern!r}")
    port = matches[0]
    print(f"opening {port} @ {baud}")
    return serial.Serial(port, baud, timeout=0.25)


def collect_until_quiet(s, quiet_s: float, max_s: float) -> str:
    """Collect output until the port has been quiet for quiet_s (cap max_s)."""
    out = []
    start = last = time.monotonic()
    while time.monotonic() - last < quiet_s and time.monotonic() - start < max_s:
        chunk = s.read(4096)
        if chunk:
            out.append(chunk.decode(errors="replace"))
            last = time.monotonic()
    return "".join(out)


def send_and_collect(s, command: str, quiet_ms: int = 1500) -> str:
    """Send one command line and collect output until the port goes quiet."""
    s.write((command + "\r").encode())
    s.flush()
    return collect_until_quiet(s, quiet_ms / 1000.0, max_s=20)


def capture(s, outdir: pathlib.Path) -> None:
    outdir.mkdir(parents=True, exist_ok=True)

    # Opening the port auto-resets the board. The legacy firmware takes ~5-10 s to
    # boot and streams debug logging continuously, so wait for the boot flood to
    # settle before sending anything.
    print("board is booting; capturing banner until output settles (up to 30 s)...")
    banner = collect_until_quiet(s, quiet_s=2.5, max_s=30)
    (outdir / "boot_banner.txt").write_text(banner)
    if "Droid Dome Controller" in banner or "Roam-A-Dome" in banner:
        print("  boot banner captured.")
    else:
        print("  warning: no recognizable banner — output may still be flooded.")

    for name, cmd in [("config", "#DPCONFIG"), ("status", "#DPSTATUS"), ("sequences", "#DPL")]:
        # Continuous debug logging means the port never truly goes quiet, so retry
        # until the response contains non-debug content.
        for attempt in range(1, 4):
            print(f"running {cmd} (attempt {attempt}) ...")
            raw = send_and_collect(s, cmd)
            useful = strip_debug(raw)
            if useful:
                break
            time.sleep(1.0)
        (outdir / f"{name}.txt").write_text(useful + "\n" if useful else raw)
        print(useful[:2000] or "(no output — saved raw)")

    print(f"\ncaptured to {outdir}/ — keep these as your settings backup.")


def replay(s, config_file: pathlib.Path) -> None:
    """Replay a captured config into v2 firmware.

    v2's #DPCONFIG emits replayable `#DPKEY=VALUE` lines; legacy output is `Key=Value`
    and needs hand-translation first (see BEHAVIOR.md D6). Lines not starting with #DP
    are skipped with a warning so a legacy capture fails loudly, not silently.
    """
    skipped = 0
    for line in config_file.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        if not line.startswith("#DP"):
            print(f"skip (not a #DP command): {line}")
            skipped += 1
            continue
        cmd = line.replace("=", "", 1) if "=" in line else line
        print(f"send: {cmd}")
        print(send_and_collect(s, cmd, quiet_ms=600).strip())
    if skipped:
        print(f"\n{skipped} line(s) skipped — legacy captures need translating to #DP commands first.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True, help="serial device or glob, e.g. /dev/cu.usbmodem*")
    ap.add_argument("--baud", type=int, default=115200)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--capture", metavar="OUTDIR", help="capture config/status/sequences to OUTDIR")
    mode.add_argument("--replay", metavar="CONFIG", help="replay a #DPKEY=VALUE config file")
    args = ap.parse_args()

    s = open_port(args.port, args.baud)
    try:
        if args.capture:
            capture(s, pathlib.Path(args.capture))
        else:
            replay(s, pathlib.Path(args.replay))
    finally:
        s.close()


if __name__ == "__main__":
    main()
