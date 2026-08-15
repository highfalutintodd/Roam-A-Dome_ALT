#!/usr/bin/env python3
"""Capture (and later replay) Roam-A-Dome configuration over USB serial.

Capture from the LEGACY firmware before flashing v2:
    python3 tools/capture_config.py --port /dev/cu.usbmodem* --capture docs/capture

Replay captured settings into the NEW firmware (Phase 1+):
    python3 tools/capture_config.py --port /dev/cu.usbmodem* --replay docs/capture/config.txt

Requires: pip install pyserial
"""
import argparse
import glob
import pathlib
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: pip3 install pyserial")


def open_port(pattern: str, baud: int) -> "serial.Serial":
    matches = sorted(glob.glob(pattern)) if any(c in pattern for c in "*?[") else [pattern]
    if not matches:
        sys.exit(f"no serial port matches {pattern!r}")
    port = matches[0]
    print(f"opening {port} @ {baud}")
    s = serial.Serial(port, baud, timeout=0.25)
    time.sleep(2.0)  # ESP32 resets on port open; let it boot
    s.reset_input_buffer()
    return s


def send_and_collect(s, command: str, quiet_ms: int = 1200) -> str:
    """Send one command line and collect output until the port goes quiet."""
    s.write((command + "\r").encode())
    s.flush()
    out, last = [], time.monotonic()
    while (time.monotonic() - last) * 1000 < quiet_ms:
        chunk = s.read(4096)
        if chunk:
            out.append(chunk.decode(errors="replace"))
            last = time.monotonic()
    return "".join(out)


def capture(s, outdir: pathlib.Path) -> None:
    outdir.mkdir(parents=True, exist_ok=True)

    print("capturing boot banner (press the board's RESET button now, 5s)...")
    banner, deadline = [], time.monotonic() + 5
    while time.monotonic() < deadline:
        chunk = s.read(4096)
        if chunk:
            banner.append(chunk.decode(errors="replace"))
    (outdir / "boot_banner.txt").write_text("".join(banner))

    for name, cmd in [("config", "#DPCONFIG"), ("status", "#DPSTATUS"), ("sequences", "#DPL")]:
        print(f"running {cmd} ...")
        text = send_and_collect(s, cmd)
        (outdir / f"{name}.txt").write_text(text)
        print(text.strip()[:2000] or "(no output)")

    print(f"\ncaptured to {outdir}/ — commit these files (they are the migration data).")


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
