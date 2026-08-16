#!/usr/bin/env python3
"""Replay every command in Sabé's DroidNet library against the v2 parser.

Each of the 72 roam-a-dome.json commands must either parse OK or be on the
explicit dropped/deferred list with a reason — silent grammar regressions fail.

Usage (from repo root; builds the parser CLI on demand):
    python3 tools/conformance.py
"""
import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
DCL = ROOT / "docs/reference/roam-a-dome.dcl.json"
NATIVE = ROOT / "RadFirmware/test/native"

# Commands intentionally not in v2, with reasons (BEHAVIOR.md §4).
DROPPED = {
    "rad.cfg.wifi": "web UI deferred (WCBClient owns the radio)",
    "rad.cfg.remote": "SMQ droid remote removed",
    "rad.cfg.rname": "SMQ droid remote removed",
    "rad.cfg.rsecret": "SMQ droid remote removed",
    "rad.cfg.pair": "SMQ droid remote removed",
    "rad.cfg.unpair": "SMQ droid remote removed",
    "rad.cfg.pwmarc": "in DCL only; absent from legacy README — confirmed dropped",
}
DEFERRED = {
    "rad.cfg.setup": "#DPSETUP non-blocking rewrite lands with bench calibration",
    "rad.cfg.setupVelocity": "with #DPSETUP",
    "rad.cfg.debug": "verbose debug port pending",
    "rad.cfg.joy": "VT100 joystick emulation pending",
}


def main() -> int:
    subprocess.run(["make", "-s", "parse_cli"], cwd=NATIVE, check=True)

    dcl = json.loads(DCL.read_text())
    rows = []
    for comp in dcl["components"]:
        for cmd in comp["commands"]:
            for example in cmd.get("examples") or []:
                rows.append((cmd["id"], example))

    lines = "\n".join(e for _, e in rows) + "\n"
    out = subprocess.run([str(NATIVE / "parse_cli")], input=lines, text=True,
                         capture_output=True, check=True).stdout.splitlines()

    failures = []
    counts = {"ok": 0, "dropped": 0, "deferred": 0}
    for (cid, example), verdict_line in zip(rows, out):
        verdict = verdict_line.split("\t", 1)[0]
        if cid in DROPPED:
            counts["dropped"] += 1
            if verdict == "OK":
                failures.append(f"{cid}: {example!r} parses but is on the DROPPED list")
        elif cid in DEFERRED:
            counts["deferred"] += 1
        elif verdict != "OK":
            failures.append(f"{cid}: {example!r} -> {verdict}")
        else:
            counts["ok"] += 1

    total = len(rows)
    print(f"{total} examples: {counts['ok']} OK, {counts['dropped']} dropped, "
          f"{counts['deferred']} deferred, {len(failures)} failures")
    for f in failures:
        print("FAIL", f)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
