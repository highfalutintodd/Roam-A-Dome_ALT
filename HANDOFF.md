# Roam-A-Dome — "dome flip-out" fix: status & handoff

**Date:** 2026-08-18 · **Branch:** `fix/dome-polarity-flipout` (NOT merged, NOT pushed)
· **RaD fw:** `2.0.0` · **Working tree:** clean, everything committed.

## TL;DR

The dome "flipping out" (violent oscillation, hangs, stuck orange square) during
K-ARDS / auto-dome is **fixed**. It was three stacked causes: a runtime polarity
re-learn that inverted the control loop, a parked-hold regression I introduced,
and a genuinely noisy absolute encoder. All three are addressed and **confirmed**.

**RECURRENCE (2026-08-19) — new root cause, fix on branch `fix/dome-hunt-plausibility-guard` (awaiting field test).**
A K-ARDS flip-out returned. Sabé log (`logs/sabe log K-ARDS flip out 8:19:26.txt`)
decoded: `:DPA323` is home-relative, so target was **203° absolute** and the dome
was already there. The real fault is a *control-loop noise chase*: the coarse arc
around 200° can't be resolved to the ±5° arrival window, and the stable 304/299
alias arrives in blocks long enough to be *confirmed as real motion* — so the
controller believed the dome jumped to 304°, drove hard, and shoved it off target
(~38 s hunt). The `kStableReads` debounce can't fix a *stable* alias, and doesn't
touch the controller's noise-chasing. Two-part RaD-side fix (no sensor reflash):
(1) **motor-plausibility guard** in `SensorRing` — reject a position jump the
*actual commanded drive* over the elapsed time couldn't have produced, even while
driving (a 100° jump in 60 ms is impossible), killing 304/299 permanently;
(2) **adaptive deadband** in `MotionController` — when reported position shows wide
spread but no net progress (noise, not travel), widen the stop-driving band toward
`fudgeMax` so it stops chasing and settles; clean arcs keep the tight `fudge`.
Simulated field scenario: motor effort 5962→165 (36×), never-arrives→arrives in
~260 ms at 202°. 62/62 host tests pass incl. an end-to-end replay with teeth;
firmware compiles. **NEXT: flash RaD, run K-ARDS, capture `#DPDEBUG1` log.** Also
under consideration: a sensor-characterization sketch to *measure* the ring's true
resolution/alias map instead of tuning to assumptions. Prior status below.

---

**CONFIRMED SOLID (2026-08-18).** Sensor was reflashed to `kStableReads=6`, then a
full K-ARDS + auto-dome + manual session was captured
(`logs/rad_log_8_18_26_8-17pm.txt`, 4382 telemetry samples). Analysis: `dir=1`
throughout with **zero polarity sign-flips across all 38 auto-drive episodes**
(each ramps monotonically into target); **all 39 target moves completed to idle**,
none hung; **zero ~299/304 aliases, zero phantom park-jumps**, longest clean-park
stretch 1031 samples at 1° spread; `sensor=OK` / `fault=NONE` on every sample. The
trailing `estop=1` is a clean operator E-stop at session end. Open loop CLOSED.
Merged to `main`.

## The system (two boards, two firmwares)

- **RaD board** (LilyGO T-Display-S3, ESP32-S3): runs `RadFirmware/` in this repo.
  Closed-loop dome positioning. Reads the sensor over serial (`#DP@<deg>` frames),
  drives the Syren motor, talks to the WCB mesh.
- **Sensor ring board** (plain ESP32): the Reeltwo Dome Sensor Ring — 9 IR
  sensors reading a printed **Mimir** absolute-encoder sticker → 9-bit code →
  `getDomeAngle()` lookup → angle. Firmware = `reeltwo/DomeSensorFirmware32`
  (LGPL-2.1). Our hardened copy is vendored at
  `SensorFirmware/DomeSensorFirmware32/`.

The two are independent MCUs on different USB ports. Mechanics are locked
(Hebel mount, high-quality Mimir sticker) — treat all remaining issues as
firmware, per the owner.

## Root causes & fixes (in causal order)

1. **Runtime polarity sign-flip → full-speed runaway.** `learnPolarity()` in
   `RadFirmware.ino` re-derived the wire→degrees sign at runtime from motion the
   sensor filter was already rejecting. Motor-current noise corrupted the sensor,
   the learner flipped the sign mid-hold, the position loop became POSITIVE
   feedback, and the dome bolted at 100% until the next flip. **Fix (commit
   `12221b5`):** polarity is now learned once and **frozen**; re-derivation only
   via the explicit **`#DPDIRLEARN`** command. `PolarityStore::clear()` added.
   BEHAVIOR.md **D14**.

2. **Parked-hold froze target moves mid-arrival (a regression I introduced).**
   To stop a parked dome from believing encoder misreads, I added a "parked-hold"
   to `SensorRing` that holds position when the motor is off. First version gated
   on `wire==0`, but a target move sits at `wire==0` while settling in the arrival
   arc → it froze `fPosition` at a value the flickering encoder never re-reported
   → arrival dwell never completed → move hung in `st=target` forever (K-ARDS
   never finished; the amber "moving" square stayed lit with the dome still).
   **Fix (commit `ed831c9`):** parked-hold is now gated on the **controller being
   idle** (`noteActive(manual || motion != idle)`), never during a move. Renamed
   `noteDrive`→`noteActive`. BEHAVIOR.md **D15/D16**.

3. **Genuinely noisy absolute encoder.** At certain arcs (notably ~195–210°) the
   9-bit code flickers between two valid codes ~15–20° apart, and occasionally
   emits a stable-but-wrong alias (~299/304). No control logic can settle at a
   target the encoder refuses to report, so the dome hunted. **Fix (commits
   `74462c6`, `d51d449`, `3e6d13d`):** `SensorFirmware/PositionDebounce.h` —
   debounce the **raw 9-bit code before decode** (getAngle() medians *after*
   decode, which can't undo a stuck code). Only emit an angle once the identical
   raw code holds for `kStableReads` reads AND is valid. This killed the flicker
   at the source; mounted test went from ~50 s of shaking to clean sweeps.

### Belt-and-suspenders on the RaD side (still active, correct)

- **Parked-hold** (idle-gated, above): once idle past a coast window, holds the
  last good position and rejects phantom jumps — protects against a parked dome
  reading a wrong angle for seconds and corrupting the next move's start.
- Existing `SensorRing` median-5 + rate gate + jump-confirm still run underneath.

## Current state / what's flashed

- **RaD:** flashed with everything through `ed831c9`. Working well.
- **Sensor:** flashed with the debounce at **`kStableReads=6`** (commit `966a70a`).
- **DONE:** the `kStableReads=6` reflash was completed and confirmed clean by the
  2026-08-18 run above. No open loop remains.

## THE OPEN ITEM — CLOSED (2026-08-18)

The sensor was reflashed to `kStableReads=6` and the confirming
`logs/rad_log_8_18_26_8-17pm.txt` run came back clean (no `~299/304` blips, no
wrong-direction `auto` kicks, `rej`/`jmp` flat at rest — ended 197/41 over the
whole session, all increments during motion). Nothing further to do.

If a residual alias ever reappears in a future run: bump `kStableReads` to **8**
(one-line change, `SensorFirmware/DomeSensorFirmware32/PositionDebounce.h`) and
reflash the sensor — OR implement **Option B** below to avoid touching the sensor
board again.

### Option B (not yet built) — RaD-side motor-plausibility guard

Add to `SensorRing` a check that rejects a confirmed position jump whose magnitude
exceeds what the motor could physically produce given how hard/long it has
actually been driving (a `200→304` jump while creeping at min-speed is impossible,
so discard it regardless of how "stable" the encoder claims). Makes the RaD immune
to residual encoder aliases with **no sensor reflash** — only the RaD is flashed.
More involved than the `kStableReads` bump; build only if the owner asks.

## Build & flash reference

Two different boards / two different ports — always `arduino-cli board list` first.

**RaD** (ESP32-S3 display board, native USB `/dev/cu.usbmodem…`):
```bash
cd "<repo>/RadFirmware" && arduino-cli compile --profile rad-display --upload --port /dev/cu.usbmodem1101
```
**Sensor** (plain ESP32, USB-serial `/dev/cu.usbserial…`):
```bash
cd "<repo>" && arduino-cli compile --fqbn esp32:esp32:esp32 --upload --port /dev/cu.usbserial-XXXX SensorFirmware/DomeSensorFirmware32
```
`compile --upload` writes only the app partition → both boards keep stored
settings (RaD keeps learned polarity; sensor keeps its baud). The global ReelTwo
(`~/Documents/Arduino/libraries/Reeltwo` 23.5.3) has a `DomeSensorRing.h`
byte-identical to upstream — confirmed safe; the owner's Uppity-project ReelTwo
mods do NOT touch the dome-sensor decode. Board target = plain ESP32 (repo `.auto`).

## Host tests

RaD core logic is host-testable (pure C++, no Arduino):
```bash
cd "<repo>/RadFirmware/test/native" && make test   # last run: 56 tests pass
```
Covers `SensorRing` (incl. parked-hold: reject-phantom, track-dither, reacquire,
`sensor_active_move_is_never_frozen`) and `MotionController`
(`motion_in_arc_jump_does_not_restart_dwell`).

## Key files

- `RadFirmware/RadFirmware.ino` — `learnPolarity()` (frozen), `#DPDIRLEARN` intercept
  in `handleCommandLine()`, `sSensor.noteActive(...)` call in `loop()`.
- `RadFirmware/src/SensorRing.h` — parked-hold (`noteActive`, `coastMs`, the
  idle-gated hold block), median/gate/jump logic.
- `RadFirmware/src/MotionController.h` — arrival dwell; in-arc-jump guard (~line 173).
- `RadFirmware/src/PolarityStore.{h,cpp}` — persisted sign + `clear()`.
- `SensorFirmware/DomeSensorFirmware32/` — vendored hardened sensor sketch
  (`.ino` = upstream + 4 tagged `RaD` edits, `PositionDebounce.h`, upstream `LICENSE`).
- `SensorFirmware/README.md` — sensor-side rationale + exact integration steps.
- `BEHAVIOR.md` — contract rows **D13–D16** describe the polarity + parked-hold behavior.
- `COMMANDS.md` — `#DPDIRLEARN` and the polarity-learning section.

## Gotchas

- **Branch merged to `main` and pushed (2026-08-18)** after the confirming run.
  `fix/dome-polarity-flipout` fast-forwarded into `main`.
- **iCloud "conflict copy" duplicates** (`Foo 2.cpp`) previously broke the
  `arduino-cli` link with multiple-definition errors. 45 were removed and
  `.gitignore` now ignores `* [0-9].*` / `* [0-9]/`. If the build ever fails with
  "multiple definition" again, `find . -name '* 2.*' -delete` (they regenerate if
  the repo keeps syncing in iCloud; real cure is a non-synced working copy).
- **Logs are git-ignored** (`logs/`). New capture logs stay local.
- **`#DPFUDGE`** stopgap (0–20, default 5): widening arrival tolerance lets the
  dome "arrive" despite flicker at the cost of accuracy. Not needed now that the
  sensor debounce works; mentioned only as a fallback.

## History (context, if digging)

Diagnosis was iterative across several field logs (some captured by a parallel
"Sabé review" session that first spotted the polarity-learner flip). The owner
updated the Sabé remote to a new version at the start, which was a red herring —
Sabé sent the correct K-ARDS commands (`#DPAUTO0` / `:DPA323` / `<CA1011>`); the
fault was entirely dome-side. `SensorRing.h` cites `DomeSensorFirmware32.ino` by
name — that's how the sensor firmware was identified and located on GitHub.
