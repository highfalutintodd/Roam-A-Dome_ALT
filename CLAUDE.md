# CLAUDE.md — Roam-A-Dome v2

ESP32 firmware for an R2-D2 dome: closed-loop dome positioning ("RaD" board)
fed by a separate sensor-ring MCU, driving a Syren motor controller, meshed to
the rest of the droid over WCB (ESP-NOW). Two boards, two firmwares:
`RadFirmware/` (the controller — most work happens here) and `SensorFirmware/`
(the ring encoder; rarely touched).

## Build & test — the definition of done

Every change must pass all three before it is done:

```
make -C RadFirmware/test/native test          # host tests (pure C++ core)
cd RadFirmware && arduino-cli compile --profile rad-display .   # default board
cd RadFirmware && arduino-cli compile --profile rad-compact .   # classic ESP32
```

Host tests are the fast loop; the two compiles catch Arduino-only code the
tests can't see. New behavior in the pure core gets a regression test in
`RadFirmware/test/native/` (harness: `rad_test.h` — use its shared `feedFrame`
/ `midRng` helpers, don't re-implement them per file).

## The contract docs (read before changing behavior)

- **BEHAVIOR.md** — the observable contract. Anything not listed under its
  Deviations table must behave identically to the legacy firmware
  (`legacy/DomeControlFirmware.ino` is the reference). Deliberate behavior
  changes get a new D-number there, not a silent divergence.
- **COMMANDS.md** — the operator-facing command grammar. Code and this file
  must never disagree; a knob documented here must actually work.
- **HANDOFF.md** — running status log. Append dated entries for significant
  work; don't rewrite history.

## Architecture rules

- **Pure core vs. glue.** `src/*.h` classes used by host tests (SensorRing,
  MotionController, Sequencer, CircularMath, Dedup, Command/Parser, Settings
  struct) are pure C++ — no Arduino includes outside `#ifdef ARDUINO`. IO and
  NVS live in Arduino-only files (CommandExec, SyrenBus, PwmIO, SeqStore,
  WcbLink, the .ino).
- **Tuning defaults have one source.** MotionTuning/SensorTuning default from
  `RadSettings{}` so bench tests and the flashed droid run the same numbers.
  Add new tunables to RadSettings first, then mirror + wire through
  `CommandExec::applyTuning`, the parser table, `setSetting` (with range
  check), `dumpConfig`, and COMMANDS.md — all six or none.
- **RadSettings is append-only.** Only grow the struct at the END, bump
  `kSettingsVersion`, and extend the prefix-copy migration in
  `RadSettingsStore::load` — stored configs must survive reflashes (D8).
- **Shared constants, not twins.** `kMaxCommandText`, `kMaxSeqSlot`,
  `kModeChars`, `kNvsNamespace` (Command.h / Settings.h) are the single
  sources for line length, slot range, report mode chars, and the NVS
  namespace. Don't re-declare literals that must agree across layers.
- **Safety invariants.** E-stop releases only on fresh manual input or a
  VALIDATED `:DP` motion command; `#DPAUTO`/`#DPHOME` are runtime-only and
  never persist across boots (D12 — and their toggles are deliberately not
  written to NVS); automation always requires a valid sensor; everything RAD
  emits can echo back over the mesh, so output formats must parse as silent
  no-ops (`KEY=` forms), never as executable commands or "Invalid" spam.
- **Alias defense ownership.** The sensor firmware's debounce owns
  transition/flicker rejection only. Stable-but-wrong codes are owned by the
  RaD-side motor-plausibility guard (earned-coast drive model in
  SensorRing.h). Don't tune one layer to patch the other's failure class.

## Environment gotchas

- The repo lives in iCloud Drive: it creates `name 2.ext` duplicate files on
  sync conflicts. Delete them; never commit them (`.gitignore` catches most).
- `sketch.yaml` pins the esp32 core (3.3.4) and vendors WCB_Client in
  `RadFirmware/libs/` — don't float either without a bench session.
- The command serial port exists only on the display board; the compact
  classic-ESP32 build has no spare UART (console/mesh only).
