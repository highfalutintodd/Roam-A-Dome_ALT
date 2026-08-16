# Roam-A-Dome_ALT

A ground-up modernization of the [Roam-a-Dome-Home](https://github.com/reeltwo/DomeControlFirmware)
dome automation firmware for R2-D2 and other astromech droids, rebuilt for tight
integration with the [WCB](https://github.com/greghulette/WCBClient) wireless mesh and
the [Sabé](https://github.com/highfalutintodd/Sabe) droid control system.

Huge thanks to Skelmir and the reeltwo project for the original Roam-a-Dome-Home —
the hardware, the sensor ring design, and the command language that made automated
dome movement accessible to the whole astromech community. This fork keeps that
command language fully intact and focuses on evolving the firmware underneath it.

## What it does

Roam-A-Dome sits between your droid control system (Shadow, Padawan, Stealth, Kyber,
Sabé, …) and your dome motor controller (Syren packet serial or PWM). Manual control
always passes straight through; when your sticks go idle, the firmware takes over with
automated behaviors — return to home, random seek, and scripted sequences — using a
position sensor ring to know exactly where the dome is pointing.

## What this rewrite improves

- **Native wireless control.** The controller joins the WCB ESP-NOW mesh as a
  first-class peer via [WCBClient](https://github.com/greghulette/WCBClient) — dome
  commands arrive over the air from Sabé or any WCB node, with the wired serial port
  retained as a hot fallback. Fleet-wide `?STOP` e-stop is honored instantly.
- **Smarter position sensing.** Readings from the sensor ring pass through a
  validation pipeline — circular median filtering, physical-plausibility checks, and
  connection health monitoring — so the dome always acts on positions it can trust.
- **Precise sequence timing.** A new non-blocking sequence engine with millisecond
  waits, inclusive random ranges (`:DPWR10,20`), and a new random-millisecond wait
  (`:DPWMR`), engineered so timing stays exact no matter what else is happening on
  the serial ports or the mesh.
- **Self-calibrating drive polarity.** Which way the motor has to turn to make the
  position reading go up is *learned* by watching the dome move under manual control,
  so a closed-loop move can never run away from its target because of a swapped wire.
- **Big-number position display.** On the display controller, dome position fills the
  screen in seven-segment digits, with an idle backlight timeout (`#DPLCDSLEEP`) so a
  parked droid isn't glowing all evening.
- **Modern toolchain.** ESP32 Arduino core 3.3.4, reproducible one-command builds via
  `arduino-cli` profiles, and a modular codebase whose core logic runs — and is
  tested — on your desktop, no droid required.
- **Full compatibility.** The `:DP` motion and `#DP` configuration command language,
  stored sequences (slots 0–100), and position reporting all work exactly as
  documented, so existing controllers, sequences, and the DroidNet command library
  keep working unchanged.

Every command, with ranges and defaults, is in [COMMANDS.md](COMMANDS.md). The
complete behavioral specification — including the handful of deliberate refinements
over the original — lives in [BEHAVIOR.md](BEHAVIOR.md).

## Repository layout

| Path | Contents |
|---|---|
| `RadFirmware/` | the v2 firmware (modular sources in `src/`, host tests in `test/native/`) |
| `COMMANDS.md` | complete command reference — syntax, ranges, defaults |
| `BEHAVIOR.md` | the observable contract the firmware is built and tested against |
| `BENCH.md` | one-time bench checklist (board identification, settings capture) |
| `tools/` | `capture_config.py` — capture/replay controller settings over USB |
| `docs/reference/` | machine-readable command library used for conformance testing |
| `legacy/` | the original firmware, kept intact as reference ([docs](legacy/README.md)) |
| `Mounts/`, `images/` | sensor-ring mounting hardware (CAD/STL) and images from upstream |

## Building

Install [arduino-cli](https://arduino.github.io/arduino-cli/), then:

```bash
cd RadFirmware && arduino-cli compile --profile rad-display
```

Profiles pin everything (ESP32 core 3.3.4, vendored WCB_Client) so builds are
reproducible: `rad-display` for the display controller, `rad-compact` for the compact
controller.

## Testing

The parser, sequencer, sensor validation, and circular math are plain C++ with no
Arduino dependencies and run natively:

```bash
make -C RadFirmware/test/native test
```

## Quick start

1. Flash the firmware (see **Building**), then open a serial monitor at 115200 baud.
2. Point the dome forward and set home: `#DPHOMEPOS`
3. Try a move: `:DPA90` — then `:DPA0` to come back.
4. Join the mesh: `#DPWCBPW<your mesh password>` then `#DPRESTART`
5. Turn on idle dome motion: `#DPAUTO1`

`#DPCONFIG` dumps every setting as replayable commands; `#DPSTATUS` and
`#DPSTATS` show what the firmware is seeing.

## Status

Built in phases — each phase left the droid fully functional:

- [x] Phase 0 — behavioral spec, bench tooling
- [x] Phase 1 — build scaffold, transparent motor passthrough
- [x] Phase 2 — validated sensor pipeline
- [x] Phase 3 — motion controller and calibration
- [x] Phase 4 — sequence engine
- [x] Phase 5 — WCB mesh integration
- [x] Phase 6 — position display, docs

Field-verified on a live droid: manual passthrough, targeted moves, sequences,
idle automation, home seek, position reporting, mesh join and e-stop, and the
position display. Still to be exercised on-droid: on-device stored sequences
(`#DPS<n>` / `:DPS<n>`) and the digital output pins (`:DPT`/`:DPP`), both
covered by host tests. `#DPSETUP` auto-calibration, the VT100 joystick, the WiFi
web UI, the rotary menu system, and the SMQ droid remote are deliberately not
carried over — see [BEHAVIOR.md](BEHAVIOR.md) §4 and §8.

## Credits

- **Skelmir / [reeltwo](https://github.com/reeltwo)** — the original
  [Roam-a-Dome-Home firmware](https://github.com/reeltwo/DomeControlFirmware) and
  hardware this project builds on
- **[Greg Hulette](https://github.com/greghulette)** — the WCB ecosystem and the
  [WCBClient](https://github.com/greghulette/WCBClient) library
- The [Astromech.net](https://astromech.net) community

Licensed under the [LGPL-2.1](LICENSE), same as upstream.
