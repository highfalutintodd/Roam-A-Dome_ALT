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
  so a closed-loop move can never run away from its target because of a swapped wire —
  and it's remembered across reboots, so the first move after power-up is right too.
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
| `tools/` | `capture_config.py` — capture/replay controller settings over USB |
| `docs/reference/` | machine-readable command library used for conformance testing |
| `legacy/` | the original firmware, kept intact as reference ([docs](legacy/README.md)) |
| `Mounts/`, `images/` | sensor-ring mounting hardware (CAD/STL) and images from upstream |

## Hardware

This is firmware for the **Roam-A-Dome controller board** from the
[original project](https://github.com/reeltwo/DomeControlFirmware) — it does not
replace your motor controller or your sensor ring, it drives them. Two board
variants are supported, and they use different build profiles:

| Variant | MCU | Profile | Notes |
|---|---|---|---|
| Display controller | ESP32-**S3** (LilyGO T-Display-S3 style, 170×320 ST7789) | `rad-display` | big-number position screen; console is USB CDC, so the command serial port is UART0 |
| Compact controller | classic ESP32 | `rad-compact` | no screen, otherwise identical |

You also need:

- a **dome position sensor ring** feeding `#DP@<degrees>` lines at 57600 or 115200 baud
  (`#DPSENSORBAUD`) — this is what makes any of the automation possible;
- a **motor controller** — Syren/Sabertooth on packet serial (default) or anything
  that takes an RC/PWM signal (`#DPPWMOUT1`);
- optionally a **WCB** mesh to talk to, if you want wireless control.

Exact pin assignments live in [`RadFirmware/pinmap.h`](RadFirmware/pinmap.h).

## Installing

**The easy way** — grab `rad-display.bin` or `rad-compact.bin` from the
[latest release](../../releases/latest) and flash it with
[esptool](https://github.com/espressif/esptool) (adjust the port for your machine):

```bash
esptool.py --port /dev/cu.usbmodem1101 write_flash 0x0 rad-display.bin
```

Nothing is erased that you don't ask to erase — your settings live in NVS and
survive a reflash unless the settings schema changed, in which case the firmware
falls back to defaults and says so in the boot banner.

**Coming from the original firmware?** Back your settings up first, then replay
them once v2 is running:

```bash
python3 tools/capture_config.py --port /dev/cu.usbmodem1101 --capture ./capture
python3 tools/capture_config.py --port /dev/cu.usbmodem1101 --replay ./capture/config.txt
```

Mesh passwords are deliberately never included in a capture — re-send
`#DPWCBPW<password>` followed by `#DPRESTART` yourself.

## Building from source

Install [arduino-cli](https://arduino.github.io/arduino-cli/), then:

```bash
cd RadFirmware && arduino-cli compile --profile rad-display
```

Profiles pin everything (ESP32 core 3.3.4, vendored WCB_Client) so builds are
reproducible. Swap in `--profile rad-compact` for the compact controller, and add
`--upload --port <your port>` to flash in one step.

## Testing

The parser, sequencer, sensor validation, and circular math are plain C++ with no
Arduino dependencies and run natively:

```bash
make -C RadFirmware/test/native test
```

## Quick start

1. Flash the firmware (see **Installing**), then open a serial monitor at 115200 baud.
2. Point the dome forward and set home: `#DPHOMEPOS`
3. Try a move: `:DPA90` — then `:DPA0` to come back.
4. Join the mesh: `#DPWCBPW<your mesh password>` then `#DPRESTART`
5. Turn on idle dome motion: `#DPAUTO1` — deliberately **not** remembered across
   reboots, so the droid never powers up and starts moving on its own.

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
