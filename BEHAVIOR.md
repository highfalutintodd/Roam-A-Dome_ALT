# Roam-A-Dome v2 — Observable Behavior Contract

This document is the specification for the ground-up rewrite. It records the externally
observable behavior of the legacy firmware (`legacy/DomeControlFirmware.ino` @ f9b1b83 +
`reeltwo/Reeltwo` library) that v2 must preserve, plus the deliberate deviations v2 makes.
Sources cited per section: `README` = legacy README.md, `DCL` =
`docs/reference/roam-a-dome.dcl.json` (Sabé's DroidNet command library, 72 commands),
`ino:<n>` = line in `legacy/DomeControlFirmware.ino`.

Anything not listed under **Deviations** must behave identically to legacy.

---

## 1. Core role (unchanged)

RAD sits between the droid control system (Shadow/Kyber/Sabé) and the dome motor
(Syren/Sabertooth packet serial, or PWM). Manual motor commands pass through with minimal
latency at all times. Automation (home seek, random auto, targeted moves, sequences) runs
only when the manual input has been idle. Dome position comes from the sensor ring on
`DOME_SENSOR_SERIAL` as ASCII lines `#DP@<degrees>\r\n` (57600 or 115200 baud,
`#DPSENSORBAUD`).

## 2. Command transports

| Transport | Direction | Notes |
|---|---|---|
| USB console serial | in/out | commands + human-readable responses |
| Command serial (`#DPSERIALBAUD`, default 9600) | in/out | same grammar; position reports out (§6) |
| WCB mesh (**new in v2**) | in/out | same grammar via WCBClient; `?STOP` e-stop; `&RAD,…` reports |
| Syren packet serial in → out | passthrough | 4-byte frames `[addr][cmd][data][(addr+cmd+data)&0x7F]`, in-addr/out-addr configurable (default 129) |
| PWM in → out | passthrough | RC pulse 800–2200 µs, neutral 1500, deadband % |

Commands are line-oriented, terminated by CR or LF. Console and command serial have
**independent buffers** in v2 (legacy shared one buffer — interleaving corrupted commands).

## 3. Motion command grammar (`:DP…`)

A motion line is `:DP` + one or more steps separated by `:`. **Only the first step carries
the `DP` prefix** (README:444): `:DPA90:W2:H` = go to 90°, wait 2 s, go home.
`:DPW5:DPA90` is invalid — the second step must be `A90`, not `DPA90`.

| Step | Syntax | Behavior | Source |
|---|---|---|---|
| `S<n>` | `:DPS1` | Play stored sequence n (0–100). Replaces any running sequence. | README:454, DCL rad.playSeq |
| `A[M]<deg>[,speed[,maxspeed]][+]` | `:DPA90,20,100` | Absolute rotate, degrees relative to home, negative = other direction. Speed = start %, maxspeed = ramp ceiling (defaults to speed). `+` suffix = fire-and-forget (sequence continues without waiting for arrival). `M` = one-shot (return mode to off after). | README:501, ino:1951 |
| `AR` | `:DPAR` | Absolute rotate to random position. | DCL rad.rotate.absRandom |
| `D[M]<deg>[,speed[,maxspeed]][+]` | `:DPD-90` | Relative rotate by degrees. | README:480, ino:1952 |
| `DR` | `:DPDR` | Relative rotate by random degrees. | DCL rad.rotate.relRandom |
| `R<speed>` | `:DPR-30` | Continuous spin at −100…100 % (sign = direction). | README:511 |
| `H[R][speed]` | `:DPH` | Seek home position. | README:473 |
| `W<sec>` | `:DPW2` | Wait seconds before next step. | README:463 |
| `WM<ms>` | `:DPWM500` | Wait milliseconds. | ino:2128 |
| `WR[<max>[,<min>]]` | `:DPWR10,20` | Random wait. Bare `WR` = 1–6 s. | README:465 |
| `WMR<min>,<max>` | `:DPWMR100,900` | **New in v2**: random wait in milliseconds, inclusive. | v2 |
| `T<pin>` | `:DPT3` | Toggle digital pin 1–8. | README:519 |
| `P<pin><0\|1>` | `:DPP21` | Set digital pin 1–8 low/high. | README:526 |
| `Q<n>,<ms>,<pos>,<easing>` | | Servo move (only if servos enabled at build time). | ino:1884 |
| `Z` | `:DPZ` | Restore settings/pins to stored defaults. | README:533 |

Responses on the console: waits print `WAIT SECONDS: <n>` / `WAIT MILLIS: <n>` (ino:2153);
malformed commands print `Invalid` and abort the whole line.

### Sequence semantics
- A new `:DP` line **replaces** any currently running sequence.
- Config commands (`#DP…`), position reports, blank lines, CR/LF noise, and mesh
  heartbeats have **no effect** on a running sequence or an in-flight wait (v2 deviation
  D1 — legacy cancelled waits on any newline).
- Stored sequences (slots 0–100) hold the raw step string exactly as given to `#DPS<n>:`.

## 4. Config command grammar (`#DP…`) — keep/drop triage

### Kept (identical syntax and semantics)

Motion tuning: `#DPMAXSPEED<0-100>` (50), `#DPHOMESPEED` (40), `#DPAUTOSPEED` (30),
`#DPTARGETSPEED` (100), `#DPMINSPEED` (15), `#DPINPUTSPEED`, `#DPFUDGE<0-20>` (5°),
`#DPSCALE<0|1>`, `#DPASCALE<0-255>`, `#DPDSCALE<0-255>`, `#DPINVERT<0|1>`,
`#DPTIMEOUT<sec>` (5).

Modes: `#DPHOME<0|1>`, `#DPAUTO<0|1>` (v2: runtime-only, off at boot — deviation D12),
`#DPAUTOSAFETY<0|1>`, `#DPAUTORESTART<0|1>`,
`#DPAUTOLEFT<0-180>` (80), `#DPAUTORIGHT<0-180>` (80), `#DPAUTOMIN/MAX<sec>` (6/8),
`#DPHOMEMIN/MAX<sec>` (6/8), `#DPTARGETMIN/MAX<sec>` (0/1).

Calibration: `#DPHOMEPOS[deg]` (bare form snapshots current position — v2 averages 1 s of
validated readings, deviation D4), `#DPSETUP` (v2: non-blocking, deviation D5),
`#DPSETUPVELOCITY<n>` (100).

I/O: `#DPSERIALIN/OUT<0|1>`, `#DPSERIALBAUD` (9600), `#DPSYRENBAUD` (9600),
`#DPSYRENADDRIN/OUT/ADDR` (129), `#DPSENSORBAUD<57600|115200>`, `#DPPWMIN/OUT<0|1>`,
`#DPPWMMIN/MAX/NEUTRAL<800-2200>`, `#DPPWMDEADBAND<0-50%>` (5), `#DPPIN<pin><0|1>`.

System: `#DPCONFIG` (v2: dumps as replayable `#DPKEY=VALUE` lines, deviation D6),
`#DPSTATUS`, `#DPRESTART`, `#DPZERO`, `#DPFACTORY`, `#DPREPORT<ms>` (0 = off),
`#DPDEBUG<0|1>`, `#DPJOY` (VT100 joystick emulation).

Sequences: `#DPS<n>:<steps>` store, `#DPL` list, `#DPD<n>` delete.

### Dropped in v2 (with reason)

| Command | Reason |
|---|---|
| `#DPREMOTE<0|1>`, `#DPRNAME`, `#DPRSECRET`, `#DPPAIR`, `#DPUNPAIR` | SMQ Droid Remote removed — WCBClient owns ESP-NOW exclusively |
| `#DPWIFI<0|1>` | Web UI deferred; if it returns it is SoftAP-on-mesh-channel only |
| `#DPPWMARC` | present in DCL only, not in legacy README/parser — verify on bench; drop if legacy no-op |

### New in v2

| Command | Meaning | Default |
|---|---|---|
| `#DPMAXRPM<1-60>` | sensor plausibility gate: max physical dome RPM (bench-measured ~41 RPM at 100% speed) | 60 |
| `#DPSENSTO<ms>` | sensor staleness timeout (must exceed the ring's 1000 ms parked-heartbeat interval) | 2500 |
| `#DPSENSN<n>` | consistent samples to confirm a position discontinuity | 3 |
| `#DPDWELL<n>` | consecutive in-arc samples counted as "arrived" | 3 |
| `#DPIDLE<ms>` | manual-neutral time before automation resumes | 3000 |
| `#DPDEDUP<ms>` | cross-transport duplicate-command window (0 = off) | 750 |
| `#DPSERIALCMD<0|1>` | command-serial ingress enable | 1 |
| `#DPWCBEN<0|1>` | WCB radio enable | 1 |
| `#DPWCBID<1-19>` | WCB device ID | 4 |
| `#DPWCBOCT<o2>,<o3>` | mesh MAC octets (hex) | 3C,4E |
| `#DPWCBQTY<n>` | fleet WCB quantity | 3 |
| `#DPWCBCH<n>` | mesh channel | 1 |
| `#DPWCBPW<str>` | mesh password | (unset) |
| `#DPWCBCS<0|1>` | mesh checksum | 1 |
| `#DPLCDSLEEP<sec>` | display backlight idle timeout, 0 = always on (display board only) | 300 |
| `#DPSTATS` | dump parser/sensor/dedup/queue counters | — |

## 5. Passthrough & arbitration (priority ladder, evaluated every loop)

1. **E-stop latched** — `?STOP` from any transport, or `&SABE,ESTOP,*` from mesh: motor
   → neutral, sequence aborted, automation off. Released by fresh non-neutral manual
   input or an explicit new `:DP` motion command.
2. **Manual active** — non-neutral Syren-in frame or PWM outside deadband: passed through
   to motor out immediately (<5 ms added latency); kills any automation move same tick.
3. **Idle** — manual neutral for `#DPIDLE` ms: automation eligible.
4. **Automation** — runs only when sensor state is VALID and not e-stopped. Sensor going
   STALE mid-move: motor → neutral immediately; passthrough unaffected.

## 6. Position reporting

- **Command serial**, on every position/mode change (rate-limited): `#DP<mode><pos>` where
  mode is `@` off, `!` home, `$` random, `%` target (ino:3326-3345). Unchanged in v2.
- **Console**, when `#DPREPORT<ms>` enabled: `DOME POSITION: <n>` every interval
  (drift-free in v2, deviation D3).
- **Mesh (new)**: `&RAD,POS,<deg>,<modechar>` unicast to Sabé (device 20), ≤1 Hz, on ≥1°
  change; `&RAD,HB,<fw>,<state>` broadcast every 10 s; `&RAD,FAULT,<code>` on faults.

## 7. Sensor input contract

Accepted frame: `#DP@<1-4 digits>\r\n`, value 0–359. Everything else is discarded and
counted (`#DPSTATS`). The ring transmits on position change plus a heartbeat resend
every 1000 ms when parked (`DomeSensorFirmware32.ino:22,336`) — so a stationary dome
delivers ~1 frame/s and a moving dome many more. Position becomes VALID after the
warm-up window fills (5 samples), or early after 3 agreeing samples (parked dome:
valid in ~3 s, not ~5 s). No frames for `#DPSENSTO` ms (default 2500 — must clear the
1000 ms heartbeat with margin) → STALE: automation disabled, display shows `---`, one
`&RAD,FAULT,SENSOR_STALE` emitted; recovery restarts warm-up.

Glitch rejection: a reading whose circular distance from the last accepted position
exceeds `max_deg_per_ms × elapsed + 2°` (from `#DPMAXRPM`) is not applied until
`#DPSENSN` consecutive consistent samples confirm it (a real jump). Filtering uses a
circular median (correct across the 0/359 seam). Relative-move accumulation sees only
validated deltas; a confirmed jump aborts an in-progress relative move with an error
instead of silently corrupting it.

## 8. Deliberate deviations from legacy (v2 behavior changes)

| # | Deviation | Legacy behavior being fixed |
|---|---|---|
| D1 | Serial/mesh traffic never cancels an in-flight wait; only a new `:DP` line replaces a sequence | any CR/LF zeroed the wait deadline (ino:1819,3280-3313) — the "wait miscounts" bug |
| D2 | `:DPW0` legal (0 ms); `WR<a>,<b>` upper bound inclusive; `WMR` added; waits rollover-safe | W0 clamped to 1 s; `random(a,b)` exclusive → `WR10,20` = 10–19 s; unsafe `millis() > deadline` |
| D3 | `#DPREPORT` interval drift-free (`next += interval`) | `next = millis() + interval` accumulated drift |
| D4 | bare `#DPHOMEPOS` averages 1 s of validated readings | snapshotted one raw, unvalidated sample |
| D5 | `#DPSETUP` non-blocking; console alive during calibration; auto-invert from filtered full-revolution data | blocking while-loop; invert decided from 2 raw samples only when pos<300 |
| D6 | `#DPCONFIG` output is replayable (`#DPKEY=VALUE` per line) | freeform dump |
| D7 | Sensor validation per §7 | median-of-5 only, no range/staleness/plausibility checks |
| D8 | Settings in NVS with schema version + migration; never silently wiped | EEPROM magic+size check wiped all settings on any struct change |
| D9 | Arrival requires `#DPDWELL` consecutive in-arc samples | single-sample arrival/departure |
| D10 | SMQ Droid Remote and WiFi web UI removed/deferred | see §4 dropped table |
| D11 | Manual input cancels a running sequence outright (console: `SEQUENCE CANCELLED`) | legacy only overrode the motor while the stick was deflected — the sequence resumed on release |
| D12 | `#DPAUTO` and `#DPHOME` are runtime-only: always **off** at boot regardless of what is stored | legacy persisted both, so a droid could power up and immediately start moving its own dome because a mode was enabled days earlier — unexpected motion is a safety hazard, so idle automation must be armed by the current session |

## 9. Display (display board only)

Dome position renders as three large 7-segment digits, `---` while the sensor is
warming up or STALE, plus a small activity marker while automation drives. The
backlight sleeps after `#DPLCDSLEEP` seconds (default 300; 0 = always on) with the
dome parked and no input, and wakes in the same loop iteration on dome rotation
(≥8°, above sensor drift), manual input, a running sequence, or any command on any
transport. The panel retains its last frame while dark, so waking costs no redraw.

## 10. Open items

**E-stop release with the remotes off.** The latch clears only on fresh manual input
or an explicit `:DP` command (§5). A droid parked in "sentry" mode — remotes off,
`#DPAUTO1` wandering — therefore stays stopped after any `?STOP` until someone
powers a remote back on. Sabé's DroidNet command library
(`docs/reference/roam-a-dome.dcl.json`) defines no e-stop *or* resume command, and
the bench log shows no mesh message on release, so RAD currently has no way to hear
"all clear". Candidate fix: an explicit re-arm command (`#DPRESUME`) honored from
any transport, so Sabé can clear the latch when its own e-stop is unlatched.

## 11. Phase 0 bench checklist

- Board variant (compact vs display) and exact pin map; boot banner + PCB photos.
- Actual sensor frame rate at the configured baud (tunes §7 constants).
- Whether legacy `#DPPWMARC` exists in the user's flashed build (DCL lists it; upstream
  README does not).
- Live `#DPCONFIG` values and stored sequences (captured via `tools/capture_config.py`).
- WCB fleet: checksum ON, channel 1, octets 3C/4E, device ID 4 unused.
