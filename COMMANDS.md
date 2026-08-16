# Roam-A-Dome v2 — Command Reference

Every command the firmware accepts, with syntax, ranges, and shipped defaults.
This is the operator-facing reference; [BEHAVIOR.md](BEHAVIOR.md) is the
specification it was built against, including the deliberate differences from the
original Roam-a-Dome-Home firmware.

Two grammars share every transport:

| Prefix | Purpose |
|---|---|
| `:DP…` | **motion** — move the dome, run sequences |
| `#DP…` | **config** — settings, status, stored sequences |

Commands are plain text lines terminated by CR or LF. They are accepted on the
USB console, the command serial port (`#DPSERIALCMD1`), and the WCB mesh — all
with identical syntax. Duplicates arriving on two transports within
`#DPDEDUP` ms are executed once.

Anything that is not a `#DP`/`:DP` line is ignored silently (mesh chatter and
RAD's own position reports must never provoke a reply). A recognized but
malformed command answers `Invalid`.

---

## Motion commands (`:DP…`)

A motion line is `:DP` followed by one or more steps separated by `:`.
**Only the first step carries the `DP` prefix:**

```
:DPA90:W2:H          go to 90°, wait 2 s, return home
:DPW5:DPA90          INVALID — the second step must be W5:A90
```

A new `:DP` line replaces any sequence already running. Manual stick input
cancels a running sequence outright (`SEQUENCE CANCELLED (manual override)`).

### Steps

| Step | Syntax | Meaning |
|---|---|---|
| Absolute | `A[M][R\|<deg>][,speed[,maxspeed]][+]` | Rotate to `<deg>`, measured **relative to home**. Negative goes the other way. |
| Relative | `D[M][R\|<deg>][,speed[,maxspeed]][+]` | Rotate `<deg>` from the current position. |
| Spin | `R<-100…100>` | Continuous spin; sign sets direction, `R0` stops. |
| Home | `H[R][speed]` | Seek the home position. |
| Wait (s) | `W<0…600>` | Wait seconds before the next step. |
| Wait (ms) | `WM<ms>` | Wait milliseconds. |
| Random wait (s) | `WR[<max>[,<min>]]` | Random wait; bare `WR` = 1–6 s. Bounds inclusive. |
| Random wait (ms) | `WMR<min>,<max>` | Random wait in milliseconds *(new in v2)*. |
| Play sequence | `S<0…100>` | Play a stored sequence; replaces the running one. |
| Toggle pin | `T<1…8>` | Toggle a digital output. |
| Set pin | `P<1…8><0\|1>` | Drive a digital output low/high. |
| Restore | `Z` | Restore settings and pins to stored defaults. |

Modifiers:

- **`R`** in place of a number — random target (`AR`, `DR`, `HR`, `RR`).
- **`M`** after `A`/`D` — one-shot: return the mode to off after the move.
- **`+`** suffix — fire-and-forget: the sequence advances immediately instead of
  waiting for arrival.
- **`speed`** is the starting speed %, **`maxspeed`** the ramp ceiling
  (defaults to `speed`).

> `Q<n>,<ms>,<pos>,<easing>` (servo) is accepted by the parser for legacy
> compatibility but is **not executed** — v2 has no servo support, and a
> sequence containing it will stop with `Invalid`.

### Examples

```
:DPA0                 point the dome at home
:DPA90,30,100         swing to 90° starting at 30%, ramping to 100%
:DPD-45+              nudge 45° the other way, don't wait for arrival
:DPAR:W2:AR:W2:H      two random looks, two seconds each, then home
:DPR-30               spin continuously counter-clockwise at 30%
:DPS3                 play stored sequence 3
```

---

## Stored sequences

| Command | Meaning |
|---|---|
| `#DPS<n>:<steps>` | Store steps in slot `n` (0–100). Validated before storing; a malformed script is rejected whole. |
| `#DPD<n>` | Delete slot `n`. |
| `#DPL` | List stored sequences. |

```
#DPS3:A50:W2:A-50:W2:H     store
:DPS3                      play
```

---

## Motion tuning

| Command | Range | Default | Meaning |
|---|---|---|---|
| `#DPMAXSPEED<n>` | 0–100 | 100 | Ceiling for all automated motion. |
| `#DPMINSPEED<n>` | 0–100 | 15 | Floor — below this the dome won't actually turn. |
| `#DPHOMESPEED<n>` | 0–100 | 40 | Speed for home seeks. |
| `#DPAUTOSPEED<n>` | 0–100 | 30 | Speed for random idle motion. |
| `#DPTARGETSPEED<n>` | 0–100 | 100 | Speed for targeted moves. |
| `#DPINPUTSPEED<n>` | 0–100 | 100 | Scale applied to manual passthrough. |
| `#DPFUDGE<n>` | 0–20 | 5 | Arrival tolerance in degrees. |
| `#DPSCALE<0\|1>` | | 0 | Enable acceleration/deceleration ramping. |
| `#DPASCALE<n>` | 0–255 | 20 | Acceleration rate when ramping. |
| `#DPDSCALE<n>` | 0–255 | 50 | Deceleration zone, in degrees. |
| `#DPINVERT<0\|1>` | | 1 | Flip manual stick direction vs. motor wiring. |
| `#DPTIMEOUT<sec>` | 0–30 | 5 | Stuck-dome watchdog; 0 disables. |

**Automation direction is learned, not configured.** `#DPINVERT` maps the
*operator's stick* to the motor. Which wire polarity makes the sensor reading
increase is learned automatically by watching the dome move under manual
control, so closed-loop moves can never chase a target the wrong way. The
console prints `[DIR] learned: …` once it locks in.

## Modes and idle automation

| Command | Range | Default | Meaning |
|---|---|---|---|
| `#DPAUTO<0\|1>` | | 0 | Random idle motion ("sentry" dome wandering). |
| `#DPHOME<0\|1>` | | 0 | Return home automatically when idle and away from home. |
| `#DPAUTOLEFT<n>` | 0–180 | 47 | Random arc limit, one side of home. |
| `#DPAUTORIGHT<n>` | 0–180 | 46 | Random arc limit, other side. |
| `#DPAUTOMIN<sec>` / `#DPAUTOMAX<sec>` | | 6 / 8 | Delay range between random moves. |
| `#DPHOMEMIN<sec>` / `#DPHOMEMAX<sec>` | | 6 / 8 | Delay range before an automatic home. |
| `#DPTARGETMIN<sec>` / `#DPTARGETMAX<sec>` | | 0 / 1 | Settle delay range after a targeted move. |
| `#DPAUTOSAFETY<0\|1>` | | 1 | Require a valid sensor before automation runs. |
| `#DPAUTORESTART<0\|1>` | | 1 | Resume automation after it is interrupted. |
| `#DPIDLE<ms>` | 0–60000 | 3000 | Manual-neutral time before automation resumes. |

Automation only runs when the sticks have been neutral for `#DPIDLE` ms **and**
the sensor is valid. Manual input wins instantly, every time.

## Calibration

| Command | Range | Default | Meaning |
|---|---|---|---|
| `#DPHOMEPOS<deg>` | 0–359 | 240 | Set the home position explicitly. |
| `#DPHOMEPOS` | — | | Bare form: snapshot the **current validated** position as home. Answers `SENSOR NOT READY` if the sensor isn't trustworthy yet. |

To calibrate: turn the dome to face forward (by hand or with the sticks), then
send `#DPHOMEPOS`. All `A` moves are measured from there.

## Sensor validation

| Command | Range | Default | Meaning |
|---|---|---|---|
| `#DPMAXRPM<n>` | 1–60 | 60 | Fastest the dome can physically turn; readings implying more are rejected as glitches. |
| `#DPSENSTO<ms>` | 1500–60000 | 2500 | No frames for this long ⇒ sensor STALE, automation disabled. |
| `#DPSENSN<n>` | 1–10 | 3 | Consistent samples needed to accept a position jump as real. |
| `#DPDWELL<n>` | 1–10 | 3 | Consecutive in-tolerance samples counted as "arrived". |
| `#DPSENSORBAUD<n>` | 57600 \| 115200 | 115200 | Sensor ring baud. **Restart required.** |

`#DPSTATS` reports how many readings were accepted, rejected, or confirmed as
jumps — useful for spotting a worn position sticker or a marginal cable.

## Motor output and transports

| Command | Range | Default | Meaning |
|---|---|---|---|
| `#DPSERIALOUT<0\|1>` | | 1 | Drive the motor over Syren/Sabertooth packet serial. |
| `#DPSERIALIN<0\|1>` | | 0 | Accept Syren packet-serial frames as manual input. |
| `#DPSYRENADDR<n>` | | 129 | Set both Syren addresses at once. |
| `#DPSYRENADDRIN<n>` / `#DPSYRENADDROUT<n>` | | 129 | Set them independently. |
| `#DPSYRENBAUD<n>` | | 9600 | Motor controller baud. **Restart required.** |
| `#DPPWMIN<0\|1>` | | 1 | Accept an RC/PWM signal as manual input. **Restart required.** |
| `#DPPWMOUT<0\|1>` | | 0 | Drive the motor with PWM instead of serial. **Restart required.** |
| `#DPPWMMIN<µs>` | 800–2200 | 1000 | PWM pulse at full reverse. |
| `#DPPWMMAX<µs>` | 800–2200 | 2000 | PWM pulse at full forward. |
| `#DPPWMNEUTRAL<µs>` | 800–2200 | 1500 | PWM pulse at rest. |
| `#DPPWMDEADBAND<%>` | 0–50 | 5 | Dead zone around neutral. |
| `#DPSERIALBAUD<n>` | | 9600 | Command serial baud. **Restart required.** |
| `#DPSERIALCMD<0\|1>` | | 1 | Accept commands on the command serial port. |
| `#DPPIN<1-8><0\|1>` | | all 0 | Set a digital output's power-on default (and apply it now). |

## WCB mesh

| Command | Range | Default | Meaning |
|---|---|---|---|
| `#DPWCBEN<0\|1>` | | 1 | Join the WCB ESP-NOW mesh. **Restart required.** |
| `#DPWCBPW<password>` | | *(unset)* | Mesh password — the radio stays off until this is set. **Restart required.** |
| `#DPWCBID<n>` | 1–19 | 4 | This board's WCB device ID. **Restart required.** |
| `#DPWCBOCT<o2>,<o3>` | hex | 3C,4E | Mesh MAC octets. **Restart required.** |
| `#DPWCBQTY<n>` | 1–19 | 3 | Number of WCBs in the fleet. **Restart required.** |
| `#DPWCBCH<n>` | 1–13 | 1 | Mesh channel. **Restart required.** |
| `#DPWCBCS<0\|1>` | | 1 | Mesh checksums. **Restart required.** |
| `#DPDEDUP<ms>` | 0–10000 | 750 | Window for suppressing a command that arrives on two transports. 0 disables. |

Set the password, then restart:

```
#DPWCBPW<your mesh password>
#DPRESTART
```

RAD advertises itself on the mesh as `Roam-A-Dome` with capabilities
`dome,seq,pos`, and emits:

| Message | When |
|---|---|
| `&RAD,POS,<deg>,<mode>` | position changed (≤1 Hz, unicast to Sabé) |
| `&RAD,HB,<fw>,<uptime>,<state>` | every 10 s, broadcast |
| `&RAD,FAULT,<code>` | on a fault, e.g. `SENSOR_STALE` |

**E-stop.** `?STOP` from any transport, or `&SABE,ESTOP` from the mesh, latches
an emergency stop: the motor goes to neutral, any sequence is cancelled, and
automation is disabled. The latch clears when the operator moves the sticks, or
when an explicit new `:DP` motion command arrives. *(An unattended droid with
its remotes off therefore stays stopped until one of those happens — see the
open item in [BEHAVIOR.md](BEHAVIOR.md) §10.)*

## Display

| Command | Range | Default | Meaning |
|---|---|---|---|
| `#DPLCDSLEEP<sec>` | 0–3600 | 300 | Backlight idle timeout in seconds; `0` keeps the screen on permanently. |

The screen blanks after the dome has been parked and untouched for the timeout,
and lights instantly on dome movement, manual input, a running sequence, or any
command arriving on any transport. The panel keeps its last frame while dark, so
waking is immediate — no redraw flash. Takes effect without a restart. Ignored
on the compact (display-less) board.

## Position reporting

| Command | Range | Default | Meaning |
|---|---|---|---|
| `#DPREPORT<ms>` | | 0 | Print `DOME POSITION: <n>` on the console every interval. 0 disables. |

On the **command serial port**, RAD reports every position change as
`#DP<mode><position>`, where mode is:

| Char | Mode |
|---|---|
| `@` | idle / manual |
| `!` | seeking home |
| `$` | random automation |
| `%` | targeted move |

## System

| Command | Meaning |
|---|---|
| `#DPCONFIG` | Dump every setting as replayable `#DPKEY=VALUE` lines. |
| `#DPSTATUS` | Firmware version, uptime, sensor state, motion state, last fault. |
| `#DPSTATS` | Parser, sensor, dedup, and mesh counters. |
| `#DPDEBUG<0\|1>` | Live ~4 Hz telemetry on the console. Not saved. |
| `#DPRESTART` | Reboot. |
| `#DPZERO` | Erase settings (sequences kept) and reboot. |
| `#DPFACTORY` | Erase settings **and** stored sequences, then reboot. |

`#DPCONFIG` output is designed to be replayed straight back into a controller —
`tools/capture_config.py` uses it to back up and restore a configuration.

### Reading `#DPDEBUG1` output

```
[DBG] pos=262 tgt=0 st=idle auto=0 man=0 wire=0 dir=1 sensor=OK rej=0 jmp=0 fault=NONE seq=0 wcb=sabe estop=0
```

| Field | Meaning |
|---|---|
| `pos` / `tgt` | current position / target, degrees |
| `st` | motion state: `idle`, `target`, `spin` |
| `auto` / `man` / `wire` | automation output %, manual input %, actual motor command % |
| `dir` | learned polarity (`0` = not yet learned) |
| `sensor` | `OK`, `WARM` (warming up), `STALE` |
| `rej` / `jmp` | readings rejected as implausible / confirmed position jumps |
| `fault` | `NONE`, `TIMEOUT`, `SENSOR_LOST`, `JUMP` |
| `seq` | a sequence is running |
| `wcb` | `off`, `up` (mesh joined), `sabe` (Sabé heard) |
| `estop` | e-stop latched |

A slowly climbing `rej`/`jmp` count during fast moves is normal on a worn
position sticker — the validation pipeline absorbs it. A climbing count while
the dome is *parked* points at a cable or power problem.
