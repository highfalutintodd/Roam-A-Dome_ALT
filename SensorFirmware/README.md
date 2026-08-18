# RaD Sensor Ring firmware hardening

The dome position sensor is a **Reeltwo-style pseudo-absolute optical encoder**:
9 IR reflective sensors read a 9-bit code off the printed ring (a Mimir sticker),
and a lookup table (`getDomeAngle(code)`) maps each valid code to an angle. The
firmware on that ESP32 is `DomeSensorFirmware32.ino` (referenced from the RaD
firmware's `SensorRing.h`).

## The problem this fixes

Even with a good mount and a high-quality sticker, the **raw** code stream lies at
two moments:

1. **Transitions.** As the dome crosses a mark edge, several sensors switch at
   slightly different instants. The in-between codes are either invalid
   (`getDomeAngle == -1`) or alias to a valid code for a *far-away* angle. This is
   how a parked dome reported **299° while actually at 194° for ~4 seconds** in
   the field log — corrupting the start point of the next move (the "semi
   flip-out").
2. **Boundary flicker.** Parked exactly on an edge, one sensor dithers and the
   code toggles between two valid patterns ~15–20° apart. A median *after* decode
   can't fix this; it just tracks the recent majority, so the angle flip-flops.

## The fix: debounce the raw code *before* decoding

`PositionDebounce.h` only emits an angle once the **exact same 9-bit code** has
been read several times in a row **and** decodes to a valid angle. A transition
never holds one code that long; a flicker never holds one code that long. What
survives is the true settled position. It keeps your existing pin map,
`getDomeAngle()` table, and serial frame — it only gates *when* a reading is
trustworthy.

## Integrate it into `DomeSensorFirmware32.ino`

Targets the upstream Reeltwo firmware
([reeltwo/DomeSensorFirmware32](https://github.com/reeltwo/DomeSensorFirmware32)),
`loop()` at lines ~273–381. That sketch reports `short angle =
sDomePosition.getAngle();` and prints `#DP@%d` on change or every
`POSITION_RESEND_INTERVAL` (1000 ms).

The catch: `getAngle()` medians *after* decode. A median can't undo a
transition-aliased or stuck raw code — it's already in the buffer and drags the
median with it. So debounce the **raw code**, which the class exposes via
`readSensors()` and `getDomeAngle()`. The change is four small edits; nothing
touches the pins, the `getDomeAngle()` table (matched to your Mimir sticker), the
`#DP@` frame, the baud, or the NeoPixel/debug output.

**Edit 1 — copy the header into the sketch folder** (same directory as the
`.ino`, so the quoted `#include` resolves):

```
cp SensorFirmware/PositionDebounce.h  <path to>/DomeSensorFirmware32/
```

**Edit 2 — add the include** after the last `#include` (line 6,
`#include "core/StringUtils.h"`):

```cpp
#include "PositionDebounce.h"
```

**Edit 3 — add the global** after the last global (line 63, `static bool sDebug;`):

```cpp
static PositionDebounce sDebounce;
```

**Edit 4 — in `loop()`, change how `angle` is obtained and gated.**

Replace line 282:

```cpp
    short angle = sDomePosition.getAngle();
```

with:

```cpp
    // Debounce the RAW sensor code before trusting it. getAngle() medians AFTER
    // decode, which can't undo a transition-aliased or stuck code; the raw code
    // must simply hold steady for a few reads. sHeldAngle keeps the last good
    // value, so a burst of invalid/transition codes holds position instead of
    // reporting a phantom. (-1 until the first lock.)
    unsigned code = sDomePosition.readSensors();
    int out;
    static short sHeldAngle = -1;
    if (sDebounce.update(code, sDomePosition.getDomeAngle(code), millis(), out))
        sHeldAngle = out;
    short angle = sHeldAngle;
```

and replace the next line (283):

```cpp
    if (sDomePosition.ready())
```

with:

```cpp
    if (angle >= 0)
```

That's it. `getAngle()` is no longer used (so `ready()`'s warm-up is replaced by
the debounce's own lock — `angle` stays `-1` until a code holds steady, which also
guarantees you never emit `#DP@-1`). Everything downstream — the `angle !=
sLastAngle || resend` gate, the `#DP@%d` frame, NeoPixel, `sDebug`/`sVerbose`
echoes — works unchanged on the now-clean `angle`.

### Two things to confirm

- **Poll fast.** `PositionDebounce` counts *reads*, not milliseconds. Upstream
  `loop()` has no `delay()`, so it free-runs at several kHz — the default
  `kStableReads = 4` is then a millisecond or two of debounce: invisible in
  motion, long enough to outlast any edge transition. If you ever add a loop
  delay, drop `kStableReads` to 2–3.
- **Heartbeat matches.** `kHeartbeatMs = 1000` equals `POSITION_RESEND_INTERVAL`,
  so the RaD side never trips its 2500 ms `STALE` timeout.

### Optional stronger check

The Reeltwo `DomeSensorRing` already computes `countChangedBits(mask, lastMask)`
and flags `> 1` as a bad state in debug. Once the debounce is in, you could also
*reject* a newly-stable code whose bit-count jumped by more than one from the last
good code unless it holds even longer — but in practice the stability gate alone
removes the transition/flicker misreads seen in the logs. Start with the edits
above and only add this if a residual slips through.

## Belt and suspenders

This is the first line of defence, at the point with the most information (the raw
code). The RaD firmware adds a second, independent line: `SensorRing` runs a
median + rate gate, and a **parked-hold** that refuses any position change while
the motor is commanded off (a parked dome can't move, so a jump is by definition a
misread). Either layer alone stops the dangerous failures; together the system is
robust to both transient and stuck sensor lies.
