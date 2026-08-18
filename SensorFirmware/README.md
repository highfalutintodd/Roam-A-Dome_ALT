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

This targets the upstream Reeltwo firmware
([reeltwo/DomeSensorFirmware32](https://github.com/reeltwo/DomeSensorFirmware32)).
That sketch reports with `short angle = sDomePosition.getAngle();` and then, on
change or every `POSITION_RESEND_INTERVAL` ms, sends `snprintf(buf,…,"#DP@%d",…)`
out `REPORT_SERIAL`.

The catch: `getAngle()` medians *after* decode. A median can't undo a
transition-aliased or stuck raw code — that value is already in the buffer and
drags the median with it. So debounce the **raw code**, which the class exposes
via `readSensors()` and `getDomeAngle()`.

**1. At the top of the sketch, next to the other globals:**

```cpp
#include "PositionDebounce.h"
static PositionDebounce sDebounce;
```

**2. In `loop()`, replace the acquisition + report block.** Where the firmware does:

```cpp
short angle = sDomePosition.getAngle();
if (angle != sLastAngle || millis() - sLastReport > POSITION_RESEND_INTERVAL) {
    snprintf(buf, sizeof(buf), "#DP@%d", angle);
    REPORT_SERIAL.println(buf);
    ...
}
```

use the raw code, gated by the debounce (which does its own change + heartbeat
logic, so `sLastAngle` / `sLastReport` are no longer needed):

```cpp
unsigned code  = sDomePosition.readSensors();      // raw 9-bit pattern
int      raw   = sDomePosition.getDomeAngle(code); // -1 if not a valid code
int      angle;
if (sDebounce.update(code, raw, millis(), angle)) {
    snprintf(buf, sizeof(buf), "#DP@%d", angle);
    REPORT_SERIAL.println(buf);                    // frame + serial unchanged
    // Serial.print("POS: "); Serial.println(angle);   // keep your console echo if wanted
}
```

Nothing else changes — same pins, same `getDomeAngle()` table (matched to your
Mimir sticker), same `REPORT_SERIAL`, same `#DP@` frame and baud.

### Two things to confirm

- **Poll fast.** `PositionDebounce` counts *reads*, not milliseconds. The upstream
  `loop()` has no `delay()`, so it already free-runs at several kHz — the default
  `kStableReads = 4` is then a millisecond or two of debounce: invisible in
  motion, long enough to outlast any edge transition. If you ever add a loop
  delay, drop `kStableReads` to 2–3.
- **Heartbeat matches.** `kHeartbeatMs = 1000` equals the firmware's
  `POSITION_RESEND_INTERVAL`, so the RaD side never trips its 2500 ms `STALE`
  timeout.

### Optional stronger check

The Reeltwo `DomeSensorRing` already computes `countChangedBits(mask, lastMask)`
and flags `> 1` as a bad state in debug. Once the debounce is in, you can also
*reject* a newly-stable code whose bit-count jumped by more than one from the last
good code unless it holds even longer — but in practice the stability gate alone
removes the transition/flicker misreads seen in the logs, so start with the drop-in
above and only add this if a residual slips through.

## Belt and suspenders

This is the first line of defence, at the point with the most information (the raw
code). The RaD firmware adds a second, independent line: `SensorRing` runs a
median + rate gate, and a **parked-hold** that refuses any position change while
the motor is commanded off (a parked dome can't move, so a jump is by definition a
misread). Either layer alone stops the dangerous failures; together the system is
robust to both transient and stuck sensor lies.
