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

## Integrate it (≈6 lines)

In `DomeSensorFirmware32.ino`:

```cpp
#include "PositionDebounce.h"
static PositionDebounce sDebounce;

void loop() {
    unsigned code  = readSensors();        // your existing 9-bit read
    int      angle = getDomeAngle(code);   // your existing table (-1 = invalid)
    int      out;
    if (sDebounce.update(code, angle, millis(), out)) {
        // your existing frame, unchanged:
        Serial.print("#DP@"); Serial.print(out); Serial.print("\r\n");
    }
    // no delay(): poll as fast as you can — see below
}
```

Two things to check:

- **Poll fast.** `PositionDebounce` counts *reads*, not milliseconds. Remove any
  `delay()` in the read loop so it runs ~1 kHz; the default `kStableReads = 4`
  is then ~4 ms of debounce — invisible in motion, but enough to outlast any
  edge transition. If your loop is slower, drop `kStableReads` to 2–3.
- **Keep your heartbeat cadence.** `kHeartbeatMs = 1000` resends the last good
  angle every second so the RaD side never marks the link `STALE` (its timeout is
  2500 ms). Match whatever your firmware already used.

If you'd rather I fold this directly into `DomeSensorFirmware32.ino` (and add the
one-bit-transition check the Reeltwo library already flags in debug), **share that
file** — I kept the change as a wrapper precisely so it can't disturb the decode
table that matches your specific sticker.

## Belt and suspenders

This is the first line of defence, at the point with the most information (the raw
code). The RaD firmware adds a second, independent line: `SensorRing` runs a
median + rate gate, and a **parked-hold** that refuses any position change while
the motor is commanded off (a parked dome can't move, so a jump is by definition a
misread). Either layer alone stops the dangerous failures; together the system is
robust to both transient and stuck sensor lies.
