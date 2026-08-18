// PositionDebounce.h — drop-in hardening for the RaD dome sensor ring firmware
// (DomeSensorFirmware32.ino, the ESP32 that reads the 9 IR sensors off the
// printed absolute-encoder ring).
//
// WHY THIS EXISTS
// ---------------
// The ring is a pseudo-absolute optical encoder: 9 IR sensors read a 9-bit code
// off the ring, and getDomeAngle(code) maps each valid pattern to an angle.
// Two things make the raw stream lie, even with a perfect sticker and mount:
//   1. TRANSITIONS. When the dome sits on (or sweeps past) a mark edge, several
//      sensors change at slightly different instants. The intermediate codes are
//      either invalid (getDomeAngle == -1) or — worse — happen to alias to a
//      valid code for a FAR-AWAY angle. That is how a parked dome can report a
//      stable-but-wrong angle for seconds (observed: parked at 194°, reported
//      299° for ~4 s), which corrupts the start point of the next move.
//   2. BOUNDARY FLICKER. Parked exactly on an edge, one sensor dithers, so the
//      code toggles between two valid patterns whose angles differ by ~15–20°.
//      A median-after-decode can't fix this — it just picks whichever value was
//      the recent majority, so the reported angle flip-flops.
//
// THE FIX: debounce the RAW 9-bit code, before decoding. Only emit an angle once
// the exact same code has been read N times in a row AND decodes to a valid
// angle. A transition never holds one code that long; a boundary flicker never
// holds one code that long. What survives is the true, settled position.
//
// This preserves YOUR existing pin map, YOUR getDomeAngle() table (which matches
// your Mimir sticker), and YOUR serial output. It only decides WHEN a reading is
// trustworthy enough to send. Nothing here assumes a particular ring pattern.
//
// INTEGRATION (see README.md for the full walkthrough):
//   - Read the sensors FAST — poll in a tight loop (aim ~1 kHz). With the default
//     kStableReads=4 that is ~4 ms of debounce: imperceptible lag, kills flicker.
//   - Feed each read through update(); emit only when it returns true.
//
//   static PositionDebounce sDebounce;
//   void loop() {
//     unsigned code = sDomePosition.readSensors();       // existing 9-bit read
//     int      raw  = sDomePosition.getDomeAngle(code);  // existing table (-1 = invalid)
//     int      out;
//     if (sDebounce.update(code, raw, millis(), out)) {
//       snprintf(buf, sizeof(buf), "#DP@%d", out);
//       REPORT_SERIAL.println(buf);                      // existing frame, unchanged
//     }
//   }
//
// (Debounce the RAW code, not sDomePosition.getAngle() — getAngle() medians after
//  decode, which can't undo a stuck or transition-aliased code.)
//
// The RaD firmware's own SensorRing filter (median + rate gate + parked-hold) is
// the second layer of defence; this is the first, at the point with the most
// information — the raw code.
#pragma once

#include <stdint.h>

class PositionDebounce {
  public:
    // Consecutive identical raw reads required before a code is believed. At a
    // ~1 kHz poll this is a few milliseconds — long enough to outlast any edge
    // transition or dither, short enough to be invisible in motion.
    static constexpr uint8_t kStableReads = 4;

    // Resend the last good angle at least this often even when nothing changes,
    // so the RaD side never times the link out as STALE. Match this to whatever
    // your firmware used (the RaD stale timeout is 2500 ms, so 1000 ms is safe).
    static constexpr uint32_t kHeartbeatMs = 1000;

    // Call once per sensor read.
    //   rawCode : the 9-bit sensor mask (readSensors()).
    //   angle   : getDomeAngle(rawCode), or -1 if the code is not a valid position.
    //   nowMs   : millis().
    //   outAngle: set to the angle to emit when this returns true.
    // Returns true when a new, stable, valid angle should be sent — or on the
    // periodic heartbeat. Returns false while the code is mid-transition,
    // invalid, or unchanged between heartbeats.
    bool update(unsigned rawCode, int angle, uint32_t nowMs, int& outAngle) {
        // Debounce the raw code: any change restarts the stability counter.
        if (rawCode != fLastCode) {
            fLastCode = rawCode;
            fStable = 1;
        } else if (fStable < kStableReads) {
            ++fStable;
        }

        const bool heartbeatDue = (fEmitted >= 0) && (nowMs - fLastEmitMs >= kHeartbeatMs);
        const bool trustworthy = (fStable >= kStableReads) && (angle >= 0);

        if (trustworthy) {
            if (angle != fEmitted) {       // settled on a genuinely new position
                fEmitted = angle;
                fLastEmitMs = nowMs;
                outAngle = angle;
                return true;
            }
            if (heartbeatDue) {            // unchanged, but keep the link alive
                fLastEmitMs = nowMs;
                outAngle = angle;
                return true;
            }
            return false;
        }

        // Mid-transition or invalid code: never emit a fresh value here — that is
        // exactly the misread we are suppressing. Still honour the heartbeat with
        // the last GOOD angle so the dome doesn't appear to have vanished.
        if (heartbeatDue) {
            fLastEmitMs = nowMs;
            outAngle = fEmitted;
            return true;
        }
        return false;
    }

    // Last angle actually emitted, or -1 before the first lock. Handy for a
    // status LED or local display.
    int lastAngle() const { return fEmitted; }

  private:
    unsigned fLastCode = ~0u; // last raw code seen (impossible sentinel to start)
    uint8_t fStable = 0;      // consecutive identical reads of fLastCode
    int fEmitted = -1;        // last angle sent (-1 = none yet)
    uint32_t fLastEmitMs = 0; // when we last sent, for the heartbeat
};
