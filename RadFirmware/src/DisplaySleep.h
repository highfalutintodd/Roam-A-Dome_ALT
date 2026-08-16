// Backlight idle policy for the position display. Pure C++ (no Arduino) so the
// timing rules are host-testable; DisplayS3 owns the actual backlight pin.
//
// The legacy OLED controller blanked its screen when the dome sat idle. v2 does
// the same for the big-number display: after `#DPLCDSLEEP` seconds without
// activity the backlight goes dark, and the next activity wakes it in the same
// loop iteration (no fade, no redraw delay — the panel keeps its last frame, so
// waking is just the backlight pin going high).
#pragma once

#include <cstdint>

namespace rad {

class DisplaySleep {
  public:
    // 0 = never sleep (always-on display).
    void setTimeout(uint32_t ms) { fTimeoutMs = ms; }
    uint32_t timeout() const { return fTimeoutMs; }

    // Register activity: restarts the idle countdown. Deliberately does NOT flip
    // the awake flag itself — tick() is the only place the state changes, so the
    // wake edge is always reported exactly once to whoever drives the backlight.
    void poke(uint32_t now) { fLastActivity = now; }

    // Advance the clock. Returns true when the awake/asleep state CHANGED this
    // tick, which is the caller's cue to drive the backlight pin.
    // Rollover-safe: unsigned subtraction, never `now > deadline`.
    bool tick(uint32_t now) {
        bool want = (fTimeoutMs == 0) || (now - fLastActivity) < fTimeoutMs;
        if (want == fAwake)
            return false;
        fAwake = want;
        return true;
    }

    bool awake() const { return fAwake; }

  private:
    uint32_t fTimeoutMs = 0;   // 0 = disabled
    uint32_t fLastActivity = 0;
    bool fAwake = true;
};

} // namespace rad
