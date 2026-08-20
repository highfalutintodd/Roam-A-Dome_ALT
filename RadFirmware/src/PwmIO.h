// RC pulse (PWM) input capture + passthrough output. Arduino/ESP32 core 3.x only.
#pragma once
#ifdef ARDUINO

#include "Settings.h"

#include <Arduino.h>

namespace rad {

class PwmIO {
  public:
    void begin(int inPin, int outPin, const RadSettings& s) {
        fSettings = s;
        fOutPin = outPin;
        if (s.pwmIn && inPin >= 0) {
            fInPin = inPin;
            pinMode(inPin, INPUT);
            attachInterruptArg(digitalPinToInterrupt(inPin), isr, this, CHANGE);
        }
        if (s.pwmOut && outPin >= 0) {
            // Core 3.x LEDC: 50 Hz servo-style output, 14-bit resolution.
            ledcAttach(outPin, 50, 14);
            writePulseUs(s.pwmNeutralUs);
        }
    }

    // A real RC receiver repeats frames at ~50 Hz with near-identical widths
    // frame to frame; require this many consecutive pulses at that cadence AND
    // width-consistent before believing the input. A lone plausible-width
    // pulse (floating pin + motor noise) must never read as stick input — it
    // was cancelling sequences as phantom "manual override" — and a streak of
    // 3 was still beaten by cadenced noise bursts during full-power drive
    // (bench 2026-08-20 evening: [MAN] 23% with the transmitter off). Eight
    // width-consistent frames is 160 ms — imperceptible on a real stick grab,
    // ~1e-6 for random-width noise.
    static constexpr uint8_t kMinPulseStreak = 8;
    static constexpr uint32_t kMaxFrameGapMs = 40; // 50 Hz frames + margin
    // Max width change between consecutive streak frames. A stick sweep moves
    // ~10-60 us/frame; noise widths are random over the whole 500-2500 band.
    // A genuine instant stick slam breaks the streak once and re-arms in
    // kMinPulseStreak frames.
    static constexpr uint16_t kMaxWidthStepUs = 150;

    // Latest captured pulse width; 0 if none seen for staleWindowMs or the
    // pulse train is too short to be a genuine transmitter.
    uint16_t pulseUs(uint32_t now, uint32_t staleWindowMs = 100) const {
        uint32_t last = fLastEdgeMs;
        if (last == 0 || (now - last) > staleWindowMs)
            return 0;
        if (fStreak < kMinPulseStreak)
            return 0; // lone blip, not a pulse train: fail neutral
        return fPulseUs;
    }

    // Pulse mapped to -100..100 around neutral, deadband-gated, scaled by
    // #DPINPUTSPEED. 0 when neutral, inside the deadband, stale, or the
    // calibration is misordered (min >= max would wrap the deadband math).
    int8_t manualPercent(uint32_t now) const {
        uint16_t us = pulseUs(now);
        if (us == 0)
            return 0;
        if (fSettings.pwmMinUs >= fSettings.pwmMaxUs)
            return 0; // misordered calibration: fail neutral, not wrapped
        uint16_t band = static_cast<uint16_t>(
            (uint32_t)(fSettings.pwmMaxUs - fSettings.pwmMinUs) * fSettings.pwmDeadbandPct / 100);
        int32_t delta = static_cast<int32_t>(us) - fSettings.pwmNeutralUs;
        if (delta > -band && delta < band)
            return 0;
        int32_t halfSpan = delta > 0 ? fSettings.pwmMaxUs - fSettings.pwmNeutralUs
                                     : fSettings.pwmNeutralUs - fSettings.pwmMinUs;
        if (halfSpan <= 0)
            return 0;
        int32_t pct = delta * 100 / halfSpan;
        pct = pct > 100 ? 100 : (pct < -100 ? -100 : pct);
        return static_cast<int8_t>(pct * fSettings.inputSpeed / 100);
    }

    void writePulseUs(uint16_t us) {
        if (!fSettings.pwmOut || fOutPin < 0)
            return;
        // duty = us / 20000us * 2^14
        ledcWrite(fOutPin, (static_cast<uint32_t>(us) << 14) / 20000);
    }

    // Emit a motor command as a servo pulse (used when #DPPWMOUT is enabled).
    void drivePercent(int8_t pct) {
        int32_t halfSpan = pct >= 0 ? fSettings.pwmMaxUs - fSettings.pwmNeutralUs
                                    : fSettings.pwmNeutralUs - fSettings.pwmMinUs;
        if (halfSpan <= 0) { // misordered calibration: neutral, never reversed
            writePulseUs(fSettings.pwmNeutralUs);
            return;
        }
        writePulseUs(static_cast<uint16_t>(fSettings.pwmNeutralUs + halfSpan * pct / 100));
    }

  private:
    // Defined out-of-line in PwmIO.cpp: IRAM_ATTR functions must not be inline in
    // headers (Xtensa "l32r literal placed after use" link error).
    static void isr(void* arg);

    RadSettings fSettings;
    int fInPin = -1;
    int fOutPin = -1;
    volatile uint32_t fRiseUs = 0;
    volatile uint16_t fPulseUs = 0;
    volatile uint32_t fLastEdgeMs = 0;
    volatile uint32_t fPrevPulseMs = 0; // previous valid pulse (train spacing)
    volatile uint8_t fStreak = 0;       // consecutive pulses at RC cadence
};

} // namespace rad

#endif // ARDUINO
