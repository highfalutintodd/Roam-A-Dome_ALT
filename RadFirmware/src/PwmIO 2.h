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

    // Latest captured pulse width, 0 if none seen for staleWindowMs.
    uint16_t pulseUs(uint32_t now, uint32_t staleWindowMs = 100) const {
        uint32_t last = fLastEdgeMs;
        if (last == 0 || (now - last) > staleWindowMs)
            return 0;
        return fPulseUs;
    }

    bool manualActive(uint32_t now) const { return manualPercent(now) != 0; }

    // Pulse mapped to -100..100 around neutral, deadband-gated, scaled by
    // #DPINPUTSPEED. 0 when neutral, inside the deadband, or stale.
    int8_t manualPercent(uint32_t now) const {
        uint16_t us = pulseUs(now);
        if (us == 0)
            return 0;
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
};

} // namespace rad

#endif // ARDUINO
