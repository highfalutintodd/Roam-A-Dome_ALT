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

    bool manualActive(uint32_t now) const {
        uint16_t us = pulseUs(now);
        if (us == 0)
            return false;
        uint16_t band = static_cast<uint16_t>(
            (uint32_t)(fSettings.pwmMaxUs - fSettings.pwmMinUs) * fSettings.pwmDeadbandPct / 100);
        return us < fSettings.pwmNeutralUs - band || us > fSettings.pwmNeutralUs + band;
    }

    void writePulseUs(uint16_t us) {
        if (!fSettings.pwmOut || fOutPin < 0)
            return;
        // duty = us / 20000us * 2^14
        ledcWrite(fOutPin, (static_cast<uint32_t>(us) << 14) / 20000);
    }

    // Passthrough: mirror input to output (arbitration hooks in later phases).
    void pump(uint32_t now) {
        if (!fSettings.pwmIn || !fSettings.pwmOut)
            return;
        uint16_t us = pulseUs(now);
        if (us != 0)
            writePulseUs(us);
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
