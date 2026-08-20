#ifdef ARDUINO

#include "PwmIO.h"

namespace rad {

void IRAM_ATTR PwmIO::isr(void* arg) {
    auto* self = static_cast<PwmIO*>(arg);
    uint32_t nowUs = micros();
    if (digitalRead(self->fInPin) == HIGH) {
        self->fRiseUs = nowUs;
    } else {
        uint32_t width = nowUs - self->fRiseUs;
        if (width >= 500 && width <= 2500) { // plausible RC pulse only
            // Pulse-TRAIN tracking: a real receiver repeats frames every
            // ~20 ms; a floating input picking up motor noise produces lone
            // plausible-width pulses at random spacing. Count consecutive
            // pulses arriving at RC cadence — pulseUs() only believes the
            // input once the streak shows a genuine transmitter (field
            // failure: noise blips cancelled sequences as phantom "manual").
            uint32_t nowMs = millis();
            if (nowMs - self->fPrevPulseMs <= kMaxFrameGapMs) {
                if (self->fStreak < 255)
                    self->fStreak = self->fStreak + 1;
            } else {
                self->fStreak = 1;
            }
            self->fPrevPulseMs = nowMs;
            self->fPulseUs = static_cast<uint16_t>(width);
            self->fLastEdgeMs = nowMs;
        }
    }
}

} // namespace rad

#endif // ARDUINO
