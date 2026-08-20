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
            // ~20 ms with near-identical widths; a floating input picking up
            // motor noise produces plausible-width pulses at random spacing
            // and random widths. Count consecutive pulses arriving at RC
            // cadence AND width-consistent — pulseUs() only believes the
            // input once the streak shows a genuine transmitter (field
            // failures: lone noise blips cancelled sequences as phantom
            // "manual", and cadence alone was still beaten during full-power
            // drive — the width check is what noise cannot fake).
            uint32_t nowMs = millis();
            int32_t step = static_cast<int32_t>(width) - self->fPulseUs;
            bool cadence = nowMs - self->fPrevPulseMs <= kMaxFrameGapMs;
            bool similar = step <= static_cast<int32_t>(kMaxWidthStepUs) &&
                           step >= -static_cast<int32_t>(kMaxWidthStepUs);
            if (cadence && similar) {
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
