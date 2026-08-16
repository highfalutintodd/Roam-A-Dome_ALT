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
            self->fPulseUs = static_cast<uint16_t>(width);
            self->fLastEdgeMs = millis();
        }
    }
}

} // namespace rad

#endif // ARDUINO
