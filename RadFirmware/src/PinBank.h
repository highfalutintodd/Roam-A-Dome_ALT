// Digital output pins for :DPT / :DPP / #DPPIN. Arduino-only.
#pragma once
#ifdef ARDUINO

#include <Arduino.h>

namespace rad {

class PinBank {
  public:
    static constexpr uint8_t kMaxPins = 8;

    // pins: board GPIO numbers, count <= 8; defaults: bitmask, bit0 = pin 1.
    void begin(const int* pins, uint8_t count, uint8_t defaults) {
        fCount = count < kMaxPins ? count : kMaxPins;
        fDefaults = defaults;
        for (uint8_t i = 0; i < fCount; ++i) {
            fPins[i] = pins[i];
            pinMode(fPins[i], OUTPUT);
        }
        restore();
    }

    // 1-based pin index, matching the command grammar.
    bool set(uint8_t index, bool value) {
        if (index < 1 || index > fCount)
            return false;
        fState = value ? (fState | (1u << (index - 1))) : (fState & ~(1u << (index - 1)));
        digitalWrite(fPins[index - 1], value ? HIGH : LOW);
        return true;
    }

    bool toggle(uint8_t index) {
        if (index < 1 || index > fCount)
            return false;
        return set(index, (fState & (1u << (index - 1))) == 0);
    }

    void restore() { // :DPZ semantics: back to stored defaults
        for (uint8_t i = 0; i < fCount; ++i)
            set(i + 1, (fDefaults & (1u << i)) != 0);
    }

    void setDefaults(uint8_t defaults) { fDefaults = defaults; }

  private:
    int fPins[kMaxPins] = {};
    uint8_t fCount = 0;
    uint8_t fState = 0;
    uint8_t fDefaults = 0;
};

} // namespace rad

#endif // ARDUINO
