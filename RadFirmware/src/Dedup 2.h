// Cross-transport duplicate-command suppression. Pure C++ — host-testable.
//
// Sabé broadcasts :DP commands onto the mesh, and the dome's WCB also fans the
// same broadcast out its serial port — so one command can arrive twice, once
// per transport. A command identical to the last executed one is suppressed if
// it arrives from the OTHER transport within the window; same-transport repeats
// always run (an operator mashing the same command twice must work).
#pragma once

#include <cstdint>

namespace rad {

inline uint32_t fnv1a(const char* s) {
    uint32_t h = 2166136261u;
    while (*s != '\0') {
        h ^= static_cast<uint8_t>(*s++);
        h *= 16777619u;
    }
    return h;
}

class DedupFilter {
  public:
    static constexpr uint8_t kSourceSerial = 0;
    static constexpr uint8_t kSourceMesh = 1;

    // Returns true if the line should execute; false if it is the cross-source
    // twin of a just-executed line. windowMs 0 disables suppression.
    bool allow(const char* line, uint8_t source, uint32_t now, uint16_t windowMs) {
        uint32_t h = fnv1a(line);
        if (windowMs != 0 && h == fLastHash && source != fLastSource &&
            (now - fLastTime) < windowMs) {
            ++fSuppressed;
            return false;
        }
        fLastHash = h;
        fLastSource = source;
        fLastTime = now;
        return true;
    }

    uint32_t suppressed() const { return fSuppressed; }

  private:
    uint32_t fLastHash = 0;
    uint8_t fLastSource = 255;
    uint32_t fLastTime = 0;
    uint32_t fSuppressed = 0;
};

} // namespace rad
