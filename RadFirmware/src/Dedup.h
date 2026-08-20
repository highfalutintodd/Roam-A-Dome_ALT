// Cross-transport duplicate-command suppression. Pure C++ — host-testable.
//
// Sabé broadcasts :DP commands onto the mesh, and the dome's WCB also fans the
// same broadcast out its serial port — so one command can arrive twice, once
// per transport. A command identical to a recently executed one is suppressed
// if it arrives from the OTHER transport within the window; same-transport
// repeats always run (an operator mashing the same command twice must work).
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

    // A small ring, not one slot: the WCB serial fan-out lags the mesh copy by
    // tens of milliseconds, and in that gap OTHER lines arrive — the next
    // command, RAD's own report echoes, unrelated mesh chatter. One slot would
    // be evicted before the twin shows up and the duplicate would execute.
    static constexpr uint8_t kEntries = 4;

    // Returns true if the line should execute; false if it is the cross-source
    // twin of a recently executed line. windowMs 0 disables suppression.
    bool allow(const char* line, uint8_t source, uint32_t now, uint16_t windowMs) {
        uint32_t h = fnv1a(line);
        if (windowMs != 0) {
            for (uint8_t i = 0; i < kEntries; ++i) {
                const Entry& e = fEntries[i];
                if (e.used && e.hash == h && e.source != source &&
                    (now - e.time) < windowMs) {
                    ++fSuppressed;
                    return false;
                }
            }
        }
        Entry& e = fEntries[fNext];
        fNext = static_cast<uint8_t>((fNext + 1) % kEntries);
        e.used = true;
        e.hash = h;
        e.source = source;
        e.time = now;
        return true;
    }

    uint32_t suppressed() const { return fSuppressed; }

  private:
    struct Entry {
        bool used = false;
        uint32_t hash = 0;
        uint8_t source = 255;
        uint32_t time = 0;
    };
    Entry fEntries[kEntries];
    uint8_t fNext = 0;
    uint32_t fSuppressed = 0;
};

} // namespace rad
