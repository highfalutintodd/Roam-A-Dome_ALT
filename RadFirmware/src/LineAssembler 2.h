// Per-source line assembly. Pure C++ — host-testable.
//
// Each command transport (console, command serial, mesh queue) owns its own
// assembler, fixing the legacy design where two serial ports interleaved bytes
// into one shared buffer.
#pragma once

#include <cstdint>

namespace rad {

class LineAssembler {
  public:
    static constexpr uint16_t kCapacity = 256;

    // Feed one byte. Returns a NUL-terminated line when CR or LF completes one,
    // nullptr otherwise. Blank lines return "" (caller ignores them — they must
    // never affect sequencer state; BEHAVIOR.md D1). Overlong lines are dropped
    // whole and counted.
    const char* feed(char ch) {
        if (ch == '\r' || ch == '\n') {
            if (fOverflowed) { // discard the tail of an overlong line
                fOverflowed = false;
                fLen = 0;
                return nullptr;
            }
            fBuf[fLen] = '\0';
            fLen = 0;
            return fBuf;
        }
        if (fOverflowed)
            return nullptr;
        if (fLen >= kCapacity - 1) {
            fOverflowed = true;
            ++fOverflows;
            fLen = 0;
            return nullptr;
        }
        fBuf[fLen++] = ch;
        return nullptr;
    }

    uint32_t overflows() const { return fOverflows; }

  private:
    char fBuf[kCapacity];
    uint16_t fLen = 0;
    bool fOverflowed = false;
    uint32_t fOverflows = 0;
};

} // namespace rad
