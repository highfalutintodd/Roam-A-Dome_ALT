// Syren/Sabertooth packet-serial frame codec. Pure C++ — host-testable.
//
// Frame: [address][command][data][checksum] where checksum = (addr+cmd+data) & 0x7F.
// Addresses are 128–135 (high bit set); command and data bytes are 0–127, which is
// what makes resynchronization on a high-bit byte reliable.
#pragma once

#include <cstdint>

namespace rad {

struct SyrenFrame {
    uint8_t addr = 0;
    uint8_t cmd = 0;
    uint8_t data = 0;

    // Motor commands 0 (forward) and 1 (reverse); data 0 = stopped.
    bool isMotorCmd() const { return cmd == 0 || cmd == 1; }
    bool isNeutral() const { return isMotorCmd() && data == 0; }
};

inline void syrenEncode(const SyrenFrame& f, uint8_t out[4]) {
    out[0] = f.addr;
    out[1] = f.cmd;
    out[2] = f.data;
    out[3] = static_cast<uint8_t>(f.addr + f.cmd + f.data) & 0x7F;
}

// Feed bytes one at a time; returns true when a checksum-valid frame for the
// expected address completes. Bad checksums and foreign addresses are counted and
// the decoder resyncs on the next high-bit byte.
class SyrenDecoder {
  public:
    explicit SyrenDecoder(uint8_t expectedAddr = 129) : fAddr(expectedAddr) {}

    void setAddress(uint8_t addr) { fAddr = addr; }

    bool feed(uint8_t byte, SyrenFrame& out) {
        if (fCount == 0) {
            if ((byte & 0x80) == 0)
                return false; // not an address byte; skip until sync
            fBuf[fCount++] = byte;
            return false;
        }
        if ((byte & 0x80) != 0) { // unexpected new address byte: resync here
            ++fResyncs;
            fBuf[0] = byte;
            fCount = 1;
            return false;
        }
        fBuf[fCount++] = byte;
        if (fCount < 4)
            return false;
        fCount = 0;
        uint8_t sum = static_cast<uint8_t>(fBuf[0] + fBuf[1] + fBuf[2]) & 0x7F;
        if (sum != fBuf[3]) {
            ++fChecksumErrors;
            return false;
        }
        if (fBuf[0] != fAddr) {
            ++fForeignFrames;
            return false;
        }
        out = SyrenFrame{fBuf[0], fBuf[1], fBuf[2]};
        return true;
    }

    uint32_t checksumErrors() const { return fChecksumErrors; }
    uint32_t resyncs() const { return fResyncs; }
    uint32_t foreignFrames() const { return fForeignFrames; }

  private:
    uint8_t fAddr;
    uint8_t fBuf[4] = {};
    uint8_t fCount = 0;
    uint32_t fChecksumErrors = 0;
    uint32_t fResyncs = 0;
    uint32_t fForeignFrames = 0;
};

} // namespace rad
