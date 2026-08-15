// Syren packet-serial passthrough: droid controller in -> motor controller out.
// Arduino-only wrapper around the pure SyrenCodec.
#pragma once
#ifdef ARDUINO

#include "Settings.h"
#include "SyrenCodec.h"

#include <Arduino.h>

namespace rad {

class SyrenBus {
  public:
    void begin(HardwareSerial& io, const RadSettings& s) {
        fIo = &io;
        fDecoder.setAddress(s.syrenAddrIn);
        fAddrOut = s.syrenAddrOut;
        fInEnabled = s.serialIn;
        fOutEnabled = s.serialOut;
    }

    // Pump all pending input bytes; forward complete valid frames re-addressed to
    // the output address. Returns true if a frame was forwarded this call.
    bool pump(uint32_t now) {
        if (fIo == nullptr || !fInEnabled)
            return false;
        bool forwarded = false;
        while (fIo->available() > 0) {
            SyrenFrame frame;
            if (!fDecoder.feed(static_cast<uint8_t>(fIo->read()), frame))
                continue;
            fLastFrameMs = now;
            fLastNeutral = frame.isNeutral();
            if (fOutEnabled) {
                frame.addr = fAddrOut;
                uint8_t buf[4];
                syrenEncode(frame, buf);
                fIo->write(buf, sizeof(buf));
                forwarded = true;
            }
        }
        return forwarded;
    }

    // Emit a motor command from RAD itself (automation; Phase 3).
    void drive(int8_t speedPct) {
        if (fIo == nullptr || !fOutEnabled)
            return;
        SyrenFrame frame;
        frame.addr = fAddrOut;
        frame.cmd = speedPct >= 0 ? 0 : 1;
        int mag = speedPct >= 0 ? speedPct : -speedPct;
        frame.data = static_cast<uint8_t>((mag * 127) / 100);
        uint8_t buf[4];
        syrenEncode(frame, buf);
        fIo->write(buf, sizeof(buf));
    }

    // Manual input considered "active" when the last frame was non-neutral, or any
    // frame arrived recently (arbitration ladder rung 2; window tuned in Phase 3).
    bool manualActive(uint32_t now, uint32_t windowMs = 250) const {
        return fLastFrameMs != 0 && !fLastNeutral && (now - fLastFrameMs) < windowMs;
    }

    uint32_t checksumErrors() const { return fDecoder.checksumErrors(); }

  private:
    HardwareSerial* fIo = nullptr;
    SyrenDecoder fDecoder;
    uint8_t fAddrOut = 129;
    bool fInEnabled = true;
    bool fOutEnabled = true;
    uint32_t fLastFrameMs = 0;
    bool fLastNeutral = true;
};

} // namespace rad

#endif // ARDUINO
