// Syren packet-serial input decode + non-motor passthrough. Motor commands from
// the droid controller are DECODED here (manualPercent) and re-driven by the
// glue through the single motor output path — never raw-forwarded, so RAD's own
// synthesized stream is the only one on the wire. Arduino-only wrapper around
// the pure SyrenCodec.
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

    // Pump all pending input bytes, decoding manual motor input; NON-motor
    // frames (timeout/ramp/config) are forwarded re-addressed to the output.
    // Motor commands are NOT raw-forwarded: RAD re-synthesizes the motor stream
    // itself (the decoded input feeds driveMotor, which applies #DPINVERT), and
    // forwarding the raw frame alongside would put two contradictory command
    // streams on the same UART. Returns true if a frame was forwarded.
    bool pump(uint32_t now) {
        if (fIo == nullptr || !fInEnabled)
            return false;
        bool forwarded = false;
        while (fIo->available() > 0) {
            SyrenFrame frame;
            if (!fDecoder.feed(static_cast<uint8_t>(fIo->read()), frame))
                continue;
            // Manual-input freshness tracks MOTOR frames only: a controller
            // that keeps sending non-motor frames (timeout/ramp keepalives)
            // after the stick went quiet must not hold the last non-zero motor
            // percent alive as phantom stick input.
            if (frame.isMotorCmd()) {
                fLastMotorMs = now;
                fLastNeutral = frame.isNeutral();
                int pct = static_cast<int>(frame.data) * 100 / 127;
                fLastPct = static_cast<int8_t>(frame.cmd == 1 ? -pct : pct);
            }
            if (fOutEnabled && !frame.isMotorCmd()) {
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
        int mag = speedPct >= 0 ? speedPct : -static_cast<int>(speedPct);
        if (mag > 100)
            mag = 100; // data > 127 would set bit7 and corrupt packet framing
        frame.data = static_cast<uint8_t>((mag * 127) / 100);
        uint8_t buf[4];
        syrenEncode(frame, buf);
        fIo->write(buf, sizeof(buf));
    }

    // Last manual motor command as -100..100 (0 when neutral or stale).
    int8_t manualPercent(uint32_t now, uint32_t windowMs = 250) const {
        if (fLastMotorMs == 0 || (now - fLastMotorMs) >= windowMs)
            return 0;
        return fLastPct;
    }

    uint32_t checksumErrors() const { return fDecoder.checksumErrors(); }

  private:
    HardwareSerial* fIo = nullptr;
    SyrenDecoder fDecoder;
    uint8_t fAddrOut = 129;
    bool fInEnabled = true;
    bool fOutEnabled = true;
    uint32_t fLastMotorMs = 0; // last MOTOR frame (freshness for manualPercent)
    bool fLastNeutral = true;
    int8_t fLastPct = 0;
};

} // namespace rad

#endif // ARDUINO
