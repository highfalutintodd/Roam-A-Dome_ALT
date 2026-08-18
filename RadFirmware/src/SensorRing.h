// Sensor ring frame parser + validation gauntlet. Pure C++ — host-testable.
// Implements BEHAVIOR.md §7 (deviation D7): the legacy pipeline accepted any
// parsed number verbatim (no range check, no staleness, linear median broken at
// the 0/359 seam, single glitches corrupting downstream state).
//
// Wire format in: ASCII lines "#DP@<deg>\r\n" from the sensor ring MCU.
// Output: a validated position + VALID/WARMUP/STALE state.
#pragma once

#include "CircularMath.h"

#include <cstdint>

namespace rad {

struct SensorTuning {
    // Bench-measured: this dome does ~41 RPM (248 deg/s) at 100% Syren output, so
    // the gate must sit well above that or full-speed moves read as "jumps".
    uint8_t maxRpm = 60;          // #DPMAXRPM: plausibility gate
    // The ring (DomeSensorFirmware32.ino:22,336) sends on change plus a heartbeat
    // resend every 1000 ms when parked — the timeout must clear that cadence with
    // margin or a stationary dome flaps between VALID and STALE.
    uint16_t staleMs = 2500;      // #DPSENSTO: no-frame timeout
    uint8_t confirmSamples = 3;   // #DPSENSN: samples to accept a discontinuity
    uint8_t slackDeg = 2;         // fixed gate slack
    uint8_t jitterDeg = 3;        // pending-jump agreement window
    // Fail-open backstop: after this many consecutive gate rejections the median
    // is adopted (flagged as a jump) so tracking can never freeze indefinitely —
    // e.g. sustained motion faster than #DPMAXRPM, or a long sticker-seam burst.
    uint8_t rejectStreakLimit = 10;
    // Parked-hold grace: once the motor has been commanded off for longer than
    // this, the dome is mechanically incapable of moving, so onFrame() holds the
    // last good position and refuses phantom jumps (see noteDrive/onFrame). The
    // window lets a just-finished move coast to rest before the hold engages.
    uint16_t coastMs = 400;
};

class SensorRing {
  public:
    enum class State : uint8_t { kWarmup, kValid, kStale };
    static constexpr uint8_t kMedianWindow = 5;

    explicit SensorRing(const SensorTuning& tuning = SensorTuning{}) : fTuning(tuning) {}

    void setTuning(const SensorTuning& t) { fTuning = t; }

    // Report whether the motor is currently commanded to move. Call once per
    // control loop. A dome with the motor off cannot change position, so once it
    // has been off longer than coastMs the parked-hold in onFrame() rejects any
    // reported jump as an encoder misread rather than believing the dome moved.
    // (Only armed after the first real drive: fLastDriveMs==0 keeps boot-time and
    // host-test behaviour on the plain gate.)
    void noteDrive(bool active, uint32_t nowMs) {
        fDriveActive = active;
        if (active)
            fLastDriveMs = nowMs;
    }

    // Feed one raw byte from the sensor serial stream; call tick() regularly too.
    void feed(uint8_t ch, uint32_t nowMs) {
        if (ch == '\r' || ch == '\n') {
            if (fParseState == ParseState::kDigits && fDigits >= 1 && fValue <= 359)
                onFrame(static_cast<int16_t>(fValue), nowMs);
            else if (fParseState != ParseState::kIdle && fParseState != ParseState::kSkip)
                ++fStats.rejectedParse; // zero digits or partial header at EOL
            fParseState = ParseState::kIdle;
            return;
        }
        switch (fParseState) {
        case ParseState::kIdle:
            fParseState = (ch == '#') ? ParseState::kHdrD : ParseState::kSkip;
            if (fParseState == ParseState::kSkip)
                ++fStats.rejectedParse;
            break;
        case ParseState::kHdrD:
            fParseState = (ch == 'D') ? ParseState::kHdrP : reject();
            break;
        case ParseState::kHdrP:
            fParseState = (ch == 'P') ? ParseState::kHdrAt : reject();
            break;
        case ParseState::kHdrAt:
            if (ch == '@') {
                fParseState = ParseState::kDigits;
                fValue = 0;
                fDigits = 0;
            } else {
                fParseState = reject();
            }
            break;
        case ParseState::kDigits:
            if (ch >= '0' && ch <= '9' && fDigits < 4) {
                fValue = fValue * 10 + (ch - '0');
                ++fDigits;
            } else {
                fParseState = reject(); // >4 digits or junk: whole frame dropped
            }
            break;
        case ParseState::kSkip:
            break; // discard until EOL
        }
    }

    // Time-based state maintenance; call every loop.
    void tick(uint32_t nowMs) {
        if (fState != State::kStale && fLastFrameMs != 0 &&
            (nowMs - fLastFrameMs) > fTuning.staleMs) {
            fState = State::kStale;
            fWarmCount = 0; // recovery restarts warm-up from scratch
            fPendingCount = 0;
            ++fStats.staleEvents;
        }
    }

    State state() const { return fState; }
    bool valid() const { return fState == State::kValid; }

    // Last accepted position (only meaningful when valid()).
    int16_t position() const { return fPosition; }

    // True once per accepted discontinuity ("the dome really did jump" — e.g. it was
    // moved while readings were stale). Consumers (relative-move tracking, watchdog)
    // must treat it as a reset, not a delta. Reading clears the flag.
    bool consumeJump() {
        bool j = fJumped;
        fJumped = false;
        return j;
    }

    // Signed validated movement since the previous accepted sample; consumers
    // accumulate this instead of raw positions. 0 across a jump.
    int16_t lastDelta() const { return fLastDelta; }

    struct Stats {
        uint32_t accepted = 0;
        uint32_t rejectedParse = 0;  // malformed frames (junk, 0 digits, >4 digits, >359)
        uint32_t rejectedRate = 0;   // plausibility-gate rejections (glitches)
        uint32_t jumps = 0;          // confirmed discontinuities
        uint32_t staleEvents = 0;
    };
    const Stats& stats() const { return fStats; }

  private:
    enum class ParseState : uint8_t { kIdle, kHdrD, kHdrP, kHdrAt, kDigits, kSkip };

    ParseState reject() {
        ++fStats.rejectedParse;
        return ParseState::kSkip;
    }

    // deg/ms * 1024 to stay in integer math: rpm * 360 / 60000 * 1024
    uint32_t maxDegPerMsQ10() const { return (static_cast<uint32_t>(fTuning.maxRpm) * 360 * 1024) / 60000; }

    void onFrame(int16_t deg, uint32_t nowMs) {
        bool wasStale = (fState == State::kStale);
        fLastFrameMs = nowMs;

        // Warm-up: fill the median window before reporting anything (no legacy
        // first-6-samples bypass). Fast path for a parked dome: the ring only
        // heartbeats at 1 Hz when stationary, so waiting for 5 frames would take
        // ~5 s — 3 agreeing frames are proof enough and seed the whole window.
        if (fWarmCount < kMedianWindow) {
            fWindow[fWarmCount++] = deg;
            fWindowIdx = fWarmCount % kMedianWindow;
            bool settled = false;
            if (fWarmCount >= 3 && fWarmCount < kMedianWindow) {
                int16_t a = fWindow[fWarmCount - 1];
                int16_t b = fWindow[fWarmCount - 2];
                int16_t c = fWindow[fWarmCount - 3];
                if (circularDistance(a, b) <= fTuning.jitterDeg &&
                    circularDistance(a, c) <= fTuning.jitterDeg) {
                    for (uint8_t i = fWarmCount; i < kMedianWindow; ++i)
                        fWindow[i] = a;
                    fWarmCount = kMedianWindow;
                    fWindowIdx = 0;
                    settled = true;
                }
            }
            if (fWarmCount == kMedianWindow || settled) {
                fPosition = circularMedian(deg);
                fLastAcceptMs = nowMs;
                fLastDelta = 0;
                if (wasStale || fEverValid) {
                    // Recovery from stale (or re-warm) counts as a jump, not motion.
                    fJumped = true;
                    ++fStats.jumps;
                }
                fState = State::kValid;
                fEverValid = true;
            }
            return;
        }

        fWindow[fWindowIdx] = deg;
        fWindowIdx = (fWindowIdx + 1) % kMedianWindow;
        int16_t med = circularMedian(fPosition);

        // Parked hold. With the motor commanded off past the coast window the dome
        // is mechanically incapable of moving, so a reported change beyond sensor
        // dither is the encoder decoding a marginal code to a stable-but-wrong
        // angle — not a real move. Hold the last good position and never adopt the
        // lie (no confirm, no fail-open); tracking re-locks on the next commanded
        // move, where genuine motion justifies the change. This is what stops a
        // parked dome from reading a wrong angle for seconds and corrupting the
        // start point of the following move. Armed only after the first real drive
        // (fLastDriveMs != 0), so boot-time acquisition and host tests are
        // unaffected.
        if (!fDriveActive && fLastDriveMs != 0 &&
            (nowMs - fLastDriveMs) > fTuning.coastMs) {
            if (circularDistance(med, fPosition) <= fTuning.slackDeg) {
                accept(med, nowMs, /*jump=*/false); // genuine dither: stay in sync
            } else {
                ++fStats.rejectedRate; // phantom move: count it, hold position
            }
            fPendingCount = 0;
            fRejectStreak = 0;
            return;
        }

        uint32_t dt = nowMs - fLastAcceptMs;
        if (dt > 10000)
            dt = 10000;
        int32_t allowed = static_cast<int32_t>((maxDegPerMsQ10() * dt) >> 10) + fTuning.slackDeg;
        if (allowed > 180)
            allowed = 180;

        int16_t dist = circularDistance(med, fPosition);
        if (dist <= allowed) {
            accept(med, nowMs, /*jump=*/false);
            fPendingCount = 0;
            fRejectStreak = 0;
        } else {
            // Discontinuity: require N consecutive samples agreeing on the new
            // position before believing it.
            ++fStats.rejectedRate;
            ++fRejectStreak;
            if (fPendingCount > 0 && circularDistance(med, fPendingValue) <= fTuning.jitterDeg) {
                if (++fPendingCount >= fTuning.confirmSamples) {
                    accept(med, nowMs, /*jump=*/true);
                    fPendingCount = 0;
                    fRejectStreak = 0;
                    return;
                }
            } else {
                fPendingValue = med;
                fPendingCount = 1;
            }
            if (fRejectStreak >= fTuning.rejectStreakLimit) {
                // Fail-open: adopt reality rather than freeze (see tuning note).
                accept(med, nowMs, /*jump=*/true);
                fPendingCount = 0;
                fRejectStreak = 0;
            }
        }
    }

    void accept(int16_t deg, uint32_t nowMs, bool jump) {
        fLastDelta = jump ? 0 : signedCircularDelta(fPosition, deg);
        if (jump) {
            fJumped = true;
            ++fStats.jumps;
        }
        fPosition = deg;
        fLastAcceptMs = nowMs;
        ++fStats.accepted;
    }

    // Median of the window, unwrapped to within ±180 of `ref` so the 0/359 seam
    // sorts correctly (legacy linear median gave ~8° errors crossing home).
    int16_t circularMedian(int16_t ref) const {
        int16_t unwrapped[kMedianWindow];
        for (uint8_t i = 0; i < kMedianWindow; ++i)
            unwrapped[i] = static_cast<int16_t>(ref + signedCircularDelta(ref, fWindow[i]));
        // insertion sort (N=5)
        for (uint8_t i = 1; i < kMedianWindow; ++i) {
            int16_t v = unwrapped[i];
            int8_t j = i - 1;
            for (; j >= 0 && unwrapped[j] > v; --j)
                unwrapped[j + 1] = unwrapped[j];
            unwrapped[j + 1] = v;
        }
        return normalizeDeg(unwrapped[kMedianWindow / 2]);
    }

    SensorTuning fTuning;
    ParseState fParseState = ParseState::kIdle;
    uint16_t fValue = 0;
    uint8_t fDigits = 0;

    State fState = State::kWarmup;
    bool fEverValid = false;
    int16_t fWindow[kMedianWindow] = {};
    uint8_t fWindowIdx = 0;
    uint8_t fWarmCount = 0;
    int16_t fPosition = 0;
    int16_t fLastDelta = 0;
    bool fJumped = false;
    int16_t fPendingValue = 0;
    uint8_t fPendingCount = 0;
    uint8_t fRejectStreak = 0;
    uint32_t fLastFrameMs = 0;
    uint32_t fLastAcceptMs = 0;
    bool fDriveActive = false;    // motor commanded to move this loop (noteDrive)
    uint32_t fLastDriveMs = 0;    // last time the motor was driving; 0 = never yet
    Stats fStats;
};

} // namespace rad
