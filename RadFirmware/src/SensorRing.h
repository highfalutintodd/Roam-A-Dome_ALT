// Sensor ring frame parser + validation gauntlet. Pure C++ — host-testable.
// Implements BEHAVIOR.md §7 (deviation D7): the legacy pipeline accepted any
// parsed number verbatim (no range check, no staleness, linear median broken at
// the 0/359 seam, single glitches corrupting downstream state).
//
// Wire format in: ASCII lines "#DP@<deg>\r\n" from the sensor ring MCU.
// Output: a validated position + VALID/WARMUP/STALE state.
#pragma once

#include "CircularMath.h"
#include "Settings.h"

#include <cstdint>

namespace rad {

// Defaults mirror RadSettings{} (one source of truth: bare-constructed rings in
// host tests run the same numbers the device runs after applyTuning). Only the
// three fields with #DP commands live here; fixed validation constants are
// static members of SensorRing below.
struct SensorTuning {
    // Bench-measured: this dome does ~41 RPM (248 deg/s) at 100% Syren output, so
    // the gate must sit well above that or full-speed moves read as "jumps".
    uint8_t maxRpm = RadSettings{}.maxRpm;         // #DPMAXRPM: plausibility gate
    // The ring (DomeSensorFirmware32.ino:22,336) sends on change plus a heartbeat
    // resend every 1000 ms when parked — the timeout must clear that cadence with
    // margin or a stationary dome flaps between VALID and STALE.
    uint16_t staleMs = RadSettings{}.sensToMs;     // #DPSENSTO: no-frame timeout
    uint8_t confirmSamples = RadSettings{}.sensN;  // #DPSENSN: samples to accept a discontinuity
};

class SensorRing {
  public:
    enum class State : uint8_t { kWarmup, kValid, kStale };
    static constexpr uint8_t kMedianWindow = 5;

    // Fixed validation constants — deliberately NOT tunables. No #DP command
    // sets them, so keeping them in SensorTuning only implied knobs that could
    // not be turned (and every settings change silently reset them). Bench-derived.
    static constexpr uint8_t kSlackDeg = 2;   // fixed gate slack
    static constexpr uint8_t kJitterDeg = 3;  // pending-jump agreement window
    // Fail-open backstop: after this many consecutive gate rejections the median
    // is adopted (flagged as a jump) so tracking cannot freeze while the dome is
    // under drive — e.g. sustained motion faster than #DPMAXRPM, or a long
    // sticker-seam burst. NOTE: with the motor-plausibility guard armed and the
    // held drive at 0 the backstop stays blocked (a parked dome cannot move
    // itself, so a large persistent change is a sensor lie); genuine external
    // motion at zero drive is held out until the next driven move re-locks
    // tracking, and the controller's no-progress watchdog bounds any move that
    // is waiting on a stalled sample stream (it faults instead of hanging).
    static constexpr uint8_t kRejectStreakLimit = 10;
    // Parked-hold grace AND the drive-memory window: once no move has been
    // active for longer than this the parked-hold engages, and noteActive holds
    // the strongest commanded drive for this long so a dome coasting after the
    // throttle drops is judged against what was actually commanded during the
    // coast, not the instantaneous zero.
    static constexpr uint16_t kCoastMs = 400;
    // Motor-plausibility coast floor: extra degrees tolerated while the held
    // drive is nonzero (mount slop, inertia). Once the held drive has decayed to
    // zero — past the coast window — the dome cannot be moving, so the floor
    // drops away and only kSlackDeg of dither is accepted, matching the
    // parked-hold's threshold (no blind band between the two guards).
    static constexpr uint8_t kCoastDeg = 8;

    explicit SensorRing(const SensorTuning& tuning = SensorTuning{}) : fTuning(tuning) {}

    void setTuning(const SensorTuning& t) { fTuning = t; }

    // Report whether the dome is under active control this loop — i.e. a move is
    // in progress (target/spin) or the operator is driving it manually. Call once
    // per control loop. Only when it has been INACTIVE (a completed move / true
    // idle) longer than coastMs does the parked-hold in onFrame() engage and hold
    // position against encoder misreads.
    //
    // Crucially this is "a move is running", NOT merely "the motor is energised":
    // a target move sits at wire==0 while it settles inside the arrival arc, and
    // if the hold froze there it would pin the position at a value a flickering
    // encoder never re-reports, so the arrival dwell could never complete and the
    // move would hang forever in `target` (observed: K-ARDS stuck, orange square
    // latched). Staying active until the controller reaches idle lets arrival run.
    // (Only armed after the first active period: fLastActiveMs==0 keeps boot-time
    // and host-test behaviour on the plain gate.)
    // driveMagPct is the magnitude (0..100) of the motor output the glue is
    // commanding this loop — |wire|. Supplying it arms the motor-plausibility
    // guard; the default sentinel leaves it disarmed so callers that don't know
    // the drive (host tests, boot) keep the pure kinematic gate.
    static constexpr uint8_t kDriveUnknown = 255;
    void noteActive(bool active, uint32_t nowMs, uint8_t driveMagPct = kDriveUnknown) {
        fActive = active;
        if (active)
            fLastActiveMs = nowMs;
        if (driveMagPct != kDriveUnknown) {
            fDriveArmed = true;
            // Coast model: after the throttle drops, plausibility decays over a
            // coast tail whose length is EARNED by how long the drive was
            // actually on (capped at kCoastMs). A dome driven for seconds
            // carries real momentum — its post-release tail must stay
            // trackable; a momentary blip never spun the dome up, so it earns
            // (almost) no tail and misreads arriving right after it are still
            // held out. Instant-zero (the old behavior) rejected the genuine
            // tail; a flat unearned hold would believe the misreads.
            if (driveMagPct > 0) {
                if (fDriveNowPct == 0) { // rising edge: new drive episode
                    fDriveOnSinceMs = nowMs;
                    fDrivePeakPct = 0;
                    fDriveRiseMs = nowMs; // see adoptAllowance
                }
                if (driveMagPct > fDrivePeakPct)
                    fDrivePeakPct = driveMagPct;
            } else if (fDriveNowPct > 0) { // falling edge: begin the coast tail
                fDriveFellAt = nowMs;
                uint32_t on = nowMs - fDriveOnSinceMs;
                fDriveCreditMs = on < kCoastMs ? static_cast<uint16_t>(on) : kCoastMs;
            }
            fDriveNowPct = driveMagPct;
        }
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

    // Effective drive for plausibility (0..100): the current commanded
    // magnitude while driving; after release, the episode's peak decayed
    // linearly across the EARNED coast credit (see noteActive). Zero once the
    // dome has had time to stop.
    uint8_t driveMag(uint32_t nowMs) const {
        if (fDriveNowPct > 0)
            return fDriveNowPct;
        if (fDrivePeakPct == 0 || fDriveCreditMs == 0)
            return 0;
        uint32_t age = nowMs - fDriveFellAt;
        if (age >= fDriveCreditMs)
            return 0;
        return static_cast<uint8_t>(static_cast<uint32_t>(fDrivePeakPct) *
                                    (fDriveCreditMs - age) / fDriveCreditMs);
    }

    // Degrees the dome could plausibly have moved from the last accepted position
    // while the current pending discontinuity has persisted, given the commanded
    // drive. A confirmed/fail-open jump beyond this is a sensor lie, not motion.
    int32_t adoptAllowance(uint32_t nowMs) const {
        uint8_t mag = driveMag(nowMs);
        uint32_t adt = nowMs - fPendingSinceMs;
        // A pending window accrued while the motor was OFF must not become
        // instantly adoptable the moment drive returns: reach counts only from
        // when the drive actually rose (plus the coast tail), not from when the
        // reading first appeared.
        if (mag > 0 && fDriveRiseMs != 0) {
            uint32_t sinceRise = nowMs - fDriveRiseMs + kCoastMs;
            if (adt > sinceRise)
                adt = sinceRise;
        }
        if (adt > 10000)
            adt = 10000;
        int32_t reach = static_cast<int32_t>((maxDegPerMsQ10() * adt) >> 10);
        return reach * mag / 100 + kSlackDeg + (mag > 0 ? kCoastDeg : 0);
    }

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
                if (circularDistance(a, b) <= kJitterDeg &&
                    circularDistance(a, c) <= kJitterDeg) {
                    for (uint8_t i = fWarmCount; i < kMedianWindow; ++i)
                        fWindow[i] = a;
                    fWarmCount = kMedianWindow;
                    fWindowIdx = 0;
                    settled = true;
                }
            }
            if (fWarmCount == kMedianWindow || settled) {
                int16_t p = circularMedian(deg);
                // Recovery from stale (or re-warm) counts as a jump, not motion.
                bool rejoin = wasStale || fEverValid;
                // Route through accept() so stats().accepted advances: the
                // controller's freshness gate must see the recovery sample, or
                // a move issued right after warm-up stalls its dwell for an
                // extra heartbeat.
                accept(p, nowMs, /*jump=*/rejoin);
                if (!rejoin)
                    fLastDelta = 0; // first-ever fix: no prior position to delta from
                fState = State::kValid;
                fEverValid = true;
            }
            return;
        }

        fWindow[fWindowIdx] = deg;
        fWindowIdx = (fWindowIdx + 1) % kMedianWindow;
        int16_t med = circularMedian(fPosition);

        // Parked hold. Once no move has been active past the coast window the dome
        // is at rest and mechanically incapable of moving, so a reported change
        // beyond sensor dither is the encoder decoding a marginal code to a
        // stable-but-wrong angle — not a real move. Hold the last good position and
        // never adopt the lie (no confirm, no fail-open); tracking re-locks on the
        // next move, where genuine motion justifies the change. This is what stops
        // a parked dome from reading a wrong angle for seconds and corrupting the
        // start point of the following move. Gated on the controller being idle
        // (noteActive), NOT on wire==0, so a target move settling inside the arrival
        // arc is never frozen mid-arrival. Armed only after the first active period
        // (fLastActiveMs != 0), so boot-time acquisition and host tests are unaffected.
        if (!fActive && fLastActiveMs != 0 &&
            (nowMs - fLastActiveMs) > kCoastMs) {
            if (circularDistance(med, fPosition) <= kSlackDeg) {
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
        int32_t full = static_cast<int32_t>((maxDegPerMsQ10() * dt) >> 10);
        int32_t allowed = full + kSlackDeg;
        if (allowed > 180)
            allowed = 180;
        // Motor-plausibility cap. When the guard is armed, a reported change
        // can't exceed what the effective drive (current command, or the recent
        // peak decayed across kCoastMs — see driveMag) over dt could produce,
        // plus a kCoastDeg floor while that drive is nonzero. While the
        // controller holds station in the arrival arc the effective drive
        // decays to 0 within the coast window, so the cap collapses to
        // kSlackDeg — the ~299/304 alias and the ±35° coarse-arc wander (both
        // physically impossible with the motor off) are rejected before they
        // ever reach the controller, so it never gets kicked out of the arc
        // and never re-pulses the motor (the self-exciting hunt). At full
        // drive the cap exceeds the kinematic `allowed`, so fast and
        // over-limit moves are unchanged; a just-released jog's coasting tail
        // is still tracked because the launching drive decays rather than
        // vanishing.
        if (fDriveArmed) {
            uint8_t mag = driveMag(nowMs);
            int32_t plaus = (full * mag) / 100 + kSlackDeg + (mag > 0 ? kCoastDeg : 0);
            if (allowed > plaus)
                allowed = plaus;
        }

        int16_t dist = circularDistance(med, fPosition);
        if (dist <= allowed) {
            accept(med, nowMs, /*jump=*/false);
            fPendingCount = 0;
            fRejectStreak = 0;
        } else {
            // Discontinuity: require N consecutive samples agreeing on the new
            // position before believing it.
            ++fStats.rejectedRate;
            if (fRejectStreak < 255)
                ++fRejectStreak; // saturate: a long hold must not wrap the byte
            bool agreed =
                fPendingCount > 0 && circularDistance(med, fPendingValue) <= kJitterDeg;
            if (agreed) {
                if (fPendingCount < 255)
                    ++fPendingCount;
            } else {
                fPendingValue = med;
                fPendingCount = 1;
                fPendingSinceMs = nowMs;
            }
            // Adopting a discontinuity (confirmed or fail-open) means "the dome
            // really moved there." Only believe it if the *actual commanded drive*
            // over the time this reading has persisted could have carried the dome
            // that far — otherwise it is a stable sensor lie (the ~299/304 alias or
            // a stuck coarse code), which the motor cannot have produced no matter
            // how many times it repeats. This is what lets the guard reject the lie
            // even while the controller is driving (the alias's 100° jump is
            // impossible at any sane speed in 60 ms), so the dome is never yanked
            // off target. Real and over-limit motion stay plausible as time passes,
            // so genuine discontinuities are still adopted and never freeze.
            bool plausibleAdopt = !fDriveArmed || dist <= adoptAllowance(nowMs);
            // `agreed` keeps the legacy floor: even #DPSENSN1 needs a SECOND
            // sample agreeing with the first sighting — one glitch frame alone
            // is never believed.
            if (agreed && fPendingCount >= fTuning.confirmSamples && plausibleAdopt) {
                accept(med, nowMs, /*jump=*/true);
                fPendingCount = 0;
                fRejectStreak = 0;
                return;
            }
            if (fRejectStreak >= kRejectStreakLimit && plausibleAdopt) {
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
    uint32_t fPendingSinceMs = 0;  // when the current pending discontinuity began
    uint8_t fRejectStreak = 0;
    uint32_t fLastFrameMs = 0;
    uint32_t fLastAcceptMs = 0;
    bool fActive = false;         // a move/manual drive is active this loop (noteActive)
    uint32_t fLastActiveMs = 0;   // last time control was active; 0 = never yet
    bool fDriveArmed = false;     // glue has supplied real drive magnitudes
    uint8_t fDriveNowPct = 0;     // |wire| commanded this loop
    uint8_t fDrivePeakPct = 0;    // peak |wire| of the current/last drive episode
    uint32_t fDriveOnSinceMs = 0; // when the current drive episode began
    uint32_t fDriveFellAt = 0;    // when drive last fell to 0 (coast tail anchor)
    uint16_t fDriveCreditMs = 0;  // earned coast tail: min(episode length, kCoastMs)
    uint32_t fDriveRiseMs = 0;    // when drive last rose from 0 (0 = never)
    Stats fStats;
};

} // namespace rad
