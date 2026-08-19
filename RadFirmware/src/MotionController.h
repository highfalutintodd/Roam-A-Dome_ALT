// Dome motion control. Pure C++ — host-testable.
//
// Owns the automation side of the arbitration ladder (BEHAVIOR.md §5):
//   e-stop > manual input > idle detection > automation.
// The caller (firmware glue) feeds Inputs each loop and routes the returned
// speed (-100..100, positive = increasing degrees) to the enabled motor
// outputs; motor inversion is applied by the caller at the output stage.
//
// Safety by construction: automation only ever drives on VALID sensor data;
// staleness, confirmed position jumps, and a no-progress watchdog all cut the
// move; arrival requires `dwell` consecutive fresh in-arc samples.
#pragma once

#include "CircularMath.h"

#include <cstdint>

namespace rad {

struct MotionTuning {
    uint8_t maxSpeed = 100;
    uint8_t minSpeed = 15;
    uint8_t homeSpeed = 40;
    uint8_t autoSpeed = 30;
    uint8_t targetSpeed = 100; // default when a move gives no speed argument
    uint8_t fudge = 5;         // arrival tolerance (degrees) in a clean arc
    // Adaptive-deadband ceiling. In a coarse-encoder arc the reported position
    // wanders many degrees while the dome is really on target; chasing that wander
    // pulses the motor and self-excites the flip-out. When the controller sees this
    // (wide sample spread but no *net* progress) it widens the "close enough, stop
    // driving" band toward this ceiling so it stops and lets the move settle. Only
    // engages on measured noise, so clean arcs keep the tight `fudge`. Set == fudge
    // to disable. The cost is honest: arrival precision in a noisy arc is only as
    // good as the encoder can resolve there.
    uint8_t fudgeMax = 22;
    uint8_t dwell = 3;         // consecutive in-arc fresh samples = arrived
    bool scaling = false;      // ramp speed up instead of stepping
    uint8_t accScale = 20;     // ramp: ~accScale*20 ms from 0 to 100%
    uint8_t decScale = 50;     // deceleration zone (degrees from target)
    uint8_t timeoutSec = 5;    // stuck-dome watchdog
    int16_t homePos = 240;     // sensor angle of "home"
    bool homeMode = false;     // idle behavior: return home
    bool autoMode = false;     // idle behavior: random seek
    uint8_t autoLeft = 47;     // max auto excursion left of home (degrees)
    uint8_t autoRight = 46;    // right of home
    uint16_t autoMinS = 6, autoMaxS = 8;
    uint16_t homeMinS = 6, homeMaxS = 8;
    uint16_t idleMs = 3000;    // manual-neutral time before automation resumes
};

class MotionController {
  public:
    using RandomFn = uint32_t (*)(uint32_t lo, uint32_t hi); // inclusive

    enum class State : uint8_t { kIdle, kTarget, kSpin };
    enum class Fault : uint8_t { kNone, kTimeout, kSensorLost, kJump };

    explicit MotionController(RandomFn rng) : fRng(rng) {}

    MotionTuning tuning;

    struct Inputs {
        uint32_t now = 0;
        bool sensorValid = false;
        int16_t position = 0;     // last accepted sensor angle
        uint32_t sampleCount = 0; // SensorRing stats().accepted — freshness counter
        bool jumped = false;      // SensorRing consumeJump()
        bool manualActive = false;
        bool estop = false;
    };

    // ---- commands -----------------------------------------------------------

    // Absolute sensor angle. speed 0 = use default; maxSpeed 0 = same as speed.
    void moveToAbsolute(int16_t absDeg, uint8_t speed, uint8_t maxSpeed, bool blocking) {
        fTarget = normalizeDeg(absDeg);
        fSpeedReq = speed != 0 ? speed : tuning.targetSpeed;
        fSpeedMax = maxSpeed != 0 ? maxSpeed : fSpeedReq;
        fBlocking = blocking;
        fState = State::kTarget;
        fFault = Fault::kNone;
        fArrived = false;
        fDwellCount = 0;
        fCurSpeed = 0;
        fProgressAt = 0;
        fProgressPos = -1;
        fNoiseCount = 0;
    }

    // Degrees relative to home (":DPA90" semantics).
    void moveHomeRelative(int16_t deg, uint8_t speed, uint8_t maxSpeed, bool blocking) {
        moveToAbsolute(static_cast<int16_t>(tuning.homePos + deg), speed, maxSpeed, blocking);
    }

    void moveRandom(uint8_t speed, bool blocking) {
        int32_t span = tuning.autoLeft + tuning.autoRight;
        int32_t off = span > 0 ? static_cast<int32_t>(fRng(0, span)) - tuning.autoLeft : 0;
        moveHomeRelative(static_cast<int16_t>(off), speed != 0 ? speed : tuning.autoSpeed, 0,
                         blocking);
    }

    void seekHome(uint8_t speed) {
        moveToAbsolute(tuning.homePos, speed != 0 ? speed : tuning.homeSpeed, 0, true);
    }

    void spin(int8_t pct) {
        if (pct == 0) {
            stop();
            return;
        }
        fSpinPct = pct;
        fState = State::kSpin;
        fFault = Fault::kNone;
        fCurSpeed = 0;
    }

    void stop() {
        fState = State::kIdle;
        fSchedAt = 0;
        fCurSpeed = 0;
    }

    // ---- status -------------------------------------------------------------

    State state() const { return fState; }
    Fault fault() const { return fFault; }
    void clearFault() { fFault = Fault::kNone; }
    static const char* faultName(Fault f) {
        switch (f) {
        case Fault::kTimeout: return "TIMEOUT (dome stuck?)";
        case Fault::kSensorLost: return "SENSOR LOST";
        case Fault::kJump: return "POSITION JUMP";
        default: return "NONE";
        }
    }
    bool arrived() const { return fArrived; }
    int16_t target() const { return fTarget; }
    // True while a blocking move is in flight — the sequencer must not advance.
    bool busy() const { return fState == State::kTarget && fBlocking; }

    // ---- per-loop tick ------------------------------------------------------

    // Returns commanded speed -100..100 (0 = neutral). Positive = toward
    // increasing sensor degrees; the output stage applies #DPINVERT.
    int8_t tick(const Inputs& in) {
        uint32_t dt = fLastTick == 0 ? 0 : in.now - fLastTick;
        fLastTick = in.now;

        if (in.estop) {
            stop();
            fLastManual = in.now;
            return 0;
        }
        if (in.manualActive) {
            if (fState != State::kIdle)
                stop(); // manual wins the same tick, automation move cancelled
            fLastManual = in.now;
            return 0;
        }

        switch (fState) {
        case State::kSpin:
            return ramp(fSpinPct, dt);
        case State::kTarget:
            return tickTarget(in, dt);
        case State::kIdle:
            tickIdleAutomation(in);
            return 0;
        }
        return 0;
    }

  private:
    int8_t tickTarget(const Inputs& in, uint32_t dt) {
        if (!in.sensorValid) {
            fFault = Fault::kSensorLost;
            stop();
            return 0;
        }
        int16_t dist = signedCircularDelta(in.position, fTarget);
        int16_t adist = dist < 0 ? -dist : dist;

        // Record fresh positions and derive the adaptive arrival/deadband tolerance
        // for this tick before anything uses it.
        if (in.sampleCount != fLastSample) {
            fNoiseBuf[fNoiseIdx] = in.position;
            fNoiseIdx = (fNoiseIdx + 1) % kNoiseWindow;
            if (fNoiseCount < kNoiseWindow)
                ++fNoiseCount;
        }
        int16_t arc = arcFudge();

        if (in.jumped && adist > arc) {
            // The tracker corrected itself (sticker-seam burst, over-limit motion,
            // recovery) and the corrected position is OUTSIDE the arrival arc, so
            // re-plan from it: fresh dwell + fresh watchdog window. But a jump that
            // lands us already inside the arc must NOT wipe an almost-complete
            // dwell — otherwise a glitch-prone arc (motor noise near the target)
            // keeps resetting arrival and the hold hunts forever instead of
            // latching idle. Inside the arc, fall through and let the dwell run.
            fDwellCount = 0;
            fProgressPos = -1;
            fProgressAt = in.now;
        }

        // Dwell + watchdog progress advance only on fresh accepted samples.
        if (in.sampleCount != fLastSample) {
            fLastSample = in.sampleCount;
            if (adist <= arc) {
                ++fDwellCount;
            } else if (fDwellCount > 0) {
                // Leaky dwell: a lone out-of-arc sample costs one tick, not the
                // whole count. The sensor guard now holds position steady while the
                // dome is parked in the arc, but if a rare outlier still slips
                // through it must not wipe an almost-complete arrival and restart
                // the hunt — a few good samples still outvote the odd stray one.
                --fDwellCount;
            }
            if (fProgressPos < 0 || circularDistance(in.position, fProgressPos) >= 1) {
                fProgressPos = in.position;
                fProgressAt = in.now;
            }
        }

        if (fDwellCount >= tuning.dwell) {
            fArrived = true;
            stop();
            return 0;
        }
        if (adist <= arc)
            return 0; // inside the (adaptive) arc, waiting out the dwell —
                      // crucially NOT driving, so the sensor guard can hold station

        if (fProgressAt == 0)
            fProgressAt = in.now;
        if (in.now - fProgressAt > static_cast<uint32_t>(tuning.timeoutSec) * 1000u) {
            fFault = Fault::kTimeout; // dome physically stuck: stop pushing
            stop();
            return 0;
        }

        uint8_t base = clampSpeed(fSpeedReq);
        uint8_t cap = clampSpeed(fSpeedMax);
        uint8_t desired = cap;
        if (tuning.decScale != 0 && adist < tuning.decScale) {
            uint32_t scaled = static_cast<uint32_t>(cap) * adist / tuning.decScale;
            desired = scaled > tuning.minSpeed ? static_cast<uint8_t>(scaled) : tuning.minSpeed;
        }
        if (fCurSpeed < base)
            fCurSpeed = base;
        int8_t mag = ramp(static_cast<int8_t>(desired), dt);
        return dist > 0 ? mag : static_cast<int8_t>(-mag);
    }

    void tickIdleAutomation(const Inputs& in) {
        if (!in.sensorValid || in.now - fLastManual < tuning.idleMs) {
            fSchedAt = 0;
            return;
        }
        bool wantAuto = tuning.autoMode;
        bool wantHome = tuning.homeMode &&
                        circularDistance(in.position, tuning.homePos) > tuning.fudge;
        if (!wantAuto && !wantHome) {
            fSchedAt = 0;
            return;
        }
        if (fSchedAt == 0) {
            uint16_t mn = wantAuto ? tuning.autoMinS : tuning.homeMinS;
            uint16_t mx = wantAuto ? tuning.autoMaxS : tuning.homeMaxS;
            fSchedAt = in.now + fRng(mn, mx) * 1000u;
            if (fSchedAt == 0)
                fSchedAt = 1; // 0 is the "unscheduled" sentinel
            return;
        }
        if (static_cast<int32_t>(in.now - fSchedAt) < 0)
            return;
        fSchedAt = 0;
        if (wantAuto)
            moveRandom(tuning.autoSpeed, true);
        else
            seekHome(tuning.homeSpeed);
    }

    // Adaptive arrival/deadband tolerance. Baseline is the tight `fudge`; it widens
    // toward `fudgeMax` only when recent fresh samples show a wide spread WITHOUT
    // net progress — i.e. the reported position is bouncing around one spot, the
    // coarse-arc noise signature, not travelling. Real motion (large net progress)
    // keeps the spread "explained" and stays tight, so clean-arc precision is
    // untouched. Widening lets the controller stop driving inside the noisy band,
    // which ends the motor-pulse self-excitation and lets the move settle.
    int16_t arcFudge() const {
        if (fNoiseCount < 4)
            return tuning.fudge;
        int16_t newest = fNoiseBuf[(fNoiseIdx + kNoiseWindow - 1) % kNoiseWindow];
        int16_t oldest = fNoiseBuf[(fNoiseIdx + kNoiseWindow - fNoiseCount) % kNoiseWindow];
        int16_t mn = 0, mx = 0; // deltas measured relative to the newest sample
        for (uint8_t k = 0; k < fNoiseCount; ++k) {
            int16_t d = signedCircularDelta(newest, fNoiseBuf[k]);
            if (d < mn)
                mn = d;
            if (d > mx)
                mx = d;
        }
        int16_t spread = mx - mn;
        int16_t net = circularDistance(oldest, newest);
        if (net > tuning.fudge || spread <= tuning.fudge)
            return tuning.fudge; // travelling, or genuinely quiet: stay tight
        int16_t widened = tuning.fudge + spread / 2;
        return widened > tuning.fudgeMax ? tuning.fudgeMax : widened;
    }

    uint8_t clampSpeed(uint8_t s) const {
        if (s < tuning.minSpeed)
            return tuning.minSpeed;
        if (s > tuning.maxSpeed)
            return tuning.maxSpeed;
        return s;
    }

    // Rate-limit magnitude changes when scaling is enabled (sign preserved).
    int8_t ramp(int8_t desired, uint32_t dt) {
        int16_t want = desired < 0 ? -desired : desired;
        if (!tuning.scaling || tuning.accScale == 0) {
            fCurSpeed = static_cast<uint8_t>(want);
        } else {
            uint32_t step = dt * 100u / (static_cast<uint32_t>(tuning.accScale) * 20u);
            if (step == 0)
                step = 1;
            if (fCurSpeed + step < static_cast<uint32_t>(want))
                fCurSpeed = static_cast<uint8_t>(fCurSpeed + step);
            else
                fCurSpeed = static_cast<uint8_t>(want);
        }
        return desired < 0 ? static_cast<int8_t>(-fCurSpeed) : static_cast<int8_t>(fCurSpeed);
    }

    RandomFn fRng;
    State fState = State::kIdle;
    Fault fFault = Fault::kNone;
    int16_t fTarget = 0;
    uint8_t fSpeedReq = 0;
    uint8_t fSpeedMax = 0;
    bool fBlocking = true;
    bool fArrived = false;
    int8_t fSpinPct = 0;
    uint8_t fCurSpeed = 0;
    uint8_t fDwellCount = 0;
    uint32_t fLastSample = 0;
    int16_t fProgressPos = -1;
    uint32_t fProgressAt = 0;
    uint32_t fLastManual = 0;
    uint32_t fSchedAt = 0;
    uint32_t fLastTick = 0;
    static constexpr uint8_t kNoiseWindow = 8;
    int16_t fNoiseBuf[kNoiseWindow] = {}; // recent fresh positions (adaptive fudge)
    uint8_t fNoiseIdx = 0;
    uint8_t fNoiseCount = 0;
};

} // namespace rad
