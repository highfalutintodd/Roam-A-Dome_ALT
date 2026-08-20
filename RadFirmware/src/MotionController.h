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
#include "Settings.h"

#include <cstdint>

namespace rad {

// Defaults mirror RadSettings{} so a bare-constructed controller (host tests)
// runs the SAME numbers the device runs after CommandExec::applyTuning — one
// source of truth, no silent drift between bench and droid.
struct MotionTuning {
    uint8_t maxSpeed = RadSettings{}.maxSpeed;
    uint8_t minSpeed = RadSettings{}.minSpeed;
    uint8_t homeSpeed = RadSettings{}.homeSpeed;
    uint8_t autoSpeed = RadSettings{}.autoSpeed;
    uint8_t targetSpeed = RadSettings{}.targetSpeed; // default when a move gives no speed argument
    uint8_t fudge = RadSettings{}.fudge; // arrival tolerance (degrees) in a clean arc
    // Adaptive-deadband ceiling (#DPFUDGEMAX). When a min-speed correction
    // overshoots the tight ±fudge arc, the dome can swing back and forth around
    // the target and never land (the overshoot limit cycle / "flip out"). Only
    // after the dome has crossed the target TWICE (see tickTarget) — a genuine
    // oscillation, not a single settling overshoot — does the controller widen
    // its "close enough, stop driving" band and latch it, stopping the motor
    // inside the swing. The latched width is sized from the MEASURED swing (so
    // a small hunt gives up only a little precision), clamped to [fudge, this
    // ceiling]. Must exceed the overshoot amplitude at the configured minSpeed
    // (~13° at 15%) or the cycle survives the latch — raise it together with
    // #DPMINSPEED on heavy domes. Set == fudge to disable widening.
    uint8_t fudgeMax = RadSettings{}.fudgeMax;
    uint8_t dwell = RadSettings{}.dwell; // consecutive in-arc fresh samples = arrived
    bool scaling = RadSettings{}.scaling;   // ramp speed up instead of stepping
    uint8_t accScale = RadSettings{}.accScale; // ramp: ~accScale*20 ms from 0 to 100%
    uint8_t decScale = RadSettings{}.decScale; // deceleration zone (degrees from target)
    uint8_t timeoutSec = RadSettings{}.timeoutSec; // stuck-dome watchdog; 0 disables (#DPTIMEOUT0)
    int16_t homePos = RadSettings{}.homePos;   // sensor angle of "home"
    bool homeMode = false;     // idle behavior: return home (runtime-only, D12)
    bool autoMode = false;     // idle behavior: random seek (runtime-only, D12)
    uint8_t autoLeft = RadSettings{}.autoLeft;   // max auto excursion left of home (degrees)
    uint8_t autoRight = RadSettings{}.autoRight; // right of home
    uint16_t autoMinS = RadSettings{}.autoMinS, autoMaxS = RadSettings{}.autoMaxS;
    uint16_t homeMinS = RadSettings{}.homeMinS, homeMaxS = RadSettings{}.homeMaxS;
    // Settle delay after a targeted move arrives, before idle automation may
    // resume (#DPTARGETMIN/#DPTARGETMAX, seconds — legacy semantics).
    uint16_t targetMinS = RadSettings{}.targetMinS, targetMaxS = RadSettings{}.targetMaxS;
    uint16_t idleMs = RadSettings{}.idleMs; // manual-neutral time before automation resumes
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
        // A sequence is executing (including its wait steps): idle automation
        // must not schedule its own moves into the middle of it.
        bool suppressAutomation = false;
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
        fBestDist = 32767;
        fMaxDwell = 0;
        fArcWide = tuning.fudge; // reset adaptive deadband for the new move
        fArcSign = 0;
        fArcCross = 0;
        fSwingPeak = 0;
        fRelative = false;
        fHomeRestArc = 0;
    }

    // Degrees relative to home (":DPA90" semantics).
    void moveHomeRelative(int16_t deg, uint8_t speed, uint8_t maxSpeed, bool blocking) {
        moveToAbsolute(static_cast<int16_t>(tuning.homePos + deg), speed, maxSpeed, blocking);
    }

    // Delta from a position the caller just read (":DPD" semantics). Unlike an
    // absolute move, a confirmed tracker jump ABORTS it with kJump instead of
    // re-planning: the delta was measured from a start point now known to be
    // wrong, so completing the move would land somewhere never asked for
    // (BEHAVIOR.md §7: a jump aborts an in-progress relative move with an error).
    void moveRelative(int16_t fromPos, int16_t deltaDeg, uint8_t speed, uint8_t maxSpeed,
                      bool blocking) {
        moveToAbsolute(static_cast<int16_t>(fromPos + deltaDeg), speed, maxSpeed, blocking);
        fRelative = true;
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
        // #DPMAXSPEED caps ALL automated motion, spin included. A request below
        // minSpeed stops instead of holding the motor energised too weak to
        // actually turn the dome (legacy zeroed sub-minimum speeds).
        int16_t mag = pct < 0 ? static_cast<int16_t>(-static_cast<int16_t>(pct)) : pct;
        if (mag < tuning.minSpeed) {
            stop();
            return;
        }
        if (mag > tuning.maxSpeed)
            mag = tuning.maxSpeed;
        fSpinPct = pct < 0 ? static_cast<int8_t>(-mag) : static_cast<int8_t>(mag);
        fState = State::kSpin;
        fFault = Fault::kNone;
        fCurSpeed = 0;
    }

    void stop() {
        fState = State::kIdle;
        fSchedArmed = false;
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

        // Adaptive deadband, updated on each fresh sample. The arc stays tight
        // (fudge) until the dome has CROSSED the target twice — overshot it, come
        // back, and overshot again. One overshoot is normal and settles at full
        // precision; two is an overshoot limit cycle that will hunt forever, so
        // widen the arrival/deadband arc to fudgeMax and latch it, which stops the
        // motor inside the swing. Requiring two crossings (not one) is what keeps
        // ordinary moves accurate: only a genuinely oscillating move gives up
        // precision, and even then it has made real correction passes first.
        if (in.sampleCount != fLastSample && !in.jumped) {
            int16_t d = signedCircularDelta(fTarget, in.position); // pos - target
            int16_t ad = d < 0 ? static_cast<int16_t>(-d) : d;
            // Count a crossing only NEAR the target: signedCircularDelta also
            // flips sign at the antipode (±180), and a genuine overshoot swing
            // is small (~13° at min speed) — a distant sign flip is approach or
            // jitter, not hunting. Jump samples are excluded outright: an
            // adopted tracker correction teleports the believed position and is
            // not a physical swing.
            int8_t sgn = 0;
            if (ad <= 90)
                sgn = d > tuning.fudge ? 1 : d < -tuning.fudge ? -1 : 0;
            if (sgn != 0) {
                bool flipped = fArcSign != 0 && sgn != fArcSign;
                if (flipped)
                    ++fArcCross;
                if (fArcCross > 0 && ad > fSwingPeak)
                    fSwingPeak = ad; // amplitude of the oscillation, once it started
                if (flipped && fArcCross >= 2) {
                    // Latch: size the arc from the MEASURED swing (+ margin) so a
                    // small hunt gives up only a little precision, clamped to
                    // [current arc, fudgeMax]. Never narrows — a fudge configured
                    // above fudgeMax must not invert widening into tightening.
                    int16_t w = static_cast<int16_t>(fSwingPeak + 2);
                    if (w > tuning.fudgeMax)
                        w = tuning.fudgeMax;
                    if (w > fArcWide)
                        fArcWide = w;
                }
                fArcSign = sgn;
            }
        }
        int16_t arc = fArcWide;

        if (in.jumped && fRelative) {
            // The relative delta was measured from a position the tracker has
            // just disowned — abort with an error rather than silently complete
            // a move to a target derived from a lie (BEHAVIOR.md §7).
            fFault = Fault::kJump;
            stop();
            return 0;
        }
        if (in.jumped && adist > arc) {
            // The tracker corrected itself (sticker-seam burst, over-limit motion,
            // recovery) and the corrected position is OUTSIDE the arrival arc, so
            // re-plan from it: fresh dwell + fresh watchdog window. But a jump that
            // lands us already inside the arc must NOT wipe an almost-complete
            // dwell — otherwise a glitch-prone arc (motor noise near the target)
            // keeps resetting arrival and the hold hunts forever instead of
            // latching idle. Inside the arc, fall through and let the dwell run.
            fDwellCount = 0;
            fBestDist = 32767;
            fMaxDwell = 0;
            fProgressAt = in.now;
            fArcSign = 0;  // restart the oscillation detector for the re-plan —
            fArcCross = 0; // pre-jump crossings came from a different geometry
            fArcWide = tuning.fudge;
            fSwingPeak = 0;
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
            // Watchdog "progress" = getting closer than ever before (approach)
            // or a new high-water dwell count (arrival progressing). Mere
            // movement is NOT progress: a wrong-polarity runaway or a sensor
            // flicker hunt moves plenty while getting nowhere, and must time
            // out rather than drive forever.
            bool progressed = adist < fBestDist;
            if (progressed)
                fBestDist = adist;
            if (fDwellCount > fMaxDwell) {
                fMaxDwell = fDwellCount;
                progressed = true;
            }
            if (progressed)
                fProgressAt = in.now;
        }

        if (fDwellCount >= tuning.dwell) {
            fArrived = true;
            // A home arrival may legitimately rest anywhere inside the latched
            // arc; remember that width so homeMode's "am I home?" test doesn't
            // measure the rest point against the tight fudge and re-seek forever
            // (the seek would just hunt, latch, and rest off-home again).
            fHomeRestArc = fTarget == normalizeDeg(tuning.homePos) ? arc : 0;
            // Settle delay before idle automation may resume (#DPTARGETMIN/MAX).
            fSettleUntil = in.now + fRng(tuning.targetMinS, tuning.targetMaxS) * 1000u;
            fSettleArmed = true;
            stop();
            return 0;
        }
        // No-progress watchdog. Checked BEFORE the in-arc return so a move
        // whose accepted-sample stream stalls (e.g. the plausibility guard
        // holding out a discontinuity at zero drive) faults instead of hanging
        // in kTarget forever with busy() latched. timeoutSec 0 disables it
        // (#DPTIMEOUT0), as documented.
        if (fProgressAt == 0)
            fProgressAt = in.now;
        if (tuning.timeoutSec != 0 &&
            in.now - fProgressAt > static_cast<uint32_t>(tuning.timeoutSec) * 1000u) {
            fFault = Fault::kTimeout; // stuck, hunting, or tracking stalled: stop pushing
            stop();
            return 0;
        }
        if (adist <= arc)
            return 0; // inside the (adaptive) arc, waiting out the dwell —
                      // crucially NOT driving, so the sensor guard can hold station

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
        if (in.suppressAutomation || !in.sensorValid ||
            in.now - fLastManual < tuning.idleMs) {
            fSchedArmed = false;
            return;
        }
        // Post-arrival settle window (#DPTARGETMIN/MAX): let the dome rest
        // before automation starts planning again.
        if (fSettleArmed) {
            if (static_cast<int32_t>(in.now - fSettleUntil) < 0) {
                fSchedArmed = false;
                return;
            }
            fSettleArmed = false;
        }
        bool wantAuto = tuning.autoMode;
        // "Am I home?" is measured against the arc the last home arrival was
        // GRANTED (a hunted seek legitimately rests up to the latched arc away),
        // never tighter than fudge — otherwise a rest point at 6-18° re-seeks
        // every few seconds forever.
        int16_t homeTol = fHomeRestArc > tuning.fudge ? fHomeRestArc
                                                      : static_cast<int16_t>(tuning.fudge);
        bool wantHome = tuning.homeMode &&
                        circularDistance(in.position, tuning.homePos) > homeTol;
        if (!wantAuto && !wantHome) {
            fSchedArmed = false;
            return;
        }
        if (!fSchedArmed) {
            uint16_t mn = wantAuto ? tuning.autoMinS : tuning.homeMinS;
            uint16_t mx = wantAuto ? tuning.autoMaxS : tuning.homeMaxS;
            fSchedAt = in.now + fRng(mn, mx) * 1000u;
            fSchedArmed = true;
            return;
        }
        if (static_cast<int32_t>(in.now - fSchedAt) < 0)
            return;
        fSchedArmed = false;
        if (wantAuto)
            moveRandom(tuning.autoSpeed, true);
        else
            seekHome(tuning.homeSpeed);
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
    int16_t fBestDist = 32767; // smallest |dist| yet this move (watchdog watermark)
    uint8_t fMaxDwell = 0;     // high-water dwell count this move (watchdog watermark)
    uint32_t fProgressAt = 0;
    uint32_t fLastManual = 0;
    uint32_t fSchedAt = 0;     // meaningful only while fSchedArmed
    bool fSchedArmed = false;  // an idle-automation move is scheduled at fSchedAt
    uint32_t fSettleUntil = 0; // post-arrival settle window (meaningful when armed)
    bool fSettleArmed = false;
    uint32_t fLastTick = 0;
    int16_t fArcWide = 0;   // latched adaptive arc; set per-move by moveToAbsolute
    int8_t fArcSign = 0;    // last side of target the dome was on (+1/-1/0)
    uint8_t fArcCross = 0;  // times it has crossed the target this move
    int16_t fSwingPeak = 0; // largest |dist| seen since the oscillation began
    bool fRelative = false; // current move is a ":DPD" relative move (jump aborts)
    int16_t fHomeRestArc = 0; // arc granted to the last HOME arrival (0 = n/a)
};

} // namespace rad
