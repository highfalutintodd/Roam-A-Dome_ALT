#include "rad_test.h"

#include "../../src/MotionController.h"
#include "../../src/SensorRing.h"

using namespace rad;

namespace {

using radtest::feedFrame;
constexpr MotionController::RandomFn midRng2 = &radtest::midRng;

MotionController::Inputs in(uint32_t now, int16_t pos, uint32_t samples,
                            bool valid = true, bool manual = false, bool jumped = false) {
    MotionController::Inputs i;
    i.now = now;
    i.sensorValid = valid;
    i.position = pos;
    i.sampleCount = samples;
    i.jumped = jumped;
    i.manualActive = manual;
    i.estop = false;
    return i;
}

} // namespace

TEST(motion_target_move_drives_decelerates_and_arrives) {
    MotionController mc(midRng2);
    mc.tuning.homePos = 240;
    mc.moveHomeRelative(90, 0, 0, true); // -> absolute 330, default speed 100
    CHECK(mc.busy());

    uint32_t now = 1000, samples = 10;
    int16_t pos = 240;

    // Far from target: full commanded speed, positive direction (increasing deg).
    int8_t out = mc.tick(in(now, pos, samples));
    CHECK(out > 0);
    CHECK_EQ(out, 100);

    // Sweep toward target; inside the decel zone (50 deg) speed must taper but
    // never below minSpeed (15).
    while (pos < 325) {
        pos += 3;
        now += 20;
        ++samples;
        out = mc.tick(in(now, pos, samples));
        int16_t dist = 330 - pos;
        if (dist >= 50) {
            CHECK_EQ(out, 100);
        } else if (dist > 5) {
            CHECK(out > 0 && out <= 100);
            CHECK(out >= 15);
            CHECK(out <= (int32_t)100 * dist / 50 + 16);
        }
    }

    // In the arrival arc: neutral output while dwell counts fresh samples.
    pos = 329;
    for (int i = 0; i < 3; ++i) {
        now += 20;
        ++samples;
        out = mc.tick(in(now, pos, samples));
        CHECK_EQ(out, 0);
    }
    CHECK(!mc.busy());
    CHECK(mc.arrived());
    CHECK(mc.state() == MotionController::State::kIdle);
    CHECK(mc.fault() == MotionController::Fault::kNone);
}

TEST(motion_arrival_needs_consecutive_samples) {
    // A single in-arc glitch must not end the move (the legacy single-sample
    // arrival bug). One in-arc sample followed by out-of-arc samples: keeps driving.
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.moveToAbsolute(180, 0, 0, true);
    uint32_t now = 0, samples = 0;
    CHECK(mc.tick(in(now, 90, ++samples)) > 0);
    CHECK_EQ(mc.tick(in(now += 20, 178, ++samples)), 0); // in arc: dwell 1, hold
    int8_t out = mc.tick(in(now += 20, 150, ++samples)); // back out: dwell resets
    CHECK(out > 0);
    CHECK(mc.busy()); // still moving — one sample was not enough to declare arrival
}

TEST(motion_manual_input_cancels_automation_same_tick) {
    MotionController mc(midRng2);
    mc.moveToAbsolute(100, 0, 0, true);
    CHECK(mc.busy());
    int8_t out = mc.tick(in(1000, 0, 1, true, /*manual=*/true));
    CHECK_EQ(out, 0);
    CHECK(!mc.busy());
    CHECK(mc.state() == MotionController::State::kIdle);
}

TEST(motion_sensor_stale_cuts_move) {
    MotionController mc(midRng2);
    mc.moveToAbsolute(100, 0, 0, true);
    int8_t out = mc.tick(in(1000, 0, 1, /*valid=*/false));
    CHECK_EQ(out, 0);
    CHECK(mc.state() == MotionController::State::kIdle);
    CHECK(mc.fault() == MotionController::Fault::kSensorLost);
}

TEST(motion_position_jump_replans_instead_of_aborting) {
    // A confirmed tracker correction (sticker-seam burst, over-limit motion) must
    // not kill an absolute move — the controller re-plans from the corrected
    // position with a fresh watchdog window and keeps driving.
    MotionController mc(midRng2);
    mc.moveToAbsolute(100, 0, 0, true);
    int8_t out = mc.tick(in(1000, 0, 1, true, false, /*jumped=*/true));
    CHECK(out > 0); // still driving toward 100 from the corrected position 0
    CHECK(mc.busy());
    CHECK(mc.fault() == MotionController::Fault::kNone);
}

TEST(motion_in_arc_jump_does_not_restart_dwell) {
    // A tracker jump whose corrected position is already inside the arrival arc
    // must NOT wipe the dwell and restart the hold — otherwise a glitch-prone arc
    // (motor-current noise near the target) resets arrival every tick and the
    // dome hunts forever instead of latching idle. Regression for the Aug 2026
    // "flip out": here the loop still settles despite a jump flagged at target.
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.moveToAbsolute(180, 0, 0, true);
    uint32_t now = 0, samples = 0;
    CHECK(mc.tick(in(now, 90, ++samples)) > 0);            // driving toward 180
    CHECK_EQ(mc.tick(in(now += 20, 179, ++samples)), 0);   // in arc: dwell 1
    CHECK_EQ(mc.tick(in(now += 20, 181, ++samples, true, false, /*jumped=*/true)), 0);
    CHECK(mc.busy());                                      // dwell 2, not reset
    CHECK_EQ(mc.tick(in(now += 20, 180, ++samples)), 0);   // dwell 3 -> arrived
    CHECK(!mc.busy());
    CHECK(mc.arrived());
    CHECK(mc.state() == MotionController::State::kIdle);
}

TEST(motion_watchdog_stops_stuck_dome) {
    MotionController mc(midRng2);
    mc.tuning.timeoutSec = 5;
    mc.moveToAbsolute(180, 0, 0, true);
    uint32_t now = 1000;
    uint32_t samples = 7;
    // Dome never moves: same position, fresh samples keep arriving.
    for (int i = 0; i < 300; ++i) {
        now += 20;
        mc.tick(in(now, 20, ++samples));
    }
    CHECK(mc.state() == MotionController::State::kIdle);
    CHECK(mc.fault() == MotionController::Fault::kTimeout);
}

TEST(motion_auto_mode_schedules_random_move_when_idle) {
    MotionController mc(midRng2);
    mc.tuning.homePos = 240;
    mc.tuning.autoMode = true;
    mc.tuning.idleMs = 3000;
    mc.tuning.autoMinS = 6;
    mc.tuning.autoMaxS = 8;
    uint32_t now = 5000; // beyond idleMs since boot (lastManual = 0)
    CHECK_EQ(mc.tick(in(now, 240, 1)), 0);      // schedules: now + 7 s (mid of 6..8)
    CHECK_EQ(mc.tick(in(now + 6900, 240, 2)), 0);
    mc.tick(in(now + 7001, 240, 3));            // due: starts a random move
    CHECK(mc.state() == MotionController::State::kTarget);
    // Target inside home-autoLeft .. home+autoRight (240-47 .. 240+46).
    int16_t off = signedCircularDelta(240, mc.target());
    CHECK(off >= -47 && off <= 46);
}

TEST(motion_no_automation_while_manual_recent) {
    MotionController mc(midRng2);
    mc.tuning.autoMode = true;
    mc.tuning.idleMs = 3000;
    uint32_t now = 10000;
    mc.tick(in(now, 100, 1, true, /*manual=*/true)); // stick input
    // 2 s later (inside idle window): no scheduling, stays idle.
    for (uint32_t t = now + 100; t < now + 2900; t += 500)
        CHECK_EQ(mc.tick(in(t, 100, 2)), 0);
    CHECK(mc.state() == MotionController::State::kIdle);
}

TEST(motion_estop_input_forces_neutral) {
    MotionController mc(midRng2);
    mc.moveToAbsolute(100, 0, 0, true);
    MotionController::Inputs i = in(1000, 0, 1);
    i.estop = true;
    CHECK_EQ(mc.tick(i), 0);
    CHECK(mc.state() == MotionController::State::kIdle);
}

TEST(motion_kards_settles_in_noisy_arc_without_hunting) {
    // End-to-end regression for the 2026-08-19 K-ARDS flip-out. SensorRing and
    // MotionController wired exactly as the firmware loop wires them. The dome sits
    // at ~205° and is commanded to absolute 203° (`:DPA323` + home 240) — it barely
    // needs to move. The encoder in this arc periodically lies: the stable 304 alias
    // and coarse ±35° wander. Before the plausibility guard this restarted the
    // arrival dwell every time and pulsed the motor for ~38 s (the flip-out). Now
    // the guard holds those lies off while the motor is idle, so the move settles
    // quickly and the motor is barely driven at all.
    SensorRing sr;
    MotionController mc(midRng2);
    mc.tuning.homePos = 240;

    uint32_t now = 1000;
    for (int i = 0; i < SensorRing::kMedianWindow; ++i, now += 20) // parked at 210
        feedFrame(sr, 210, now);
    CHECK(sr.valid());

    mc.moveHomeRelative(323, 50, 0, true); // -> absolute 203
    CHECK_EQ(mc.target(), 203);

    // Start just OUTSIDE the arrival arc (210 vs 203): the controller must nudge, so
    // it commands drive — which is exactly the case the guard-alone can't fix (drive
    // disarms the per-frame hold). Only the adaptive deadband stopping the chase +
    // the plausibility-gated guard rejecting the alias together settle it. Without
    // the fix this ran the motor hard and never arrived (measured effort ~5962);
    // with it, a dozen small nudges (~165).
    double truePos = 210.0;      // real dome angle; ~0.0496 deg per % per 20 ms tick
    long driveEffort = 0;        // sum of |commanded speed| over the run
    bool everArrived = false;
    for (int i = 0; i < 150 && !everArrived; ++i, now += 20) { // up to 3 s
        MotionController::Inputs in;
        in.now = now;
        in.sensorValid = sr.valid();
        in.position = sr.position();
        in.sampleCount = sr.stats().accepted;
        in.jumped = sr.consumeJump();
        int8_t out = mc.tick(in);
        driveEffort += out < 0 ? -out : out;

        // Physics: the motor actually moves the dome (dir=1, no invert here).
        truePos += out * 0.0496;
        while (truePos >= 360.0)
            truePos -= 360.0;
        while (truePos < 0.0)
            truePos += 360.0;

        // Sensor: mostly truth, but this arc injects the field lies as multi-frame
        // BLOCKS (a stable-but-wrong code the ring sits on), not single spikes — the
        // median absorbs lone spikes on its own; blocks are what survived to reset
        // the arrival dwell and drive the flip-out. A block long enough to pass the
        // median (>=3 frames) lands every ~14 frames.
        int reported = (int)(truePos + 0.5);
        int phase = i % 20;
        if (phase < 6)
            reported = 304;                        // stable-alias block
        else if (phase >= 9 && phase < 15)
            reported = (i % 40 < 20) ? 239 : 168;  // coarse-wander block

        uint8_t driveMag = out < 0 ? -out : out;
        sr.noteActive(mc.state() != MotionController::State::kIdle, now, driveMag);
        feedFrame(sr, reported, now);

        if (mc.arrived())
            everArrived = true;
    }

    CHECK(everArrived);                                   // it settled...
    CHECK(mc.state() == MotionController::State::kIdle);
    CHECK(mc.fault() == MotionController::Fault::kNone);  // ...cleanly, not via watchdog
    // Lands within the arc the coarse encoder can actually resolve here.
    CHECK(circularDistance((int)(truePos + 0.5), 203) <= mc.tuning.fudgeMax);
    // The whole point: the motor was barely used. The old flip-out ran it hard for
    // ~38 s (effort ~5962 in this model); a clean settle is a handful of nudges.
    CHECK(driveEffort < 1200);
}

TEST(motion_overshoot_limit_cycle_stops_driving) {
    // Regression for the 2026-08-19 field flip-out, replayed from the ACTUAL RaD DBG
    // capture. Target 203; the dome swept up cleanly then overshot the tight ±5° arc
    // on each min-speed correction and swung 190↔214 with wire pulsing ±15 for
    // seconds — a pure motor overshoot limit cycle (all readings real, no aliasing).
    // The controller must recognise the swing (samples straddling the target) and
    // STOP driving, instead of pumping the oscillation forever.
    MotionController mc(midRng2);
    mc.tuning.homePos = 240;
    mc.moveToAbsolute(203, 50, 0, true);

    // The position stream exactly as logged (st=target rows).
    int seq[] = {56, 81, 121, 155, 190, 194, 210, 210, 191, 194, 213, 211,
                 193, 190, 213, 214, 211, 190, 191, 211, 211, 211};
    uint32_t now = 1000, samples = 0;
    int8_t out = 0;
    int drivenAfterStraddle = 0;
    bool sawAbove = false, sawBelow = false, straddled = false;
    for (int p : seq) {
        now += 250;
        out = mc.tick(in(now, (int16_t)p, ++samples));
        if (p > 203) sawAbove = true;
        if (p < 203) sawBelow = true;
        // Once the dome has been seen both past and short of target, every further
        // min-speed pulse is the controller feeding the oscillation.
        if (straddled && out != 0) ++drivenAfterStraddle;
        if (sawAbove && sawBelow) straddled = true;
    }
    // After it has clearly straddled the target, the controller must have quit
    // driving (deadband widened to enclose the swing). Pre-fix it kept pulsing ±15
    // the whole time — drivenAfterStraddle would be large.
    CHECK(straddled);
    CHECK(drivenAfterStraddle <= 3);
    CHECK_EQ(out, 0);                                 // ends parked, motor off
    CHECK(mc.arrived());
    CHECK(mc.state() == MotionController::State::kIdle);
}

TEST(motion_single_overshoot_still_lands_accurately) {
    // Accuracy guard (the 2026-08-19 post-fix log showed some moves stopping ~27°
    // off): a move that overshoots the target ONCE and settles must still arrive at
    // full precision — a lone overshoot must NOT trip the adaptive deadband. Only a
    // sustained oscillation (two crossings) is allowed to widen the arc.
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.moveToAbsolute(200, 30, 0, true);
    uint32_t now = 1000, samples = 0;

    // Approach from below, overshoot once to 208, then come back and settle at 200.
    int seq[] = {150, 175, 195, 208, 204, 201, 200, 200, 200};
    int arrivedAtPos = -1;
    bool droveAfterOvershoot = false;
    for (int p : seq) {
        now += 250;
        int8_t out = mc.tick(in(now, (int16_t)p, ++samples));
        if (p == 208 && out != 0)
            droveAfterOvershoot = true;   // kept correcting, did NOT stop at 208
        if (arrivedAtPos < 0 && mc.arrived())
            arrivedAtPos = p;
    }
    CHECK(mc.state() == MotionController::State::kIdle);
    CHECK(droveAfterOvershoot);                                   // no premature stop
    CHECK(arrivedAtPos >= 0);
    CHECK(circularDistance(arrivedAtPos, 200) <= mc.tuning.fudge); // landed tight, not at 208
}

TEST(motion_spin_runs_without_sensor) {
    // Continuous spin is operator-commanded and does not require the sensor
    // (legacy behavior); staleness only gates position-based automation.
    MotionController mc(midRng2);
    mc.spin(-30);
    int8_t out = mc.tick(in(1000, 0, 0, /*valid=*/false));
    CHECK_EQ(out, -30);
    mc.spin(0); // R0 = stop
    CHECK_EQ(mc.tick(in(1020, 0, 0, false)), 0);
}

// ---- regression tests for the 2026-08 code-review fixes --------------------

TEST(motion_watchdog_times_out_on_flicker_hunt) {
    // Same-side sensor flicker (never straddling the target) used to count as
    // "progress" forever: the position changed every sample, the two-crossing
    // latch never fired (no sign flip across the target), and the dome pulsed
    // min-speed indefinitely. Progress is now an approach watermark, so a hunt
    // that never gets CLOSER faults within timeoutSec.
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.moveToAbsolute(215, 0, 0, true);
    uint32_t now = 0, samples = 0;
    mc.tick(in(now, 195, ++samples)); // dist 20: driving
    bool faulted = false;
    for (int i = 0; i < 70 && !faulted; ++i) {
        now += 100;
        mc.tick(in(now, (i % 2) ? 195 : 213, ++samples)); // 213 in-arc, 195 out
        faulted = mc.fault() == MotionController::Fault::kTimeout;
    }
    CHECK(faulted);
    CHECK(mc.state() == MotionController::State::kIdle);
    CHECK(!mc.busy());
}

TEST(motion_watchdog_times_out_driving_away_from_target) {
    // Wrong learned polarity drives the dome AWAY from the target; movement
    // used to refresh the watchdog, so no fault ever fired and the dome hunted
    // around the antipode at full speed forever.
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.moveToAbsolute(90, 0, 0, true);
    uint32_t now = 0, samples = 0;
    int16_t pos = 60;
    bool faulted = false;
    for (int i = 0; i < 70 && !faulted; ++i) {
        now += 100;
        pos = normalizeDeg(pos - 3); // moving away, 30 deg/s
        mc.tick(in(now, pos, ++samples));
        faulted = mc.fault() == MotionController::Fault::kTimeout;
    }
    CHECK(faulted);
}

TEST(motion_timeout_zero_disables_watchdog) {
    // #DPTIMEOUT0 is documented as "0 disables"; it used to fault every move on
    // the first tick instead.
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.tuning.timeoutSec = 0;
    mc.moveToAbsolute(180, 0, 0, true);
    uint32_t now = 0, samples = 0;
    for (int i = 0; i < 100; ++i) {
        now += 100;
        int8_t out = mc.tick(in(now, 90, ++samples)); // stuck at 90, fresh samples
        CHECK(out > 0);                               // still driving
    }
    CHECK(mc.state() == MotionController::State::kTarget);
    CHECK(mc.fault() == MotionController::Fault::kNone);
}

TEST(motion_stalled_tracking_inside_arc_faults_instead_of_hanging) {
    // The drive-0 freeze class: the dome sits inside the arrival arc while the
    // tracker rejects every frame (accepted counter frozen), so the dwell can
    // never complete. The move must fault within timeoutSec — never hang in
    // kTarget with busy() latched until someone touches the stick.
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.moveToAbsolute(100, 0, 0, true);
    uint32_t now = 0;
    mc.tick(in(now, 98, 5)); // one fresh in-arc sample: dwell 1 of 3
    for (int i = 0; i < 70 && mc.fault() == MotionController::Fault::kNone; ++i) {
        now += 100;
        mc.tick(in(now, 98, 5)); // sample counter frozen from here on
    }
    CHECK(mc.fault() == MotionController::Fault::kTimeout);
    CHECK(!mc.busy());
}

TEST(motion_spin_respects_speed_bounds) {
    // :DPR bypassed clampSpeed entirely: #DPMAXSPEED did not cap it, and a
    // below-minimum spin held the motor energised too weak to turn the dome.
    MotionController mc(midRng2);
    mc.tuning.maxSpeed = 30;
    uint32_t now = 0, samples = 0;
    mc.spin(100);
    CHECK_EQ(mc.tick(in(now += 20, 0, ++samples)), 30);
    mc.spin(-100);
    CHECK_EQ(mc.tick(in(now += 20, 0, ++samples)), -30);
    mc.spin(10); // below minSpeed (15): stop, like legacy's sub-minimum zeroing
    CHECK(mc.state() == MotionController::State::kIdle);
    CHECK_EQ(mc.tick(in(now += 20, 0, ++samples)), 0);
}

TEST(motion_antipode_jitter_does_not_widen_deadband) {
    // signedCircularDelta flips sign at the target's ANTIPODE too: jitter near
    // 180-away used to count as two crossings and latch the wide arc before the
    // move even started, silently costing up to fudgeMax-fudge degrees.
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.moveToAbsolute(0, 0, 0, true);
    uint32_t now = 0, samples = 0;
    mc.tick(in(now += 100, 179, ++samples)); // antipode jitter: d = +179
    mc.tick(in(now += 100, 181, ++samples)); // d = -179 (would have been cross 1)
    mc.tick(in(now += 100, 179, ++samples)); // d = +179 (would have latched wide)
    // Clean approach: at 8 deg out (inside fudgeMax=18, outside fudge=5) the
    // arc must still be tight, i.e. the controller keeps driving.
    mc.tick(in(now += 100, 90, ++samples));
    int8_t out = mc.tick(in(now += 100, 8, ++samples));
    CHECK(out != 0);
    // And it still lands: three fresh samples inside the tight arc.
    for (int i = 0; i < 3; ++i)
        mc.tick(in(now += 100, 2, ++samples));
    CHECK(mc.arrived());
}

TEST(motion_jump_does_not_count_as_deadband_crossing) {
    // An adopted tracker jump teleports the BELIEVED position across the
    // target; that is not a physical swing and must not feed the oscillation
    // detector (it used to combine with one real overshoot to latch wide).
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.moveToAbsolute(200, 0, 0, true);
    uint32_t now = 0, samples = 0;
    mc.tick(in(now += 100, 230, ++samples));                          // approach, d=+30
    mc.tick(in(now += 100, 185, ++samples, true, false, true));       // JUMP across target
    mc.tick(in(now += 100, 195, ++samples));                          // real crossing 1 (d=-5.. below fudge? use 190)
    mc.tick(in(now += 100, 190, ++samples));                          // d=-10: side -1
    mc.tick(in(now += 100, 208, ++samples));                          // d=+8: one real flip
    int8_t out = mc.tick(in(now += 100, 208, ++samples));
    CHECK(out != 0); // arc still tight at 8 deg out: no premature "arrived" hold
}

TEST(motion_relative_move_aborts_on_jump) {
    // BEHAVIOR §7: a confirmed tracker jump ABORTS an in-progress relative move
    // with an error (the delta was measured from a disowned start point);
    // absolute moves re-plan instead (covered elsewhere).
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.moveRelative(/*fromPos=*/100, /*delta=*/-90, 0, 0, true); // -> absolute 10
    CHECK_EQ(mc.target(), 10);
    uint32_t now = 0, samples = 0;
    mc.tick(in(now += 20, 100, ++samples));
    mc.tick(in(now += 20, 60, ++samples, true, false, /*jumped=*/true));
    CHECK(mc.fault() == MotionController::Fault::kJump);
    CHECK(mc.state() == MotionController::State::kIdle);
    CHECK(!mc.busy());
}

TEST(motion_sequence_suppresses_idle_automation) {
    // A running sequence's W steps leave the controller idle; automation must
    // not schedule its own moves into that window.
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.tuning.autoMode = true;
    mc.tuning.idleMs = 0;
    uint32_t now = 10000, samples = 0;
    for (int i = 0; i < 200; ++i) {
        now += 100;
        MotionController::Inputs i2 = in(now, 100, ++samples);
        i2.suppressAutomation = true; // sequence active (e.g. inside a W step)
        mc.tick(i2);
        CHECK(mc.state() == MotionController::State::kIdle);
    }
}

TEST(motion_settle_delay_after_arrival) {
    // #DPTARGETMIN/#DPTARGETMAX: after a targeted move arrives, automation
    // waits out the settle window before planning again.
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.tuning.autoMode = true;
    mc.tuning.idleMs = 0;
    mc.tuning.targetMinS = 4;
    mc.tuning.targetMaxS = 4;
    mc.tuning.autoMinS = 1;
    mc.tuning.autoMaxS = 1;
    uint32_t now = 5000, samples = 0;
    mc.moveToAbsolute(100, 0, 0, true);
    for (int i = 0; i < 3; ++i)
        mc.tick(in(now += 20, 100, ++samples)); // in-arc: dwell to arrival
    CHECK(mc.arrived());
    // Inside the 4 s settle window nothing may even be SCHEDULED; after it,
    // the 1 s auto delay runs and a move starts. Check at 3.5 s: still idle.
    uint32_t arrivedAt = now;
    while (now - arrivedAt < 3500) {
        mc.tick(in(now += 100, 100, ++samples));
        CHECK(mc.state() == MotionController::State::kIdle);
    }
    // By 4 s (settle) + 1 s (autoMin..Max) + slack, the random move is running.
    while (now - arrivedAt < 5600 && mc.state() == MotionController::State::kIdle)
        mc.tick(in(now += 100, 100, ++samples));
    CHECK(mc.state() == MotionController::State::kTarget);
}

TEST(motion_home_mode_accepts_wide_arrival_rest) {
    // A hunted home-seek legitimately rests anywhere inside its latched arc;
    // homeMode must NOT measure that rest point against the tight fudge and
    // re-seek forever (seek -> hunt -> rest off-home -> repeat).
    MotionController mc(midRng2);
    mc.tuning.homePos = 200;
    mc.tuning.homeMode = true;
    mc.tuning.idleMs = 0;
    mc.tuning.homeMinS = 1;
    mc.tuning.homeMaxS = 1;
    uint32_t now = 5000, samples = 0;
    mc.seekHome(0);
    // Oscillate around home to latch the wide arc: 210 -> 190 -> 210 (two
    // crossings at ±10), then rest at 208 (inside the latched arc).
    mc.tick(in(now += 100, 210, ++samples));
    mc.tick(in(now += 100, 190, ++samples));
    mc.tick(in(now += 100, 210, ++samples));
    for (int i = 0; i < 5 && !mc.arrived(); ++i)
        mc.tick(in(now += 100, 208, ++samples));
    CHECK(mc.arrived());
    // Idle at 208 (8 deg off home, inside the granted arc): no re-seek, ever.
    for (int i = 0; i < 100; ++i) {
        mc.tick(in(now += 100, 208, ++samples));
        CHECK(mc.state() == MotionController::State::kIdle);
    }
    // But a real displacement beyond the granted arc re-arms the home seek.
    bool sought = false;
    for (int i = 0; i < 30 && !sought; ++i) {
        mc.tick(in(now += 100, 150, ++samples));
        sought = mc.state() == MotionController::State::kTarget;
    }
    CHECK(sought);
}

TEST(motion_latch_sizes_arc_from_measured_swing) {
    // The latched deadband is sized from the swing actually observed (+margin),
    // not slammed to fudgeMax: a small oscillation gives up only a little
    // precision. Swing here peaks at ±8, so the arc latches ~10 — a rest point
    // 12 deg out must still be driven at, not accepted as "arrived".
    MotionController mc(midRng2);
    mc.tuning.homePos = 0;
    mc.moveToAbsolute(200, 0, 0, true);
    uint32_t now = 0, samples = 0;
    mc.tick(in(now += 100, 208, ++samples)); // +8
    mc.tick(in(now += 100, 192, ++samples)); // -8: crossing 1
    mc.tick(in(now += 100, 208, ++samples)); // +8: crossing 2 -> latch ~10
    int8_t out = mc.tick(in(now += 100, 212, ++samples)); // 12 out: beyond latched arc
    CHECK(out != 0);
    for (int i = 0; i < 4 && !mc.arrived(); ++i)
        mc.tick(in(now += 100, 208, ++samples)); // 8 out: inside latched arc
    CHECK(mc.arrived());
}
