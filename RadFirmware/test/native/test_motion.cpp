#include "rad_test.h"

#include "../../src/MotionController.h"
#include "../../src/SensorRing.h"

using namespace rad;

namespace {

uint32_t midRng2(uint32_t lo, uint32_t hi) {
    return (lo + hi) / 2;
}

void feedFrame(SensorRing& sr, int deg, uint32_t now) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#DP@%d\r\n", deg);
    for (const char* p = buf; *p; ++p)
        sr.feed(static_cast<uint8_t>(*p), now);
    sr.tick(now);
}

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
