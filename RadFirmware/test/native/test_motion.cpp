#include "rad_test.h"

#include "../../src/MotionController.h"

using namespace rad;

namespace {

uint32_t midRng2(uint32_t lo, uint32_t hi) {
    return (lo + hi) / 2;
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
