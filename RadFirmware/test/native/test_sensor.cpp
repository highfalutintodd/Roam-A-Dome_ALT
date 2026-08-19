#include "rad_test.h"

#include "../../src/CircularMath.h"
#include "../../src/SensorRing.h"

#include <cstdio>
#include <cstring>

using namespace rad;

namespace {

// Feed a complete "#DP@<deg>\r\n" frame at time `now`.
void frame(SensorRing& sr, int deg, uint32_t now) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#DP@%d\r\n", deg);
    for (const char* p = buf; *p; ++p)
        sr.feed(static_cast<uint8_t>(*p), now);
    sr.tick(now);
}

void feedRaw(SensorRing& sr, const char* s, uint32_t now) {
    for (const char* p = s; *p; ++p)
        sr.feed(static_cast<uint8_t>(*p), now);
    sr.tick(now);
}

// Sensor streams ~50 frames/s in the field; 20 ms spacing.
constexpr uint32_t kDt = 20;

SensorRing warmedUp(int deg = 200, uint32_t* nowOut = nullptr) {
    SensorRing sr;
    uint32_t now = 1000;
    for (uint8_t i = 0; i < SensorRing::kMedianWindow; ++i, now += kDt)
        frame(sr, deg, now);
    if (nowOut != nullptr)
        *nowOut = now;
    return sr;
}

} // namespace

// ------------------------------------------------------------------- CircularMath

TEST(circular_math_basics) {
    CHECK_EQ(normalizeDeg(-1), 359);
    CHECK_EQ(normalizeDeg(360), 0);
    CHECK_EQ(normalizeDeg(725), 5);
    CHECK_EQ(signedCircularDelta(350, 10), 20);
    CHECK_EQ(signedCircularDelta(10, 350), -20);
    CHECK_EQ(signedCircularDelta(0, 180), 180);
    CHECK_EQ(circularDistance(359, 1), 2);
    CHECK(withinArc(350, 10, 0));   // arc across the seam
    CHECK(withinArc(350, 10, 355));
    CHECK(!withinArc(350, 10, 180));
    CHECK(withinArc(80, 100, 90));
}

// ------------------------------------------------------------------- parsing

TEST(sensor_parser_rejects_malformed_frames) {
    SensorRing sr;
    uint32_t now = 1000;
    feedRaw(sr, "#DP@\r\n", now);        // zero digits — legacy read this as position 0!
    feedRaw(sr, "#DP@12345\r\n", now);   // >4 digits
    feedRaw(sr, "#DP@999\r\n", now);     // out of range
    feedRaw(sr, "#DP@1x2\r\n", now);     // junk mid-number
    feedRaw(sr, "garbage line\r\n", now);
    CHECK_EQ(sr.stats().accepted, 0u);
    CHECK(sr.state() == SensorRing::State::kWarmup);
    CHECK(sr.stats().rejectedParse >= 5u);
}

TEST(sensor_no_position_until_warmup_completes) {
    // Moving dome: samples differ, so the stationary fast path stays out of it
    // and the full 5-frame window must fill (legacy reported raw values from the
    // very first sample).
    SensorRing sr;
    uint32_t now = 1000;
    int degs[] = {100, 105, 110, 115};
    for (int d : degs) {
        frame(sr, d, now);
        now += kDt;
        CHECK(!sr.valid());
    }
    frame(sr, 120, now);
    CHECK(sr.valid());
    CHECK(circularDistance(sr.position(), 110) <= 5); // median of the window
}

TEST(sensor_parked_dome_warms_up_in_three_heartbeats) {
    // Stationary dome: the ring only heartbeats every 1000 ms, so warm-up must
    // settle on 3 agreeing frames (~3 s), not wait ~5 s for the full window.
    SensorRing sr;
    uint32_t now = 1000;
    frame(sr, 210, now);
    frame(sr, 210, now + 1000);
    CHECK(!sr.valid());
    frame(sr, 210, now + 2000);
    CHECK(sr.valid());
    CHECK_EQ(sr.position(), 210);
}

TEST(sensor_parked_dome_stays_valid_on_heartbeat_cadence) {
    // Frames every 1000 ms (the ring's parked resend interval) must never trip
    // the staleness timeout (default 2500 ms).
    uint32_t now;
    SensorRing sr = warmedUp(90, &now);
    for (int i = 0; i < 20; ++i) {
        now += 1000;
        frame(sr, 90, now);
        sr.tick(now + 999); // just before the next heartbeat lands
        CHECK(sr.valid());
    }
    CHECK_EQ(sr.stats().staleEvents, 0u);
}

// ------------------------------------------------------------------- glitch rejection

TEST(sensor_single_glitch_rejected_users_scenario) {
    // The complaint that started this: dome moving 200 -> 250, sensor misreads "25"
    // in between. Legacy passed it straight through; v2 must not.
    uint32_t now;
    SensorRing sr = warmedUp(200, &now);
    for (int deg = 202; deg <= 224; deg += 2, now += kDt)
        frame(sr, deg, now);
    CHECK(sr.valid());
    int16_t before = sr.position();
    CHECK(circularDistance(before, 224) <= 4);

    frame(sr, 25, now); // the misread
    now += kDt;
    CHECK(circularDistance(sr.position(), before) <= 6); // did NOT jump to 25

    for (int deg = 226; deg <= 250; deg += 2, now += kDt)
        frame(sr, deg, now);
    CHECK(circularDistance(sr.position(), 250) <= 4); // tracking resumed unharmed
    // A single misread never becomes a jump: it is absorbed by the circular median
    // (the rate gate is the second line of defense, for bursts — tested below).
    CHECK(sr.stats().jumps == 0u);
}

TEST(sensor_short_glitch_burst_rejected) {
    uint32_t now;
    SensorRing sr = warmedUp(180, &now);
    // Two consecutive bogus reads (below confirm threshold of 3) then normal.
    frame(sr, 10, now);
    now += kDt;
    frame(sr, 11, now);
    now += kDt;
    CHECK_EQ(sr.position(), 180);
    for (int i = 0; i < 6; ++i, now += kDt)
        frame(sr, 180, now);
    CHECK_EQ(sr.position(), 180);
    CHECK_EQ(sr.stats().jumps, 0u);
}

TEST(sensor_sustained_jump_eventually_accepted_and_flagged) {
    // A real discontinuity (dome physically moved while readings glitched, or ring
    // slipped) must be adopted after N consistent samples — flagged as a jump so
    // relative tracking resets instead of accumulating a bogus delta.
    uint32_t now;
    SensorRing sr = warmedUp(300, &now);
    (void)sr.consumeJump();
    // Median window holds history, so several frames are needed before the median
    // lands on the new value and the confirm counter can run.
    for (int i = 0; i < 8 && sr.position() == 300; ++i, now += kDt)
        frame(sr, 90, now);
    CHECK_EQ(sr.position(), 90);
    CHECK(sr.consumeJump());
    CHECK_EQ(sr.lastDelta(), 0); // jump contributes no relative motion
}

TEST(sensor_parked_hold_rejects_phantom_jump) {
    // Motor off past the coast window: the dome cannot move, so a stable-but-wrong
    // encoder reading (the field "reads 299 for 4 s while parked" failure) is held
    // off and the position stays frozen — no confirm, no fail-open adoption.
    uint32_t now;
    SensorRing sr = warmedUp(200, &now);
    sr.noteActive(true, now);         // arm: control was active at least once
    now += kDt;
    sr.noteActive(false, now);        // move finished, controller idle
    now += 500;                      // past coastMs (400)
    for (int i = 0; i < 8; ++i, now += kDt) {
        sr.noteActive(false, now);
        frame(sr, 299, now);         // encoder lies: a wrong-but-stable code
    }
    CHECK(sr.valid());
    CHECK_EQ(sr.position(), 200);    // phantom rejected; dome held where it parked
    CHECK(!sr.consumeJump());        // and never flagged as a real move
}

TEST(sensor_parked_hold_tracks_dither) {
    // Parked hold still follows genuine sensor dither within slack, so the held
    // position never drifts away from truth.
    uint32_t now;
    SensorRing sr = warmedUp(200, &now);
    sr.noteActive(true, now);
    now += kDt;
    sr.noteActive(false, now);
    now += 500;
    for (int i = 0; i < 4; ++i, now += kDt) {
        sr.noteActive(false, now);
        frame(sr, 201, now);         // 1 deg: within slackDeg
    }
    CHECK(circularDistance(sr.position(), 201) <= 1);
}

TEST(sensor_parked_hold_reacquires_when_driven) {
    // The hold is not a latch: once the motor drives again, real motion is
    // tracked. (A dome hand-moved while parked re-locks on the next commanded move.)
    uint32_t now;
    SensorRing sr = warmedUp(200, &now);
    sr.noteActive(true, now);
    now += kDt;
    sr.noteActive(false, now);
    now += 500;
    for (int i = 0; i < 4; ++i, now += kDt) { // parked: 260 rejected, held at 200
        sr.noteActive(false, now);
        frame(sr, 260, now);
    }
    CHECK_EQ(sr.position(), 200);
    for (int i = 0; i < 4; ++i, now += kDt) { // active again: 260 is now believed
        sr.noteActive(true, now);
        frame(sr, 260, now);
    }
    CHECK(circularDistance(sr.position(), 260) <= 6);
}

TEST(sensor_active_move_is_never_frozen) {
    // Regression for the K-ARDS hang: while a move is active (noteActive true),
    // the parked-hold must NEVER engage — even past coastMs — so a target move
    // settling inside the arrival arc keeps getting fresh accepted samples and
    // its dwell can complete. Freezing here (the old wire==0 gate) pinned the
    // position at a value the flickering encoder never re-reported, so arrival
    // never finished and the move hung in `target` forever.
    uint32_t now;
    SensorRing sr = warmedUp(203, &now);
    uint32_t acceptedBefore = sr.stats().accepted;
    for (int i = 0; i < 12; ++i, now += kDt) {
        sr.noteActive(true, now);          // move still in progress (well past coast)
        frame(sr, (i % 2) ? 205 : 201, now); // small in-arc dither around target
    }
    CHECK(sr.stats().accepted > acceptedBefore); // frames keep flowing, not frozen
    CHECK(circularDistance(sr.position(), 203) <= 3);
}

TEST(sensor_fast_legit_motion_tracks) {
    // 30 RPM = 180 deg/s = 3.6 deg per 20 ms frame. Feed 3 deg per frame — fast
    // but plausible; the gate must not reject it.
    uint32_t now;
    SensorRing sr = warmedUp(0, &now);
    uint32_t rejectedBefore = sr.stats().rejectedRate;
    int deg = 0;
    for (int i = 0; i < 60; ++i, now += kDt) {
        deg = normalizeDeg(deg + 3);
        frame(sr, deg, now);
    }
    CHECK_EQ(sr.stats().rejectedRate, rejectedBefore);
    CHECK(circularDistance(sr.position(), deg) <= 6);
}

TEST(sensor_over_limit_motion_recovers_via_streak) {
    // Dome moving faster than #DPMAXRPM allows: the gate rejects every frame and
    // consecutive medians never agree, so pending confirmation can't fire. The
    // reject-streak backstop must adopt reality (flagged as a jump) instead of
    // freezing the position until the dome slows down.
    SensorTuning t;
    t.maxRpm = 10; // artificially low: 60 deg/s allowed
    uint32_t now = 1000;
    SensorRing sr(t);
    for (int i = 0; i < SensorRing::kMedianWindow; ++i, now += 20)
        frame(sr, 0, now);
    CHECK(sr.valid());
    (void)sr.consumeJump();

    // 6 deg per 20 ms = 300 deg/s, 5x the limit.
    int deg = 0;
    for (int i = 0; i < 40; ++i, now += 20) {
        deg = normalizeDeg(deg + 6);
        frame(sr, deg, now);
    }
    // Position must have kept (roughly) up rather than staying frozen at ~0.
    // Streak recovery fires every 10 rejected frames, so worst-case lag is
    // ~10 frames of motion (60 deg here) plus median lag.
    CHECK(circularDistance(sr.position(), deg) <= 75);
    CHECK(circularDistance(sr.position(), 0) > 60); // definitely not frozen at start
    CHECK(sr.stats().jumps >= 2u); // recoveries are flagged, never silent
}

// ------------------------------------------------------------------- seam

TEST(sensor_median_correct_across_seam) {
    // Legacy linear median of {355,357,359,1,3} returned 355 (~8 deg error).
    uint32_t now;
    SensorRing sr = warmedUp(353, &now);
    int degs[] = {355, 357, 359, 1, 3, 5, 7};
    for (int d : degs) {
        frame(sr, d, now);
        now += kDt;
    }
    CHECK(circularDistance(sr.position(), 5) <= 3);
    CHECK_EQ(sr.stats().rejectedRate, 0u); // smooth crossing, no gate trips
}

// ------------------------------------------------------------------- staleness

TEST(sensor_goes_stale_then_recovers_with_jump_flag) {
    uint32_t now;
    SensorRing sr = warmedUp(120, &now);
    CHECK(sr.valid());
    (void)sr.consumeJump();

    sr.tick(now + 3000); // cable yanked: no frames past the 2500 ms timeout
    CHECK(sr.state() == SensorRing::State::kStale);
    CHECK(!sr.valid());

    // Frames return (dome moved to 240 meanwhile): re-warm, then valid again,
    // flagged as jump so no phantom relative motion is accumulated.
    now += 5000;
    for (uint8_t i = 0; i < SensorRing::kMedianWindow; ++i, now += kDt)
        frame(sr, 240, now);
    CHECK(sr.valid());
    CHECK_EQ(sr.position(), 240);
    CHECK(sr.consumeJump());
}

// ------------------------------------------------------ motor-plausibility guard

// Feed a frame while telling the tracker the move is active and how hard the
// motor is actually being driven (0..100). This is what the firmware glue does
// each loop: noteActive(active, now, |wire|).
void frameDriven(SensorRing& sr, int deg, uint32_t now, uint8_t driveMag) {
    sr.noteActive(true, now, driveMag);
    frame(sr, deg, now);
}

TEST(sensor_undriven_active_holds_against_alias) {
    // The 2026-08-19 field flip-out: a target move settling at ~203 (dome already
    // there, so the controller commands 0% — in-arc). The encoder spits the stable
    // 304 alias. With the motor commanded off the dome CANNOT have moved 100°, so
    // the guard must hold position and never feed the lie to the controller (which
    // would kick it out of the arrival arc and restart the motor-pulse hunt).
    uint32_t now;
    SensorRing sr = warmedUp(203, &now);
    sr.noteActive(true, now, 40); // a moment of real drive to arm the guard
    now += kDt;
    for (int i = 0; i < 12; ++i, now += kDt) {
        int deg = (i % 4 == 2) ? 304 : 203; // periodic stable-alias spikes
        frameDriven(sr, deg, now, /*driveMag=*/0); // controller holding: 0% drive
    }
    CHECK(sr.valid());
    CHECK_EQ(sr.position(), 203);      // alias never adopted
    CHECK(!sr.consumeJump());          // and never flagged as a real move
}

TEST(sensor_undriven_active_holds_against_wander) {
    // Same arc also wanders ±35° between valid-but-coarse codes (167..239), not
    // just the alias. The sensor-side debounce means each wrong code arrives as a
    // short block (not frame-to-frame flicker), the way the ring actually delivers
    // it. Undriven, none of it is physically possible, so the reported position
    // stays put and the controller sees a clean, settleable signal.
    uint32_t now;
    SensorRing sr = warmedUp(205, &now);
    sr.noteActive(true, now, 40);
    now += kDt;
    int blocks[] = {239, 168, 218, 190, 175, 223}; // each a wrong-but-settled code
    for (int b : blocks)
        for (int i = 0; i < 4; ++i, now += kDt)
            frameDriven(sr, b, now, /*driveMag=*/0);
    CHECK(circularDistance(sr.position(), 205) <= 3); // held where it settled
    CHECK(!sr.consumeJump());                         // never mistaken for a move
}

TEST(sensor_driven_still_tracks_real_motion) {
    // The guard must only reject what the drive can't explain. While actually
    // driving at full output the dome really is sweeping, so motion must track and
    // over-limit bursts must still recover — the guard opens up with the throttle.
    uint32_t now;
    SensorRing sr = warmedUp(0, &now);
    (void)sr.consumeJump();
    int deg = 0;
    for (int i = 0; i < 60; ++i, now += kDt) {
        deg = normalizeDeg(deg + 3); // 150 deg/s: real full-speed motion
        frameDriven(sr, deg, now, /*driveMag=*/100);
    }
    CHECK(circularDistance(sr.position(), deg) <= 6); // kept up, not frozen
}

TEST(sensor_driven_guard_reacquires_after_undriven_hold) {
    // Undriven hold is not a latch: once the motor drives again the dome can move,
    // so a genuine new position is believed. (Mirrors parked-hold reacquire, but
    // via the drive signal during an active move rather than the idle timeout.)
    uint32_t now;
    SensorRing sr = warmedUp(200, &now);
    sr.noteActive(true, now, 30);
    now += kDt;
    for (int i = 0; i < 5; ++i, now += kDt)      // undriven: 260 rejected, held
        frameDriven(sr, 260, now, 0);
    CHECK_EQ(sr.position(), 200);
    // Driving again, the dome really can move: a 60° reacquisition at 60% output
    // takes ~280 ms of drive to become physically plausible, so give it that time.
    for (int i = 0; i < 20; ++i, now += kDt)
        frameDriven(sr, 260, now, 60);
    CHECK(circularDistance(sr.position(), 260) <= 6);
}

TEST(sensor_undriven_hold_still_follows_true_dither) {
    // Holding must not go blind: genuine sub-slack dither at the settled position
    // is still tracked, so the held value never drifts from truth.
    uint32_t now;
    SensorRing sr = warmedUp(203, &now);
    sr.noteActive(true, now, 40);
    now += kDt;
    for (int i = 0; i < 8; ++i, now += kDt)
        frameDriven(sr, (i % 2) ? 204 : 202, now, 0); // 1 deg dither, within slack
    CHECK(circularDistance(sr.position(), 203) <= 2);
}

TEST(sensor_relative_deltas_accumulate_only_validated_motion) {
    uint32_t now;
    SensorRing sr = warmedUp(100, &now);
    int32_t accum = 0;
    for (int deg = 102; deg <= 120; deg += 2, now += kDt) {
        frame(sr, deg, now);
        accum += sr.lastDelta();
    }
    frame(sr, 300, now); // glitch — rejected, delta must be 0
    now += kDt;
    accum += sr.lastDelta();
    CHECK(accum >= 16 && accum <= 22); // ~20 degrees of real motion, no glitch spike
}
