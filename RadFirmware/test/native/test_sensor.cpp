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
