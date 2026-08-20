#include "rad_test.h"

#include "../../src/Sequencer.h"

using namespace rad;

namespace {

// Deterministic RNG capturing its arguments; returns the midpoint.
uint32_t gRngLo, gRngHi;
uint32_t midRng(uint32_t lo, uint32_t hi) {
    gRngLo = lo;
    gRngHi = hi;
    return (lo + hi) / 2;
}

} // namespace

// ------------------------------------------------------------------- parsing

TEST(seq_parse_step_forms) {
    SeqStep s;
    CHECK(parseSeqStep("A90", 3, s) && s.op == 'A' && s.a == 90 && !s.random);
    CHECK(parseSeqStep("A-90", 4, s) && s.a == -90);
    CHECK(parseSeqStep("A90,20,100", 10, s) && s.b == 20 && s.c == 100 && s.argc == 3);
    CHECK(parseSeqStep("A90,20,100+", 11, s) && s.fireAndForget);
    CHECK(parseSeqStep("AM180", 5, s) && s.oneshot && s.a == 180);
    CHECK(parseSeqStep("AR", 2, s) && s.random);
    CHECK(parseSeqStep("DR", 2, s) && s.op == 'D' && s.random);
    CHECK(parseSeqStep("R-30", 4, s) && s.op == 'R' && s.a == -30);
    CHECK(parseSeqStep("H", 1, s) && s.op == 'H');
    CHECK(parseSeqStep("W2", 2, s) && s.op == 'W' && !s.millis && s.a == 2);
    CHECK(parseSeqStep("WM500", 5, s) && s.millis && s.a == 500);
    CHECK(parseSeqStep("WR10,20", 7, s) && s.random && s.a == 10 && s.b == 20);
    CHECK(parseSeqStep("WMR100,900", 10, s) && s.random && s.millis);
    CHECK(parseSeqStep("W0", 2, s) && s.a == 0); // W0 legal in v2 (BEHAVIOR D2)
    CHECK(parseSeqStep("T3", 2, s) && s.op == 'T' && s.a == 3);
    CHECK(parseSeqStep("P21", 3, s) && s.a == 2 && s.b == 1);
    CHECK(parseSeqStep("S12", 3, s) && s.op == 'S' && s.a == 12);
    CHECK(parseSeqStep("Z", 1, s));

    CHECK(!parseSeqStep("W", 1, s));       // W needs a number
    CHECK(!parseSeqStep("W999", 4, s));    // >600 s
    CHECK(!parseSeqStep("A90x", 4, s));    // trailing junk
    CHECK(!parseSeqStep("R150", 4, s));    // out of range
    CHECK(!parseSeqStep("P91", 3, s));     // pin out of range
    CHECK(!parseSeqStep("X1", 2, s));      // unknown op
}

// ------------------------------------------------------------------- execution

TEST(seq_runs_steps_and_absorbs_waits_exactly) {
    Sequencer seq(midRng);
    uint32_t now = 5000;
    CHECK(seq.start("A90:W2:H", now));

    const SeqStep* s = seq.tick(now);
    CHECK(s != nullptr && s->op == 'A' && s->a == 90);

    // Next tick hits W2 -> waiting; H must not fire before 2000 ms elapse.
    CHECK(seq.tick(now + 1) == nullptr);
    CHECK(seq.waiting());
    CHECK(seq.tick(now + 1999) == nullptr);
    s = seq.tick(now + 2001);
    CHECK(s != nullptr && s->op == 'H');
    CHECK(seq.tick(now + 2002) == nullptr);
    CHECK(!seq.active());
}

TEST(seq_wait_survives_unrelated_activity) {
    // The legacy bug (BEHAVIOR D1): any newline on any serial port cancelled the
    // wait. In v2 the sequencer has no such input path at all — simulate the
    // scenario by ticking many times during the wait window (each loop pass in
    // firmware may process serial lines between ticks) and verify timing is exact.
    Sequencer seq(midRng);
    uint32_t now = 100000;
    CHECK(seq.start("W10:H", now));
    for (uint32_t t = now; t < now + 10000; t += 7)
        CHECK(seq.tick(t) == nullptr); // lots of loop passes; nothing fires early
    const SeqStep* s = seq.tick(now + 10000);
    CHECK(s != nullptr && s->op == 'H');
}

TEST(seq_wait_rollover_safe) {
    Sequencer seq(midRng);
    uint32_t now = 0xFFFFFF00u; // 256 ms before millis() wraps
    CHECK(seq.start("W1:H", now));
    CHECK(seq.tick(now) == nullptr);
    CHECK(seq.tick(0xFFFFFFF0u) == nullptr);       // still pre-wrap, 240 ms in
    CHECK(seq.tick(0x00000010u) == nullptr);       // wrapped, ~784 ms elapsed
    const SeqStep* s = seq.tick(now + 1000);       // wraps to 0x2C4: exactly 1 s
    CHECK(s != nullptr && s->op == 'H');
}

TEST(seq_w0_advances_same_tick) {
    Sequencer seq(midRng);
    uint32_t now = 42;
    CHECK(seq.start("W0:H", now));
    const SeqStep* s = seq.tick(now);
    CHECK(s != nullptr && s->op == 'H'); // zero wait absorbed within one tick
}

TEST(seq_random_wait_bounds_inclusive) {
    Sequencer seq(midRng);
    uint32_t now = 1000;

    CHECK(seq.start("WR10,20:H", now));
    seq.tick(now);
    CHECK_EQ(gRngLo, 10u);
    CHECK_EQ(gRngHi, 20u); // legacy random(10,20) could never return 20
    // midpoint = 15 s
    CHECK(seq.tick(now + 14999) == nullptr);
    CHECK(seq.tick(now + 15000) != nullptr);

    CHECK(seq.start("WR:H", now)); // bare WR = 1..6 s
    seq.tick(now);
    CHECK_EQ(gRngLo, 1u);
    CHECK_EQ(gRngHi, 6u);

    CHECK(seq.start("WMR100,900:H", now)); // random milliseconds (new in v2)
    seq.tick(now);
    CHECK_EQ(gRngLo, 100u);
    CHECK_EQ(gRngHi, 900u);
    CHECK(seq.tick(now + 499) == nullptr);
    CHECK(seq.tick(now + 500) != nullptr);
}

TEST(seq_start_replaces_running_sequence) {
    Sequencer seq(midRng);
    uint32_t now = 1000;
    CHECK(seq.start("W600:H", now)); // 10-minute wait
    CHECK(seq.tick(now + 5) == nullptr);
    CHECK(seq.start("A45", now + 10)); // operator sends a new :DP line
    const SeqStep* s = seq.tick(now + 11);
    CHECK(s != nullptr && s->op == 'A' && s->a == 45);
    CHECK(seq.tick(now + 12) == nullptr);
    CHECK(!seq.active());
}

TEST(seq_invalid_script_rejected_atomically) {
    Sequencer seq(midRng);
    // Third step malformed -> whole line refused, nothing executes (legacy would
    // run the first two then print Invalid mid-flight).
    CHECK(!seq.start("A90:W2:BOGUS", 0));
    CHECK(!seq.active());
    CHECK(seq.tick(1) == nullptr);
}

TEST(seq_field_sequence_from_sabe_library) {
    // Real stored sequence shape from the DCL: "#DPS3:D50:W2:D-50"
    Sequencer seq(midRng);
    uint32_t now = 0;
    CHECK(seq.start("D50:W2:D-50", now));
    const SeqStep* s = seq.tick(now);
    CHECK(s != nullptr && s->op == 'D' && s->a == 50);
    // Next tick (1 ms later, like the real loop) consumes W2 — the wait clock
    // starts there, so D-50 fires 2 s after that.
    CHECK(seq.tick(now + 1) == nullptr);
    CHECK(seq.tick(now + 2000) == nullptr);
    s = seq.tick(now + 2001);
    CHECK(s != nullptr && s->op == 'D' && s->a == -50);
}

// ---- regression tests for the 2026-08 code-review fixes --------------------

TEST(parse_rejects_out_of_range_step_args) {
    // exec-stage casts used to silently narrow these: A90,-50 ran at FULL
    // speed (uint8 wrap), A33000 landed 16 deg off (mod-65536 wrap), S300
    // played stored slot 44 and T260 toggled pin 4 (mod-256 alias).
    SeqStep s;
    CHECK(!parseSeqStep("A90,-50", 7, s));
    CHECK(!parseSeqStep("A90,300", 7, s));
    CHECK(!parseSeqStep("A33000", 6, s));
    CHECK(!parseSeqStep("D-70000", 7, s));
    CHECK(!parseSeqStep("S300", 4, s));
    CHECK(!parseSeqStep("T260", 4, s));
    CHECK(!parseSeqStep("T0", 2, s));
    CHECK(!parseSeqStep("H150", 4, s));
    // In-range forms still parse.
    CHECK(parseSeqStep("A-359", 5, s));
    CHECK(parseSeqStep("A90,100,100", 11, s));
    CHECK(parseSeqStep("AR,40", 5, s));
    CHECK(parseSeqStep("D-90", 4, s));
    CHECK(parseSeqStep("S100", 4, s));
    CHECK(parseSeqStep("T8", 2, s));
    CHECK(parseSeqStep("H100", 4, s));
}

TEST(sequencer_surfaces_wait_start_events) {
    // Waits are absorbed inside tick(); the one-shot event lets the caller
    // print the legacy "WAIT SECONDS/MILLIS" console feedback.
    Sequencer seq(midRng);
    uint32_t ms;
    bool wasMillis;
    CHECK(seq.start("W2:A90", 1000));
    CHECK(seq.tick(1000) == nullptr); // enters the wait
    CHECK(seq.consumeWaitStarted(ms, wasMillis));
    CHECK_EQ(ms, 2000u);
    CHECK(!wasMillis);
    CHECK(!seq.consumeWaitStarted(ms, wasMillis)); // one-shot
    CHECK(seq.tick(3001) != nullptr);              // A90 after the wait

    CHECK(seq.start("WM500:H", 1000));
    seq.tick(1000);
    CHECK(seq.consumeWaitStarted(ms, wasMillis));
    CHECK_EQ(ms, 500u);
    CHECK(wasMillis);
}

TEST(parse_random_forms_carry_shifted_args) {
    // ":DPAR,40" has no degree arg, so the tail shifts down one slot: speed
    // lands in b at argc 1 (the exec stage reads it from there).
    SeqStep s;
    CHECK(parseSeqStep("AR,40", 5, s));
    CHECK(s.random);
    CHECK_EQ(s.argc, 1);
    CHECK_EQ(s.b, 40);
    CHECK(parseSeqStep("AR,40,80", 8, s));
    CHECK_EQ(s.argc, 2);
    CHECK_EQ(s.b, 40);
    CHECK_EQ(s.c, 80);
    CHECK(parseSeqStep("RR", 2, s));
    CHECK(s.random);
    CHECK(parseSeqStep("HR", 2, s));
    CHECK(s.random);
    CHECK(parseSeqStep("AM90", 4, s));
    CHECK(s.oneshot);
}
