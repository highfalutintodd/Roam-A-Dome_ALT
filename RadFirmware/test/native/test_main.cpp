#include "rad_test.h"

#include "../../src/Command.h"
#include "../../src/Dedup.h"
#include "../../src/DisplaySleep.h"
#include "../../src/LineAssembler.h"
#include "../../src/SyrenCodec.h"

using namespace rad;

// ---------------------------------------------------------------- CommandParser

TEST(parser_blank_and_unknown_lines_are_silent) {
    Command c;
    CHECK(parseLine("", c) == ParseStatus::kEmpty);
    CHECK(parseLine("   ", c) == ParseStatus::kEmpty);
    CHECK(parseLine("&SABE,HB,0.36.0,120", c) == ParseStatus::kUnknown);
    CHECK(parseLine("READ: X [88]", c) == ParseStatus::kUnknown);
    // Position-report echoes: the dome WCB port broadcasts serial-in onto the
    // mesh, so RAD's own "#DP<mode><pos>" reports can arrive back — silence, not
    // "Invalid", or RAD would argue with its own telemetry.
    CHECK(parseLine("#DP@123", c) == ParseStatus::kUnknown);
    CHECK(parseLine("#DP!240", c) == ParseStatus::kUnknown);
    CHECK(parseLine("#DP$77", c) == ParseStatus::kUnknown);
    CHECK(parseLine("#DP%180", c) == ParseStatus::kUnknown);
}

TEST(parser_config_basic) {
    Command c;
    CHECK(parseLine("#DPCONFIG", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kConfig);
    CHECK(parseLine("#DPSTATUS", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kStatus);
    CHECK(parseLine("#DPRESTART", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kRestart);
}

TEST(parser_config_int_args) {
    Command c;
    CHECK(parseLine("#DPSERIALBAUD9600", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSerialBaud);
    CHECK_EQ(c.arg, 9600);

    CHECK(parseLine("#DPREPORT100", c) == ParseStatus::kOk);
    CHECK_EQ(c.arg, 100);

    CHECK(parseLine("#DPSERIALIN0", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSerialIn);
    CHECK_EQ(c.arg, 0);
}

TEST(parser_longest_prefix_wins) {
    // Legacy dispatch was order-dependent (#DPSYRENADDR had to come after
    // #DPSYRENADDRIN). v2 must resolve by longest match regardless of table order.
    Command c;
    CHECK(parseLine("#DPSYRENADDRIN128", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSyrenAddrIn);
    CHECK_EQ(c.arg, 128);

    CHECK(parseLine("#DPSYRENADDROUT130", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSyrenAddrOut);

    CHECK(parseLine("#DPSYRENADDR129", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSyrenAddrBoth);
    CHECK_EQ(c.arg, 129);

    CHECK(parseLine("#DPSERIALBAUD19200", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSerialBaud);
    CHECK(parseLine("#DPSERIALIN1", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSerialIn);
}

TEST(parser_malformed_config_is_invalid) {
    Command c;
    CHECK(parseLine("#DPCONFIGX", c) == ParseStatus::kInvalid);   // trailing junk
    CHECK(parseLine("#DPSERIALBAUD", c) == ParseStatus::kInvalid); // missing arg
    CHECK(parseLine("#DPSERIALBAUD96k", c) == ParseStatus::kInvalid);
    CHECK(parseLine("#DPBOGUS", c) == ParseStatus::kInvalid);
}

TEST(parser_sequence_storage_commands) {
    Command c;
    CHECK(parseLine("#DPS3:D50:W2:D-50", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSeqStore);
    CHECK_EQ(c.arg, 3);
    CHECK(std::strcmp(c.text, "D50:W2:D-50") == 0);

    CHECK(parseLine("#DPS100:H", c) == ParseStatus::kOk);
    CHECK_EQ(c.arg, 100);
    CHECK(parseLine("#DPS101:H", c) == ParseStatus::kInvalid); // slot > 100
    CHECK(parseLine("#DPS3:", c) == ParseStatus::kInvalid);    // empty body

    CHECK(parseLine("#DPD0", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSeqDelete);
    CHECK_EQ(c.arg, 0);
    CHECK(parseLine("#DPD12x", c) == ParseStatus::kInvalid);

    CHECK(parseLine("#DPL", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSeqList);

    // 'S'+digit special case must not shadow table commands starting with S.
    CHECK(parseLine("#DPSTATUS", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kStatus);
    CHECK(parseLine("#DPSERIALBAUD9600", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSerialBaud);
    // 'D'+digit delete must not shadow DSCALE/DWELL.
    CHECK(parseLine("#DPDSCALE50", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kDScale);
    CHECK(parseLine("#DPDWELL3", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kDwell);
}

TEST(parser_motion_config_longest_prefix_families) {
    Command c;
    // The AUTO* family: bare AUTO is the mode toggle; longer names win.
    CHECK(parseLine("#DPAUTO1", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kAutoModeSet);
    CHECK(parseLine("#DPAUTOSPEED35", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kAutoSpeed);
    CHECK(parseLine("#DPAUTOSAFETY1", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kAutoSafety);
    CHECK(parseLine("#DPAUTOLEFT47", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kAutoLeft && c.arg == 47);
    CHECK(parseLine("#DPAUTOMIN6", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kAutoMin);

    // The HOME* family: HOMEPOS with and without argument.
    CHECK(parseLine("#DPHOME1", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kHomeModeSet);
    CHECK(parseLine("#DPHOMEPOS", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kHomePos && !c.hasArg);
    CHECK(parseLine("#DPHOMEPOS240", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kHomePos && c.hasArg && c.arg == 240);
    CHECK(parseLine("#DPHOMESPEED40", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kHomeSpeed);

    // v2 sensor tuning.
    CHECK(parseLine("#DPMAXRPM30", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kMaxRpm);
    CHECK(parseLine("#DPSENSTO2500", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kSensTo);
    CHECK(parseLine("#DPIDLE3000", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kIdle);
}

TEST(parser_wcb_commands) {
    Command c;
    CHECK(parseLine("#DPWCBEN1", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kWcbEn && c.arg == 1);
    CHECK(parseLine("#DPWCBID4", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kWcbId && c.arg == 4);
    CHECK(parseLine("#DPWCBOCT3C,4E", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kWcbOct && std::strcmp(c.text, "3C,4E") == 0);
    CHECK(parseLine("#DPWCBPWsecret123", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kWcbPw && std::strcmp(c.text, "secret123") == 0);
    CHECK(parseLine("#DPWCBPW", c) == ParseStatus::kInvalid); // empty password
    CHECK(parseLine("#DPWCBCH1", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kWcbCh);
    CHECK(parseLine("#DPDEDUP750", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kDedup && c.arg == 750);
}

// ---------------------------------------------------------------- DedupFilter

TEST(dedup_suppresses_cross_source_twin_only) {
    DedupFilter d;
    // Command arrives via mesh, then its wired twin 100 ms later: twin suppressed.
    CHECK(d.allow(":DPA90", DedupFilter::kSourceMesh, 1000, 750));
    CHECK(!d.allow(":DPA90", DedupFilter::kSourceSerial, 1100, 750));
    CHECK_EQ(d.suppressed(), 1u);

    // Same source repeating is ALWAYS allowed (operator mashing a command).
    CHECK(d.allow(":DPH", DedupFilter::kSourceMesh, 2000, 750));
    CHECK(d.allow(":DPH", DedupFilter::kSourceMesh, 2100, 750));

    // Outside the window the cross-source copy runs.
    CHECK(d.allow(":DPA45", DedupFilter::kSourceMesh, 5000, 750));
    CHECK(d.allow(":DPA45", DedupFilter::kSourceSerial, 5900, 750));

    // Different command from the other source is never suppressed.
    CHECK(d.allow(":DPA10", DedupFilter::kSourceMesh, 7000, 750));
    CHECK(d.allow(":DPA20", DedupFilter::kSourceSerial, 7050, 750));

    // Window 0 disables suppression entirely.
    CHECK(d.allow(":DPZ", DedupFilter::kSourceMesh, 9000, 0));
    CHECK(d.allow(":DPZ", DedupFilter::kSourceSerial, 9010, 0));
}

TEST(parser_motion_lines_preserved_verbatim) {
    Command c;
    CHECK(parseLine(":DPA90:W2:H", c) == ParseStatus::kOk);
    CHECK(c.id == CmdId::kMotion);
    CHECK(std::strcmp(c.text, "A90:W2:H") == 0);

    CHECK(parseLine(":DP", c) == ParseStatus::kInvalid);
}

// ---------------------------------------------------------------- LineAssembler

TEST(line_assembler_crlf_yields_one_line_and_one_blank) {
    LineAssembler la;
    CHECK(la.feed('h') == nullptr);
    CHECK(la.feed('i') == nullptr);
    const char* line = la.feed('\r');
    CHECK(line != nullptr && std::strcmp(line, "hi") == 0);
    // The LF of CRLF produces an empty line — parser maps it to kEmpty (silent).
    const char* blank = la.feed('\n');
    CHECK(blank != nullptr && blank[0] == '\0');
}

TEST(line_assembler_overflow_drops_whole_line) {
    LineAssembler la;
    for (int i = 0; i < 400; ++i)
        CHECK(la.feed('x') == nullptr);
    const char* line = la.feed('\n');
    CHECK(line == nullptr); // overlong line dropped entirely
    CHECK_EQ(la.overflows(), 1u);
    // Next line works normally.
    la.feed('o');
    la.feed('k');
    const char* ok = la.feed('\n');
    CHECK(ok != nullptr && std::strcmp(ok, "ok") == 0);
}

// ---------------------------------------------------------------- SyrenCodec

TEST(syren_roundtrip) {
    SyrenFrame in{129, 0, 64};
    uint8_t buf[4];
    syrenEncode(in, buf);
    CHECK_EQ(buf[3], (129 + 0 + 64) & 0x7F);

    SyrenDecoder dec(129);
    SyrenFrame out;
    bool got = false;
    for (int i = 0; i < 4; ++i)
        got = dec.feed(buf[i], out);
    CHECK(got);
    CHECK_EQ(out.addr, 129);
    CHECK_EQ(out.cmd, 0);
    CHECK_EQ(out.data, 64);
    CHECK(!out.isNeutral());
}

TEST(syren_bad_checksum_rejected_and_resyncs) {
    SyrenDecoder dec(129);
    SyrenFrame out;
    uint8_t bad[4] = {129, 0, 64, 0x00}; // wrong checksum
    bool got = false;
    for (uint8_t b : bad)
        got = dec.feed(b, out);
    CHECK(!got);
    CHECK_EQ(dec.checksumErrors(), 1u);

    uint8_t good[4];
    syrenEncode(SyrenFrame{129, 1, 32}, good);
    for (uint8_t b : good)
        got = dec.feed(b, out);
    CHECK(got);
    CHECK_EQ(out.cmd, 1);
}

TEST(syren_mid_stream_join_resyncs_on_address_byte) {
    SyrenDecoder dec(129);
    SyrenFrame out;
    bool got = false;
    // Two garbage payload bytes (as if we joined mid-frame), then a clean frame.
    dec.feed(0x10, out);
    dec.feed(0x22, out);
    uint8_t good[4];
    syrenEncode(SyrenFrame{129, 0, 0}, good);
    for (uint8_t b : good)
        got = dec.feed(b, out);
    CHECK(got);
    CHECK(out.isNeutral());
}

TEST(syren_foreign_address_ignored) {
    SyrenDecoder dec(129);
    SyrenFrame out;
    uint8_t other[4];
    syrenEncode(SyrenFrame{130, 0, 50}, other); // Sabertooth on another address
    bool got = false;
    for (uint8_t b : other)
        got = dec.feed(b, out);
    CHECK(!got);
    CHECK_EQ(dec.foreignFrames(), 1u);
}

// ---------------------------------------------------------------- DisplaySleep

TEST(display_sleep_disabled_stays_awake_forever) {
    DisplaySleep s;
    s.setTimeout(0); // #DPLCDSLEEP0 = always on
    s.poke(0);
    CHECK(!s.tick(1000000));
    CHECK(s.awake());
}

TEST(display_sleep_sleeps_after_timeout_and_wakes_on_activity) {
    DisplaySleep s;
    s.setTimeout(300000); // 5 minutes
    s.poke(1000);

    CHECK(!s.tick(200000)); // still inside the window: no edge
    CHECK(s.awake());

    CHECK(s.tick(301000)); // edge: fell asleep
    CHECK(!s.awake());
    CHECK(!s.tick(400000)); // stays asleep; the edge is reported only once

    s.poke(400000);
    CHECK(s.tick(400000)); // edge: woke in the same tick as the activity
    CHECK(s.awake());
}

TEST(display_sleep_survives_millis_rollover) {
    DisplaySleep s;
    s.setTimeout(60000);
    uint32_t justBeforeWrap = 0xFFFFFFFFu - 10000u;
    s.poke(justBeforeWrap);
    // 20 s later the clock has wrapped past zero; a naive `now > deadline`
    // comparison would blank the screen instantly here.
    CHECK(!s.tick(justBeforeWrap + 20000u));
    CHECK(s.awake());
    CHECK(s.tick(justBeforeWrap + 61000u));
    CHECK(!s.awake());
}

int main() {
    return radtest::runAll();
}
