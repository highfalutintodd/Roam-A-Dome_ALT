#include "rad_test.h"

#include "../../src/Command.h"
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

int main() {
    return radtest::runAll();
}
