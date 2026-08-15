// Typed command representation + parser. Pure C++ (no Arduino) — host-testable.
#pragma once

#include <cstdint>

namespace rad {

enum class CmdId : uint8_t {
    kNone = 0,
    // Motion line (":DP..."): steps kept as raw text for the Sequencer (Phase 4).
    kMotion,
    // Config commands (Phase 1 subset; grows through Phase 5).
    kConfig,
    kStatus,
    kStats,
    kRestart,
    kZero,
    kFactory,
    kSerialBaud,
    kSyrenBaud,
    kSyrenAddrIn,
    kSyrenAddrOut,
    kSyrenAddrBoth,
    kSensorBaud,
    kSerialIn,
    kSerialOut,
    kPwmIn,
    kPwmOut,
    kReport,
};

constexpr uint16_t kMaxCommandText = 256; // matches legacy sequence-length ceiling

struct Command {
    CmdId id = CmdId::kNone;
    int32_t arg = 0;             // single numeric argument, if the command takes one
    bool hasArg = false;
    char text[kMaxCommandText];  // for kMotion: the step string after ":DP"
};

enum class ParseStatus : uint8_t {
    kOk,
    kEmpty,    // blank line — ignore silently (never affects sequencer state; BEHAVIOR D1)
    kUnknown,  // not a #DP/:DP line — ignore (mesh chatter, debug echo, etc.)
    kInvalid,  // recognized prefix but malformed — respond "Invalid"
};

// Parse one CR/LF-stripped line. Longest-prefix-wins across the whole config table,
// eliminating the legacy ordering fragility (#DPS had to be registered last, etc.).
ParseStatus parseLine(const char* line, Command& out);

} // namespace rad
