#include "Command.h"

#include <cstdlib>
#include <cstring>

namespace rad {
namespace {

enum class ArgKind : uint8_t {
    kNone,     // bare command, trailing chars invalid
    kInt,      // decimal integer immediately follows the name
    kOptInt,   // integer optional (e.g. #DPHOMEPOS vs #DPHOMEPOS90 in later phases)
    kString,   // non-empty free text into Command::text (passwords, hex pairs)
};

struct ConfigDef {
    const char* name; // after the "#DP" prefix
    ArgKind kind;
    CmdId id;
};

// Order does not matter: the parser picks the longest matching name.
constexpr ConfigDef kConfigTable[] = {
    {"CONFIG", ArgKind::kNone, CmdId::kConfig},
    {"STATUS", ArgKind::kNone, CmdId::kStatus},
    {"STATS", ArgKind::kNone, CmdId::kStats},
    {"RESTART", ArgKind::kNone, CmdId::kRestart},
    {"ZERO", ArgKind::kNone, CmdId::kZero},
    {"FACTORY", ArgKind::kNone, CmdId::kFactory},
    {"SERIALBAUD", ArgKind::kInt, CmdId::kSerialBaud},
    {"SYRENBAUD", ArgKind::kInt, CmdId::kSyrenBaud},
    {"SYRENADDRIN", ArgKind::kInt, CmdId::kSyrenAddrIn},
    {"SYRENADDROUT", ArgKind::kInt, CmdId::kSyrenAddrOut},
    {"SYRENADDR", ArgKind::kInt, CmdId::kSyrenAddrBoth},
    {"SENSORBAUD", ArgKind::kInt, CmdId::kSensorBaud},
    {"SERIALIN", ArgKind::kInt, CmdId::kSerialIn},
    {"SERIALOUT", ArgKind::kInt, CmdId::kSerialOut},
    {"SERIALCMD", ArgKind::kInt, CmdId::kSerialCmd},
    {"PWMIN", ArgKind::kInt, CmdId::kPwmIn},
    {"PWMOUT", ArgKind::kInt, CmdId::kPwmOut},
    {"PWMMIN", ArgKind::kInt, CmdId::kPwmMin},
    {"PWMMAX", ArgKind::kInt, CmdId::kPwmMax},
    {"PWMNEUTRAL", ArgKind::kInt, CmdId::kPwmNeutral},
    {"PWMDEADBAND", ArgKind::kInt, CmdId::kPwmDeadband},
    {"REPORT", ArgKind::kInt, CmdId::kReport},
    // Motion tuning + modes
    {"MAXSPEED", ArgKind::kInt, CmdId::kMaxSpeed},
    {"MINSPEED", ArgKind::kInt, CmdId::kMinSpeed},
    {"HOMESPEED", ArgKind::kInt, CmdId::kHomeSpeed},
    {"AUTOSPEED", ArgKind::kInt, CmdId::kAutoSpeed},
    {"TARGETSPEED", ArgKind::kInt, CmdId::kTargetSpeed},
    {"INPUTSPEED", ArgKind::kInt, CmdId::kInputSpeed},
    {"FUDGE", ArgKind::kInt, CmdId::kFudge},
    {"SCALE", ArgKind::kInt, CmdId::kScale},
    {"ASCALE", ArgKind::kInt, CmdId::kAScale},
    {"DSCALE", ArgKind::kInt, CmdId::kDScale},
    {"INVERT", ArgKind::kInt, CmdId::kInvert},
    {"TIMEOUT", ArgKind::kInt, CmdId::kTimeout},
    {"AUTOSAFETY", ArgKind::kInt, CmdId::kAutoSafety},
    {"AUTORESTART", ArgKind::kInt, CmdId::kAutoRestart},
    {"HOME", ArgKind::kInt, CmdId::kHomeModeSet},
    {"AUTO", ArgKind::kInt, CmdId::kAutoModeSet},
    {"AUTOLEFT", ArgKind::kInt, CmdId::kAutoLeft},
    {"AUTORIGHT", ArgKind::kInt, CmdId::kAutoRight},
    {"AUTOMIN", ArgKind::kInt, CmdId::kAutoMin},
    {"AUTOMAX", ArgKind::kInt, CmdId::kAutoMax},
    {"HOMEMIN", ArgKind::kInt, CmdId::kHomeMin},
    {"HOMEMAX", ArgKind::kInt, CmdId::kHomeMax},
    {"TARGETMIN", ArgKind::kInt, CmdId::kTargetMin},
    {"TARGETMAX", ArgKind::kInt, CmdId::kTargetMax},
    {"HOMEPOS", ArgKind::kOptInt, CmdId::kHomePos},
    // Sensor validation + arbitration (new in v2)
    {"DEBUG", ArgKind::kInt, CmdId::kDebug},
    // WCB mesh
    {"WCBEN", ArgKind::kInt, CmdId::kWcbEn},
    {"WCBID", ArgKind::kInt, CmdId::kWcbId},
    {"WCBOCT", ArgKind::kString, CmdId::kWcbOct},
    {"WCBQTY", ArgKind::kInt, CmdId::kWcbQty},
    {"WCBCH", ArgKind::kInt, CmdId::kWcbCh},
    {"WCBPW", ArgKind::kString, CmdId::kWcbPw},
    {"WCBCS", ArgKind::kInt, CmdId::kWcbCs},
    {"DEDUP", ArgKind::kInt, CmdId::kDedup},
    {"MAXRPM", ArgKind::kInt, CmdId::kMaxRpm},
    {"SENSTO", ArgKind::kInt, CmdId::kSensTo},
    {"SENSN", ArgKind::kInt, CmdId::kSensN},
    {"DWELL", ArgKind::kInt, CmdId::kDwell},
    {"IDLE", ArgKind::kInt, CmdId::kIdle},
    {"LCDSLEEP", ArgKind::kInt, CmdId::kLcdSleep},
    // Sequence list ("S"/"D" forms are special-cased in parseConfig)
    {"L", ArgKind::kNone, CmdId::kSeqList},
};

bool parseIntArg(const char* text, int32_t& value, bool required) {
    if (*text == '\0') {
        value = 0;
        return !required;
    }
    char* end = nullptr;
    long v = std::strtol(text, &end, 10);
    if (end == text || *end != '\0')
        return false;
    value = static_cast<int32_t>(v);
    return true;
}

ParseStatus parseConfig(const char* body, Command& out) {
    // Position-report echoes ("#DP@123" etc. — mode chars @ ! $ %). The dome WCB
    // port has serial-in broadcast enabled, so RAD's own reports can come back
    // at it over the mesh; they must be ignored silently, never answered.
    if (*body == '@' || *body == '!' || *body == '$' || *body == '%')
        return ParseStatus::kUnknown;

    // Sequence store: #DPS<n>:<body>. Special-cased because the body is free text
    // and 'S'+digit must not collide with SERIALBAUD/STATUS in the table.
    if (body[0] == 'S' && body[1] >= '0' && body[1] <= '9') {
        const char* p = body + 1;
        long slot = std::strtol(p, const_cast<char**>(&p), 10);
        if (slot < 0 || slot > 100 || *p != ':')
            return ParseStatus::kInvalid;
        ++p;
        size_t len = std::strlen(p);
        if (len == 0 || len >= kMaxCommandText)
            return ParseStatus::kInvalid;
        std::memcpy(out.text, p, len + 1);
        out.id = CmdId::kSeqStore;
        out.arg = static_cast<int32_t>(slot);
        out.hasArg = true;
        return ParseStatus::kOk;
    }
    // Pin default: #DPPIN<pin 1-8><0|1> — two adjacent digits, so the generic
    // int-arg table can't express it.
    if (std::strncmp(body, "PIN", 3) == 0 && body[3] >= '1' && body[3] <= '8' &&
        (body[4] == '0' || body[4] == '1') && body[5] == '\0') {
        out.id = CmdId::kPinDefault;
        out.arg = (body[3] - '0') * 10 + (body[4] - '0');
        out.hasArg = true;
        return ParseStatus::kOk;
    }

    // Sequence delete: #DPD<n> (all digits to end of line).
    if (body[0] == 'D' && body[1] >= '0' && body[1] <= '9') {
        const char* p = body + 1;
        long slot = std::strtol(p, const_cast<char**>(&p), 10);
        if (*p != '\0' || slot < 0 || slot > 100)
            return ParseStatus::kInvalid;
        out.id = CmdId::kSeqDelete;
        out.arg = static_cast<int32_t>(slot);
        out.hasArg = true;
        return ParseStatus::kOk;
    }

    const ConfigDef* best = nullptr;
    size_t bestLen = 0;
    for (const ConfigDef& def : kConfigTable) {
        size_t len = std::strlen(def.name);
        if (len > bestLen && std::strncmp(body, def.name, len) == 0) {
            best = &def;
            bestLen = len;
        }
    }
    if (best == nullptr)
        return ParseStatus::kInvalid;

    const char* args = body + bestLen;
    switch (best->kind) {
    case ArgKind::kNone:
        if (*args != '\0')
            return ParseStatus::kInvalid;
        break;
    case ArgKind::kInt:
        if (!parseIntArg(args, out.arg, /*required=*/true))
            return ParseStatus::kInvalid;
        out.hasArg = true;
        break;
    case ArgKind::kOptInt:
        if (!parseIntArg(args, out.arg, /*required=*/false))
            return ParseStatus::kInvalid;
        out.hasArg = (*args != '\0');
        break;
    case ArgKind::kString: {
        size_t len = std::strlen(args);
        if (len == 0 || len >= kMaxCommandText)
            return ParseStatus::kInvalid;
        std::memcpy(out.text, args, len + 1);
        out.hasArg = true;
        break;
    }
    }
    out.id = best->id;
    return ParseStatus::kOk;
}

} // namespace

ParseStatus parseLine(const char* line, Command& out) {
    out = Command{};
    while (*line == ' ' || *line == '\t')
        ++line;
    if (*line == '\0')
        return ParseStatus::kEmpty;

    if (std::strncmp(line, "#DP", 3) == 0)
        return parseConfig(line + 3, out);

    if (std::strncmp(line, ":DP", 3) == 0) {
        const char* body = line + 3;
        if (*body == '\0')
            return ParseStatus::kInvalid;
        size_t len = std::strlen(body);
        if (len >= sizeof(out.text))
            return ParseStatus::kInvalid;
        std::memcpy(out.text, body, len + 1);
        out.id = CmdId::kMotion;
        return ParseStatus::kOk;
    }

    return ParseStatus::kUnknown;
}

} // namespace rad
