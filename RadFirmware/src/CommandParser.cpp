#include "Command.h"

#include <cstdlib>
#include <cstring>

namespace rad {
namespace {

enum class ArgKind : uint8_t {
    kNone,     // bare command, trailing chars invalid
    kInt,      // decimal integer immediately follows the name
    kOptInt,   // integer optional (e.g. #DPHOMEPOS vs #DPHOMEPOS90 in later phases)
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
    {"PWMIN", ArgKind::kInt, CmdId::kPwmIn},
    {"PWMOUT", ArgKind::kInt, CmdId::kPwmOut},
    {"REPORT", ArgKind::kInt, CmdId::kReport},
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
