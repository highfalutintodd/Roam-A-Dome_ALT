#ifdef ARDUINO

#include "CommandExec.h"

#include "RadVersion.h"

namespace rad {

void CommandExec::handleLine(const char* line, Print& reply) {
    Command cmd;
    switch (parseLine(line, cmd)) {
    case ParseStatus::kEmpty:
    case ParseStatus::kUnknown:
        return; // silent: mesh chatter / blank lines never produce output or side effects
    case ParseStatus::kInvalid:
        if (fStats != nullptr)
            ++fStats->invalidLines;
        reply.println(F("Invalid"));
        return;
    case ParseStatus::kOk:
        execute(cmd, reply);
        return;
    }
}

void CommandExec::execute(const Command& cmd, Print& reply) {
    switch (cmd.id) {
    case CmdId::kConfig:
        dumpConfig(reply);
        break;
    case CmdId::kStatus:
        reply.printf("Roam-A-Dome v2 %s\n", RAD_FW_VERSION);
        reply.printf("Uptime: %lus\n", static_cast<unsigned long>(millis() / 1000));
        break;
    case CmdId::kStats:
        if (fStats != nullptr) {
            reply.printf("LinesConsole=%lu\n", (unsigned long)fStats->linesConsole);
            reply.printf("LinesCmdSerial=%lu\n", (unsigned long)fStats->linesCmdSerial);
            reply.printf("InvalidLines=%lu\n", (unsigned long)fStats->invalidLines);
            reply.printf("LineOverflows=%lu\n", (unsigned long)fStats->lineOverflows);
            reply.printf("SyrenChecksumErrors=%lu\n", (unsigned long)fStats->syrenChecksumErrors);
        }
        break;
    case CmdId::kRestart:
        reply.println(F("Restarting"));
        reply.flush();
        ESP.restart();
        break;
    case CmdId::kZero:
    case CmdId::kFactory:
        fStore->clear();
        reply.println(F("Cleared"));
        reply.flush();
        ESP.restart();
        break;
    case CmdId::kMotion:
        // Sequencer/MotionController land in Phases 3-4; be honest until then.
        reply.println(F("MOTION COMMANDS NOT IMPLEMENTED YET (v2 phase 1 build)"));
        break;
    default:
        setSetting(cmd, reply);
        break;
    }
}

void CommandExec::setSetting(const Command& cmd, Print& reply) {
    RadSettings& s = *fSettings;
    bool needsRestart = false;
    switch (cmd.id) {
    case CmdId::kSerialBaud:
        s.serialBaud = cmd.arg;
        needsRestart = true;
        break;
    case CmdId::kSyrenBaud:
        s.syrenBaud = cmd.arg;
        needsRestart = true;
        break;
    case CmdId::kSensorBaud:
        if (cmd.arg != 57600 && cmd.arg != 115200) {
            reply.println(F("Invalid"));
            return;
        }
        s.sensorBaud = cmd.arg;
        needsRestart = true;
        break;
    case CmdId::kSyrenAddrIn:
        s.syrenAddrIn = cmd.arg;
        break;
    case CmdId::kSyrenAddrOut:
        s.syrenAddrOut = cmd.arg;
        break;
    case CmdId::kSyrenAddrBoth:
        s.syrenAddrIn = s.syrenAddrOut = cmd.arg;
        break;
    case CmdId::kSerialIn:
        s.serialIn = (cmd.arg != 0);
        break;
    case CmdId::kSerialOut:
        s.serialOut = (cmd.arg != 0);
        break;
    case CmdId::kPwmIn:
        s.pwmIn = (cmd.arg != 0);
        needsRestart = true;
        break;
    case CmdId::kPwmOut:
        s.pwmOut = (cmd.arg != 0);
        needsRestart = true;
        break;
    case CmdId::kReport:
        s.reportMs = cmd.arg;
        break;
    default:
        reply.println(F("Invalid"));
        return;
    }
    fStore->save(s);
    reply.println(F("OK"));
    if (needsRestart)
        reply.println(F("(#DPRESTART to apply)"));
}

void CommandExec::dumpConfig(Print& reply) const {
    // Replayable format (BEHAVIOR.md D6): each line is a valid #DP command with '='
    // inserted after the key; capture_config.py strips it on replay.
    const RadSettings& s = *fSettings;
    reply.printf("#DPSERIALBAUD=%lu\n", (unsigned long)s.serialBaud);
    reply.printf("#DPSYRENBAUD=%lu\n", (unsigned long)s.syrenBaud);
    reply.printf("#DPSENSORBAUD=%lu\n", (unsigned long)s.sensorBaud);
    reply.printf("#DPSYRENADDRIN=%u\n", s.syrenAddrIn);
    reply.printf("#DPSYRENADDROUT=%u\n", s.syrenAddrOut);
    reply.printf("#DPSERIALIN=%d\n", s.serialIn ? 1 : 0);
    reply.printf("#DPSERIALOUT=%d\n", s.serialOut ? 1 : 0);
    reply.printf("#DPPWMIN=%d\n", s.pwmIn ? 1 : 0);
    reply.printf("#DPPWMOUT=%d\n", s.pwmOut ? 1 : 0);
    reply.printf("#DPREPORT=%u\n", s.reportMs);
}

} // namespace rad

#endif // ARDUINO
