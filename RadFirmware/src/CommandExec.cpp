#ifdef ARDUINO

#include "CommandExec.h"

#include "RadVersion.h"

namespace rad {

void CommandExec::applyTuning() {
    const RadSettings& s = *fSettings;
    MotionTuning& t = fMotion->tuning;
    t.maxSpeed = s.maxSpeed;
    t.minSpeed = s.minSpeed;
    t.homeSpeed = s.homeSpeed;
    t.autoSpeed = s.autoSpeed;
    t.targetSpeed = s.targetSpeed;
    t.fudge = s.fudge;
    t.dwell = s.dwell;
    t.scaling = s.scaling;
    t.accScale = s.accScale;
    t.decScale = s.decScale;
    t.timeoutSec = s.timeoutSec;
    t.homePos = s.homePos;
    t.homeMode = s.homeMode;
    t.autoMode = s.autoMode;
    t.autoLeft = s.autoLeft;
    t.autoRight = s.autoRight;
    t.autoMinS = s.autoMinS;
    t.autoMaxS = s.autoMaxS;
    t.homeMinS = s.homeMinS;
    t.homeMaxS = s.homeMaxS;
    t.idleMs = s.idleMs;

    SensorTuning st;
    st.maxRpm = s.maxRpm;
    st.staleMs = s.sensToMs;
    st.confirmSamples = s.sensN;
    fSensor->setTuning(st);
    fPins->setDefaults(s.digitalPins);
}

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

void CommandExec::pump(uint32_t now, Print& console) {
    if (!fSeq->active())
        return;
    if (fMotion->busy())
        return; // blocking move in flight: sequencer holds (legacy sWaitTarget)
    if (fMotion->fault() != MotionController::Fault::kNone) {
        console.println(F("SEQUENCE ABORTED (motion fault)"));
        fSeq->stop();
        return;
    }
    const SeqStep* step = fSeq->tick(now);
    if (step != nullptr)
        execStep(*step, now, console);
}

void CommandExec::execStep(const SeqStep& step, uint32_t now, Print& reply) {
    switch (step.op) {
    case 'A': { // absolute (home-relative degrees)
        uint8_t speed = step.argc >= 2 ? static_cast<uint8_t>(step.b) : 0;
        uint8_t maxSpeed = step.argc >= 3 ? static_cast<uint8_t>(step.c) : 0;
        if (step.random)
            fMotion->moveRandom(speed, !step.fireAndForget);
        else
            fMotion->moveHomeRelative(static_cast<int16_t>(step.a), speed, maxSpeed,
                                      !step.fireAndForget);
        break;
    }
    case 'D': { // relative to current position
        if (!fSensor->valid()) {
            reply.println(F("SENSOR NOT READY"));
            fSeq->stop();
            return;
        }
        uint8_t speed = step.argc >= 2 ? static_cast<uint8_t>(step.b) : 0;
        uint8_t maxSpeed = step.argc >= 3 ? static_cast<uint8_t>(step.c) : 0;
        int16_t delta = static_cast<int16_t>(step.a);
        if (step.random) {
            fMotion->moveRandom(speed, !step.fireAndForget);
        } else {
            fMotion->moveToAbsolute(static_cast<int16_t>(fSensor->position() + delta), speed,
                                    maxSpeed, !step.fireAndForget);
        }
        break;
    }
    case 'R': // continuous spin (sequence continues; a wait or end stops nothing)
        fMotion->spin(static_cast<int8_t>(step.a));
        break;
    case 'H':
        fMotion->seekHome(step.argc >= 1 ? static_cast<uint8_t>(step.a) : 0);
        break;
    case 'S': { // play stored sequence (replaces the current one)
        char body[SeqStore::kMaxBody];
        if (fSeqStore->load(static_cast<uint8_t>(step.a), body, sizeof(body))) {
            fSeq->start(body, now);
        } else {
            reply.println(F("NO SUCH SEQUENCE"));
            fSeq->stop();
        }
        break;
    }
    case 'T':
        if (!fPins->toggle(static_cast<uint8_t>(step.a)))
            reply.println(F("Invalid"));
        break;
    case 'P':
        if (!fPins->set(static_cast<uint8_t>(step.a), step.b != 0))
            reply.println(F("Invalid"));
        break;
    case 'Z':
        applyTuning();
        fPins->restore();
        fMotion->stop();
        break;
    default:
        reply.println(F("Invalid"));
        fSeq->stop();
        break;
    }
}

void CommandExec::execute(const Command& cmd, Print& reply) {
    switch (cmd.id) {
    case CmdId::kMotion:
        if (fSeq->start(cmd.text, millis())) {
            // Steps dispatch from pump(); nothing else to do here.
        } else {
            reply.println(F("Invalid"));
        }
        break;
    case CmdId::kConfig:
        dumpConfig(reply);
        break;
    case CmdId::kStatus: {
        reply.printf("Roam-A-Dome v2 %s\n", RAD_FW_VERSION);
        reply.printf("Uptime: %lus\n", static_cast<unsigned long>(millis() / 1000));
        switch (fSensor->state()) {
        case SensorRing::State::kValid:
            reply.printf("Sensor: OK, position %d (home %d)\n", fSensor->position(),
                         fSettings->homePos);
            break;
        case SensorRing::State::kWarmup:
            reply.println(F("Sensor: warming up (no frames yet?)"));
            break;
        case SensorRing::State::kStale:
            reply.println(F("Sensor: STALE — check cable/power"));
            break;
        }
        const char* ms = "idle";
        if (fMotion->state() == MotionController::State::kTarget)
            ms = "moving";
        else if (fMotion->state() == MotionController::State::kSpin)
            ms = "spinning";
        reply.printf("Motion: %s", ms);
        switch (fMotion->fault()) {
        case MotionController::Fault::kTimeout:
            reply.print(F(" (last fault: TIMEOUT — dome stuck?)"));
            break;
        case MotionController::Fault::kSensorLost:
            reply.print(F(" (last fault: SENSOR LOST)"));
            break;
        case MotionController::Fault::kJump:
            reply.print(F(" (last fault: POSITION JUMP)"));
            break;
        case MotionController::Fault::kNone:
            break;
        }
        reply.println();
        reply.printf("Sequence: %s\n", fSeq->active() ? "running" : "none");
        break;
    }
    case CmdId::kStats: {
        reply.printf("LinesConsole=%lu\n", (unsigned long)fStats->linesConsole);
        reply.printf("LinesCmdSerial=%lu\n", (unsigned long)fStats->linesCmdSerial);
        reply.printf("InvalidLines=%lu\n", (unsigned long)fStats->invalidLines);
        reply.printf("LineOverflows=%lu\n", (unsigned long)fStats->lineOverflows);
        reply.printf("SyrenChecksumErrors=%lu\n", (unsigned long)fStats->syrenChecksumErrors);
        const SensorRing::Stats& ss = fSensor->stats();
        reply.printf("SensorAccepted=%lu\n", (unsigned long)ss.accepted);
        reply.printf("SensorRejectedParse=%lu\n", (unsigned long)ss.rejectedParse);
        reply.printf("SensorRejectedRate=%lu\n", (unsigned long)ss.rejectedRate);
        reply.printf("SensorJumps=%lu\n", (unsigned long)ss.jumps);
        reply.printf("SensorStaleEvents=%lu\n", (unsigned long)ss.staleEvents);
        break;
    }
    case CmdId::kRestart:
        reply.println(F("Restarting"));
        reply.flush();
        ESP.restart();
        break;
    case CmdId::kZero:
        fStore->clear();
        reply.println(F("Cleared"));
        reply.flush();
        ESP.restart();
        break;
    case CmdId::kFactory:
        fStore->clear();
        fSeqStore->clearAll();
        reply.println(F("Factory reset"));
        reply.flush();
        ESP.restart();
        break;
    case CmdId::kSeqStore:
        // Validate before storing — a malformed sequence is rejected whole.
        if (Sequencer::validateScript(cmd.text) &&
            fSeqStore->save(static_cast<uint8_t>(cmd.arg), cmd.text))
            reply.println(F("Stored"));
        else
            reply.println(F("Invalid"));
        break;
    case CmdId::kSeqDelete:
        reply.println(fSeqStore->erase(static_cast<uint8_t>(cmd.arg)) ? F("Deleted")
                                                                      : F("Invalid"));
        break;
    case CmdId::kSeqList:
        fSeqStore->list(reply);
        break;
    default:
        setSetting(cmd, reply);
        break;
    }
}

void CommandExec::setSetting(const Command& cmd, Print& reply) {
    RadSettings& s = *fSettings;
    bool needsRestart = false;
    int32_t v = cmd.arg;
    auto inRange = [&](int32_t lo, int32_t hi) { return v >= lo && v <= hi; };

    switch (cmd.id) {
    case CmdId::kSerialBaud: s.serialBaud = v; needsRestart = true; break;
    case CmdId::kSyrenBaud: s.syrenBaud = v; needsRestart = true; break;
    case CmdId::kSensorBaud:
        if (v != 57600 && v != 115200) { reply.println(F("Invalid")); return; }
        s.sensorBaud = v; needsRestart = true; break;
    case CmdId::kSyrenAddrIn: s.syrenAddrIn = v; break;
    case CmdId::kSyrenAddrOut: s.syrenAddrOut = v; break;
    case CmdId::kSyrenAddrBoth: s.syrenAddrIn = s.syrenAddrOut = v; break;
    case CmdId::kSerialIn: s.serialIn = (v != 0); break;
    case CmdId::kSerialOut: s.serialOut = (v != 0); break;
    case CmdId::kSerialCmd: s.serialCmdIn = (v != 0); break;
    case CmdId::kPwmIn: s.pwmIn = (v != 0); needsRestart = true; break;
    case CmdId::kPwmOut: s.pwmOut = (v != 0); needsRestart = true; break;
    case CmdId::kPwmMin:
        if (!inRange(800, 2200)) { reply.println(F("Invalid")); return; }
        s.pwmMinUs = v; break;
    case CmdId::kPwmMax:
        if (!inRange(800, 2200)) { reply.println(F("Invalid")); return; }
        s.pwmMaxUs = v; break;
    case CmdId::kPwmNeutral:
        if (!inRange(800, 2200)) { reply.println(F("Invalid")); return; }
        s.pwmNeutralUs = v; break;
    case CmdId::kPwmDeadband:
        if (!inRange(0, 50)) { reply.println(F("Invalid")); return; }
        s.pwmDeadbandPct = v; break;
    case CmdId::kReport: s.reportMs = v; break;
    case CmdId::kMaxSpeed:
        if (!inRange(0, 100)) { reply.println(F("Invalid")); return; }
        s.maxSpeed = v; break;
    case CmdId::kMinSpeed:
        if (!inRange(0, 100)) { reply.println(F("Invalid")); return; }
        s.minSpeed = v; break;
    case CmdId::kHomeSpeed:
        if (!inRange(0, 100)) { reply.println(F("Invalid")); return; }
        s.homeSpeed = v; break;
    case CmdId::kAutoSpeed:
        if (!inRange(0, 100)) { reply.println(F("Invalid")); return; }
        s.autoSpeed = v; break;
    case CmdId::kTargetSpeed:
        if (!inRange(0, 100)) { reply.println(F("Invalid")); return; }
        s.targetSpeed = v; break;
    case CmdId::kInputSpeed:
        if (!inRange(0, 100)) { reply.println(F("Invalid")); return; }
        s.inputSpeed = v; break;
    case CmdId::kFudge:
        if (!inRange(0, 20)) { reply.println(F("Invalid")); return; }
        s.fudge = v; break;
    case CmdId::kScale: s.scaling = (v != 0); break;
    case CmdId::kAScale:
        if (!inRange(0, 255)) { reply.println(F("Invalid")); return; }
        s.accScale = v; break;
    case CmdId::kDScale:
        if (!inRange(0, 255)) { reply.println(F("Invalid")); return; }
        s.decScale = v; break;
    case CmdId::kInvert: s.inverted = (v != 0); break;
    case CmdId::kTimeout:
        if (!inRange(0, 30)) { reply.println(F("Invalid")); return; }
        s.timeoutSec = v; break;
    case CmdId::kAutoSafety: s.autoSafety = (v != 0); break;
    case CmdId::kAutoRestart: s.autoRestart = (v != 0); break;
    case CmdId::kHomeModeSet: s.homeMode = (v != 0); break;
    case CmdId::kAutoModeSet: s.autoMode = (v != 0); break;
    case CmdId::kAutoLeft:
        if (!inRange(0, 180)) { reply.println(F("Invalid")); return; }
        s.autoLeft = v; break;
    case CmdId::kAutoRight:
        if (!inRange(0, 180)) { reply.println(F("Invalid")); return; }
        s.autoRight = v; break;
    case CmdId::kAutoMin: s.autoMinS = v; break;
    case CmdId::kAutoMax: s.autoMaxS = v; break;
    case CmdId::kHomeMin: s.homeMinS = v; break;
    case CmdId::kHomeMax: s.homeMaxS = v; break;
    case CmdId::kTargetMin: s.targetMinS = v; break;
    case CmdId::kTargetMax: s.targetMaxS = v; break;
    case CmdId::kHomePos:
        if (cmd.hasArg) {
            if (!inRange(0, 359)) { reply.println(F("Invalid")); return; }
            s.homePos = v;
        } else {
            // Bare form: snapshot current VALIDATED position (BEHAVIOR D4 — the
            // gauntlet's median already smooths it; refuse when not valid).
            if (!fSensor->valid()) { reply.println(F("SENSOR NOT READY")); return; }
            s.homePos = fSensor->position();
            reply.printf("CURRENT POSITION: %d\n", s.homePos);
        }
        break;
    case CmdId::kMaxRpm:
        if (!inRange(1, 60)) { reply.println(F("Invalid")); return; }
        s.maxRpm = v; break;
    case CmdId::kSensTo:
        if (!inRange(1500, 60000)) { reply.println(F("Invalid")); return; }
        s.sensToMs = v; break;
    case CmdId::kSensN:
        if (!inRange(1, 10)) { reply.println(F("Invalid")); return; }
        s.sensN = v; break;
    case CmdId::kDwell:
        if (!inRange(1, 10)) { reply.println(F("Invalid")); return; }
        s.dwell = v; break;
    case CmdId::kIdle:
        if (!inRange(0, 60000)) { reply.println(F("Invalid")); return; }
        s.idleMs = v; break;
    default:
        reply.println(F("Invalid"));
        return;
    }
    fStore->save(s);
    applyTuning();
    reply.println(F("OK"));
    if (needsRestart)
        reply.println(F("(#DPRESTART to apply)"));
}

void CommandExec::dumpConfig(Print& reply) const {
    // Replayable format (BEHAVIOR.md D6): each line is a valid #DP command with '='
    // inserted after the key; capture_config.py strips it on replay.
    const RadSettings& s = *fSettings;
    reply.printf("#DPHOMEPOS=%d\n", s.homePos);
    reply.printf("#DPMAXSPEED=%u\n", s.maxSpeed);
    reply.printf("#DPMINSPEED=%u\n", s.minSpeed);
    reply.printf("#DPHOMESPEED=%u\n", s.homeSpeed);
    reply.printf("#DPAUTOSPEED=%u\n", s.autoSpeed);
    reply.printf("#DPTARGETSPEED=%u\n", s.targetSpeed);
    reply.printf("#DPINPUTSPEED=%u\n", s.inputSpeed);
    reply.printf("#DPHOME=%d\n", s.homeMode ? 1 : 0);
    reply.printf("#DPAUTO=%d\n", s.autoMode ? 1 : 0);
    reply.printf("#DPSCALE=%d\n", s.scaling ? 1 : 0);
    reply.printf("#DPASCALE=%u\n", s.accScale);
    reply.printf("#DPDSCALE=%u\n", s.decScale);
    reply.printf("#DPINVERT=%d\n", s.inverted ? 1 : 0);
    reply.printf("#DPTIMEOUT=%u\n", s.timeoutSec);
    reply.printf("#DPAUTOSAFETY=%d\n", s.autoSafety ? 1 : 0);
    reply.printf("#DPAUTORESTART=%d\n", s.autoRestart ? 1 : 0);
    reply.printf("#DPAUTOLEFT=%u\n", s.autoLeft);
    reply.printf("#DPAUTORIGHT=%u\n", s.autoRight);
    reply.printf("#DPAUTOMIN=%u\n", s.autoMinS);
    reply.printf("#DPAUTOMAX=%u\n", s.autoMaxS);
    reply.printf("#DPHOMEMIN=%u\n", s.homeMinS);
    reply.printf("#DPHOMEMAX=%u\n", s.homeMaxS);
    reply.printf("#DPTARGETMIN=%u\n", s.targetMinS);
    reply.printf("#DPTARGETMAX=%u\n", s.targetMaxS);
    reply.printf("#DPFUDGE=%u\n", s.fudge);
    reply.printf("#DPSYRENADDRIN=%u\n", s.syrenAddrIn);
    reply.printf("#DPSYRENADDROUT=%u\n", s.syrenAddrOut);
    reply.printf("#DPSYRENBAUD=%lu\n", (unsigned long)s.syrenBaud);
    reply.printf("#DPSERIALBAUD=%lu\n", (unsigned long)s.serialBaud);
    reply.printf("#DPSENSORBAUD=%lu\n", (unsigned long)s.sensorBaud);
    reply.printf("#DPSERIALIN=%d\n", s.serialIn ? 1 : 0);
    reply.printf("#DPSERIALOUT=%d\n", s.serialOut ? 1 : 0);
    reply.printf("#DPSERIALCMD=%d\n", s.serialCmdIn ? 1 : 0);
    reply.printf("#DPPWMIN=%d\n", s.pwmIn ? 1 : 0);
    reply.printf("#DPPWMOUT=%d\n", s.pwmOut ? 1 : 0);
    reply.printf("#DPPWMMIN=%u\n", s.pwmMinUs);
    reply.printf("#DPPWMMAX=%u\n", s.pwmMaxUs);
    reply.printf("#DPPWMNEUTRAL=%u\n", s.pwmNeutralUs);
    reply.printf("#DPPWMDEADBAND=%u\n", s.pwmDeadbandPct);
    reply.printf("#DPREPORT=%u\n", s.reportMs);
    reply.printf("#DPMAXRPM=%u\n", s.maxRpm);
    reply.printf("#DPSENSTO=%u\n", s.sensToMs);
    reply.printf("#DPSENSN=%u\n", s.sensN);
    reply.printf("#DPDWELL=%u\n", s.dwell);
    reply.printf("#DPIDLE=%u\n", s.idleMs);
}

} // namespace rad

#endif // ARDUINO
