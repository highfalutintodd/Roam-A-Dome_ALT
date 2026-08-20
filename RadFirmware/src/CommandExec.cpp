#ifdef ARDUINO

#include "CommandExec.h"

#include "RadVersion.h"

#include <cstdarg>

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
    t.targetMinS = s.targetMinS;
    t.targetMaxS = s.targetMaxS;
    t.fudgeMax = s.fudgeMax;
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

void CommandExec::pump(uint32_t now, bool manualActive, Print& console) {
    // Bare #DPHOMEPOS: the 1 s averaging capture runs regardless of sequences.
    if (fHomeCapArmed)
        tickHomeCapture(now, console);
    // 'M' one-shot: the decorated move has arrived — return the self-running
    // modes to off (runtime-only, like the modes themselves: no NVS write).
    if (fModeOffAfterMove && fMotion->arrived()) {
        fModeOffAfterMove = false;
        fSettings->homeMode = false;
        fSettings->autoMode = false;
        applyTuning();
        console.println(F("MODE OFF (one-shot)"));
    }
    if (!fSeq->active())
        return;
    if (manualActive) {
        // Legacy only overrode the motor while the stick was deflected — the
        // sequence resumed on release, which reads as the dome "fighting back".
        // v2: the operator's input cancels the sequence outright (BEHAVIOR D11).
        console.println(F("SEQUENCE CANCELLED (manual override)"));
        fSeq->stop();
        fMotion->clearFault();
        fModeOffAfterMove = false;
        return;
    }
    if (fMotion->busy())
        return; // blocking move in flight: sequencer holds (legacy sWaitTarget)
    if (fMotion->fault() != MotionController::Fault::kNone) {
        console.printf("SEQUENCE ABORTED (motion fault: %s)\n",
                       MotionController::faultName(fMotion->fault()));
        fSeq->stop();
        fMotion->clearFault(); // a fault kills THIS sequence, not all future ones
        fModeOffAfterMove = false;
        return;
    }
    const SeqStep* step = fSeq->tick(now);
    // Waits are absorbed inside tick(); print the legacy console feedback so a
    // stored sequence's pacing is visible ("WAIT SECONDS: 2" etc.).
    uint32_t waitMs;
    bool waitWasMillis;
    if (fSeq->consumeWaitStarted(waitMs, waitWasMillis)) {
        if (waitWasMillis)
            console.printf("WAIT MILLIS: %lu\n", static_cast<unsigned long>(waitMs));
        else
            console.printf("WAIT SECONDS: %lu\n", static_cast<unsigned long>(waitMs / 1000));
    }
    if (step != nullptr)
        execStep(*step, now, console);
}

// Bare #DPHOMEPOS (BEHAVIOR D4): average 1 s of validated readings — circular
// mean via deltas unwrapped around the first sample, integer-only.
void CommandExec::tickHomeCapture(uint32_t now, Print& fallback) {
    Print& reply = fHomeCapReply != nullptr ? *fHomeCapReply : fallback;
    if (fSensor->valid() && fSensor->stats().accepted != fHomeCapLastSample) {
        fHomeCapLastSample = fSensor->stats().accepted;
        int16_t pos = fSensor->position();
        if (fHomeCapCount == 0)
            fHomeCapBase = pos;
        fHomeCapSum += signedCircularDelta(fHomeCapBase, pos);
        ++fHomeCapCount;
    }
    if (static_cast<int32_t>(now - fHomeCapUntil) < 0)
        return;
    fHomeCapArmed = false;
    if (fHomeCapCount == 0) {
        reply.println(F("SENSOR NOT READY"));
        return;
    }
    RadSettings& s = *fSettings;
    s.homePos =
        normalizeDeg(fHomeCapBase + fHomeCapSum / static_cast<int32_t>(fHomeCapCount));
    fStore->save(s);
    applyTuning();
    reply.printf("CURRENT POSITION: %d\n", s.homePos);
    reply.println(F("OK"));
}

void CommandExec::execStep(const SeqStep& step, uint32_t now, Print& reply) {
    switch (step.op) {
    case 'A': { // absolute (home-relative degrees)
        // Random form has no degree arg, so the tail shifts down one: speed is
        // step.b at argc>=1 (":DPAR,40" runs at 40, not the default).
        uint8_t speed = step.argc >= (step.random ? 1 : 2) ? static_cast<uint8_t>(step.b) : 0;
        uint8_t maxSpeed = step.argc >= (step.random ? 2 : 3) ? static_cast<uint8_t>(step.c) : 0;
        // Legacy AR: a random target over the FULL circle — the autoLeft/Right
        // arc belongs to idle automation (moveRandom), not the AR step.
        int16_t deg = step.random ? static_cast<int16_t>(rng(0, 359))
                                  : static_cast<int16_t>(step.a);
        fMotion->moveHomeRelative(deg, speed, maxSpeed, !step.fireAndForget);
        fModeOffAfterMove = step.oneshot; // 'M': modes off once this move arrives
        break;
    }
    case 'D': { // relative to current position
        if (!fSensor->valid()) {
            reply.println(F("SENSOR NOT READY"));
            fSeq->stop();
            return;
        }
        uint8_t speed = step.argc >= (step.random ? 1 : 2) ? static_cast<uint8_t>(step.b) : 0;
        uint8_t maxSpeed = step.argc >= (step.random ? 2 : 3) ? static_cast<uint8_t>(step.c) : 0;
        // Legacy DR: rotate by a random 0-359 from the CURRENT position (not an
        // absolute pick inside the auto arc). moveRelative (vs moveToAbsolute)
        // makes a confirmed tracker jump abort the move instead of completing
        // it from a start point now known to be wrong (BEHAVIOR §7).
        int16_t delta = step.random ? static_cast<int16_t>(rng(0, 359))
                                    : static_cast<int16_t>(step.a);
        fMotion->moveRelative(fSensor->position(), delta, speed, maxSpeed,
                              !step.fireAndForget);
        fModeOffAfterMove = step.oneshot;
        break;
    }
    case 'R': // continuous spin (sequence continues; a wait or end stops nothing)
        if (step.random) {
            // RR: random direction, random speed within the configured band.
            int32_t mag =
                static_cast<int32_t>(rng(fMotion->tuning.minSpeed, fMotion->tuning.maxSpeed));
            fMotion->spin(static_cast<int8_t>(rng(0, 1) != 0 ? mag : -mag));
        } else {
            fMotion->spin(static_cast<int8_t>(step.a));
        }
        break;
    case 'H':
        fMotion->seekHome(
            step.random
                ? static_cast<uint8_t>(rng(fMotion->tuning.minSpeed, fMotion->tuning.maxSpeed))
                : (step.argc >= 1 ? static_cast<uint8_t>(step.a) : 0));
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
        // A new :DP line replaces any running sequence AND its in-flight move —
        // but only once it validates: garbage must not stop motion or clear
        // state. Stopping the old move here keeps the new line from silently
        // serializing behind it on pump()'s busy() hold, and clearing the old
        // fault keeps it from aborting the brand-new sequence.
        if (!Sequencer::validateScript(cmd.text)) {
            reply.println(F("Invalid"));
            break;
        }
        fMotion->stop();
        fMotion->clearFault();
        fModeOffAfterMove = false;
        fSeq->start(cmd.text, millis());
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
        if (fMotion->fault() != MotionController::Fault::kNone)
            reply.printf(" (last fault: %s)", MotionController::faultName(fMotion->fault()));
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
        reply.printf("DedupSuppressed=%lu\n", (unsigned long)fStats->dedupSuppressed);
        reply.printf("MeshRx=%lu\n", (unsigned long)fStats->meshRx);
        reply.printf("MeshDropped=%lu\n", (unsigned long)fStats->meshDropped);
        reply.printf("MeshEstops=%lu\n", (unsigned long)fStats->meshEstops);
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
        fSeqStore->list(reply, fYield); // yield: a full listing is seconds of output
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
    // NOTE on needsRestart below: SyrenBus and PwmIO snapshot RadSettings at
    // begin(), so serial/addressing/PWM-calibration changes only take effect on
    // the next boot — the hint tells the operator so, instead of a bare "OK"
    // that silently changes nothing until an unrelated power cycle.
    case CmdId::kSerialBaud:
        if (!inRange(1200, 115200)) { reply.println(F("Invalid")); return; }
        s.serialBaud = v; needsRestart = true; break;
    case CmdId::kSyrenBaud:
        if (!inRange(1200, 115200)) { reply.println(F("Invalid")); return; }
        s.syrenBaud = v; needsRestart = true; break;
    case CmdId::kSensorBaud:
        if (v != 57600 && v != 115200) { reply.println(F("Invalid")); return; }
        s.sensorBaud = v; needsRestart = true; break;
    case CmdId::kSyrenAddrIn: // packet-serial addresses are 128-135 (high bit set)
        if (!inRange(128, 135)) { reply.println(F("Invalid")); return; }
        s.syrenAddrIn = v; needsRestart = true; break;
    case CmdId::kSyrenAddrOut:
        if (!inRange(128, 135)) { reply.println(F("Invalid")); return; }
        s.syrenAddrOut = v; needsRestart = true; break;
    case CmdId::kSyrenAddrBoth:
        if (!inRange(128, 135)) { reply.println(F("Invalid")); return; }
        s.syrenAddrIn = s.syrenAddrOut = v; needsRestart = true; break;
    case CmdId::kSerialIn: s.serialIn = (v != 0); needsRestart = true; break;
    case CmdId::kSerialOut: s.serialOut = (v != 0); needsRestart = true; break;
    case CmdId::kSerialCmd: s.serialCmdIn = (v != 0); break;
    case CmdId::kPwmIn: s.pwmIn = (v != 0); needsRestart = true; break;
    case CmdId::kPwmOut: s.pwmOut = (v != 0); needsRestart = true; break;
    // PWM calibration: each bound is range-checked AND cross-checked against
    // its partners — min >= max (or neutral outside them) wraps the deadband
    // math and drives on the wrong side of neutral. Raise max before min.
    case CmdId::kPwmMin:
        if (!inRange(800, 2200) || v >= s.pwmMaxUs || v >= s.pwmNeutralUs) {
            reply.println(F("Invalid"));
            return;
        }
        s.pwmMinUs = v; needsRestart = true; break;
    case CmdId::kPwmMax:
        if (!inRange(800, 2200) || v <= s.pwmMinUs || v <= s.pwmNeutralUs) {
            reply.println(F("Invalid"));
            return;
        }
        s.pwmMaxUs = v; needsRestart = true; break;
    case CmdId::kPwmNeutral:
        if (!inRange(800, 2200) || v <= s.pwmMinUs || v >= s.pwmMaxUs) {
            reply.println(F("Invalid"));
            return;
        }
        s.pwmNeutralUs = v; needsRestart = true; break;
    case CmdId::kPwmDeadband:
        if (!inRange(0, 50)) { reply.println(F("Invalid")); return; }
        s.pwmDeadbandPct = v; needsRestart = true; break;
    case CmdId::kReport:
        if (!inRange(0, 60000)) { reply.println(F("Invalid")); return; }
        s.reportMs = v; break;
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
        s.inputSpeed = v; needsRestart = true; break; // PwmIO snapshots at begin()
    case CmdId::kFudge:
        if (!inRange(0, 20)) { reply.println(F("Invalid")); return; }
        s.fudge = v; break;
    case CmdId::kFudgeMax: // adaptive-deadband ceiling; raise with #DPMINSPEED
        if (!inRange(0, 45)) { reply.println(F("Invalid")); return; }
        s.fudgeMax = v; break;
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
    case CmdId::kAutoSafety:
        // Accepted for legacy-capture compat, but v2's arbitration ladder
        // (BEHAVIOR §5 rung 4) ALWAYS requires a valid sensor before automation
        // — running the dome blind is not a supported mode, so 0 has no effect.
        s.autoSafety = (v != 0);
        if (v == 0)
            reply.println(F("(note: v2 always requires a valid sensor for automation)"));
        break;
    case CmdId::kAutoRestart: s.autoRestart = (v != 0); break;
    case CmdId::kHomeModeSet:
        // Runtime-only (forced off at every boot, D12): persisting the toggle
        // would burn an NVS write per show cue for a value load() discards.
        s.homeMode = (v != 0);
        applyTuning();
        reply.println(F("OK"));
        return;
    case CmdId::kAutoModeSet:
        s.autoMode = (v != 0); // runtime-only: see kHomeModeSet
        applyTuning();
        reply.println(F("OK"));
        return;
    case CmdId::kAutoLeft:
        if (!inRange(0, 180)) { reply.println(F("Invalid")); return; }
        s.autoLeft = v; break;
    case CmdId::kAutoRight:
        if (!inRange(0, 180)) { reply.println(F("Invalid")); return; }
        s.autoRight = v; break;
    case CmdId::kAutoMin:
        if (!inRange(0, 600)) { reply.println(F("Invalid")); return; }
        s.autoMinS = v; break;
    case CmdId::kAutoMax:
        if (!inRange(0, 600)) { reply.println(F("Invalid")); return; }
        s.autoMaxS = v; break;
    case CmdId::kHomeMin:
        if (!inRange(0, 600)) { reply.println(F("Invalid")); return; }
        s.homeMinS = v; break;
    case CmdId::kHomeMax:
        if (!inRange(0, 600)) { reply.println(F("Invalid")); return; }
        s.homeMaxS = v; break;
    case CmdId::kTargetMin:
        if (!inRange(0, 600)) { reply.println(F("Invalid")); return; }
        s.targetMinS = v; break;
    case CmdId::kTargetMax:
        if (!inRange(0, 600)) { reply.println(F("Invalid")); return; }
        s.targetMaxS = v; break;
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
    case CmdId::kDebug:
        fStats->debug = (v != 0);
        reply.println(fStats->debug ? F("Debug on") : F("Debug off"));
        return; // RAM-only: no settings save
    case CmdId::kWcbEn: s.wcbEnabled = (v != 0); needsRestart = true; break;
    case CmdId::kWcbId:
        if (!inRange(1, 19)) { reply.println(F("Invalid")); return; }
        s.wcbDeviceId = v; needsRestart = true; break;
    case CmdId::kWcbOct: {
        // "#DPWCBOCT3C,4E" — two hex octets
        char* end = nullptr;
        long o2 = strtol(cmd.text, &end, 16);
        if (end == cmd.text || *end != ',' || o2 < 0 || o2 > 255) {
            reply.println(F("Invalid"));
            return;
        }
        const char* p2 = end + 1;
        long o3 = strtol(p2, &end, 16);
        if (end == p2 || *end != '\0' || o3 < 0 || o3 > 255) {
            reply.println(F("Invalid"));
            return;
        }
        s.wcbOct2 = static_cast<uint8_t>(o2);
        s.wcbOct3 = static_cast<uint8_t>(o3);
        needsRestart = true;
        break;
    }
    case CmdId::kWcbQty:
        if (!inRange(1, 19)) { reply.println(F("Invalid")); return; }
        s.wcbQuantity = v; needsRestart = true; break;
    case CmdId::kWcbCh:
        if (!inRange(1, 13)) { reply.println(F("Invalid")); return; }
        s.wcbChannel = v; needsRestart = true; break;
    case CmdId::kWcbPw:
        strlcpy(s.wcbPassword, cmd.text, sizeof(s.wcbPassword));
        needsRestart = true;
        break;
    case CmdId::kWcbCs: s.wcbChecksum = (v != 0); needsRestart = true; break;
    case CmdId::kDedup:
        if (!inRange(0, 10000)) { reply.println(F("Invalid")); return; }
        s.dedupMs = v; break;
    case CmdId::kLcdSleep:
        // Up to 1 h; 0 = always on. Applied live by the display each loop, so no
        // restart is needed.
        if (!inRange(0, 3600)) { reply.println(F("Invalid")); return; }
        s.lcdSleepSec = v; break;
    case CmdId::kPinDefault: {
        uint8_t pin = static_cast<uint8_t>(v / 10);
        bool val = (v % 10) != 0;
        if (val)
            s.digitalPins |= (1u << (pin - 1));
        else
            s.digitalPins &= ~(1u << (pin - 1));
        fPins->set(pin, val);
        break;
    }
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

// One line of a bulk dump. Waits (bounded) for TX room before writing so a big
// dump over a slow reply port cannot stall the control loop: at 9600 baud with
// the core's default zero TX buffer, a raw #DPCONFIG blocks loop() ~1 s — long
// enough for the Syren's serial-timeout to cut the motor mid-move. fYield keeps
// the keepalive and sensor drain running while we wait. The 250 ms cap is a
// backstop for a sink that never drains or never reports TX room.
void CommandExec::dumpLine(Print& reply, const char* fmt, ...) const {
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    if (n >= static_cast<int>(sizeof(buf)))
        n = sizeof(buf) - 1;
    if (fYield != nullptr) {
        uint32_t start = millis();
        while (reply.availableForWrite() < n && millis() - start < 250)
            fYield();
    }
    reply.print(buf);
}

void CommandExec::dumpConfig(Print& reply) const {
    // Replayable format (BEHAVIOR.md D6): each line is a valid #DP command with '='
    // inserted after the key; capture_config.py strips it on replay.
    const RadSettings& s = *fSettings;
    dumpLine(reply, "#DPHOMEPOS=%d\n", s.homePos);
    dumpLine(reply, "#DPMAXSPEED=%u\n", s.maxSpeed);
    dumpLine(reply, "#DPMINSPEED=%u\n", s.minSpeed);
    dumpLine(reply, "#DPHOMESPEED=%u\n", s.homeSpeed);
    dumpLine(reply, "#DPAUTOSPEED=%u\n", s.autoSpeed);
    dumpLine(reply, "#DPTARGETSPEED=%u\n", s.targetSpeed);
    dumpLine(reply, "#DPINPUTSPEED=%u\n", s.inputSpeed);
    dumpLine(reply, "#DPHOME=%d\n", s.homeMode ? 1 : 0);
    dumpLine(reply, "#DPAUTO=%d\n", s.autoMode ? 1 : 0);
    dumpLine(reply, "#DPSCALE=%d\n", s.scaling ? 1 : 0);
    dumpLine(reply, "#DPASCALE=%u\n", s.accScale);
    dumpLine(reply, "#DPDSCALE=%u\n", s.decScale);
    dumpLine(reply, "#DPINVERT=%d\n", s.inverted ? 1 : 0);
    dumpLine(reply, "#DPTIMEOUT=%u\n", s.timeoutSec);
    dumpLine(reply, "#DPAUTOSAFETY=%d\n", s.autoSafety ? 1 : 0);
    dumpLine(reply, "#DPAUTORESTART=%d\n", s.autoRestart ? 1 : 0);
    dumpLine(reply, "#DPAUTOLEFT=%u\n", s.autoLeft);
    dumpLine(reply, "#DPAUTORIGHT=%u\n", s.autoRight);
    dumpLine(reply, "#DPAUTOMIN=%u\n", s.autoMinS);
    dumpLine(reply, "#DPAUTOMAX=%u\n", s.autoMaxS);
    dumpLine(reply, "#DPHOMEMIN=%u\n", s.homeMinS);
    dumpLine(reply, "#DPHOMEMAX=%u\n", s.homeMaxS);
    dumpLine(reply, "#DPTARGETMIN=%u\n", s.targetMinS);
    dumpLine(reply, "#DPTARGETMAX=%u\n", s.targetMaxS);
    dumpLine(reply, "#DPFUDGE=%u\n", s.fudge);
    dumpLine(reply, "#DPFUDGEMAX=%u\n", s.fudgeMax);
    // Digital-output power-on defaults: one replayable line per pin, so a
    // capture/factory-reset/replay round trip restores them (they were the one
    // setting #DPCONFIG used to omit).
    for (uint8_t pin = 1; pin <= 8; ++pin)
        dumpLine(reply, "#DPPIN=%u%u\n", pin, (s.digitalPins >> (pin - 1)) & 1u);
    dumpLine(reply, "#DPSYRENADDRIN=%u\n", s.syrenAddrIn);
    dumpLine(reply, "#DPSYRENADDROUT=%u\n", s.syrenAddrOut);
    dumpLine(reply, "#DPSYRENBAUD=%lu\n", (unsigned long)s.syrenBaud);
    dumpLine(reply, "#DPSERIALBAUD=%lu\n", (unsigned long)s.serialBaud);
    dumpLine(reply, "#DPSENSORBAUD=%lu\n", (unsigned long)s.sensorBaud);
    dumpLine(reply, "#DPSERIALIN=%d\n", s.serialIn ? 1 : 0);
    dumpLine(reply, "#DPSERIALOUT=%d\n", s.serialOut ? 1 : 0);
    dumpLine(reply, "#DPSERIALCMD=%d\n", s.serialCmdIn ? 1 : 0);
    dumpLine(reply, "#DPPWMIN=%d\n", s.pwmIn ? 1 : 0);
    dumpLine(reply, "#DPPWMOUT=%d\n", s.pwmOut ? 1 : 0);
    dumpLine(reply, "#DPPWMMIN=%u\n", s.pwmMinUs);
    dumpLine(reply, "#DPPWMMAX=%u\n", s.pwmMaxUs);
    dumpLine(reply, "#DPPWMNEUTRAL=%u\n", s.pwmNeutralUs);
    dumpLine(reply, "#DPPWMDEADBAND=%u\n", s.pwmDeadbandPct);
    dumpLine(reply, "#DPREPORT=%u\n", s.reportMs);
    dumpLine(reply, "#DPMAXRPM=%u\n", s.maxRpm);
    dumpLine(reply, "#DPSENSTO=%u\n", s.sensToMs);
    dumpLine(reply, "#DPSENSN=%u\n", s.sensN);
    dumpLine(reply, "#DPDWELL=%u\n", s.dwell);
    dumpLine(reply, "#DPIDLE=%u\n", s.idleMs);
    dumpLine(reply, "#DPDEDUP=%u\n", s.dedupMs);
    dumpLine(reply, "#DPLCDSLEEP=%u\n", s.lcdSleepSec);
    dumpLine(reply, "#DPWCBEN=%d\n", s.wcbEnabled ? 1 : 0);
    dumpLine(reply, "#DPWCBID=%u\n", s.wcbDeviceId);
    dumpLine(reply, "#DPWCBOCT=%02X,%02X\n", s.wcbOct2, s.wcbOct3);
    dumpLine(reply, "#DPWCBQTY=%u\n", s.wcbQuantity);
    dumpLine(reply, "#DPWCBCH=%u\n", s.wcbChannel);
    dumpLine(reply, "#DPWCBCS=%d\n", s.wcbChecksum ? 1 : 0);
    // Password deliberately not dumped (public logs); shown only as set/unset.
    dumpLine(reply, "; WCB password %s\n", s.wcbPassword[0] != '\0' ? "SET" : "NOT SET");
}

} // namespace rad

#endif // ARDUINO
