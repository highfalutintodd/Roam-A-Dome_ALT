// Roam-A-Dome v2 — Phase 5: WCB mesh + big-number display on the validated core.
// See ../BEHAVIOR.md for the observable contract this firmware implements.

#include "pinmap.h"
#include "src/CommandExec.h"
#include "src/Dedup.h"
#include "src/LineAssembler.h"
#include "src/MotionController.h"
#include "src/PinBank.h"
#include "src/PolarityStore.h"
#include "src/PwmIO.h"
#include "src/RadVersion.h"
#include "src/SensorRing.h"
#include "src/SeqStore.h"
#include "src/Sequencer.h"
#include "src/Settings.h"
#include "src/SyrenBus.h"
#include "src/WcbLink.h"

#if RAD_HAS_DISPLAY && defined(CONFIG_IDF_TARGET_ESP32S3)
#define RAD_USE_DISPLAY 1
#include "src/DisplayS3.h"
#endif

#include <esp_random.h>
#include <strings.h> // strcasecmp — case-insensitive match for #DPDIRLEARN

using namespace rad;

// UART assignment (see pinmap.h): UART1 = sensor ring, UART2 = Syren in/out.
// Display board (S3): console is native USB CDC, so UART0 serves the command port.
HardwareSerial& syrenSerial = Serial2;
HardwareSerial& sensorSerial = Serial1;
#ifdef RAD_BOARD_DISPLAY
HardwareSerial cmdSerial(0);
#define RAD_HAS_CMD_SERIAL 1
#endif

static uint32_t inclusiveRandom(uint32_t lo, uint32_t hi) {
    return hi > lo ? lo + esp_random() % (hi - lo + 1) : lo;
}

static RadSettings sSettings;
static RadSettingsStore sStore;
static RuntimeStats sStats;
static SensorRing sSensor;
static MotionController sMotion(inclusiveRandom);
static Sequencer sSeq(inclusiveRandom);
static SeqStore sSeqStore;
static PinBank sPins;
static PolarityStore sPolarity;
static CommandExec sExec;
static SyrenBus sSyren;
static PwmIO sPwm;
static DedupFilter sDedup;
static LineAssembler sConsoleLine;
#ifdef RAD_HAS_CMD_SERIAL
static LineAssembler sCmdLine;
#endif
#ifdef RAD_USE_DISPLAY
static DisplayS3 sDisplay;
#endif

// Motor sign state; see the motor-output section below for the two conventions.
static int8_t sWirePct = 0;   // last wire-level command actually sent
static int8_t sDirSign = 0;   // learned wire->degrees sign; 0 = not yet learned
static bool sDirRelearn = false; // #DPDIRLEARN re-armed a one-shot polarity learn

static void loopCriticalYield(); // defined after driveMotor

void setup() {
    Serial.begin(115200);

    bool loaded = sStore.load(sSettings);

    syrenSerial.begin(sSettings.syrenBaud, SERIAL_8N1, RAD_PIN_SYREN_IN_RX, RAD_PIN_SYREN_OUT_TX);
    sensorSerial.begin(sSettings.sensorBaud, SERIAL_8N1, RAD_PIN_SENSOR_RX, /*tx*/ -1);
#ifdef RAD_HAS_CMD_SERIAL
    // TX buffer big enough to absorb a #DPCONFIG dump: without one the core
    // busy-blocks past the 128-byte FIFO and a dump at 9600 baud stalls loop().
    cmdSerial.setTxBufferSize(2048);
    cmdSerial.begin(sSettings.serialBaud, SERIAL_8N1, RAD_PIN_CMD_RX, RAD_PIN_CMD_TX);
#endif

    static const int kDoutPins[] = RAD_PIN_DOUT;
    sPins.begin(kDoutPins, sizeof(kDoutPins) / sizeof(kDoutPins[0]), sSettings.digitalPins);
    sExec.begin(&sSettings, &sStore, &sStats, &sSensor, &sMotion, &sSeq, &sSeqStore, &sPins);
    sExec.setYield(&loopCriticalYield); // keep motor keepalive alive during bulk dumps
    sExec.setRandom(&inclusiveRandom);  // :DPAR/:DPDR/:DPRR/:DPHR random forms
    sSyren.begin(syrenSerial, sSettings);
    sPwm.begin(RAD_PIN_PWM_IN, RAD_PIN_PWM_OUT, sSettings);
#ifdef RAD_USE_DISPLAY
    if (!sDisplay.begin())
        Serial.println("[LCD] init failed — running headless");
#endif
    gWcb.begin(sSettings, RAD_FW_VERSION);

    // Restore learned drive polarity: without it the first automated move after
    // every boot runs on the #DPINVERT guess, which is backwards on some wiring.
    sDirSign = sPolarity.load();

    Serial.printf("\nRoam-A-Dome v2 %s (%s)\n", RAD_FW_VERSION,
                  loaded ? "settings loaded" : "fresh defaults");
    if (sDirSign != 0)
        Serial.printf("[DIR] restored: positive wire %s degrees\n",
                      sDirSign > 0 ? "increases" : "decreases");
    Serial.println("#DPCONFIG settings, #DPSTATUS state, :DPA<deg> to move.");
#ifndef RAD_HAS_CMD_SERIAL
    // Classic-ESP32 compact board: no free UART for the command port (UART0 =
    // console, 1 = sensor, 2 = Syren). Say so once instead of silently
    // accepting #DPSERIALCMD for a transport that does not exist.
    if (sSettings.serialCmdIn)
        Serial.println(F("[CMD] no command-serial UART on this board — commands via console/mesh only"));
#endif
}

// ---------------------------------------------------------------------------
// Motor output. Two sign conventions meet here:
//  - Manual input: #DPINVERT maps stick direction to motor wire, exactly as the
//    operator calibrated it on legacy firmware.
//  - Automation: MotionController speaks in sensor degrees (+ = increasing).
//    Which wire polarity increases degrees is LEARNED by watching the sensor
//    while the operator drives manually (falls back to #DPINVERT until known),
//    so the closed loop can never chase its target in the wrong direction.
// Output is re-sent every 60 ms even when unchanged: Syren's serial-timeout
// safety cuts the motor if packets stop (the "hold left and it stutters" bug).
// ---------------------------------------------------------------------------
static void driveMotor(int8_t wire, uint32_t now) {
    static int8_t sLast = 127;     // force first write
    static uint32_t sLastSent = 0;
    sWirePct = wire;
    if (wire == sLast && now - sLastSent < 60)
        return;
    sLast = wire;
    sLastSent = now;
    if (sSettings.serialOut)
        sSyren.drive(wire);
    if (sSettings.pwmOut)
        sPwm.drivePercent(wire);
}

// Keep the loop-critical outputs alive while a bulk reply (#DPCONFIG, #DPL)
// waits for TX room on a slow port: drain sensor frames and keep re-sending the
// Syren keepalive so its serial-timeout never cuts the motor mid-move.
static void loopCriticalYield() {
    uint32_t now = millis();
    while (sensorSerial.available() > 0)
        sSensor.feed(static_cast<uint8_t>(sensorSerial.read()), now);
    sSensor.tick(now);
    driveMotor(sWirePct, now); // re-sends every 60 ms even when unchanged
}

// Learn drive polarity from MANUAL motion only: correlate the wire command with
// the validated sensor delta. ±10 degrees of consistent evidence locks it in.
// Automation output must never feed the learner — a closed-loop move running on
// a wrong #DPINVERT guess produces exactly the consistent wrong-way motion that
// would teach (and then freeze) the wrong sign.
//
// Learning is a ONE-TIME event, never a running background process. Drive
// polarity is physical wiring — it cannot change while the droid is powered — so
// once the sign is known (restored from NVS or learned this session) the learner
// goes dormant and never second-guesses it. Re-arming is explicit: #DPDIRLEARN,
// for after a rewire or belt swap.
//
// Why this matters (field failure, Aug 2026): motor-current noise corrupted the
// sensor line during a target hold. The tracker's own filter was rejecting the
// glitches, but the learner kept re-deriving the sign from the real, consistent
// runaway motion that a wrong sign produced — flipping polarity mid-hold. Each
// flip inverted the closed loop into POSITIVE feedback, so the dome bolted at
// full speed until the next flip yanked it back: the "flip out". Freezing the
// sign once known removes that feedback path entirely; the electrical noise is a
// separate, physical fix.
static void learnPolarity(uint32_t now, bool manualActive) {
    static uint32_t sLastAccepted = 0;
    static uint32_t sLastJumps = 0;
    static int32_t sAccum = 0;
    static int8_t sPrevSign = 0;
    static uint32_t sSignChangedAt = 0;

    if (sDirRelearn) { // #DPDIRLEARN re-armed calibration: forget prior evidence
        sDirRelearn = false;
        sAccum = 0;
        sLastAccepted = sSensor.stats().accepted;
        sLastJumps = sSensor.stats().jumps;
    }

    // Locked once known — this is the fix for the runtime sign-flip runaway.
    if (sDirSign != 0)
        return;

    // Manual drive only (see the header comment): while automation is the one
    // moving the dome, the learner stays mute rather than integrating evidence
    // from a possibly wrong-signed closed loop.
    if (!manualActive)
        return;

    // The dome coasts the old way for a moment after every stick reversal, which
    // reads as contradictory evidence — ignore samples until the command
    // direction has been stable for a while (field log showed 7 [DIR] flips
    // during back-and-forth jogging).
    int8_t sign = sWirePct > 0 ? 1 : (sWirePct < 0 ? -1 : 0);
    if (sign != sPrevSign) {
        sPrevSign = sign;
        sSignChangedAt = now;
    }
    if (sign == 0 || now - sSignChangedAt < 400)
        return;

    uint32_t accepted = sSensor.stats().accepted;
    if (accepted == sLastAccepted || !sSensor.valid())
        return;
    sLastAccepted = accepted;

    // Confidence gate: learn only from motion the tracker itself trusts. If it
    // flagged a discontinuity since the last accepted sample, this window is
    // glitch-adjacent — discard the accumulated evidence rather than integrate
    // it. (Gate-rejected samples never reach here; they don't advance `accepted`.)
    uint32_t jumps = sSensor.stats().jumps;
    if (jumps != sLastJumps) {
        sLastJumps = jumps;
        sAccum = 0;
        return;
    }

    if (sWirePct >= -25 && sWirePct <= 25)
        return; // too gentle to attribute motion confidently
    int16_t d = sSensor.lastDelta();
    sAccum += (sWirePct > 0) ? d : -d;
    if (sAccum >= 10) {
        sAccum = 10;
        sDirSign = 1;
        sPolarity.save(sDirSign); // survives reboots; frozen until #DPDIRLEARN
        Serial.println(F("[DIR] learned: positive wire increases degrees"));
    } else if (sAccum <= -10) {
        sAccum = -10;
        sDirSign = -1;
        sPolarity.save(sDirSign);
        Serial.println(F("[DIR] learned: positive wire decreases degrees"));
    }
}

// Current report mode char (the kModeChars set): '@' off, '!' home mode,
// '$' random automation, '%' targeted move. Legacy semantics: this is the
// ENGAGED MODE, not the transient controller state — an auto-mode move in
// flight is still "random automation" ('$'), and homeMode parked at home is
// still "home mode" ('!'); '%' means an explicit targeted move with no
// self-running mode engaged.
static char modeChar() {
    if (sMotion.tuning.autoMode)
        return '$';
    if (sMotion.tuning.homeMode)
        return '!';
    if (sMotion.state() == MotionController::State::kTarget)
        return '%';
    return '@';
}

// Position report out the command serial on change (BEHAVIOR.md §6).
static void reportPosition() {
#ifdef RAD_HAS_CMD_SERIAL
    static int16_t sLastReported = -1;
    if (!sSensor.valid())
        return;
    // Home-RELATIVE, matching the legacy client contract (§6 "unchanged in
    // v2"; legacy fed this line from getHomeRelativeDomePosition): parked at
    // home reports 0, not the raw sensor angle.
    int16_t pos = normalizeDeg(sSensor.position() - sSettings.homePos);
    if (pos == sLastReported)
        return;
    char buf[16];
    snprintf(buf, sizeof(buf), "#DP%c%d", modeChar(), pos);
    // Coalesce under backpressure: full-speed rotation produces more report
    // than a 9600-baud line can carry, and blocking here would throttle the
    // control loop. Skipping WITHOUT updating sLastReported re-sends the
    // latest position as soon as the buffer drains.
    if (cmdSerial.availableForWrite() < static_cast<int>(strlen(buf)) + 2)
        return;
    sLastReported = pos;
    cmdSerial.println(buf);
#endif
}

// Console report every #DPREPORT ms, drift-free (BEHAVIOR.md D3).
static void reportConsole(uint32_t now) {
    static uint32_t sNext = 0;
    if (sSettings.reportMs == 0)
        return;
    if (sNext == 0)
        sNext = now + sSettings.reportMs;
    if ((int32_t)(now - sNext) >= 0) {
        sNext += sSettings.reportMs;
        if ((int32_t)(now - sNext) >= 0)
            sNext = now + sSettings.reportMs; // fell behind: resync, don't burst
        if (sSensor.valid()) {
            Serial.print(F("DOME POSITION: "));
            // Home-relative, like the command-serial report (legacy contract).
            Serial.println(normalizeDeg(sSensor.position() - sSettings.homePos));
        } else {
            Serial.println(F("DOME POSITION: ---"));
        }
    }
}

// Ingress helper: ?STOP latches the e-stop from any transport; an explicit,
// VALID new :DP motion command releases it (BEHAVIOR.md §5) before executing.
static void handleCommandLine(const char* line, Print& reply) {
    // The mesh transport latches ?STOP in its RX callback (WcbLink::onCommand);
    // console and command-serial lines land here. COMMANDS.md: "?STOP from any
    // transport ... latches an emergency stop."
    if (strncmp(line, "?STOP", 5) == 0) {
        gWcb.latchEstop();
        reply.println(F("ESTOP"));
        return;
    }
    // Release only on a line that actually parses as a motion command AND
    // validates: garbage answered "Invalid" (":DPQ7#", bare ":DP") must never
    // release a safety latch. (The line is re-parsed by handleLine below —
    // a few microseconds, and it keeps the release decision in one place.)
    if (line[0] == ':' && line[1] == 'D' && line[2] == 'P') {
        Command cmd;
        if (parseLine(line, cmd) == ParseStatus::kOk && cmd.id == CmdId::kMotion &&
            Sequencer::validateScript(cmd.text))
            gWcb.clearEstop();
    }

    // #DPDIRLEARN: forget the learned drive polarity and re-derive it from the
    // next manual jog. Polarity is otherwise frozen for life once known (see
    // learnPolarity), so this is the deliberate, operator-driven recalibration
    // after a rewire or belt swap. Intercepted here rather than in the parser:
    // the "#DPD" prefix belongs to #DPD<n> (sequence delete), and this keeps the
    // two from colliding.
    if (!strcasecmp(line, "#DPDIRLEARN")) {
        sDirSign = 0;
        sDirRelearn = true;
        sPolarity.clear();
        reply.println(F("[DIR] cleared — jog the dome to re-learn polarity"));
        return;
    }
#ifdef RAD_USE_DISPLAY
    sDisplay.noteActivity(millis()); // someone is talking to the droid: wake the screen
#endif
    sExec.handleLine(line, reply);
}

void loop() {
    uint32_t now = millis();

    // --- inputs ---------------------------------------------------------------
    sSyren.pump(now); // decode manual input (#DPSERIALIN1); non-motor frames forwarded
    while (sensorSerial.available() > 0)
        sSensor.feed(static_cast<uint8_t>(sensorSerial.read()), now);
    sSensor.tick(now);
    sStats.syrenChecksumErrors = sSyren.checksumErrors();

    int8_t manualPct = sPwm.manualPercent(now);
    if (manualPct == 0 && sSettings.serialIn)
        manualPct = sSyren.manualPercent(now);
    bool manualActive = manualPct != 0;

    // --- command ingress ------------------------------------------------------
    while (Serial.available() > 0) {
        if (const char* line = sConsoleLine.feed(static_cast<char>(Serial.read()))) {
            ++sStats.linesConsole;
            handleCommandLine(line, Serial);
        }
    }
#ifdef RAD_HAS_CMD_SERIAL
    if (sSettings.serialCmdIn) {
        while (cmdSerial.available() > 0) {
            if (const char* line = sCmdLine.feed(static_cast<char>(cmdSerial.read()))) {
                ++sStats.linesCmdSerial;
                if ((line[0] == ':' || line[0] == '#') &&
                    !sDedup.allow(line, DedupFilter::kSourceSerial, now, sSettings.dedupMs))
                    continue; // mesh twin already executed
                handleCommandLine(line, cmdSerial);
            }
        }
    }
    sStats.lineOverflows = sConsoleLine.overflows() + sCmdLine.overflows();
#else
    sStats.lineOverflows = sConsoleLine.overflows();
#endif

    // Mesh ingress: commands queued by the WCB RX callback, drained here.
    gWcb.update();
    WcbLink::RxLine rx;
    while (gWcb.receive(rx)) {
        if (!sDedup.allow(rx.text, DedupFilter::kSourceMesh, now, sSettings.dedupMs))
            continue; // wired twin already executed
        handleCommandLine(rx.text, Serial);
    }
    sStats.dedupSuppressed = sDedup.suppressed();
    sStats.meshRx = gWcb.stats().rx;
    sStats.meshDropped = gWcb.stats().dropped;
    sStats.meshEstops = gWcb.stats().estops;

    // --- arbitration ladder (BEHAVIOR.md §5) ----------------------------------
    // Rung 1: e-stop. Latched by ?STOP/&SABE,ESTOP; released by manual input or
    // an explicit new :DP command (handled at ingress).
    bool estop = gWcb.estopLatched();
    if (estop) {
        if (sSeq.active()) {
            Serial.println(F("SEQUENCE CANCELLED (e-stop)"));
            sSeq.stop();
        }
        if (manualActive) {
            gWcb.clearEstop(); // operator stick input is unambiguous human intent
            estop = false;
        }
    }

    // One-shot diagnostic when manual input engages: phantom "manual" (a
    // floating PWM pin picking up motor noise) has cancelled sequences and
    // silently killed moves before — make the culprit and its decoded value
    // visible in any bench log. Rate-limited so a noise storm can't spam.
    {
        static bool sManWas = false;
        static uint32_t sManLogAt = 0;
        if (manualActive && !sManWas && now - sManLogAt > 2000) {
            sManLogAt = now;
            Serial.printf("[MAN] manual input engaged: %d%% (%s)\n", manualPct,
                          sPwm.manualPercent(now) != 0 ? "PWM" : "Syren serial");
        }
        sManWas = manualActive;
    }

    // #DPAUTORESTART0: manual input DISARMS the self-running modes outright
    // (until re-enabled) instead of merely pausing them for #DPIDLE.
    {
        static bool sWasManual = false;
        if (manualActive && !sWasManual && !sSettings.autoRestart &&
            (sSettings.autoMode || sSettings.homeMode)) {
            sSettings.autoMode = false;
            sSettings.homeMode = false;
            sExec.applyTuning();
            Serial.println(F("AUTOMATION OFF (manual override, #DPAUTORESTART0)"));
        }
        sWasManual = manualActive;
    }

    sExec.pump(now, manualActive, Serial);

    MotionController::Inputs in;
    in.now = now;
    in.sensorValid = sSensor.valid();
    in.position = sSensor.position();
    in.sampleCount = sSensor.stats().accepted;
    in.jumped = sSensor.consumeJump();
    in.manualActive = manualActive;
    in.estop = estop;
    in.suppressAutomation = sSeq.active(); // no idle automation mid-sequence (W steps)
    int8_t autoPct = sMotion.tick(in);

    // Mesh fault telemetry: BEHAVIOR §6 promises &RAD,FAULT on faults — motion
    // faults included, not just SENSOR_STALE. Watch transitions right after
    // tick() (pump() clears consumed faults next loop).
    {
        static MotionController::Fault sLastFault = MotionController::Fault::kNone;
        MotionController::Fault f = sMotion.fault();
        if (f != sLastFault && f != MotionController::Fault::kNone)
            gWcb.sendFault(f == MotionController::Fault::kTimeout      ? "TIMEOUT"
                           : f == MotionController::Fault::kSensorLost ? "SENSOR_LOST"
                                                                       : "JUMP");
        sLastFault = f;
    }

    // Manual always wins; automation output is only used when manual is neutral.
    int8_t wire;
    if (estop) {
        wire = 0;
    } else if (manualActive) {
        wire = sSettings.inverted ? static_cast<int8_t>(-manualPct) : manualPct;
    } else {
        int8_t dir = sDirSign != 0 ? sDirSign : (sSettings.inverted ? -1 : 1);
        wire = static_cast<int8_t>(autoPct * dir);
    }
    driveMotor(wire, now);
    // Tell the sensor tracker whether a move is in progress. "Active" means the
    // controller is targeting/spinning or the operator is driving manually — NOT
    // merely that the motor is energised, because a target move sits at wire==0
    // while it settles inside the arrival arc. Only when the controller is idle
    // does the tracker hold position and reject encoder misreads (see
    // SensorRing::noteActive). Gating on wire==0 instead would freeze a settling
    // move at a value a flickering encoder never re-reports, hanging arrival.
    bool controlActive = manualActive || sMotion.state() != MotionController::State::kIdle;
    // Also hand the tracker how hard we are actually driving (|wire|, 0..100). That
    // arms its motor-plausibility guard: a reported position change the commanded
    // drive could not physically have produced is a sensor lie, not motion. In
    // particular a move holding station in its arrival arc commands 0%, so the
    // ~299/304 alias and the coarse-arc ±35° wander are rejected before they can
    // kick the controller back out of the arc and restart the motor-pulse hunt.
    uint8_t driveMag = static_cast<uint8_t>(wire < 0 ? -wire : wire);
    sSensor.noteActive(controlActive, now, driveMag);
    learnPolarity(now, manualActive);

    // --- telemetry ------------------------------------------------------------
    // Live console debug (#DPDEBUG1), ~4 Hz.
    if (sStats.debug) {
        static uint32_t sNextDbg = 0;
        if ((int32_t)(now - sNextDbg) >= 0) {
            sNextDbg = now + 250;
            const char* st = "idle";
            if (sMotion.state() == MotionController::State::kTarget)
                st = "target";
            else if (sMotion.state() == MotionController::State::kSpin)
                st = "spin";
            Serial.printf(
                "[DBG] pos=%d tgt=%d st=%s auto=%d man=%d wire=%d dir=%d sensor=%s rej=%lu "
                "jmp=%lu fault=%s seq=%d wcb=%s estop=%d\n",
                sSensor.valid() ? sSensor.position() : -1, sMotion.target(), st, autoPct,
                manualPct, sWirePct, sDirSign,
                sSensor.valid() ? "OK"
                                : (sSensor.state() == SensorRing::State::kStale ? "STALE" : "WARM"),
                (unsigned long)sSensor.stats().rejectedRate, (unsigned long)sSensor.stats().jumps,
                MotionController::faultName(sMotion.fault()), sSeq.active() ? 1 : 0,
                gWcb.active() ? (gWcb.sabeOnline() ? "sabe" : "up") : "off",
                gWcb.estopLatched() ? 1 : 0);
        }
    }

    reportPosition();
    reportConsole(now);
    {
        static bool sWasValid = false;
        if (sSensor.valid()) {
            gWcb.sendPosition(sSensor.position(), modeChar(), now);
            sWasValid = true;
        } else if (sWasValid && sSensor.state() == SensorRing::State::kStale) {
            gWcb.sendFault("SENSOR_STALE");
            sWasValid = false;
        }
    }
    {
        const char* state = gWcb.estopLatched() ? "ESTOP"
                            : sSeq.active()     ? "SEQ"
                            : manualActive      ? "MANUAL"
                                                : "IDLE";
        gWcb.sendHeartbeat(now, state);
    }
#ifdef RAD_USE_DISPLAY
    // Anything the droid is actively doing keeps the backlight up; the screen
    // only sleeps once the dome is parked and nobody is driving it.
    bool busy = manualActive || sMotion.state() != MotionController::State::kIdle || sSeq.active();
    if (busy)
        sDisplay.noteActivity(now);
    sDisplay.setSleepTimeout(sSettings.lcdSleepSec); // live #DPLCDSLEEP changes
    sDisplay.update(now, sSensor.valid(), sSensor.position(),
                    sMotion.state() != MotionController::State::kIdle);
#endif

    delay(1);
}
