// Roam-A-Dome v2 — Phase 3: motion control + sequencer on the validated pipeline.
// See ../BEHAVIOR.md for the observable contract this firmware implements.

#include "pinmap.h"
#include "src/CommandExec.h"
#include "src/LineAssembler.h"
#include "src/MotionController.h"
#include "src/PinBank.h"
#include "src/PwmIO.h"
#include "src/RadVersion.h"
#include "src/SensorRing.h"
#include "src/SeqStore.h"
#include "src/Sequencer.h"
#include "src/Settings.h"
#include "src/SyrenBus.h"

#include <esp_random.h>

using namespace rad;

// UART assignment (see pinmap.h): UART1 = sensor ring, UART2 = Syren in/out.
// Display board (S3): console is native USB CDC, so UART0 serves the command port.
// Compact board (classic ESP32): UART0 is the USB bridge console; the command
// port would need a software serial there — gated off until such a board shows up.
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
static CommandExec sExec;
static SyrenBus sSyren;
static PwmIO sPwm;
static LineAssembler sConsoleLine;
#ifdef RAD_HAS_CMD_SERIAL
static LineAssembler sCmdLine;
#endif

void setup() {
    Serial.begin(115200);

    bool loaded = sStore.load(sSettings);

    syrenSerial.begin(sSettings.syrenBaud, SERIAL_8N1, RAD_PIN_SYREN_IN_RX, RAD_PIN_SYREN_OUT_TX);
    sensorSerial.begin(sSettings.sensorBaud, SERIAL_8N1, RAD_PIN_SENSOR_RX, /*tx*/ -1);
#ifdef RAD_HAS_CMD_SERIAL
    cmdSerial.begin(sSettings.serialBaud, SERIAL_8N1, RAD_PIN_CMD_RX, RAD_PIN_CMD_TX);
#endif

    static const int kDoutPins[] = RAD_PIN_DOUT;
    sPins.begin(kDoutPins, sizeof(kDoutPins) / sizeof(kDoutPins[0]), sSettings.digitalPins);
    sExec.begin(&sSettings, &sStore, &sStats, &sSensor, &sMotion, &sSeq, &sSeqStore, &sPins);
    sSyren.begin(syrenSerial, sSettings);
    sPwm.begin(RAD_PIN_PWM_IN, RAD_PIN_PWM_OUT, sSettings);

    Serial.printf("\nRoam-A-Dome v2 %s (%s)\n", RAD_FW_VERSION,
                  loaded ? "settings loaded" : "fresh defaults");
    Serial.println("#DPCONFIG settings, #DPSTATUS state, :DPA<deg> to move.");
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
static int8_t sWirePct = 0;   // last wire-level command actually sent
static int8_t sDirSign = 0;   // learned wire->degrees sign; 0 = not yet learned

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

// Learn drive polarity from manual motion: correlate the wire command with the
// validated sensor delta. ±10 degrees of consistent evidence locks it in;
// contradicting evidence can re-flip it later (belt slip, rewiring).
static void learnPolarity() {
    static uint32_t sLastAccepted = 0;
    static int32_t sAccum = 0;
    uint32_t accepted = sSensor.stats().accepted;
    if (accepted == sLastAccepted || !sSensor.valid())
        return;
    sLastAccepted = accepted;
    if (sWirePct >= -25 && sWirePct <= 25)
        return; // too gentle to attribute motion confidently
    int16_t d = sSensor.lastDelta();
    sAccum += (sWirePct > 0) ? d : -d;
    if (sAccum >= 10) {
        sAccum = 10;
        if (sDirSign != 1) {
            sDirSign = 1;
            Serial.println(F("[DIR] learned: positive wire increases degrees"));
        }
    } else if (sAccum <= -10) {
        sAccum = -10;
        if (sDirSign != -1) {
            sDirSign = -1;
            Serial.println(F("[DIR] learned: positive wire decreases degrees"));
        }
    }
}

// Position report out the command serial on change: "#DP<mode><pos>" where mode is
// '@' off, '!' home-seek, '$' random, '%' target (BEHAVIOR.md §6).
static void reportPosition() {
#ifdef RAD_HAS_CMD_SERIAL
    static int16_t sLastReported = -1;
    if (!sSensor.valid())
        return;
    int16_t pos = sSensor.position();
    if (pos == sLastReported)
        return;
    sLastReported = pos;
    char mode = '@';
    if (sMotion.state() == MotionController::State::kTarget)
        mode = sMotion.target() == sMotion.tuning.homePos ? '!' : '%';
    else if (sMotion.tuning.autoMode)
        mode = '$';
    char buf[16];
    snprintf(buf, sizeof(buf), "#DP%c%d", mode, pos);
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
            Serial.println(sSensor.position());
        } else {
            Serial.println(F("DOME POSITION: ---"));
        }
    }
}

void loop() {
    uint32_t now = millis();

    // --- inputs ---------------------------------------------------------------
    sSyren.pump(now); // decodes frames; raw passthrough only if #DPSERIALIN1
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
            sExec.handleLine(line, Serial);
        }
    }
#ifdef RAD_HAS_CMD_SERIAL
    if (sSettings.serialCmdIn) {
        while (cmdSerial.available() > 0) {
            if (const char* line = sCmdLine.feed(static_cast<char>(cmdSerial.read()))) {
                ++sStats.linesCmdSerial;
                sExec.handleLine(line, cmdSerial);
            }
        }
    }
    sStats.lineOverflows = sConsoleLine.overflows() + sCmdLine.overflows();
#else
    sStats.lineOverflows = sConsoleLine.overflows();
#endif

    // --- sequencer + motion (arbitration ladder, BEHAVIOR.md §5) --------------
    sExec.pump(now, manualActive, Serial);

    MotionController::Inputs in;
    in.now = now;
    in.sensorValid = sSensor.valid();
    in.position = sSensor.position();
    in.sampleCount = sSensor.stats().accepted;
    in.jumped = sSensor.consumeJump();
    in.manualActive = manualActive;
    in.estop = false; // WCB ?STOP latch lands in Phase 5
    int8_t autoPct = sMotion.tick(in);

    // Manual always wins; automation output is only used when manual is neutral.
    int8_t wire;
    if (manualActive) {
        wire = sSettings.inverted ? static_cast<int8_t>(-manualPct) : manualPct;
    } else {
        int8_t dir = sDirSign != 0 ? sDirSign : (sSettings.inverted ? -1 : 1);
        wire = static_cast<int8_t>(autoPct * dir);
    }
    driveMotor(wire, now);
    learnPolarity();

    // Live telemetry (#DPDEBUG1), ~4 Hz.
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
                "jmp=%lu fault=%s seq=%d\n",
                sSensor.valid() ? sSensor.position() : -1, sMotion.target(), st, autoPct,
                manualPct, sWirePct, sDirSign,
                sSensor.valid() ? "OK"
                                : (sSensor.state() == SensorRing::State::kStale ? "STALE" : "WARM"),
                (unsigned long)sSensor.stats().rejectedRate, (unsigned long)sSensor.stats().jumps,
                MotionController::faultName(sMotion.fault()), sSeq.active() ? 1 : 0);
        }
    }

    reportPosition();
    reportConsole(now);

    delay(1);
}
