// Roam-A-Dome v2 — Phase 1: transparent passthrough + config surface.
// See ../BEHAVIOR.md for the observable contract this firmware implements.

#include "pinmap.h"
#include "src/CommandExec.h"
#include "src/LineAssembler.h"
#include "src/PwmIO.h"
#include "src/RadVersion.h"
#include "src/Settings.h"
#include "src/SyrenBus.h"

using namespace rad;

// UART assignment (see pinmap.h): UART1 = sensor ring (Phase 2), UART2 = Syren
// in/out, UART0 = command serial. Console is USB (native CDC on S3).
HardwareSerial& syrenSerial = Serial2;
#ifdef RAD_BOARD_DISPLAY
HardwareSerial cmdSerial(0);
#else
HardwareSerial& cmdSerial = Serial1; // classic ESP32: sensor gets UART1 in Phase 2 via pin remap
#endif

static RadSettings sSettings;
static RadSettingsStore sStore;
static RuntimeStats sStats;
static CommandExec sExec;
static SyrenBus sSyren;
static PwmIO sPwm;
static LineAssembler sConsoleLine;
static LineAssembler sCmdLine;

void setup() {
    Serial.begin(115200);

    bool loaded = sStore.load(sSettings);

    syrenSerial.begin(sSettings.syrenBaud, SERIAL_8N1, RAD_PIN_SYREN_IN_RX, RAD_PIN_SYREN_OUT_TX);
    cmdSerial.begin(sSettings.serialBaud, SERIAL_8N1, RAD_PIN_CMD_RX, RAD_PIN_CMD_TX);

    sExec.begin(&sSettings, &sStore, &sStats);
    sSyren.begin(syrenSerial, sSettings);
    sPwm.begin(RAD_PIN_PWM_IN, RAD_PIN_PWM_OUT, sSettings);

    Serial.printf("\nRoam-A-Dome v2 %s (%s)\n", RAD_FW_VERSION,
                  loaded ? "settings loaded" : "fresh defaults");
    Serial.println("Passthrough active. #DPCONFIG for settings.");
}

void loop() {
    uint32_t now = millis();

    sSyren.pump(now);
    sPwm.pump(now);
    sStats.syrenChecksumErrors = sSyren.checksumErrors();

    // Console: drain everything available this pass (legacy read one byte per loop).
    while (Serial.available() > 0) {
        if (const char* line = sConsoleLine.feed(static_cast<char>(Serial.read()))) {
            ++sStats.linesConsole;
            sExec.handleLine(line, Serial);
        }
    }
    if (sSettings.serialCmdIn) {
        while (cmdSerial.available() > 0) {
            if (const char* line = sCmdLine.feed(static_cast<char>(cmdSerial.read()))) {
                ++sStats.linesCmdSerial;
                sExec.handleLine(line, cmdSerial);
            }
        }
    }
    sStats.lineOverflows = sConsoleLine.overflows() + sCmdLine.overflows();

    delay(1);
}
