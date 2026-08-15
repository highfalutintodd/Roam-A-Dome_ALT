// Roam-A-Dome v2 — Phase 2: passthrough + validated sensor pipeline.
// See ../BEHAVIOR.md for the observable contract this firmware implements.

#include "pinmap.h"
#include "src/CommandExec.h"
#include "src/LineAssembler.h"
#include "src/PwmIO.h"
#include "src/RadVersion.h"
#include "src/SensorRing.h"
#include "src/Settings.h"
#include "src/SyrenBus.h"

using namespace rad;

// UART assignment (see pinmap.h): UART1 = sensor ring, UART2 = Syren in/out.
// Display board (S3): console is native USB CDC, so UART0 serves the command port.
// Compact board (classic ESP32): UART0 is the USB bridge console; the command
// port needs a software serial there — deferred until the bench session confirms
// which board this actually is (BENCH.md §1).
HardwareSerial& syrenSerial = Serial2;
HardwareSerial& sensorSerial = Serial1;
#ifdef RAD_BOARD_DISPLAY
HardwareSerial cmdSerial(0);
#define RAD_HAS_CMD_SERIAL 1
#endif

static RadSettings sSettings;
static RadSettingsStore sStore;
static RuntimeStats sStats;
static CommandExec sExec;
static SyrenBus sSyren;
static PwmIO sPwm;
static SensorRing sSensor;
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

    sExec.begin(&sSettings, &sStore, &sStats, &sSensor);
    sSyren.begin(syrenSerial, sSettings);
    sPwm.begin(RAD_PIN_PWM_IN, RAD_PIN_PWM_OUT, sSettings);

    Serial.printf("\nRoam-A-Dome v2 %s (%s)\n", RAD_FW_VERSION,
                  loaded ? "settings loaded" : "fresh defaults");
    Serial.println("Passthrough active. #DPCONFIG for settings, #DPSTATUS for sensor.");
}

// Position report out the command serial on change: "#DP<mode><pos>" where mode is
// '@' off / '!' home / '$' random / '%' target (BEHAVIOR.md §6). Automation modes
// arrive in Phase 3; until then mode is always '@'.
static void reportPosition(uint32_t now) {
    (void)now;
#ifdef RAD_HAS_CMD_SERIAL
    static int16_t sLastReported = -1;
    if (!sSensor.valid())
        return;
    int16_t pos = sSensor.position();
    if (pos == sLastReported)
        return;
    sLastReported = pos;
    char buf[16];
    snprintf(buf, sizeof(buf), "#DP@%d", pos);
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

    sSyren.pump(now);
    sPwm.pump(now);
    sStats.syrenChecksumErrors = sSyren.checksumErrors();

    while (sensorSerial.available() > 0)
        sSensor.feed(static_cast<uint8_t>(sensorSerial.read()), now);
    sSensor.tick(now);

    reportPosition(now);
    reportConsole(now);

    // Console: drain everything available this pass (legacy read one byte per loop).
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

    delay(1);
}
