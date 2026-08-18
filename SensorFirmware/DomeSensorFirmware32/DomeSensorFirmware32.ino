// DomeSensorFirmware32.ino — Roam-A-Dome (RaD) hardened build.
//
// Source: https://github.com/reeltwo/DomeSensorFirmware32  (LGPL-2.1; see LICENSE).
// This is the upstream sketch with four small Roam-A-Dome edits, each tagged
// "RaD" inline:
//   1. #include "PositionDebounce.h"
//   2. a global  static PositionDebounce sDebounce;
//   3. loop(): the reported angle now comes from the DEBOUNCED RAW sensor code
//      (readSensors() + getDomeAngle(), held in sHeldAngle) instead of the
//      median-after-decode sDomePosition.getAngle().
//   4. the report is gated on  angle >= 0  (debounce locked) instead of ready().
// This removes transition-aliased / stuck-code misreads at the source without
// touching the pin map or the getDomeAngle() decode table. Build target: ESP32
// (esp32:esp32:esp32), matching upstream .auto. See SensorFirmware/README.md.
//
#define USE_DEBUG
#undef USE_DOME_SENSOR_DEBUG
#include "ReelTwo.h"
#include "core/AnimatedEvent.h"
#include "drive/DomeSensorRing.h"
#include "core/StringUtils.h"
#include "PositionDebounce.h"  // RaD: raw-code debounce (see README)
#ifdef ESP32
#define USE_PREFERENCES
#endif
#ifdef USE_PREFERENCES
#include <Preferences.h>
#endif

///////////////////////////////////
// CONFIGURABLE OPTIONS
///////////////////////////////////

#define SERIAL_BAUD_RATE                    115200
#define CONSOLE_BUFFER_SIZE                 300
#define COMMAND_BUFFER_SIZE                 256

#define POSITION_RESEND_INTERVAL            1000  // milliseconds
#define TEST_DURATION_SECONDS               60

///////////////////////////////////

#if defined(ARDUINO_ARCH_RP2040)
 #define TXD1_PIN    5
 #define RXD1_PIN    20   /* not used */
 #define REPORT_SERIAL Serial2
 #define REPORT_SERIAL_SETUP(baud,rx,tx) {\
    REPORT_SERIAL.begin(baud); \
 }
#elif defined(ESP32)
 #ifdef PIN_NEOPIXEL
  #define TXD1_PIN    32
  #define RXD1_PIN    7   /* not used */
 #else
  #define TXD1_PIN    17
  #define RXD1_PIN    16  /* not used */
 #endif
 #define REPORT_SERIAL Serial1
 #define REPORT_SERIAL_SETUP(baud,rx,tx) REPORT_SERIAL.begin(baud, SERIAL_8N1, RXD1_PIN, TXD1_PIN);
#else
#error Unsupported board
#endif

///////////////////////////////////

struct DomeSensorSettings
{
    uint32_t fBaudRate = SERIAL_BAUD_RATE;
};

DomeSensorSettings sSettings;

#ifdef USE_PREFERENCES
Preferences sPreferences;
#define PREFERENCE_KEY  "settings"
#endif

///////////////////////////////////

static bool sNextCommand;
static bool sProcessing;
static unsigned sPos;
static uint32_t sWaitNextSerialCommand;
static char sBuffer[CONSOLE_BUFFER_SIZE];
static bool sCmdNextCommand;
static char sCmdBuffer[COMMAND_BUFFER_SIZE];
static uint32_t sEndTesting;
static bool sTestOne;
static unsigned sSensorMask;
static bool sVerbose;
static bool sDebug;
static PositionDebounce sDebounce;  // RaD: hardens the reported angle

///////////////////////////////////

DomeSensorRing sDomePosition;
#ifdef PIN_NEOPIXEL
#include "core/SingleStatusLED.h"
enum {
    kNormalModeHome = 0,
    kNormalModeMoving = 1,
};
unsigned sCurrentMode = kNormalModeHome;
static constexpr uint8_t kStatusColors[][4][3] = {
      { {  0,   2,    0} , {   0,    2,    0} , {  0,   2,    0} , {   0,    2,    0}  },  // normal mode home (all green)
      { {  0,   2,    0} , {   2,    2,    0} , {  0,   2,    0} , {   2,    2,    0}  },  // normal mode moving (green,yellow,green,yellow)
};
SingleStatusLED<PIN_NEOPIXEL> statusLED(kStatusColors, SizeOfArray(kStatusColors));
#endif

///////////////////////////////////

void setup()
{
    REELTWO_READY();
#ifndef USE_DEBUG
    Serial.begin(DEFAULT_BAUD_RATE);
#endif
#ifdef USE_PREFERENCES
    if (sPreferences.begin("domesensor"))
    {
        DomeSensorSettings settings;
        if (sPreferences.getBytes(PREFERENCE_KEY, &settings, sizeof(settings)) == sizeof(settings))
        {
            Serial.println("Settings Restored");
            sSettings = settings;
        }
        else
        {
            Serial.println("First Time Settings");
            sPreferences.putBytes(PREFERENCE_KEY, &sSettings, sizeof(sSettings));
            if (sPreferences.getBytes(PREFERENCE_KEY, &settings, sizeof(settings)) == sizeof(settings))
            {
                Serial.println("Readback Success");
            }
        }
    }
    else
    {
        Serial.println("Failed to initialize preferences.");
    }
#endif
#ifdef REPORT_SERIAL_SETUP
    REPORT_SERIAL_SETUP(sSettings.fBaudRate, RXD1_PIN, TXD1_PIN)
#endif

    SetupEvent::ready();
#ifdef PIN_NEOPIXEL
    statusLED.setMode(kNormalModeHome);
    statusLED.setDelay(300); // Fast blink
#endif

#if defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
}

///////////////////////////////////

void reboot()
{
    Serial.println("Restarting...");
#ifdef ESP32
  #ifdef USE_PREFERENCES
    sPreferences.end();
  #endif
    ESP.restart();
#elif defined(REELTWO_AVR)
    void (*resetArduino)() = NULL;
    resetArduino();
#else
    Serial.println("Restart not supported.");
#endif
}

///////////////////////////////////

static void updateSettings()
{
#ifdef USE_PREFERENCES
    sPreferences.putBytes(PREFERENCE_KEY, &sSettings, sizeof(sSettings));
#endif
    Serial.println("Updated");
}

///////////////////////////////////

static void runSerialCommand()
{
    sWaitNextSerialCommand = 0;
    sProcessing = true;
}

static void resetSerialCommand()
{
    sWaitNextSerialCommand = 0;
    sNextCommand = false;
    sProcessing = (sCmdBuffer[0] == ':');
    sPos = 0;
}

bool processDomeRingCommand(const char* cmd)
{
    switch (*cmd++)
    {
        default:
            // No dome ring commands
            break;
    }
    return true;
}

void processConfigureCommand(const char* cmd)
{
    if (startswith(cmd, "#DPZERO"))
    {
        DomeSensorSettings defaultSettings;
        sSettings = defaultSettings;
        updateSettings();
    }
    else if (startswith(cmd, "#DPDEBUG"))
    {
        bool debug = (*cmd == '1');
        if (debug != sDebug)
        {
            if (debug)
            {
                Serial.println("Debug mode enabled.");
            }
            else
            {
                Serial.println("Debug mode disabled.");
            }
            sDebug = debug;
        }
    }
    else if (startswith(cmd, "#DPVERBOSE"))
    {
        bool verbose = (*cmd == '1');
        if (verbose != sVerbose)
        {
            if (verbose)
            {
                Serial.println("Verbose mode enabled.");
            }
            else
            {
                Serial.println("Verbose mode disabled.");
            }
            sVerbose = verbose;
        }
    }
    else if (startswith(cmd, "#DPCONFIG"))
    {
        Serial.print("BaudRate="); Serial.println(sSettings.fBaudRate);
    }
    else if (startswith(cmd, "#DPTEST"))
    {
        sTestOne = (*cmd == '1');
        if (sTestOne)
        {
            Serial.print("Test one sensor at a time. ");
        }
        else
        {
            Serial.print("Testing. Rotate dome 360 degrees. ");
        }
        Serial.println("Test ends in "+String(TEST_DURATION_SECONDS)+" seconds or when all sensors have registered.");
        sEndTesting = millis() + TEST_DURATION_SECONDS * 1000L;
        sSensorMask = 0;
    }
    else if (startswith(cmd, "#DPBAUD"))
    {
        uint32_t baudrate = strtolu(cmd, &cmd);
        if (baudrate > 1200 && sSettings.fBaudRate != baudrate)
        {
            sSettings.fBaudRate = baudrate;
            Serial.print("Reboot baud rate: "); Serial.println(sSettings.fBaudRate);
            updateSettings();
        }
    }
    else if (startswith(cmd, "#DPRESTART"))
    {
        reboot();
    }
}

bool processCommand(const char* cmd, bool firstCommand)
{
    sWaitNextSerialCommand = 0;
    if (*cmd == '\0')
        return true;
    if (!firstCommand)
    {
        if (cmd[0] != ':')
        {
            Serial.println("Invalid");
            return false;
        }
        return processDomeRingCommand(cmd+1);
    }
    switch (cmd[0])
    {
        case ':':
            if (cmd[1] == 'D')
                return processDomeRingCommand(cmd+2);
            break;
        case '#':
            processConfigureCommand(cmd);
            return true;
        default:
            Serial.println("Invalid");
            break;
    }
    return false;
}

///////////////////////////////////

static unsigned countChangedBits(unsigned a, unsigned b)
{
    unsigned n = 0;
    for (unsigned i = 0; i < 9; i++) 
    {
        if ((a & (1 << i)) != (b & (1 << i)))
            n++;
    }
    return n;
}

static void printBinary(unsigned num, unsigned places)
{
    if (places)
        printBinary(num >> 1, places-1);
    Serial.print((num & 1) ? 'B' : '-');
    Serial.print(' ');
}

void loop()
{
    AnimatedEvent::process();

#ifdef PIN_NEOPIXEL
    static uint32_t sBlinkOut;
#endif
    static short sLastAngle = -1;
    static uint32_t sLastReport = 0;
    // RaD hardening: debounce the RAW sensor code before trusting it. getAngle()
    // medians AFTER decode, which cannot undo a transition-aliased or stuck code;
    // the raw code must simply hold steady for a few reads. sHeldAngle keeps the
    // last good value, so a burst of invalid/transition codes holds position
    // instead of reporting a phantom. (-1 until the first lock.)
    unsigned code = sDomePosition.readSensors();
    int out;
    static short sHeldAngle = -1;
    if (sDebounce.update(code, sDomePosition.getDomeAngle(code), millis(), out))
        sHeldAngle = out;
    short angle = sHeldAngle;
    if (angle >= 0)  // RaD: report only once the debounce has locked a valid angle
    {
        // Output the position if it changed or one second has passed
        if (angle != sLastAngle || millis() - sLastReport > POSITION_RESEND_INTERVAL)
        {
            char buf[20];
            snprintf(buf, sizeof(buf), "#DP@%d", angle);
        #ifdef REPORT_SERIAL
            REPORT_SERIAL.println(buf);
        #endif
            if (sDebug)
            {
                auto sensors = sDomePosition.readSensors();
                Serial.print("POS: ");
                auto domeAngle = sDomePosition.getDomeAngle(sensors);
                if (domeAngle > 0)
                {
                    Serial.print(domeAngle);
                    Serial.print((domeAngle < 10) ? "   " : (domeAngle < 100) ? "  " : " ");
                }
                else
                {
                    Serial.print("--- ");
                }
                printBinary(sensors, 8);
                Serial.print(": ");
                Serial.println(angle);
            }
            else if (sVerbose)
            {
                Serial.print("POS: ");
                Serial.println(angle);
            }
        #ifdef PIN_NEOPIXEL
            if (angle != sLastAngle)
            {
                statusLED.setMode(kNormalModeMoving);
                sBlinkOut = millis() + POSITION_RESEND_INTERVAL;
            }
        #endif
            sLastAngle = angle;
            sLastReport = millis();
        }
        else
        {
        #ifdef PIN_NEOPIXEL
            if (sBlinkOut < millis())
            {
                statusLED.setMode(kNormalModeHome);
            }
        #endif
        }
    }

    if (sEndTesting > 0)
    {
        unsigned sensors = (~sDomePosition.readSensors() & 0x1FF);
        sSensorMask |= sensors;
        if (sTestOne && countChangedBits(0, sensors) > 1)
        {
            Serial.print("Test failed. These sensors are reading at the same time: ");
            sEndTesting = 0;
            for (unsigned i = 0; i < 9; i++)
            {
                if ((sensors & (1<<i)) != 0)
                {
                    Serial.print(i+1);
                    Serial.print(" ");
                }
            }
            Serial.println();
        }
        if (sSensorMask == 0x1FF)
        {
            Serial.println("All sensors registered. Ending test.");
            sEndTesting = 0;
        }
        else if (sEndTesting < millis())
        {
            Serial.print("Test timeout. Following sensors did not register: ");
            for (unsigned i = 0; i < 9; i++)
            {
                if ((sSensorMask & (1<<i)) == 0)
                {
                    Serial.print(i+1);
                    Serial.print(" ");
                }
            }
            Serial.println();
            sEndTesting = 0;
        }
    }
    // append commands to command buffer
    if (Serial.available())
    {
        int ch = Serial.read();
        if (ch == '\r' || ch == '\n')
        {
            runSerialCommand();
        }
        else if (sPos < SizeOfArray(sBuffer)-1)
        {
            sBuffer[sPos++] = ch;
            sBuffer[sPos] = '\0';
        }
    }
    if (sProcessing && millis() > sWaitNextSerialCommand)
    {
        if (sCmdBuffer[0] == ':')
        {
            char* end = strchr(sCmdBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sCmdBuffer, !sCmdNextCommand))
            {
                // command invalid abort buffer
                Serial.print("Unrecognized: "); Serial.println(sCmdBuffer);
                sWaitNextSerialCommand = 0;
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sCmdBuffer, end);
                DEBUG_PRINT("REMAINS: ");
                DEBUG_PRINTLN(sCmdBuffer);
                sCmdNextCommand = true;
            }
            else
            {
                sCmdBuffer[0] = '\0';
                sCmdNextCommand = false;
            }
        }
        else if (sBuffer[0] == ':')
        {
            char* end = strchr(sBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sBuffer, !sNextCommand))
            {
                // command invalid abort buffer
                Serial.print("Unrecognized: "); Serial.println(sBuffer);
                sWaitNextSerialCommand = 0;
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sBuffer, end);
                sPos = strlen(sBuffer);
                DEBUG_PRINT("REMAINS: ");
                DEBUG_PRINTLN(sBuffer);
                sNextCommand = true;
            }
            else
            {
                resetSerialCommand();
                sBuffer[0] = '\0';
            }
        }
        else
        {
            processCommand(sBuffer, true);
            resetSerialCommand();
        }
    }
}
