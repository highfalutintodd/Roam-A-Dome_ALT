// Versioned settings. The struct and defaults are pure C++ (host-testable);
// persistence lives behind RadSettingsStore (NVS Preferences, Arduino-only).
#pragma once

#include <cstdint>

namespace rad {

// RULE: bump this on ANY change to RadSettings — even one that doesn't change
// sizeof. Alignment padding can absorb an inserted field, so the size check
// alone cannot detect a layout shift (field-shifted garbage loaded fine once:
// old wcbOct2 0x3C read back as "WCBEN=60").
constexpr uint16_t kSettingsVersion = 2;

// Defaults are seeded from the live droid's captured legacy config
// (docs/capture/config.txt, bench session 2) so a fresh v2 flash behaves like
// Todd's droid, not a generic build.
struct RadSettings {
    // Serial / transports
    uint32_t serialBaud = 9600;    // #DPSERIALBAUD (command serial)
    uint32_t syrenBaud = 9600;     // #DPSYRENBAUD
    uint32_t sensorBaud = 115200;  // #DPSENSORBAUD (57600 or 115200)
    uint8_t syrenAddrIn = 129;     // #DPSYRENADDRIN
    uint8_t syrenAddrOut = 129;    // #DPSYRENADDROUT
    bool serialIn = false;         // #DPSERIALIN (captured: 0 — no body controller on packet serial)
    bool serialOut = true;         // #DPSERIALOUT (Syren packet-serial output enable)
    bool serialCmdIn = true;       // #DPSERIALCMD (command-serial ingress; new in v2)
    bool pwmIn = true;             // #DPPWMIN (captured: 1 — Sabé's dome PWM is the manual input)
    bool pwmOut = false;           // #DPPWMOUT
    uint16_t pwmMinUs = 1000;      // #DPPWMMIN
    uint16_t pwmMaxUs = 2000;      // #DPPWMMAX
    uint16_t pwmNeutralUs = 1500;  // #DPPWMNEUTRAL
    uint8_t pwmDeadbandPct = 5;    // #DPPWMDEADBAND
    uint16_t reportMs = 0;         // #DPREPORT (0 = off)

    // Motion (captured values)
    uint8_t maxSpeed = 100;        // #DPMAXSPEED
    uint8_t minSpeed = 15;         // #DPMINSPEED
    uint8_t homeSpeed = 40;        // #DPHOMESPEED
    uint8_t autoSpeed = 30;        // #DPAUTOSPEED
    uint8_t targetSpeed = 100;     // #DPTARGETSPEED
    uint8_t inputSpeed = 100;      // #DPINPUTSPEED (manual passthrough scale)
    uint8_t fudge = 5;             // #DPFUDGE (arrival tolerance, degrees)
    bool scaling = false;          // #DPSCALE (speed ramping)
    uint8_t accScale = 20;         // #DPASCALE
    uint8_t decScale = 50;         // #DPDSCALE (deceleration zone, degrees)
    bool inverted = true;          // #DPINVERT (captured: 1)
    uint8_t timeoutSec = 5;        // #DPTIMEOUT (stuck-dome watchdog)
    bool autoSafety = true;        // #DPAUTOSAFETY
    bool autoRestart = true;       // #DPAUTORESTART
    bool homeMode = false;         // #DPHOME
    bool autoMode = false;         // #DPAUTO
    uint8_t autoLeft = 47;         // #DPAUTOLEFT (captured: 47)
    uint8_t autoRight = 46;        // #DPAUTORIGHT (captured: 46)
    uint16_t autoMinS = 6;         // #DPAUTOMIN
    uint16_t autoMaxS = 8;         // #DPAUTOMAX
    uint16_t homeMinS = 6;         // #DPHOMEMIN
    uint16_t homeMaxS = 8;         // #DPHOMEMAX
    uint16_t targetMinS = 0;       // #DPTARGETMIN
    uint16_t targetMaxS = 1;       // #DPTARGETMAX
    int16_t homePos = 240;         // #DPHOMEPOS (captured: 240)
    uint8_t digitalPins = 0;       // #DPPIN defaults (bitmask, captured: 00000000)

    // Sensor validation + arbitration (new in v2)
    uint8_t maxRpm = 60;           // #DPMAXRPM (dome measured ~41 RPM at full speed)
    uint16_t sensToMs = 2500;      // #DPSENSTO
    uint8_t sensN = 3;             // #DPSENSN
    uint8_t dwell = 3;             // #DPDWELL
    uint16_t idleMs = 3000;        // #DPIDLE (manual-neutral time before automation)

    uint16_t dedupMs = 750;        // #DPDEDUP: cross-transport duplicate window (0 = off)

    // WCB mesh (Phase 5; stored from day one so bench config survives reflashes)
    bool wcbEnabled = true;        // #DPWCBEN
    uint8_t wcbDeviceId = 4;       // #DPWCBID
    uint8_t wcbOct2 = 0x3C;        // #DPWCBOCT
    uint8_t wcbOct3 = 0x4E;
    uint8_t wcbQuantity = 3;       // #DPWCBQTY
    uint8_t wcbChannel = 1;        // #DPWCBCH
    bool wcbChecksum = true;       // #DPWCBCS
    char wcbPassword[33] = "";     // #DPWCBPW
};

#ifdef ARDUINO
class RadSettingsStore {
  public:
    // Loads into `out`. Unknown/absent/newer version -> defaults, returns false
    // (never silently wipes stored data; see BEHAVIOR.md D8).
    bool load(RadSettings& out);
    void save(const RadSettings& s);
    void clear(); // #DPZERO / #DPFACTORY
};
#endif

} // namespace rad
