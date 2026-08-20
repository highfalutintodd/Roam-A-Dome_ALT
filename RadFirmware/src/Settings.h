// Versioned settings. The struct and defaults are pure C++ (host-testable);
// persistence lives behind RadSettingsStore (NVS Preferences, Arduino-only).
#pragma once

#include <cstdint>

namespace rad {

// RULE: bump this on ANY change to RadSettings — even one that doesn't change
// sizeof. Alignment padding can absorb an inserted field, so the size check
// alone cannot detect a layout shift (field-shifted garbage loaded fine once:
// old wcbOct2 0x3C read back as "WCBEN=60").
// MIGRATION RULE: only ever grow the struct at the END (extend the last array /
// append fields) — load() migrates older versions by prefix-copy, which is only
// valid while every existing field keeps its offset.
constexpr uint16_t kSettingsVersion = 4;

// NVS namespace shared by the settings blob and the learned-polarity key, so
// #DPZERO/#DPFACTORY (which clear the namespace) wipe both together.
constexpr const char* kNvsNamespace = "rad";

// Defaults are seeded from a live droid's captured legacy config, so a fresh
// flash behaves like a working install rather than a generic build. Re-capture
// your own with tools/capture_config.py before flashing if yours differs.
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
    // Self-starting modes. Stored like everything else, but FORCED OFF on load
    // (RadSettingsStore::load, BEHAVIOR.md D12) — the dome must never start
    // moving on its own just because the droid was powered on.
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

    // Display (display board only; accepted and stored on every board)
    uint16_t lcdSleepSec = 300;    // #DPLCDSLEEP: backlight idle timeout (0 = always on)

    // WCB mesh (Phase 5; stored from day one so bench config survives reflashes)
    bool wcbEnabled = true;        // #DPWCBEN
    uint8_t wcbDeviceId = 4;       // #DPWCBID
    uint8_t wcbOct2 = 0x3C;        // #DPWCBOCT
    uint8_t wcbOct3 = 0x4E;
    uint8_t wcbQuantity = 3;       // #DPWCBQTY
    uint8_t wcbChannel = 1;        // #DPWCBCH
    bool wcbChecksum = true;       // #DPWCBCS
    // 39 chars + NUL: WCB_Client's own limit (structPassword[40], "max 39
    // chars") — a shorter cap here silently truncated 33-39 char fleet
    // passwords and the mesh join failed with no error pointing at it.
    char wcbPassword[40] = "";     // #DPWCBPW

    // --- v4 additions (appended: see MIGRATION RULE above) -------------------
    // Adaptive-deadband ceiling for the overshoot-oscillation latch
    // (MotionController). Must exceed the overshoot amplitude at the configured
    // #DPMINSPEED or the limit cycle survives the latch; == fudge disables.
    uint8_t fudgeMax = 18;         // #DPFUDGEMAX
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
