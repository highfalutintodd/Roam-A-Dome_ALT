// Versioned settings. The struct and defaults are pure C++ (host-testable);
// persistence lives behind RadSettingsStore (NVS Preferences, Arduino-only).
#pragma once

#include <cstdint>

namespace rad {

constexpr uint16_t kSettingsVersion = 1;

struct RadSettings {
    // Serial / transports
    uint32_t serialBaud = 9600;    // #DPSERIALBAUD (command serial)
    uint32_t syrenBaud = 9600;     // #DPSYRENBAUD
    uint32_t sensorBaud = 115200;  // #DPSENSORBAUD (57600 or 115200)
    uint8_t syrenAddrIn = 129;     // #DPSYRENADDRIN
    uint8_t syrenAddrOut = 129;    // #DPSYRENADDROUT
    bool serialIn = true;          // #DPSERIALIN  (Syren packet-serial input enable)
    bool serialOut = true;         // #DPSERIALOUT (Syren packet-serial output enable)
    bool serialCmdIn = true;       // #DPSERIALCMD (command-serial ingress; new in v2)
    bool pwmIn = false;            // #DPPWMIN
    bool pwmOut = false;           // #DPPWMOUT
    uint16_t pwmMinUs = 1000;      // #DPPWMMIN
    uint16_t pwmMaxUs = 2000;      // #DPPWMMAX
    uint16_t pwmNeutralUs = 1500;  // #DPPWMNEUTRAL
    uint8_t pwmDeadbandPct = 5;    // #DPPWMDEADBAND
    uint16_t reportMs = 0;         // #DPREPORT (0 = off)

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
