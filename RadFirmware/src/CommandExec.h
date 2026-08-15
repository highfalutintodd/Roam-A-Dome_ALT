// Applies parsed Commands to the system. Arduino-only (touches NVS, reboot, IO).
#pragma once
#ifdef ARDUINO

#include "Command.h"
#include "Settings.h"

#include <Arduino.h>

namespace rad {

struct RuntimeStats {
    uint32_t linesConsole = 0;
    uint32_t linesCmdSerial = 0;
    uint32_t invalidLines = 0;
    uint32_t lineOverflows = 0;
    uint32_t syrenChecksumErrors = 0;
};

class CommandExec {
  public:
    void begin(RadSettings* settings, RadSettingsStore* store, RuntimeStats* stats) {
        fSettings = settings;
        fStore = store;
        fStats = stats;
    }

    // Execute one parsed line; human-readable response goes to `reply`.
    void execute(const Command& cmd, Print& reply);

    // Handle a raw line end-to-end (parse + execute + Invalid handling).
    void handleLine(const char* line, Print& reply);

  private:
    void dumpConfig(Print& reply) const;
    void setSetting(const Command& cmd, Print& reply);

    RadSettings* fSettings = nullptr;
    RadSettingsStore* fStore = nullptr;
    RuntimeStats* fStats = nullptr;
};

} // namespace rad

#endif // ARDUINO
