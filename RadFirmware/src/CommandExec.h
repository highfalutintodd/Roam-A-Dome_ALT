// Applies parsed Commands to the system. Arduino-only (touches NVS, reboot, IO).
#pragma once
#ifdef ARDUINO

#include "Command.h"
#include "MotionController.h"
#include "PinBank.h"
#include "SensorRing.h"
#include "SeqStore.h"
#include "Sequencer.h"
#include "Settings.h"

#include <Arduino.h>

namespace rad {

struct RuntimeStats {
    uint32_t linesConsole = 0;
    uint32_t linesCmdSerial = 0;
    uint32_t invalidLines = 0;
    uint32_t lineOverflows = 0;
    uint32_t syrenChecksumErrors = 0;
    bool debug = false; // #DPDEBUG — not persisted; off after every boot
};

class CommandExec {
  public:
    void begin(RadSettings* settings, RadSettingsStore* store, RuntimeStats* stats,
               SensorRing* sensor, MotionController* motion, Sequencer* seq, SeqStore* seqStore,
               PinBank* pins) {
        fSettings = settings;
        fStore = store;
        fStats = stats;
        fSensor = sensor;
        fMotion = motion;
        fSeq = seq;
        fSeqStore = seqStore;
        fPins = pins;
        applyTuning();
    }

    // Execute one parsed line; human-readable response goes to `reply`.
    void execute(const Command& cmd, Print& reply);

    // Handle a raw line end-to-end (parse + execute + Invalid handling).
    void handleLine(const char* line, Print& reply);

    // Per-loop: advance the sequencer (holds while a blocking move is in flight)
    // and dispatch its steps. `console` receives step feedback.
    void pump(uint32_t now, Print& console);

    // Push settings values into MotionController tuning + SensorRing tuning.
    void applyTuning();

  private:
    void execStep(const SeqStep& step, uint32_t now, Print& reply);
    void dumpConfig(Print& reply) const;
    void setSetting(const Command& cmd, Print& reply);

    RadSettings* fSettings = nullptr;
    RadSettingsStore* fStore = nullptr;
    RuntimeStats* fStats = nullptr;
    SensorRing* fSensor = nullptr;
    MotionController* fMotion = nullptr;
    Sequencer* fSeq = nullptr;
    SeqStore* fSeqStore = nullptr;
    PinBank* fPins = nullptr;
};

} // namespace rad

#endif // ARDUINO
