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
    uint32_t dedupSuppressed = 0;
    uint32_t meshRx = 0;
    uint32_t meshDropped = 0;
    uint32_t meshEstops = 0;
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
    // and dispatch its steps. Manual input cancels the whole sequence (BEHAVIOR
    // D11) — the operator's stick always ends automation, it never just pauses
    // it. `console` receives step feedback.
    void pump(uint32_t now, bool manualActive, Print& console);

    // Push settings values into MotionController tuning + SensorRing tuning.
    void applyTuning();

    // Pumped while a bulk reply (#DPCONFIG, #DPL) waits for TX room, so the
    // control loop's critical work (Syren keepalive, sensor drain) keeps
    // running instead of blocking behind a slow serial port.
    using YieldFn = void (*)();
    void setYield(YieldFn f) { fYield = f; }

    // Random source for the R-forms (:DPAR/:DPDR/:DPRR/:DPHR); inclusive bounds.
    using RandomFn = MotionController::RandomFn;
    void setRandom(RandomFn f) { fRng = f; }

  private:
    void execStep(const SeqStep& step, uint32_t now, Print& reply);
    void dumpConfig(Print& reply) const;
    void dumpLine(Print& reply, const char* fmt, ...) const;
    void setSetting(const Command& cmd, Print& reply);
    void tickHomeCapture(uint32_t now, Print& fallback);
    uint32_t rng(uint32_t lo, uint32_t hi) const { return fRng != nullptr ? fRng(lo, hi) : lo; }

    RadSettings* fSettings = nullptr;
    RadSettingsStore* fStore = nullptr;
    RuntimeStats* fStats = nullptr;
    SensorRing* fSensor = nullptr;
    MotionController* fMotion = nullptr;
    Sequencer* fSeq = nullptr;
    SeqStore* fSeqStore = nullptr;
    PinBank* fPins = nullptr;
    YieldFn fYield = nullptr;
    RandomFn fRng = nullptr;
    bool fModeOffAfterMove = false; // 'M' one-shot: modes off when the move arrives
    // Bare #DPHOMEPOS: 1 s averaging capture (BEHAVIOR D4), completed in pump().
    bool fHomeCapArmed = false;
    uint32_t fHomeCapUntil = 0;
    uint32_t fHomeCapLastSample = 0;
    int16_t fHomeCapBase = 0;
    int32_t fHomeCapSum = 0;
    uint16_t fHomeCapCount = 0;
    Print* fHomeCapReply = nullptr; // the port that asked (global streams only)
};

} // namespace rad

#endif // ARDUINO
