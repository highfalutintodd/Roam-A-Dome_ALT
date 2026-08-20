// Non-blocking sequence engine. Pure C++ — host-testable.
//
// Runs a motion line ("A90:W2:H" — the text after ":DP") one step per tick.
// Wait steps are consumed internally with rollover-safe (start, duration) pairs.
//
// Fixes the legacy wait-timer bugs by construction (BEHAVIOR.md D1/D2):
//  - No serial/mesh traffic can touch wait state; only start()/stop() change it.
//  - Waits compare `now - start >= duration` (uint32 wrap-safe); no 0-sentinel.
//  - Random bounds are inclusive; WMR (random milliseconds) exists; W0 is legal.
#pragma once

#include "Command.h"

#include <cstdint>
#include <cstring>

namespace rad {

struct SeqStep {
    char op = 0;            // 'A','D','R','H','W','T','P','Z','S','Q'
    bool random = false;    // AR / DR / HR / WR
    bool oneshot = false;   // AM / DM ("M" modifier: return mode to off after move)
    bool millis = false;    // WM / WMR
    bool fireAndForget = false; // trailing '+' on A/D
    int32_t a = 0, b = 0, c = 0, d = 0;
    uint8_t argc = 0;       // how many of a..d were given
};

// Parse one step token (between ':' separators). Returns false on malformed input.
bool parseSeqStep(const char* tok, uint16_t len, SeqStep& out);

class Sequencer {
  public:
    // One ceiling for the whole pipeline: a Command's text, a stored slot body,
    // and a running script are the same line at different stages.
    static constexpr uint16_t kMaxScript = kMaxCommandText;
    static constexpr uint8_t kMaxStepsPerTick = 8; // bound work per loop pass

    // Inclusive-bounds random source; injected so tests are deterministic and the
    // firmware can pass esp_random-backed randomness.
    using RandomFn = uint32_t (*)(uint32_t lo, uint32_t hi);

    explicit Sequencer(RandomFn rng) : fRng(rng) {}

    // Begin a script, replacing whatever is running (the ONLY preemption path,
    // plus stop()). Validates every step up front; returns false and stays idle
    // on malformed input.
    bool start(const char* script, uint32_t now) {
        stop();
        size_t len = std::strlen(script);
        if (len == 0 || len >= kMaxScript)
            return false;
        if (!validate(script))
            return false;
        std::memcpy(fScript, script, len + 1);
        fCursor = 0;
        fPhase = Phase::kExec;
        (void)now;
        return true;
    }

    void stop() {
        fPhase = Phase::kIdle;
        fCursor = 0;
    }

    bool active() const { return fPhase != Phase::kIdle; }
    bool waiting() const { return fPhase == Phase::kWaiting; }

    // One-shot event: a wait step began since the last call (duration in ms,
    // wasMillis = the WM form). Lets the caller print the legacy
    // "WAIT SECONDS: <n>" / "WAIT MILLIS: <n>" console feedback — waits are
    // absorbed inside tick() and would otherwise be invisible.
    bool consumeWaitStarted(uint32_t& ms, bool& wasMillis) {
        if (!fWaitEvtPending)
            return false;
        fWaitEvtPending = false;
        ms = fWaitEvtMs;
        wasMillis = fWaitEvtMillis;
        return true;
    }

    // Validate a script without running it (used before storing #DPS slots).
    static bool validateScript(const char* script) {
        uint16_t cursor = 0;
        bool any = false;
        while (script[cursor] != '\0') {
            const char* tok = script + cursor;
            uint16_t len = 0;
            while (tok[len] != '\0' && tok[len] != ':')
                ++len;
            cursor += len + (tok[len] == ':' ? 1 : 0);
            if (len == 0)
                continue;
            SeqStep step;
            if (!parseSeqStep(tok, len, step))
                return false;
            any = true;
        }
        return any;
    }

    // Advance; returns the next step the caller must execute (valid until the next
    // tick() call), or nullptr. Wait steps never surface — they are absorbed here.
    const SeqStep* tick(uint32_t now) {
        for (uint8_t budget = 0; budget < kMaxStepsPerTick; ++budget) {
            switch (fPhase) {
            case Phase::kIdle:
                return nullptr;
            case Phase::kWaiting:
                if (now - fWaitStart < fWaitDuration)
                    return nullptr;
                fPhase = Phase::kExec;
                break;
            case Phase::kExec: {
                uint16_t tokLen = 0;
                const char* tok = nextToken(tokLen);
                if (tok == nullptr) {
                    fPhase = Phase::kIdle;
                    return nullptr;
                }
                SeqStep step;
                if (!parseSeqStep(tok, tokLen, step)) {
                    // validate() should make this unreachable; fail closed anyway.
                    fPhase = Phase::kIdle;
                    return nullptr;
                }
                if (step.op == 'W') {
                    beginWait(step, now);
                    break; // loop again: a zero-length wait advances this tick
                }
                fCurrent = step;
                return &fCurrent;
            }
            }
        }
        return nullptr; // budget exhausted; resume next tick
    }

  private:
    enum class Phase : uint8_t { kIdle, kExec, kWaiting };

    void beginWait(const SeqStep& w, uint32_t now) {
        uint32_t ms;
        if (w.random) {
            // WR[max[,min]] seconds (bare WR = 1..6s, WR10 = 1..10s, WR10,20 = 10..20s)
            // WMR<min>,<max> milliseconds. Bounds inclusive (BEHAVIOR.md D2).
            uint32_t lo, hi;
            if (w.millis) {
                lo = w.argc >= 1 ? (uint32_t)w.a : 0;
                hi = w.argc >= 2 ? (uint32_t)w.b : lo;
            } else {
                hi = w.argc >= 1 && w.a > 0 ? (uint32_t)w.a : 6;
                lo = w.argc >= 2 ? (uint32_t)w.b : 1;
            }
            if (lo > hi) {
                uint32_t t = lo;
                lo = hi;
                hi = t;
            }
            ms = fRng(lo, hi);
            if (!w.millis)
                ms *= 1000;
        } else {
            ms = (uint32_t)w.a;
            if (!w.millis)
                ms *= 1000;
        }
        if (ms > 600000)
            ms = 600000; // legacy ceiling: 600 s
        fWaitStart = now;
        fWaitDuration = ms;
        fPhase = Phase::kWaiting;
        fWaitEvtPending = true; // surface for the console print (legacy contract)
        fWaitEvtMs = ms;
        fWaitEvtMillis = w.millis;
    }

    const char* nextToken(uint16_t& lenOut) {
        if (fScript[fCursor] == '\0')
            return nullptr;
        const char* tok = fScript + fCursor;
        uint16_t len = 0;
        while (tok[len] != '\0' && tok[len] != ':')
            ++len;
        fCursor += len + (tok[len] == ':' ? 1 : 0);
        if (len == 0)
            return nextToken(lenOut); // tolerate "::" — skip empty token
        lenOut = len;
        return tok;
    }

    bool validate(const char* script) const { return validateScript(script); }

    RandomFn fRng;
    char fScript[kMaxScript] = {};
    uint16_t fCursor = 0;
    Phase fPhase = Phase::kIdle;
    uint32_t fWaitStart = 0;
    uint32_t fWaitDuration = 0;
    bool fWaitEvtPending = false; // consumeWaitStarted() one-shot
    uint32_t fWaitEvtMs = 0;
    bool fWaitEvtMillis = false;
    SeqStep fCurrent;
};

} // namespace rad
