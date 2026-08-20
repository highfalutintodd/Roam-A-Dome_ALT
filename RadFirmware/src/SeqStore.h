// Stored-sequence persistence (slots 0-100) in NVS. Arduino-only.
#pragma once
#ifdef ARDUINO

#include "Command.h"

#include <Arduino.h>
#include <Preferences.h>

namespace rad {

class SeqStore {
  public:
    // Shared ceilings: the parser, the sequencer, and this store must agree on
    // slot range and body length or one layer accepts what another rejects.
    static constexpr uint8_t kMaxSlot = kMaxSeqSlot;
    static constexpr uint16_t kMaxBody = kMaxCommandText;

    bool save(uint8_t slot, const char* body) {
        if (slot > kMaxSlot || strlen(body) >= kMaxBody)
            return false;
        Preferences p;
        if (!p.begin(kNamespace, false))
            return false;
        char key[8];
        keyFor(slot, key);
        bool ok = p.putString(key, body) > 0;
        p.end();
        return ok;
    }

    // Returns false if the slot is empty.
    bool load(uint8_t slot, char* out, uint16_t outSize) {
        if (slot > kMaxSlot)
            return false;
        Preferences p;
        if (!p.begin(kNamespace, true))
            return false;
        char key[8];
        keyFor(slot, key);
        size_t n = p.getString(key, out, outSize);
        p.end();
        return n > 0;
    }

    bool erase(uint8_t slot) {
        if (slot > kMaxSlot)
            return false;
        Preferences p;
        if (!p.begin(kNamespace, false))
            return false;
        char key[8];
        keyFor(slot, key);
        bool ok = p.remove(key);
        p.end();
        return ok;
    }

    // `yield` (optional) is pumped while waiting for TX room: a full listing is
    // up to ~26 KB, which at 9600 baud would otherwise block loop() for tens of
    // seconds and starve the Syren keepalive mid-move (see loopCriticalYield).
    void list(Print& reply, void (*yield)() = nullptr) {
        Preferences p;
        if (!p.begin(kNamespace, true)) {
            reply.println(F("Done"));
            return;
        }
        char key[8], body[kMaxBody];
        for (uint8_t slot = 0; slot <= kMaxSlot; ++slot) {
            keyFor(slot, key);
            if (p.getString(key, body, sizeof(body)) > 0) {
                if (yield != nullptr) {
                    int need = static_cast<int>(strlen(body)) + 10;
                    uint32_t start = millis();
                    while (reply.availableForWrite() < need && millis() - start < 250)
                        yield();
                }
                // "#DPS<n>=:<body>": NOT directly executable — a byte-exact
                // "#DPS<n>:<body>" echoing back over the mesh would silently
                // RE-STORE every listed slot. The parser drops the '=' form as
                // an echo, and capture_config.py's replay strips the first '='
                // to recover the executable command.
                reply.printf("#DPS%u=:%s\n", slot, body);
            }
        }
        p.end();
        reply.println(F("Done"));
    }

    void clearAll() {
        Preferences p;
        if (p.begin(kNamespace, false)) {
            p.clear();
            p.end();
        }
    }

  private:
    static constexpr const char* kNamespace = "radseq";
    static void keyFor(uint8_t slot, char out[8]) { snprintf(out, 8, "s%u", slot); }
};

} // namespace rad

#endif // ARDUINO
