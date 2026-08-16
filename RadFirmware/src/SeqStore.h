// Stored-sequence persistence (slots 0-100) in NVS. Arduino-only.
#pragma once
#ifdef ARDUINO

#include <Arduino.h>
#include <Preferences.h>

namespace rad {

class SeqStore {
  public:
    static constexpr uint8_t kMaxSlot = 100;
    static constexpr uint16_t kMaxBody = 256;

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

    void list(Print& reply) {
        Preferences p;
        if (!p.begin(kNamespace, true)) {
            reply.println(F("Done"));
            return;
        }
        char key[8], body[kMaxBody];
        for (uint8_t slot = 0; slot <= kMaxSlot; ++slot) {
            keyFor(slot, key);
            if (p.getString(key, body, sizeof(body)) > 0)
                reply.printf("#DPS%u:%s\n", slot, body); // replayable form
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
