#ifdef ARDUINO

#include "PolarityStore.h"

#include "Settings.h"

#include <Preferences.h>

namespace rad {
namespace {
// Same namespace as the settings blob (different key), so #DPZERO/#DPFACTORY
// clear the learned polarity too — see the header for why that is intended.
// The shared constant makes the coupling structural, not a comment.
constexpr const char* kNamespace = kNvsNamespace;
constexpr const char* kKey = "dirsign";
} // namespace

int8_t PolarityStore::load() {
    Preferences prefs;
    if (!prefs.begin(kNamespace, /*readOnly=*/true))
        return 0;
    int8_t v = prefs.getChar(kKey, 0);
    prefs.end();
    if (v != 1 && v != -1)
        v = 0; // absent, or corrupt: fall back to "not learned"
    fLast = v;
    return v;
}

void PolarityStore::save(int8_t sign) {
    if (sign == fLast || (sign != 1 && sign != -1))
        return;
    Preferences prefs;
    if (!prefs.begin(kNamespace, /*readOnly=*/false))
        return;
    prefs.putChar(kKey, sign);
    prefs.end();
    fLast = sign;
}

void PolarityStore::clear() {
    fLast = 0;
    Preferences prefs;
    if (!prefs.begin(kNamespace, /*readOnly=*/false))
        return;
    prefs.remove(kKey);
    prefs.end();
}

} // namespace rad

#endif // ARDUINO
