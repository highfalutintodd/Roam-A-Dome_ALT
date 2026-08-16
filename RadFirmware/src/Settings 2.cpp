#ifdef ARDUINO

#include "Settings.h"

#include <Preferences.h>

namespace rad {
namespace {
constexpr const char* kNamespace = "rad";
constexpr const char* kBlobKey = "settings";

struct Header {
    uint16_t version;
    uint16_t size;
};
} // namespace

bool RadSettingsStore::load(RadSettings& out) {
    out = RadSettings{};
    Preferences prefs;
    if (!prefs.begin(kNamespace, /*readOnly=*/true))
        return false;
    size_t len = prefs.getBytesLength(kBlobKey);
    bool ok = false;
    if (len == sizeof(Header) + sizeof(RadSettings)) {
        uint8_t buf[sizeof(Header) + sizeof(RadSettings)];
        prefs.getBytes(kBlobKey, buf, sizeof(buf));
        Header hdr;
        memcpy(&hdr, buf, sizeof(hdr));
        if (hdr.version == kSettingsVersion && hdr.size == sizeof(RadSettings)) {
            memcpy(&out, buf + sizeof(Header), sizeof(RadSettings));
            ok = true;
        }
        // Older known versions get field-wise migration here as the schema grows.
    }
    prefs.end();

    // SAFETY: the self-starting modes never survive a reboot (BEHAVIOR.md D12).
    // A droid that powers up and starts moving its own dome — because someone
    // left #DPAUTO1 set days ago — is a hazard and a surprise. Both modes are
    // runtime-only state: settable any time from any transport, always off at
    // boot, so the dome moves only when something explicitly asks it to.
    out.autoMode = false;
    out.homeMode = false;
    return ok;
}

void RadSettingsStore::save(const RadSettings& s) {
    Preferences prefs;
    if (!prefs.begin(kNamespace, /*readOnly=*/false))
        return;
    uint8_t buf[sizeof(Header) + sizeof(RadSettings)];
    Header hdr{kSettingsVersion, sizeof(RadSettings)};
    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(Header), &s, sizeof(RadSettings));
    prefs.putBytes(kBlobKey, buf, sizeof(buf));
    prefs.end();
}

void RadSettingsStore::clear() {
    Preferences prefs;
    if (prefs.begin(kNamespace, /*readOnly=*/false)) {
        prefs.clear();
        prefs.end();
    }
}

} // namespace rad

#endif // ARDUINO
