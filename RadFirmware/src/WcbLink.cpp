#ifdef ARDUINO

#include "WcbLink.h"

#include "RadVersion.h"

#include <WCB_Client.h>

namespace rad {

WcbLink gWcb;

namespace {
WCB_Client* sClient = nullptr;
constexpr uint8_t kSabeId = 20; // Sabé lives at the special out-of-band slot
} // namespace

bool WcbLink::begin(const RadSettings& s, const char* fwVersion) {
    if (!s.wcbEnabled) {
        Serial.println(F("[WCB] disabled (#DPWCBEN1 to enable)"));
        return false;
    }
    if (s.wcbPassword[0] == '\0') {
        Serial.println(F("[WCB] no mesh password set — #DPWCBPW<password>, then #DPRESTART"));
        return false;
    }
    fQueue = xQueueCreate(16, sizeof(RxLine));
    if (fQueue == nullptr)
        return false;

    sClient = new WCB_Client(s.wcbOct2, s.wcbOct3, s.wcbPassword, s.wcbQuantity, s.wcbDeviceId,
                             &WcbLink::onCommand, &WcbLink::onStatus);
    sClient->setMeshChannel(s.wcbChannel);
    sClient->setChecksum(s.wcbChecksum);
    sClient->setIdentity("Roam-A-Dome", fwVersion, nullptr, "dome,seq,pos");
    if (!sClient->begin()) {
        Serial.println(F("[WCB] begin() failed — mesh disabled this boot"));
        return false;
    }
    sClient->enableSpecialPeer(kSabeId); // so unicasts can reach Sabé at 20
    fActive = true;
    Serial.printf("[WCB] joined mesh as device %u (octets %02X:%02X, ch %u)\n", s.wcbDeviceId,
                  s.wcbOct2, s.wcbOct3, s.wcbChannel);
    return true;
}

// ESP-NOW RX task (core 0): flag or copy, nothing else — no NVS/flash/motor.
void WcbLink::onCommand(uint8_t senderId, const char* command) {
    ++gWcb.fStats.rx;
    if (strncmp(command, "?STOP", 5) == 0 || strncmp(command, "&SABE,ESTOP", 11) == 0) {
        gWcb.fEstop = true; // polled by the motor output stage every loop
        ++gWcb.fStats.estops;
        return;
    }
    if (command[0] != ':' && command[0] != '#')
        return; // heartbeats, WDP chatter, other boards' traffic: ignore silently
    RxLine line;
    line.senderId = senderId;
    strlcpy(line.text, command, sizeof(line.text));
    if (gWcb.fQueue == nullptr || xQueueSend(gWcb.fQueue, &line, 0) != pdTRUE)
        ++gWcb.fStats.dropped;
}

void WcbLink::onStatus(uint8_t wcbId, bool online) {
    // Informational only; printing from the RX task is tolerated by the core's
    // thread-safe Serial, but keep it terse.
    (void)wcbId;
    (void)online;
}

void WcbLink::update() {
    if (fActive)
        sClient->update();
}

bool WcbLink::receive(RxLine& out) {
    if (fQueue == nullptr)
        return false;
    return xQueueReceive(fQueue, &out, 0) == pdTRUE;
}

bool WcbLink::sabeOnline() const {
    return fActive && sClient->isSpecialPeerOnline();
}

void WcbLink::sendPosition(int16_t deg, char mode, uint32_t now) {
    if (!fActive || deg == fLastSentPos || now - fLastPosMs < 1000)
        return;
    fLastSentPos = deg;
    fLastPosMs = now;
    char buf[32];
    snprintf(buf, sizeof(buf), "&RAD,POS,%d,%c", deg, mode);
    sClient->sendToSpecialPeer(buf, /*ensured=*/false);
}

void WcbLink::sendHeartbeat(uint32_t now, const char* state) {
    if (!fActive || now - fLastHbMs < 10000)
        return;
    fLastHbMs = now;
    char buf[64];
    snprintf(buf, sizeof(buf), "&RAD,HB,%s,%lu,%s", RAD_FW_VERSION,
             static_cast<unsigned long>(now / 1000), state);
    sClient->broadcast(buf, /*ensured=*/false);
}

void WcbLink::sendFault(const char* code) {
    if (!fActive)
        return;
    char buf[48];
    snprintf(buf, sizeof(buf), "&RAD,FAULT,%s", code);
    if (!sClient->sendToSpecialPeer(buf, /*ensured=*/true))
        sClient->broadcast(buf, /*ensured=*/false); // pending slots full: best effort
}

} // namespace rad

#endif // ARDUINO
