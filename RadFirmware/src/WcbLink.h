// Native WCB mesh membership via the vendored WCB_Client library. Arduino-only.
//
// The dome joins the ESP-NOW mesh as its own peer (device ID 4 by default) and
// receives Sabé's :DP broadcasts wirelessly; the wired serial port stays as a
// hot fallback with cross-transport de-dup (Dedup.h). Hard rules honored here:
//  - WCB_Client is a singleton and owns the radio; begin() only when enabled
//    AND a mesh password is set (#DPWCBPW).
//  - The command callback runs on the ESP-NOW RX task (core 0): it only sets a
//    volatile flag (?STOP) or copies the line into a FreeRTOS queue. All real
//    work happens on the main loop.
//  - ?STOP / &SABE,ESTOP latch the e-stop; release is manual input or an
//    explicit new :DP command (BEHAVIOR.md §5).
#pragma once
#ifdef ARDUINO

#include "Settings.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace rad {

class WcbLink {
  public:
    struct RxLine {
        uint8_t senderId;
        // WCB_Client can deliver up to 199 chars with checksums off
        // (structCommand[200]); sizing below that silently truncated long
        // stored-sequence commands mid-body.
        char text[200];
    };

    // Returns false (and stays inactive) when disabled or no password is set.
    bool begin(const RadSettings& s, const char* fwVersion);

    void update(); // every loop: pumps WCB_Client

    bool receive(RxLine& out); // drain one queued command line, false if none

    bool active() const { return fActive; }
    bool sabeOnline() const;

    bool estopLatched() const { return fEstop; }
    void clearEstop() { fEstop = false; }
    // ?STOP arriving on the console/command-serial transports latches here too —
    // the mesh path latches in its own RX callback (COMMANDS.md: any transport).
    void latchEstop() { fEstop = true; }

    // Outbound telemetry (all no-ops when inactive).
    void sendPosition(int16_t deg, char mode, uint32_t now); // <=1 Hz, on change
    void sendHeartbeat(uint32_t now, const char* state);     // every 10 s
    void sendFault(const char* code);                        // ensured

    // Written on the ESP-NOW RX task (core 0), read by loop() (core 1):
    // volatile so the reader always sees fresh values. Aligned 32-bit accesses
    // are single instructions on Xtensa, so no torn reads.
    struct Stats {
        volatile uint32_t rx = 0;
        volatile uint32_t dropped = 0; // queue-full drops
        volatile uint32_t estops = 0;
    };
    const Stats& stats() const { return fStats; }

  private:
    static void onCommand(uint8_t senderId, const char* command);
    static void onStatus(uint8_t wcbId, bool online);

    bool fActive = false;
    volatile bool fEstop = false;
    QueueHandle_t fQueue = nullptr;
    Stats fStats;
    int16_t fLastSentPos = -1000;
    uint32_t fLastPosMs = 0;
    uint32_t fLastHbMs = 0;
};

extern WcbLink gWcb; // single instance (WCB_Client supports exactly one)

} // namespace rad

#endif // ARDUINO
