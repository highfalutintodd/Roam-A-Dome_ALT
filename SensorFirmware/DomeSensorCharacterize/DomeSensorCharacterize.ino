// DomeSensorCharacterize.ino — Roam-A-Dome sensor-ring characterization tool.
//
// PURPOSE. A one-time diagnostic that MEASURES what the Reeltwo dome sensor ring
// actually reports, instead of us guessing. Flash it to the SENSOR board (the same
// plain-ESP32 that normally runs DomeSensorFirmware32), slowly turn the dome
// through at least one full revolution, and capture the USB serial output. It
// tells us, for real hardware:
//   - the ring's true resolution (how many distinct valid codes/angles exist),
//   - which raw 9-bit codes decode to which angle,
//   - which arcs are noisy and how noisy,
//   - exactly which codes are ALIASES (decode to an angle far from their neighbors
//     — e.g. the ~299/304 liar behind the K-ARDS flip-out).
// With that map we can tune the RaD guards to reality — or better, blocklist the
// specific rotten codes deterministically.
//
// It does NOT touch the getDomeAngle() decode table or the pin map — it only reads
// (readSensors + getDomeAngle) and reports, so it inherits the exact same wiring
// as the working firmware. Uses Reeltwo (LGPL-2.1); see the sibling
// DomeSensorFirmware32/LICENSE. Build target: plain ESP32 (esp32:esp32:esp32),
// same as the sensor firmware.
//
// USAGE
//   1. arduino-cli compile --fqbn esp32:esp32:esp32 --upload \
//        --port /dev/cu.usbserial-XXXX SensorFirmware/DomeSensorCharacterize
//   2. Open the USB serial monitor at 115200 baud.
//   3. Slowly rotate the dome through a FULL turn (by hand, or drive it very slowly).
//      Each time the raw code changes you get a CSV line: millis,code,bits,angle.
//   4. Type 'd' + Enter to dump the accumulated code->angle->count map.
//      Type 'c' + Enter to clear it and start a fresh pass. 'h' prints help.
//   5. Capture the log (stream + final dump) and hand it back for analysis.
//   6. Reflash DomeSensorFirmware32 when done (this sketch does NOT emit #DP@ frames,
//      so the dome won't position while it's loaded).

#define USE_DEBUG
#undef USE_DOME_SENSOR_DEBUG
#include "ReelTwo.h"
#include "core/AnimatedEvent.h"
#include "drive/DomeSensorRing.h"

///////////////////////////////////

#define CONSOLE_BAUD_RATE 115200
#define SAMPLE_INTERVAL_MS 5      // how often we poll the raw code
#define HEARTBEAT_MS 2000         // periodic line even if nothing changes

DomeSensorRing sDomePosition;

// Accumulated map: for every raw 9-bit code (0..511) seen, the angle it decodes to
// and how many samples landed on it. This is the ring's fingerprint.
static uint32_t sCount[512];
static int16_t sAngle[512];       // decoded angle, -1 if invalid
static uint32_t sSamples;         // total samples this pass

static void printBits9(unsigned v)
{
    for (int b = 8; b >= 0; --b)
        Serial.print((v >> b) & 1);
}

static void dumpMap()
{
    Serial.println();
    Serial.println("==== CODE MAP (code, bits, angle, count, %) ====");
    int distinctCodes = 0, validCodes = 0;
    for (int c = 0; c < 512; ++c)
    {
        if (sCount[c] == 0)
            continue;
        ++distinctCodes;
        if (sAngle[c] >= 0)
            ++validCodes;
        Serial.print(c);
        Serial.print(",");
        printBits9(c);
        Serial.print(",");
        Serial.print(sAngle[c]);
        Serial.print(",");
        Serial.print(sCount[c]);
        Serial.print(",");
        Serial.print(sSamples ? (100.0 * sCount[c] / sSamples) : 0.0, 2);
        Serial.println();
    }
    Serial.print("==== distinct codes=");
    Serial.print(distinctCodes);
    Serial.print("  valid(angle>=0)=");
    Serial.print(validCodes);
    Serial.print("  total samples=");
    Serial.print(sSamples);
    Serial.println(" ====");
    Serial.println("Sort by angle offline to see resolution + spot aliases (a code");
    Serial.println("whose angle is far from its bit-neighbors is the liar).");
    Serial.println();
}

static void clearMap()
{
    for (int c = 0; c < 512; ++c) { sCount[c] = 0; sAngle[c] = -1; }
    sSamples = 0;
    Serial.println("[map cleared]");
}

static void help()
{
    Serial.println("commands: d=dump map  c=clear map  h=help");
    Serial.println("stream columns: millis,code,bits,angle  (printed on each code change)");
}

void setup()
{
    REELTWO_READY();
    Serial.begin(CONSOLE_BAUD_RATE);
    SetupEvent::ready();
    for (int c = 0; c < 512; ++c) sAngle[c] = -1;
    Serial.println();
    Serial.println("DomeSensorCharacterize ready. Slowly turn the dome a full turn.");
    help();
    Serial.println("millis,code,bits,angle");
}

void loop()
{
    AnimatedEvent::process();

    static uint32_t sNextSample = 0;
    static uint32_t sLastPrint = 0;
    static int sLastCode = -1;
    uint32_t now = millis();

    if ((int32_t)(now - sNextSample) >= 0)
    {
        sNextSample = now + SAMPLE_INTERVAL_MS;
        unsigned code = sDomePosition.readSensors() & 0x1FF;
        int16_t angle = (int16_t)sDomePosition.getDomeAngle(code);

        ++sSamples;
        if (sCount[code] < 0xFFFFFFFFu) ++sCount[code];
        sAngle[code] = angle;

        // Print on any code change, plus a heartbeat so a still dome still logs.
        if ((int)code != sLastCode || (now - sLastPrint) > HEARTBEAT_MS)
        {
            Serial.print(now);
            Serial.print(",");
            Serial.print(code);
            Serial.print(",");
            printBits9(code);
            Serial.print(",");
            Serial.println(angle);
            sLastCode = (int)code;
            sLastPrint = now;
        }
    }

    // Simple single-char console.
    while (Serial.available())
    {
        int ch = Serial.read();
        if (ch == 'd' || ch == 'D') dumpMap();
        else if (ch == 'c' || ch == 'C') clearMap();
        else if (ch == 'h' || ch == 'H' || ch == '?') help();
    }
}
