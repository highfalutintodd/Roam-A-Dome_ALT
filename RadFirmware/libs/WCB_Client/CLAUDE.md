# WCB_Client — Working Notes

Arduino library that lets any ESP32 join a
[WCB](https://github.com/greghulette/Wireless_Communication_Board-WCB) ESP-NOW network as a
first-class peer: unicast and broadcast commands with automatic fragmentation, raw binary
forwarding to Pololu Maestros via `WCBStream`, ETM heartbeats and ACK tracking, WDP identity
and neighbour discovery, bulk transfer, and a raw-packet hook that OTA rides on.

`architectures=esp32`. The main consumer is
[NaviCore](https://github.com/greghulette/NaviCore).

---

## Rules that are easy to break

1. **This repo is the only copy that ships.** `Arduino-Code/libraries/WCB_Client` is a local
   bench mirror — and because `Arduino-Code` *is* the arduino-cli sketchbook, that mirror
   shadows this repo in every local compile. Consumer CI clones from GitHub. A change is not
   real until it is pushed to master here, and then mirrored into the sketchbook so local
   builds agree. See `~/.claude/docs/ECOSYSTEM.md` § Shared libraries.
2. **Callbacks fire from the ESP-NOW receive context, not `loop()`.** `onWCBCommand`,
   `onNeighbor`, `onRawPacket`, and the bulk-transfer hooks all run there — on Core 0 in
   NaviCore's case. Anything a consumer does in them that touches flash, NVS, or hardware has
   to be queued to their main loop. Document that constraint on any new callback you add,
   because a consumer will get it wrong otherwise.
3. **An ESP-NOW payload is capped well under 250 bytes** once the bridge's framing and CRC
   suffix are accounted for. That is why fragmentation and bulk transfer exist. Anything new
   that can carry variable-length user data must go through one of them, not assume it fits.
4. **Timing constants are fleet-wide agreements.** The heartbeat/offline window (currently 5
   missed heartbeats) and peer aging are matched against WCB firmware behaviour. Changing one
   side alone makes boards flicker in and out of the roster.
5. **Bump `version=` in `library.properties` on every behaviour change** and say what changed
   in the commit subject — that version is what appears in WDP adverts and in users' rosters,
   and it is how a mismatch gets diagnosed in the field.

## Verifying

No test suite. The checks are:

```bash
# Compile the examples against the ESP32 core baseline (esp32:esp32@3.3.4)
arduino-cli compile --fqbn esp32:esp32:esp32s3 examples/AllFeatures
```

Real verification is on hardware, with at least one actual WCB present — mesh behaviour
(joining, aging, ACKs, fragmentation) does not show up in a compile. Say plainly when a
change has only been compiled.

## Layout

| File | What |
|---|---|
| `src/WCB_Client.{h,cpp}` | the library — join, send, receive, WDP, heartbeats, bulk |
| `src/WCBStream.{h,cpp}` | `Stream` adapter that forwards raw bytes (Maestro packets) over the mesh |
| `examples/` | one per capability — `AllFeatures` is the broadest |

Key entry points: `setIdentity()` (WDP advert — name + firmware), `setPortLabel()` (per-serial
-port labels the Wizard displays), `setMaestroIds()` (which Maestros this device hosts),
`enableSpecialPeer()` (out-of-band peer such as NaviCore), `onNeighbor()`/`getNeighbor()`
(consume adverts), `onRawPacket()`/`sendRawPacket()` (custom protocols, e.g. OTA).

Auto-join is **on by default**: the device registers WCBs it hears as peers and remembers
them across reboots, so `wcb_quantity` does not need to cover the fleet.

## Conventions

- **The code is the source of truth.** `src/WCB_Client.h` and the examples define the API, not
  the README paragraph. Never assert a callback signature, timing constant, or TLV shape from
  prose — open the header and cite `file:line`.
- **WDP is a wire format shared with the WCB firmware.** New TLVs must be added on both sides
  and must not break an older peer's parse. The authoritative description lives in the WCB
  repo's `docs/WDP_DESIGN.md`; if this library and that page disagree, the code wins and the
  page gets fixed.
- **Keep documentation current — same commit as the code.** README, `keywords.txt`, and the
  `library.properties` paragraph all describe the API and go stale silently. Update them for
  **a new or changed public method, a WDP TLV, a callback's contract or calling context, a
  timing/window constant, or a payload-size limit** — plus any fix whose cause is a trap a
  consumer could hit again. An internal fix that leaves the API and the wire format unchanged
  needs nothing. Full test in `~/.claude/docs/CONVENTIONS.md` § What earns a doc update.
  If a design doc is added here, it takes a **Revision log**.
- Examples are documentation — a new capability without an example is half-shipped.
