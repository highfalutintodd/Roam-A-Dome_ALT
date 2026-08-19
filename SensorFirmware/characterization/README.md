# Dome sensor ring — characterization results (2026-08-19)

Captured with `DomeSensorCharacterize.ino` on the actual hardware (MarkIII/Tim-Hebel
mount): four full revolutions, two by hand + two by remote, alternating directions,
turned as slowly as possible. Raw dump analyzed into `code_angle_map.csv` (every
valid 9-bit code → the angle it decodes to). 96,088 samples.

## Headline: the hardware is fully capable. The flip-out was never resolution.

| Question | Answer from the data |
|---|---|
| Resolution | **342 distinct angles over 360° ≈ 1° per step.** Only 18 degrees uncovered. |
| Is it a clean encoder? | **Yes — a proper Gray code.** All 330 existing consecutive-angle transitions differ by **exactly one bit** (0 multi-bit transitions). So *smooth rotation reads cleanly, one degree at a time* — confirmed by the long 1°-increment runs in the capture stream. |
| So what caused the flip-out? | **Single-bit sensor glitches landing on a far-away valid code.** |

## The real mechanism (and why our fix is the *right* fix)

The 9-bit code space is dense — 342 of 512 codes are valid — so **most single-bit
errors land on another valid code**, and that code is often far away in angle:

- **1,212 single-bit "alias traps":** for a typical valid code, ~3–4 of its 9
  one-bit neighbors are valid codes **>20° away** (many exactly **179°** away).
- **Uniform across all 9 bits** (124–142 traps each) — so it is *not* one bad
  sensor. Every channel can produce a far alias if it momentarily misreads.

The field flip-out, decoded exactly:

```
code 232 = angle 205  (a park spot)
  flip bit1  -> code 234 = angle 304   <-- the exact 299/304 alias
  flip bit6  -> code 168 = angle 204   (harmless, adjacent)
  flip bit5  -> code 248 = angle 196
  ...
```

One flaky read on **bit 1** while parked at 205° reports **304°**. That is the whole
bug.

### Why you cannot fix this by blocklisting codes

Code 234 is **not a bad code** — it is the legitimate encoding of **angle 304°**.
You can't blacklist it without breaking real position 304. The reading is only
*wrong in context* (you're at 205, not 304).

### Why motion-plausibility is essentially the only correct fix

A single-bit glitch is **exactly one bit** away from the current code — and so is
**legitimate motion to the adjacent degree**. In bit-space they are identical. The
*only* thing that distinguishes "glitched to 304" from "really moved" is whether the
motor could have carried the dome there. That is precisely what the RaD-side
**motor-plausibility guard** (branch `fix/dome-hunt-plausibility-guard`) tests. The
characterization proves this is not one option among many — it is the correct
discriminator, and blocklisting / more debouncing cannot substitute for it.

## Practical implications

- **Mechanically we're at the ring's best.** The Hebel mount already gives clean,
  level, rigid, constant-distance reads (the capture is a tidy 1° Gray sweep). Better
  IR margin would lower the glitch *rate* but cannot remove the trap *structure* —
  that's inherent to a dense 9-bit absolute code.
- **The firmware guard covers the residual.** Debounce (`kStableReads`) kills
  transient glitches at the source; the plausibility guard rejects stable/stuck
  glitches the debounce can't (a stuck bit gives a *stable* wrong code). Together
  they cover both.
- **Uncovered angles** (86, 92, 166, 172, 186–188, 192, 198–201, 206, 212, 216–217,
  228, 246) simply have no code — the ring will report the nearest neighbor there.
  Harmless for a show dome.

## Reproduce

Flash `SensorFirmware/DomeSensorCharacterize` to the sensor board, open serial at
**115200**, turn the dome slowly through a full turn, type `d` to dump the map,
capture the output. Reflash `DomeSensorFirmware32` afterward. See `code_angle_map.csv`
for this run's result.
