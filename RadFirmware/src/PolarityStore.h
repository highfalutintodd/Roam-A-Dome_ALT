// Persistence for the learned drive polarity.
//
// Which motor-wire polarity makes the sensor reading INCREASE is a property of
// how this particular droid is wired. It is learned by watching the dome move
// under manual control (learnPolarity() in RadFirmware.ino), but it lived only
// in RAM: every boot restarted at "unknown", so the first automated move ran on
// the #DPINVERT-derived guess. On a droid where that guess is backwards the dome
// travels the wrong way — ~54 degrees on the bench — until 10 degrees of
// evidence accumulate and the learner corrects it. Once learned, it should stay
// learned.
//
// Deliberately stored under its own NVS key instead of as a RadSettings field:
// adding a field there requires bumping kSettingsVersion, which discards the
// stored blob — and the mesh password with it. Learned calibration must never
// cost the operator their configuration.
//
// It shares the "rad" NVS namespace, so #DPZERO / #DPFACTORY clear it along with
// everything else: a factory-reset controller has legitimately learned nothing.
#pragma once

#include <cstdint>

#ifdef ARDUINO

namespace rad {

class PolarityStore {
  public:
    // Returns the stored sign (+1 or -1), or 0 if nothing has been learned yet.
    // Any other stored value is treated as "unknown" rather than trusted.
    int8_t load();

    // Writes only when the sign actually changed — this is called from the
    // learner, and an unconditional write would burn a flash cycle every boot.
    void save(int8_t sign);

    // Erases the stored sign back to "not learned". Used by the #DPDIRLEARN
    // calibration command so the learner re-derives polarity from scratch.
    void clear();

  private:
    int8_t fLast = 0;
};

} // namespace rad

#endif // ARDUINO
