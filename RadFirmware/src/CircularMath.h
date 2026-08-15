// Integer circular (0-359 degree) math. Pure C++ — host-testable.
//
// Replaces the legacy shortestDistance/normalize/withinArc trio that was
// duplicated across DomePosition.h and DomeDrive.h and used fmod() on integers
// (double-precision soft-float per call on ESP32).
#pragma once

#include <cstdint>

namespace rad {

// Normalize any degree value into [0, 360).
constexpr int16_t normalizeDeg(int32_t deg) {
    int32_t d = deg % 360;
    return static_cast<int16_t>(d < 0 ? d + 360 : d);
}

// Signed shortest rotation from `from` to `to`, in (-180, 180].
constexpr int16_t signedCircularDelta(int16_t from, int16_t to) {
    int32_t d = normalizeDeg(to) - normalizeDeg(from);
    if (d > 180)
        d -= 360;
    else if (d <= -180)
        d += 360;
    return static_cast<int16_t>(d);
}

// Unsigned shortest angular distance, in [0, 180].
constexpr int16_t circularDistance(int16_t a, int16_t b) {
    int16_t d = signedCircularDelta(a, b);
    return d < 0 ? static_cast<int16_t>(-d) : d;
}

// True if `pos` lies within the arc swept from `start` to `end` going clockwise
// (increasing degrees). Handles arcs spanning the 0/359 seam.
constexpr bool withinArc(int16_t start, int16_t end, int16_t pos) {
    int16_t s = normalizeDeg(start);
    int16_t e = normalizeDeg(end);
    int16_t p = normalizeDeg(pos);
    if (s <= e)
        return p >= s && p <= e;
    return p >= s || p <= e; // arc wraps through 0
}

} // namespace rad
