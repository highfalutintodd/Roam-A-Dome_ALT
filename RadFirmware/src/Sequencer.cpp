#include "Sequencer.h"

#include <cstdlib>

namespace rad {
namespace {

// Parse a decimal integer at *p (optional leading '-'); advances *p.
bool readInt(const char** p, const char* end, int32_t& out) {
    const char* s = *p;
    bool neg = false;
    if (s < end && *s == '-') {
        neg = true;
        ++s;
    }
    if (s >= end || *s < '0' || *s > '9')
        return false;
    int32_t v = 0;
    while (s < end && *s >= '0' && *s <= '9') {
        if (v > 100000)
            return false; // no legitimate argument is this large
        v = v * 10 + (*s - '0');
        ++s;
    }
    *p = s;
    out = neg ? -v : v;
    return true;
}

// Parse ",n[,n[,n]]" argument tail into step.b/c/d (a already used or not).
bool readArgTail(const char** p, const char* end, SeqStep& step) {
    int32_t* slots[3] = {&step.b, &step.c, &step.d};
    int idx = 0;
    while (*p < end && **p == ',') {
        if (idx >= 3)
            return false;
        ++(*p);
        if (!readInt(p, end, *slots[idx]))
            return false;
        ++idx;
        ++step.argc;
    }
    return true;
}

} // namespace

bool parseSeqStep(const char* tok, uint16_t len, SeqStep& out) {
    out = SeqStep{};
    const char* p = tok;
    const char* end = tok + len;
    if (p >= end)
        return false;

    out.op = *p++;
    switch (out.op) {
    case 'W': {
        // W<sec> | WM<ms> | WR[max[,min]] | WMR<min>,<max>
        if (p < end && *p == 'M') {
            out.millis = true;
            ++p;
        }
        if (p < end && *p == 'R') {
            out.random = true;
            ++p;
        }
        if (p < end) {
            if (!readInt(&p, end, out.a) || out.a < 0)
                return false;
            ++out.argc;
            if (p < end && *p == ',') {
                ++p;
                if (!readInt(&p, end, out.b) || out.b < 0)
                    return false;
                ++out.argc;
            }
        } else if (!out.random) {
            return false; // bare W/WM needs a number; bare WR defaults to 1..6 s
        }
        if (!out.random && out.argc != 1)
            return false;
        if (!out.random && !out.millis && out.a > 600)
            return false; // legacy range: seconds 0..600
        return p == end;
    }
    case 'A':
    case 'D': {
        // [M](R | deg)[,speed[,maxspeed]][+]
        if (p < end && *p == 'M') {
            out.oneshot = true;
            ++p;
        }
        if (p < end && *p == 'R') {
            out.random = true;
            ++p;
        } else {
            if (!readInt(&p, end, out.a))
                return false;
            ++out.argc;
        }
        if (!readArgTail(&p, end, out))
            return false;
        // Range-check here, at parse time, so both direct :DP lines and stored
        // sequences reject out-of-range values with "Invalid" instead of the
        // exec stage silently narrowing them (a negative speed used to wrap
        // through uint8 to near-255 and run at FULL speed).
        if (!out.random && (out.a < -359 || out.a > 359))
            return false; // one revolution max; larger values silently wrapped
        if (out.b < 0 || out.b > 100 || out.c < 0 || out.c > 100)
            return false; // speed/maxspeed are percentages
        if (p < end && *p == '+') {
            out.fireAndForget = true;
            ++p;
        }
        return p == end;
    }
    case 'R': {
        // R[R]<speed> — continuous spin, -100..100
        if (p < end && *p == 'R') {
            out.random = true;
            ++p;
        }
        if (p < end) {
            if (!readInt(&p, end, out.a))
                return false;
            ++out.argc;
            if (out.a < -100 || out.a > 100)
                return false;
        } else if (!out.random) {
            return false;
        }
        return p == end;
    }
    case 'H': {
        // H[R][speed]
        if (p < end && *p == 'R') {
            out.random = true;
            ++p;
        }
        if (p < end) {
            if (!readInt(&p, end, out.a))
                return false;
            ++out.argc;
            if (out.a < 0 || out.a > 100)
                return false; // speed percentage
        }
        return p == end;
    }
    case 'S': // S<n> — play stored sequence, slot 0-100 (caller swaps scripts)
    case 'T': // T<pin>, pin 1-8
    {
        if (!readInt(&p, end, out.a) || out.a < 0)
            return false;
        // Reject out-of-range slots/pins at parse time: the exec stage casts to
        // uint8, so S300 used to alias onto real slot 44 and play it.
        if (out.op == 'S' ? out.a > kMaxSeqSlot : (out.a < 1 || out.a > 8))
            return false;
        ++out.argc;
        return p == end;
    }
    case 'P': {
        // P<pin><0|1> — two adjacent digits
        if (p + 2 != end || p[0] < '1' || p[0] > '8' || (p[1] != '0' && p[1] != '1'))
            return false;
        out.a = p[0] - '0';
        out.b = p[1] - '0';
        out.argc = 2;
        return true;
    }
    case 'Q': {
        // Q<n>,<ms>,<pos>,<easing>
        if (!readInt(&p, end, out.a))
            return false;
        ++out.argc;
        if (!readArgTail(&p, end, out))
            return false;
        return p == end && out.argc == 4;
    }
    case 'Z':
        return p == end;
    default:
        return false;
    }
}

} // namespace rad
