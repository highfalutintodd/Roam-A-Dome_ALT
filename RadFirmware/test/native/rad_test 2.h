// Minimal host-side test harness (no external deps).
#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace radtest {

struct Case {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}

inline int gFailures = 0;
inline const char* gCurrent = "";

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, fn}); }
};

#define TEST(name)                                                                                 \
    static void test_##name();                                                                     \
    static radtest::Registrar reg_##name(#name, test_##name);                                      \
    static void test_##name()

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            ++radtest::gFailures;                                                                  \
            std::printf("FAIL %s: %s (%s:%d)\n", radtest::gCurrent, #cond, __FILE__, __LINE__);    \
        }                                                                                          \
    } while (0)

#define CHECK_EQ(a, b)                                                                             \
    do {                                                                                           \
        auto va = (a);                                                                             \
        auto vb = (b);                                                                             \
        if (!(va == vb)) {                                                                         \
            ++radtest::gFailures;                                                                  \
            std::printf("FAIL %s: %s == %s (%lld != %lld) (%s:%d)\n", radtest::gCurrent, #a, #b,   \
                        (long long)va, (long long)vb, __FILE__, __LINE__);                         \
        }                                                                                          \
    } while (0)

inline int runAll() {
    for (auto& c : registry()) {
        gCurrent = c.name;
        c.fn();
    }
    if (gFailures == 0)
        std::printf("OK: %zu tests passed\n", registry().size());
    else
        std::printf("FAILED: %d check(s) failed across %zu tests\n", gFailures, registry().size());
    return gFailures == 0 ? 0 : 1;
}

} // namespace radtest
