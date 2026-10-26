// The PWM divider/TOP search (PwmMath.hpp) at three clk_sys speeds: the values it finds, that they give the frequency
// asked for exactly (period = (TOP + 1) * DIV / clk_sys, RP2040 datasheet 4.5.2.6, RP2350 datasheet 12.5.2.6), that a
// topMin below the TOP found changes nothing, and each refusal reason. Every case is checked twice: by static_assert in
// a constant evaluation (the firmware's compile-time sites), and at run time with the frequency read through a volatile
// (the firmware's isValidFrequency / setFrequency), so a constexpr-only path cannot hide a run-time difference.
//
// Not here: the PIO DREQ rows (the chips' Blocks.hpp, Block.hpp dreqTxOf / dreqRxOf) need the generated DMA enum of a chip. Their
// proofs are the static_asserts in PioStateMachine.hpp (DREQ = 8n + m / 8n + 4 + m) and a firmware example that runs
// DMA on every PIO instance.
#include "PwmMath.hpp"

#include <array>
#include <cstdint>
#include <cstdio>

namespace D = Kvasir::PWM::detail;
using R     = D::DivTopRefusal;

namespace {
struct Case {
    std::uint32_t frequency;
    std::uint32_t topMin;
    R             refusal;
    std::uint16_t div16;   // when found
    std::uint16_t top;
};

template<std::uint32_t Clk>
struct Cases;

// Values from the search as it stands; the 200 Hz pair is the same with topMin 0 and 1024 (the minTop fix leaves
// a firmware's compile-time DIV/TOP alone). A frequency above clk_sys needs a divider below 1.0, 1 Hz one of 256 or
// more, and above about clk_sys / 1025 no TOP above 1024 is left.
template<>
struct Cases<125'000'000> {
    static constexpr std::array value{
      Case{        200,    0,              R::none,  153, 65358},
      Case{        200, 1024,              R::none,  153, 65358},
      Case{       1000,    0,              R::none,   31, 64515},
      Case{      32000,    0,              R::none,   20,  3124},
      Case{    100'000, 1024,              R::none,   16,  1249},
      Case{         10,    0,              R::none, 3052, 65529},
      Case{          1,    0,   R::frequencyTooLow,    0,     0},
      Case{187'500'000,    0,  R::frequencyTooHigh,    0,     0},
      Case{    130'000, 1024, R::minTopUnreachable,    0,     0},
    };
};

template<>
struct Cases<133'000'000> {
    static constexpr std::array value{
      Case{        200,    0,              R::none,  163, 65275},
      Case{        200, 1024,              R::none,  163, 65275},
      Case{       1000,    0,              R::none,   33, 64483},
      Case{      32000,    0,              R::none,   19,  3499},
      Case{    100'000, 1024,              R::none,   19,  1119},
      Case{         10,    0,              R::none, 3248, 65516},
      Case{          1,    0,   R::frequencyTooLow,    0,     0},
      Case{199'500'000,    0,  R::frequencyTooHigh,    0,     0},
      Case{    130'000, 1024, R::minTopUnreachable,    0,     0},
    };
};

template<>
struct Cases<150'000'000> {
    static constexpr std::array value{
      Case{        200,    0,              R::none,  184, 65216},
      Case{        200, 1024,              R::none,  184, 65216},
      Case{       1000,    0,              R::none,   37, 64863},
      Case{      32000,    0,              R::none,   20,  3749},
      Case{    100'000, 1024,              R::none,   20,  1199},
      Case{         10,    0,              R::none, 3663, 65519},
      Case{          1,    0,   R::frequencyTooLow,    0,     0},
      Case{225'000'000,    0,  R::frequencyTooHigh,    0,     0},
      Case{    150'000, 1024, R::minTopUnreachable,    0,     0},
    };
};

// One case: the search's reason, its value, and that a found value gives the frequency exactly and lies above topMin.
template<std::uint32_t Clk>
constexpr bool matches(Case const&     c,
                       D::DivTopSearch s) {
    if(s.refusal != c.refusal) { return false; }
    if(c.refusal != R::none) { return !s.value.has_value(); }
    return s.value.has_value() && s.value->first == c.div16 && s.value->second == c.top
        && s.value->second > c.topMin
        && Kvasir::PWM::calcFrequency<Clk>(c.div16, c.top) == c.frequency
        && D::calcDivAndTop<Clk>(c.frequency, c.topMin) == s.value;
}

template<std::uint32_t Clk>
constexpr bool allMatch() {
    for(auto const& c : Cases<Clk>::value) {
        if(!matches<Clk>(c, D::searchDivAndTop<Clk>(c.frequency, c.topMin))) { return false; }
    }
    return true;
}

static_assert(allMatch<125'000'000>());
static_assert(allMatch<133'000'000>());
static_assert(allMatch<150'000'000>());

// the static_assert the driver writes, on a reachable frequency, in a constant evaluation (an unreachable one is
// bad/pwm_unreachable.cpp)
static_assert(D::calcDivAndTop<133'000'000>(200,
                                            1024)
                .has_value());
static_assert(!D::calcDivAndTop<133'000'000>(130'000,
                                             1024)
                 .has_value());

int failures = 0;

template<std::uint32_t Clk>
void runTime() {
    for(auto const& c : Cases<Clk>::value) {
        std::uint32_t volatile frequency = c.frequency;
        std::uint32_t volatile topMin    = c.topMin;
        auto const s                     = D::searchDivAndTop<Clk>(frequency, topMin);
        if(!matches<Clk>(c, s)) {
            std::printf("FAIL %u Hz clk, %u Hz, topMin %u: refusal %u\n",
                        static_cast<unsigned>(Clk),
                        static_cast<unsigned>(c.frequency),
                        static_cast<unsigned>(c.topMin),
                        static_cast<unsigned>(s.refusal));
            ++failures;
        }
    }
}
}   // namespace

int main() {
    runTime<125'000'000>();
    runTime<133'000'000>();
    runTime<150'000'000>();
    std::printf("pwm math: %s\n", failures == 0 ? "all cases match" : "FAILED");
    return failures == 0 ? 0 : 1;
}
