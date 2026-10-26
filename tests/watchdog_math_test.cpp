// The RP watchdog's tick and LOAD (WatchdogMath.hpp) against the datasheets: LOAD is 24 bits (RP2350 Table 1249, md
// l.58285-58290; RP2040 Table 547, md l.25591-25597), the RP2040 counter goes down twice per tick (erratum RP2040-E1,
// md l.29693-29698: LOAD = 2 x ticks, ~8.39 s at a 1 us tick), the tick divider is nine bits (RP2350 Table 631, md
// l.28726-28731; RP2040 Table 550, md l.25639-25647). Building is the test (static_assert); the refusals are
// bad/watchdog_*.cpp.
#include "WatchdogMath.hpp"

#include <chrono>
#include <cstdint>

namespace {
template<std::uint64_t Hz, std::uint64_t Us, bool Rp2040>
using T = Kvasir::Detail::WatchdogTiming<Hz, Us, Rp2040>;

constexpr std::uint64_t mhz12 = 12'000'000;

// RP2350, 12 MHz clk_ref, 1 s: a 1 us tick (CYCLES = clk_ref in MHz), LOAD = the microseconds
using A = T<mhz12, 1'000'000, false>;
static_assert(A::maxLoad == 0xff'ffffU && A::decrementsPerTick == 1 && A::maxTicks == 0xff'ffffU);
static_assert(A::cyclesPerTick == 12 && A::ticks == 1'000'000 && A::reloadValue == 1'000'000U);
static_assert(A::timeout == std::chrono::seconds{1}
              && A::tickPeriod == std::chrono::microseconds{1});

// RP2040, the same: RP2040-E1, LOAD is twice the ticks, half the 24 bits of ticks at most
using B = T<mhz12, 1'000'000, true>;
static_assert(B::decrementsPerTick == 2 && B::maxTicks == 0x7f'ffffU);
static_assert(B::cyclesPerTick == 12 && B::ticks == 1'000'000 && B::reloadValue == 2'000'000U);
static_assert(B::timeout == std::chrono::seconds{1});

// RP2040 at its longest: 0xffffff / 2 ticks of 1 us = 8.388607 s
using C = T<mhz12, 8'388'607, true>;
static_assert(C::ticks == C::maxTicks && C::reloadValue == 0xff'fffeU);

// RP2350 past 16.78 s: a coarser tick so the ticks fit LOAD - 100 s at 12 MHz needs ceil(1.2e9 / 0xffffff) = 72
// cycles a tick, 6 us; the timeout rounds up to whole ticks
using D = T<mhz12, 100'000'000, false>;
static_assert(D::cyclesPerTick == 72 && D::ticks == 16'666'667 && D::ticks <= D::maxLoad);
static_assert(D::tickPeriod == std::chrono::microseconds{6});
static_assert(D::timeout == std::chrono::nanoseconds{100'000'002'000});

// RP2350 at the nine-bit divider's end: 714 s at 12 MHz is 511 cycles a tick
using E = T<mhz12, 714'000'000, false>;
static_assert(E::cyclesPerTick == 511);

// RP2350 with clk_ref no whole number of MHz: no 1 us tick, the divider the timeout needs (at least 1)
using F = T<12'500'000, 1'000'000, false>;
static_assert(F::microsecondCycles == 0 && F::cyclesPerTick == 1 && F::ticks == 12'500'000);
}   // namespace

int main() { return 0; }
