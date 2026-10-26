#pragma once
// The RP watchdog's tick and LOAD (watchdog.hpp's Watchdog takes its numbers from here): register-free, the standard
// library only, so tests/watchdog_math_test.cpp checks the same arithmetic the driver writes.
//
// The facts (RP2040 datasheet, RP2350 datasheet; md lines of their Markdown conversions):
//   - LOAD is 24 bits, 0xffffff at most (RP2350 Table 1249, md l.58285-58290; RP2040 Table 547, md l.25591-25597).
//   - RP2040-E1: the RP2040's counter is decremented twice per tick, so LOAD is twice the ticks (4.7.3, md l.25482-25488;
//     errata RP2040-E1, md l.29693-29698): 0xffffff / 2 ticks at most.
//   - The tick divides clk_ref by a nine-bit cycle count (RP2350 TICKS WATCHDOG_CYCLES, Table 631, md l.28726-28731;
//     RP2040 WATCHDOG TICK.CYCLES, Table 550, md l.25639-25647): 511 at most. On the RP2040 that tick is the TIMER's
//     too (4.7.2), so it stays at 1 us.
#include <algorithm>
#include <chrono>
#include <cstdint>

namespace Kvasir { namespace Detail {
    template<std::uint64_t ClockHz, std::uint64_t OverrunUs, bool Rp2040>
    struct WatchdogTiming {
        static constexpr std::uint64_t maxLoad = 0xffffff;   // LOAD[23:0]

        /// RP2040-E1: the counter goes down by two per tick (RP2040 only; the RP2350 data sheet
        /// 12.9 has no such note and its TIME field counts microseconds, Table 1248).
        static constexpr std::uint64_t decrementsPerTick = Rp2040 ? 2 : 1;

        static constexpr std::uint64_t totalCycles{OverrunUs * ClockHz / 1'000'000};

        // clk_ref cycles of a 1 us tick, 0 when clk_ref is not a whole number of MHz
        static constexpr std::uint64_t microsecondCycles
          = ClockHz % 1'000'000 == 0 ? ClockHz / 1'000'000 : 0;

        static_assert(!Rp2040 || microsecondCycles != 0,
                      "RP2040: the watchdog tick is the TIMER's 1 us tick (datasheet 4.7.2), so "
                      "clk_ref must be a whole number of MHz");

        static constexpr std::uint64_t maxTicks = maxLoad / decrementsPerTick;

        static constexpr std::uint32_t cyclesPerTick = static_cast<std::uint32_t>(
          Rp2040 ? microsecondCycles
                 : std::max<std::uint64_t>(microsecondCycles == 0 ? 1 : microsecondCycles,
                                           (totalCycles + maxTicks - 1) / maxTicks));

        static_assert(cyclesPerTick <= 511,
                      "Watchdog timeout too long: CyclesPerTick exceeds the 9-bit tick divider "
                      "(max 511, RP2350 "
                      "datasheet Table 631; ~714 s at 12 MHz)");

        /// Ticks from a feed to the reset, and what LOAD is written with.
        static constexpr std::uint64_t ticks = (totalCycles + cyclesPerTick - 1) / cyclesPerTick;

        static_assert(ticks <= maxTicks,
                      "Watchdog timeout too long: more ticks than the 24-bit LOAD holds (RP2040: "
                      "0xffffff / 2 us = 8.39 s because of erratum RP2040-E1)");
        static_assert(ticks > 0,
                      "Watchdog timeout too short: less than one tick");

        static constexpr std::uint32_t reloadValue
          = static_cast<std::uint32_t>(ticks * decrementsPerTick);

        /// The timeout the registers really give (rounded up to whole ticks).
        static constexpr std::chrono::nanoseconds timeout{
          static_cast<std::int64_t>(ticks * cyclesPerTick * 1'000'000'000ULL / ClockHz)};

        /// One tick, the resolution of the timeout.
        static constexpr std::chrono::nanoseconds tickPeriod{
          static_cast<std::int64_t>(cyclesPerTick * 1'000'000'000ULL / ClockHz)};
    };
}}   // namespace Kvasir::Detail
