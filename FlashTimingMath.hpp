#pragma once
// The QMI's M0_TIMING fields for the flash (clock_config.hpp's flashTiming() deposits them into the register):
// register-free, the standard library only, so tests/flash_timing_math_test.cpp checks the arithmetic the clock init
// writes. The field ranges (RP2350 datasheet 12.14, Mx_TIMING, md l.60486-60489): CLKDIV [7:0] divisors 1..255 (256 as
// 0), RXDELAY [10:8] in half clk_sys cycles, MIN_DESELECT [16:12] clk_sys cycles; input sampling 12.14.3.1 (md
// l.60103-60105). The divider floor of 2 and the 7.5 ns sample point are this package's choices (below).
#include <cstdint>

namespace Kvasir { namespace DefaultClockSettings { namespace detail {
    // The QMI's divider for the flash at the final clock: SCK no faster than MaxFlashFreq (what the flash is rated
    // for, 100 MHz by default), 2..255.
    template<auto ClockSpeed,
             auto MaxFlashFreq>
    constexpr std::uint32_t flashClkdiv() {
        constexpr std::uint32_t minClkdiv = 2;
        constexpr std::uint32_t maxClkdiv = 255;

        constexpr std::uint32_t calculatedClkdiv = (ClockSpeed + MaxFlashFreq - 1) / MaxFlashFreq;

        if(calculatedClkdiv < minClkdiv) { return minClkdiv; }
        if(calculatedClkdiv > maxClkdiv) { return maxClkdiv; }
        return calculatedClkdiv;
    }

    // The QMI divider every boot runs on until peripheryClockInit() (clock_config.hpp flashInit): the final one, at
    // least 4.
    template<auto ClockSpeed,
             auto MaxFlashFreq>
    constexpr std::uint32_t bootClkdiv() {
        constexpr std::uint32_t finalClkdiv = flashClkdiv<ClockSpeed, MaxFlashFreq>();
        return finalClkdiv > 4 ? finalClkdiv : 4;
    }

    // M0_TIMING's fields for the flash at the final clock; pagebreak none, select_setup 0, select_hold 0, max_select 0.
    struct FlashTimingFields {
        std::uint32_t cooldown;
        std::uint32_t minDeselect;
        std::uint32_t rxdelay;
        std::uint32_t clkdiv;
    };

    template<auto ClockSpeed,
             auto MaxFlashFreq>
    constexpr FlashTimingFields flashTimingFields() {
        constexpr std::uint32_t clkdiv = flashClkdiv<ClockSpeed, MaxFlashFreq>();

        constexpr std::uint32_t flashFreq = ClockSpeed / clkdiv;
        static_assert(flashFreq <= MaxFlashFreq, "Flash frequency exceeds MaxFlashFreq");

        // rxdelay: when the QMI samples the data lines, in half clk_sys cycles after
        // the SCK edge. The flash launches a bit ~9 ns (clock-to-output plus the pad
        // round trip) after the opposite edge, half an SCK period earlier, so the bit
        // is there from (9 ns - clkdiv/2 cycles) to (9 ns + clkdiv/2 cycles) after
        // the sample edge. Sampling 7.5 ns after the edge sits mid-window at every
        // clk_sys the divider rule allows: 2 at 150 MHz (window 1..3), 3 at 200 MHz
        // (window 2..4). Below about 33 MHz that rounds to 0, which is clamped to 1.
        constexpr std::uint32_t rxdelayRounded = static_cast<std::uint32_t>(
          (static_cast<unsigned long long>(ClockSpeed) * 15ULL + 500'000'000ULL)
          / 1'000'000'000ULL);   // round(7.5 ns * 2 * clk_sys)
        constexpr std::uint32_t rxdelay = rxdelayRounded < 1 ? 1 : rxdelayRounded;
        static_assert(rxdelay <= 7, "rxdelay out of the QMI's range");

        // min_deselect: chip select high for half an SCK period plus this many clk_sys
        // cycles between transfers. The W25Q64JV wants 10 ns (tSHSL, read); 2 cycles
        // give 3 x 5 ns at 200 MHz clk_sys, 3 x 6.7 ns at 150 MHz. (The bootrom uses 7.)
        constexpr std::uint32_t minDeselect = 2;

        constexpr std::uint32_t cooldown = 1;   // hold CS, append sequential accesses
        return FlashTimingFields{.cooldown    = cooldown,
                                 .minDeselect = minDeselect,
                                 .rxdelay     = rxdelay,
                                 .clkdiv      = clkdiv};
    }
}}}   // namespace Kvasir::DefaultClockSettings::detail
