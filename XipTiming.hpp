#pragma once
// Whether the QMI reads the flash right at the clock clk_sys runs at now - the condition for writing it. A flash write
// itself (FlashOps.hpp, the boot ROM's erase and program in direct mode; Bootrom::explicitBuy, whose sector read runs
// under XipGuard at the boot ROM's own 03h timing, BootInfo.hpp) does not depend on it, but what decides a write does:
// a program verified by reading back, a cell store's mount and verify, a copy from one flash range to another - all
// read through XIP with the M0_TIMING in effect, so a store that reads its sectors shifted takes them for garbage and
// erases or rewrites them.
//
// The QMI samples a read RXDELAY half clk_sys cycles after the SCK edge it launches the next bit on (RP2350 datasheet
// 12.14.3, M0_TIMING Table 1297), and the flash puts its bit out half an SCK period - CLKDIV half clk_sys cycles -
// before that edge. An RXDELAY below CLKDIV therefore samples inside the bit at any clk_sys up to the one the divider
// was chosen for (the boot's timing, a degraded boot's, the resus's at clk_ref: clock_config.hpp degradedFlashTiming,
// Flash::XipReadMode::timingAtClkRef). The full clock's timing does not: its RXDELAY (3 at 200 MHz, 2 at 150 MHz) is
// at or above its CLKDIV (2), right only because the flash and the pads take their nanoseconds at that clock; at a
// slower clk_sys it samples the next bit, and every word read comes back shifted. An image that runs from flash
// cannot run on such a timing at all; a RAM image (copy-to-RAM, or one a decryption stage loaded) reads only data
// through XIP and runs on, reading it wrong.
//
// So the clock init records where it applies a timing of that kind and for which clock (clock_config.hpp
// peripheryClockInit, flashAfterSwitchUp - each only after counting clk_sys at that speed against the crystal), and
// xipTimingTrusted() counts clk_sys again against the crystal before a write (Flash::Region). Registers and plain
// variables; the count masks interrupts for its ~0.25 ms alone. Clocks::XoscStartupConfig's verifyClkSys (on by
// default) switches the counts off: the registers' word for it then (XoscStartupConfig.hpp).
#include "XoscStartupConfig.hpp"
#include "core/InterruptMask.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/CLOCKS.hpp"

#ifdef KVASIR_CHIP_RP2350
    #include "peripherals/QMI.hpp"
#endif

#include <cstdint>
#include <utility>

namespace Kvasir { namespace Clocks {
    namespace Detail {
        // The full clock clk_sys was counted at against the crystal where the clock init made the tree's flash timing
        // final (peripheryClockInit, flashAfterSwitchUp), in kHz; 0 while it was not. RP2350: the clock the QMI's
        // full-clock timing is right at alone - 0 means the timing in effect reads at any slower clock. RP2040: the
        // SSI runs on boot2's divider whatever the clock; only onVerifiedFullClock() reads it.
        inline std::uint32_t fastTimingKhz{0};
        // The crystal clk_ref runs from on the full tree, in kHz: the counts' reference. 0 before peripheryClockInit.
        inline std::uint32_t crystalKhz{0};
        // How far a count of clk_sys may be off the clock a timing was made for: a crystal is tens of ppm off, a clock
        // tree that is not the one the timing was made for percent and more.
        inline constexpr std::uint32_t fastTimingTolerancePpm = 10'000;

        // How long an FC0 status poll is waited for: a count lasts 2^interval of the 0.98 us FC0 calls a us at the
        // reference it is told (interval 10: ~1 ms, more at a slower clk_ref), a poll takes some cycles
        inline constexpr std::uint32_t fc0PollLimit = 4'000'000;

        // One FC0 count of the source @p src (an FC0_SRC value) against clk_ref, which FC0 is told runs at @p refKhz,
        // for 2^@p interval of its 0.98 us: in 1/32 kHz (FC0_RESULT KHZ << 5 | FRAC), 0 when FC0 did not go idle, the
        // count did not finish or the source stopped in it (DIED). Registers only, the FC0 value registers written
        // whole (one field each) as FrequencyCounter::measureRaw does; a flash function, so the boot's crystal check
        // (clock_config.hpp xoscCount) calls it before initMemory() - measureRaw is a RAM function, for the resus
        // handler. The caller owns FC0 (interrupts masked where another counter may run).
        [[gnu::noinline]] inline std::uint32_t fc0Count(std::uint32_t src,
                                                        std::uint32_t refKhz,
                                                        std::uint32_t interval) {
            using Clk    = Kvasir::Peripheral::CLOCKS::Registers<>;
            using Status = Clk::FC0_STATUS;
            using Kvasir::Register::writeWhole;
            std::uint32_t polls = fc0PollLimit;
            while(get<0>(apply(read(Status::running))) != 0) {
                if(--polls == 0) { return 0; }
            }
            writeWhole(Clk::FC0_REF_KHZ::FULLREGISTER, refKhz);
            writeWhole(Clk::FC0_MIN_KHZ::FULLREGISTER, 0U);
            writeWhole(Clk::FC0_MAX_KHZ::FULLREGISTER, 0x1FF'FFFFU);
            writeWhole(Clk::FC0_DELAY::FULLREGISTER, 1U);
            apply(write(Clk::FC0_INTERVAL::fc0_interval, interval));
            writeWhole(Clk::FC0_SRC::FULLREGISTER, src);   // starts the count
            polls = fc0PollLimit;
            while(get<0>(apply(read(Status::done))) == 0) {
                if(--polls == 0) { return 0; }
            }
            if(get<0>(apply(read(Status::died))) != 0) { return 0; }
            auto const r
              = apply(read(Clk::FC0_RESULT::khz), read(Clk::FC0_RESULT::frac));   // one load
            return (static_cast<std::uint32_t>(get<0>(r)) << 5)
                 | static_cast<std::uint32_t>(get<1>(r));
        }

        // One FC0 count of clk_sys against clk_ref, which FC0 is told runs at refKhz; kHz, 0 when the counter did
        // not go idle, did not finish, or clk_sys stopped in the count. Interval 8: ~0.25 ms. Registers only.
        //
        // FC0 has no lock (ClockOutput.hpp, FrequencyCounter): the resus handler counts from its interrupt, and another
        // core or an interrupted caller may count too. So the wait for a count in progress runs with interrupts on,
        // they are masked only for this count, and it starts only when FC0 is still idle once masked. A
        // FrequencyCounter::measure() this call interrupts may come back with this count's result, as with the resus
        // handler's (measureGuarded() is the one that cannot).
        [[nodiscard]] inline std::uint32_t countClkSysKhz(std::uint32_t refKhz) {
            using Clk             = Kvasir::Peripheral::CLOCKS::Registers<>;
            using Status          = Clk::FC0_STATUS;
            constexpr auto clkSys = static_cast<std::uint32_t>(Clk::FC0_SRC::FC0_SRCVal::clk_sys);
            for(std::uint32_t polls = fc0PollLimit; polls != 0; --polls) {
                if(get<0>(apply(read(Status::running))) != 0) { continue; }
                Kvasir::Core::IrqGuard const guard{};
                if(get<0>(apply(read(Status::running))) != 0) {
                    continue;
                }   // the resus took it just now
                return fc0Count(clkSys, refKhz, 8) >> 5;
            }
            return 0;
        }

        // Whether @p khz is within fastTimingTolerancePpm of @p expectedKhz. No division (the RP2040 has none) and no
        // 64-bit product: kHz up to 2^32 / 100.
        inline constexpr std::uint32_t fastTimingToleranceShare
          = 1'000'000 / fastTimingTolerancePpm;
        static_assert(fastTimingToleranceShare * fastTimingTolerancePpm == 1'000'000,
                      "fastTimingTolerancePpm divides a million");

        [[nodiscard]] constexpr bool withinTolerance(std::uint32_t khz,
                                                     std::uint32_t expectedKhz) {
            std::uint32_t const diff = khz > expectedKhz ? khz - expectedKhz : expectedKhz - khz;
            return expectedKhz != 0 && expectedKhz <= 0xFFFF'FFFFU / fastTimingToleranceShare
                && diff <= expectedKhz && diff * fastTimingToleranceShare <= expectedKhz;
        }

        static_assert(withinTolerance(200'000,
                                      200'000)
                        && withinTolerance(201'999,
                                           200'000)
                        && withinTolerance(198'000,
                                           200'000)
                        && !withinTolerance(202'001,
                                            200'000)
                        && !withinTolerance(25'000,
                                            200'000)
                        && !withinTolerance(0,
                                            200'000)
                        && !withinTolerance(0,
                                            0),
                      "withinTolerance: 1 % either way, never for an unknown clock");
    }   // namespace Detail

    /// clk_sys counted against the crystal, in kHz: 0 when clk_ref does not run from the crystal undivided, the
    /// crystal is not known yet (before peripheryClockInit) or the count did not finish.
    [[nodiscard]] inline std::uint32_t clkSysKhz() {
        using Clk = Kvasir::Peripheral::CLOCKS::Registers<>;
        if(Detail::crystalKhz == 0) { return 0; }
        // CLK_REF_SELECTED is one-hot over CLK_REF_CTRL.SRC (RP2040 datasheet Table 222, RP2350 Table 558)
        constexpr std::uint32_t onXosc = 1U
                                      << std::to_underlying(Clk::CLK_REF_CTRL::SRCVal::xosc_clksrc);
        bool const              onCrystal
          = static_cast<std::uint32_t>(apply(read(Clk::CLK_REF_SELECTED::clk_ref_selected)))
           == onXosc
         && static_cast<std::uint32_t>(apply(read(Clk::CLK_REF_DIV::_int))) == 1U;
        return onCrystal ? Detail::countClkSysKhz(Detail::crystalKhz) : 0U;
    }

    /// Whether clk_sys runs at @p khz, counted against the crystal now.
    [[nodiscard]] inline bool clkSysAt(std::uint32_t khz) {
        return Detail::withinTolerance(clkSysKhz(), khz);
    }

    namespace Detail {
        // Whether an M0_TIMING's sample delay lies below its divider (the top of this header): it reads right at any
        // clk_sys up to the one the divider was made for. CLKDIV 0 divides by 256 (RP2350 Table 1297).
        [[nodiscard]] constexpr bool readsAtAnySlowerClock(std::uint32_t rxdelay,
                                                           std::uint32_t clkdivField) {
            return rxdelay < (clkdivField == 0 ? 256U : clkdivField);
        }

        static_assert(readsAtAnySlowerClock(2,
                                            4)
                        && readsAtAnySlowerClock(3,
                                                 4)
                        && readsAtAnySlowerClock(7,
                                                 0),
                      "the boot's and the resus's timing (CLKDIV 4, RXDELAY 2), a degraded boot's");
        static_assert(
          !readsAtAnySlowerClock(3,
                                 2)
            && !readsAtAnySlowerClock(2,
                                      2),
          "the full clock's at 200 MHz (CLKDIV 2, RXDELAY 3) and at 150 MHz (RXDELAY 2)");
    }   // namespace Detail

    /// Whether XIP reads come back right at the clock clk_sys runs at now (the top of this header): a timing whose
    /// sample delay lies below its divider reads at any slower clock; a full-clock one only at the clock it was applied
    /// for, counted again now. What every flash write waits for (Flash::Region, an update channel, an A/B trial's buy).
    /// Without XoscStartupConfig's verifyClkSys no count: whether the clock init applied the full-clock timing for the
    /// tree the registers showed. The RP2040's SSI is not timed this way: true there.
    template<typename... Dep>
    [[nodiscard]] inline bool xipTimingTrusted() {
#ifdef KVASIR_CHIP_RP2350
        using Timing = Kvasir::Peripheral::QMI::Registers<0>::M0_TIMING;
        auto const t = apply(read(Timing::rxdelay), read(Timing::clkdiv));   // one load
        if(Detail::readsAtAnySlowerClock(static_cast<std::uint32_t>(get<0>(t)),
                                         static_cast<std::uint32_t>(get<1>(t))))
        {
            return true;
        }
        if constexpr(Detail::verifiesClkSys<Dep...>()) {
            return Detail::fastTimingKhz != 0 && clkSysAt(Detail::fastTimingKhz);
        } else {
            return Detail::fastTimingKhz != 0;
        }
#else
        return true;
#endif
    }
}}   // namespace Kvasir::Clocks
