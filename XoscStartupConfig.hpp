#pragma once
// Clocks::XoscStartupConfig: what the RP clock init does around the crystal - when it counts as started, whether the
// ring oscillator's randomisation goes off first, and what the init records and verifies on the way (clock_config.hpp,
// XipTiming.hpp, DegradedBoot.hpp). Its own header: XipTiming.hpp's queries look it up too.
#include "WaitBounds.hpp"
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Util/Config.hpp"

#include <cstdint>

namespace Kvasir { namespace Clocks {
    // When the crystal counts as started, a hook (DefaultClockSettings::detail::enableCrystal and crystalSettled):
    //
    //     struct Xosc : Kvasir::Clocks::XoscDelay {};   // or XoscMeasured with its own agree / maxCounts
    //     template<> struct Kvasir::Clocks::XoscStartupConfig<Kvasir::Tag::User> { using type = Xosc; };
    //
    // XOSC.STATUS.STABLE is only XOSC.STARTUP.DELAY running out: nothing in the XOSC looks at the oscillation.
    //   - XoscDelay: STABLE after startupMs, and that is it. Long enough for the crystals Kvasir has met
    //     (Chip::xoscStartupMs), a time that holds for a crystal only as far as somebody measured it.
    //   - XoscMeasured (the default): STABLE after the short startupMs, then the frequency counter counts clk_xosc
    //     against clk_ref -
    //     still the ROSC - until `agree` counts in a row lie within 1/2^toleranceShift of the one before. clk_ref's
    //     frequency is not known on the ROSC, so the counts are compared with each other, not with CrystalSpeed: a
    //     crystal still building up drops or adds edges, and its counts disagree. At most maxCounts counts (each
    //     0.98 us * 2^interval of clk_ref at the ROSC's ceiling, longer at a slower ROSC); a crystal that does not
    //     settle in them is switched off again, so the boot stops at InitResult::xoscNotStable as for no crystal.
    //
    // Both also carry three switches, each on by default; a config that leaves a member out gets the default:
    //   - roscRandomisationOff: the clock init switches the ring oscillator's drive randomisation off
    //     (Clocks::roscRandomisationOff). clk_ref runs from the ROSC while the crystal is counted, and a stage before
    //     the image may leave its randomisation on (picotool's decryption stage does) - a reference that jitters on
    //     purpose, against which a good crystal's counts need not agree.
    //   - recordCheck: each crystal check keeps a Clocks::CrystalCheck - why the crystal did or did not count as
    //     started (Clocks::lastInitCrystalCheck(), lastTryCrystalCheck(), DegradedBoot::Report, the degraded boot's
    //     note). Off: none of it is built, the accessors say none.
    //   - verifyClkSys: clk_sys is counted against the crystal before the full clock's flash timing goes on
    //     (peripheryClockInit, flashAfterSwitchUp), before a flash write (Clocks::xipTimingTrusted(), Flash::Region)
    //     and for Clocks::onVerifiedFullClock(); a full tree whose count fails keeps the slow timing and
    //     Clocks::DegradedBoot tries again (XipTiming.hpp). Off: the registers' word for it, as for an image that runs
    //     from flash, which a wrong timing stops at once - the count matters to an image that reads only data
    //     through XIP (a RAM image).
    //
    //     struct Lean : Kvasir::Clocks::XoscMeasured {
    //         static constexpr bool recordCheck  = false;
    //         static constexpr bool verifyClkSys = false;
    //     };
    enum class XoscCheck : std::uint8_t { delay, measured };

    struct XoscDelay {
        static constexpr XoscCheck     check                = XoscCheck::delay;
        static constexpr std::uint32_t startupMs            = Kvasir::Chip::xoscStartupMs;
        static constexpr bool          roscRandomisationOff = true;
        static constexpr bool          recordCheck          = true;
        static constexpr bool          verifyClkSys         = true;
    };

    struct XoscMeasured {
        static constexpr XoscCheck     check = XoscCheck::measured;
        static constexpr std::uint32_t startupMs
          = 1;   // pico-sdk's default: the measurement does the waiting
        static constexpr std::uint32_t agree          = 4;
        static constexpr std::uint32_t toleranceShift = 8;   // 1/256
        static constexpr std::uint32_t interval = 10;   // FC0_INTERVAL: ~1 ms at the ROSC's ceiling
        static constexpr std::uint32_t maxCounts            = 256;
        static constexpr bool          roscRandomisationOff = true;
        static constexpr bool          recordCheck          = true;
        static constexpr bool          verifyClkSys         = true;
    };

    template<typename T, typename... Ts>
    struct XoscStartupConfig {
        using type = XoscMeasured;
    };

    namespace Detail {
        // The image's XoscStartupConfig, named late on Dep (the hook's specialisation comes after this header): a
        // type of the caller's, or none in a function template of its own
        template<typename... Dep>
        using XoscStartupConfigOf =
          typename Kvasir::Port::LateT<XoscStartupConfig<Kvasir::Tag::User>, Dep...>::type;

        template<typename... Dep>
        [[nodiscard]] consteval bool recordsCrystalCheck() {
            using Config = XoscStartupConfigOf<Dep...>;
            return KVASIR_CONFIG_OR(Config, recordCheck, true);
        }

        template<typename... Dep>
        [[nodiscard]] consteval bool verifiesClkSys() {
            using Config = XoscStartupConfigOf<Dep...>;
            return KVASIR_CONFIG_OR(Config, verifyClkSys, true);
        }
    }   // namespace Detail
}}   // namespace Kvasir::Clocks
