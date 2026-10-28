#pragma once
#include "Clocks.hpp"
#include "FlashTimingMath.hpp"
#include "PllSearch.hpp"
#include "WaitBounds.hpp"
#include "core/Barrier.hpp"
#include "core/Delay.hpp"
#include "core/Special.hpp"
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Port/Hooks.hpp"
#include "kvasir/Register/Memory.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/CLOCKS.hpp"
#include "peripherals/PLL.hpp"
#include "peripherals/RESETS.hpp"
#include "peripherals/ROSC.hpp"
#include "peripherals/XOSC.hpp"

#ifdef KVASIR_CHIP_RP2350
    #include "XipControl.hpp"
    #include "peripherals/QMI.hpp"
#endif

#include "peripherals/XIP_CTRL.hpp"   // both chips have the XIP cache controller

#ifdef KVASIR_CHIP_RP2350
    #include "chip/WatchdogDomains.hpp"   // releasePowmanLposc (vregInit)
    #include "peripherals/POWMAN.hpp"
    #include "peripherals/TICKS.hpp"
#endif
#ifdef KVASIR_CHIP_RP2040
    #include "peripherals/VREG_AND_CHIP_RESET.hpp"
    #include "peripherals/WATCHDOG.hpp"
#endif
#include "peripherals/TIMER.hpp"

#include <bit>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <kvasir/Runtime/Panic.hpp>
#include <string_view>
#include <type_traits>
#include <uc_log/uc_log.hpp>
#include <utility>

namespace Kvasir { namespace Clocks {
    // The two PLLs by name: the system PLL feeds clk_sys, the USB PLL clk_usb and clk_adc.
    using PllSys = Peripheral::PLL::Registers<0>;
    using PllUsb = Peripheral::PLL::Registers<1>;

    /// How far DefaultClockSettings::coreClockInit() got (initResult() below). Every wait in it is bounded and gives
    /// up instead of hanging; each way it stops leaves the clocks in a state of its own. Also what a run-time switch-up
    /// (DefaultClockSettings::startSources / switchToSources) stopped at.
    enum class InitResult : std::uint8_t {
        ok,
        voltageNotSet,   // RP2350 above 150 MHz: the regulator's update did not finish; clk_sys and clk_ref on the ROSC
        // (also after a core reset that left them on PLL_SYS and the crystal), clk_peri stopped
        xoscNotStable,   // no crystal, or one that did not start in XoscStableBoundFor or did not settle in
        // XoscMeasured's counts (Clocks::XoscStartupConfig): clk_sys and clk_ref on the ROSC
        pllSysNotLocked,   // PLL_SYS did not lock (or leave reset): clk_sys on clk_ref = the crystal, the TIMER true
        pllUsbNotLocked,   // PLL_USB did not lock (or leave reset): clk_sys on PLL_SYS, clk_usb and clk_adc off
        clockSwitchNotTaken,   // a glitchless switch did not show (CLK_REF/SYS_SELECTED), clk_peri did not stop, or the
                               // TIMER's reset did not finish
        degraded,   // no init's stop: Clocks::DegradedBoot (DegradedBoot.hpp) runs this boot on a fallback tree
        // (DefaultClockSettings::degradedClockInit, Clocks::degradedTree()) after the init failed in too many boots
    };

    /// The tree DefaultClockSettings::degradedClockInit() runs a degraded boot on (Clocks::DegradedBoot): only as far
    /// down as the failure needs - the crystal whenever it runs. Read back by Clocks::degradedTree().
    enum class DegradedTree : std::uint8_t {
        none,   // not degraded
        ringOscillator,   // no crystal: clk_ref and clk_sys on the ROSC, clk_ref at most the crystal's frequency
        crystal,   // the crystal runs, PLL_SYS does not: clk_ref and clk_sys on the crystal
        noPllUsb,   // the crystal and PLL_SYS run, PLL_USB does not: clk_sys on PLL_SYS at full speed
    };

    [[nodiscard]] constexpr std::string_view name(DegradedTree t) {
        switch(t) {
        case DegradedTree::none:           return "not degraded";
        case DegradedTree::ringOscillator: return "the ring oscillator";
        case DegradedTree::crystal:        return "the crystal";
        case DegradedTree::noPllUsb:       return "PLL_SYS without PLL_USB";
        }
        return "?";
    }

    [[nodiscard]] constexpr std::string_view name(InitResult r) {
        switch(r) {
        case InitResult::ok:                  return "ok";
        case InitResult::voltageNotSet:       return "core voltage not set";
        case InitResult::xoscNotStable:       return "crystal oscillator not stable";
        case InitResult::pllSysNotLocked:     return "PLL_SYS did not lock";
        case InitResult::pllUsbNotLocked:     return "PLL_USB did not lock";
        case InitResult::clockSwitchNotTaken: return "clock switch did not take";
        case InitResult::degraded:            return "degraded, on a fallback tree";
        }
        return "?";
    }
}}   // namespace Kvasir::Clocks

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
    enum class XoscCheck : std::uint8_t { delay, measured };

    struct XoscDelay {
        static constexpr XoscCheck     check     = XoscCheck::delay;
        static constexpr std::uint32_t startupMs = Kvasir::Chip::xoscStartupMs;
    };

    struct XoscMeasured {
        static constexpr XoscCheck     check = XoscCheck::measured;
        static constexpr std::uint32_t startupMs
          = 1;   // pico-sdk's default: the measurement does the waiting
        static constexpr std::uint32_t agree          = 4;
        static constexpr std::uint32_t toleranceShift = 8;   // 1/256
        static constexpr std::uint32_t interval = 10;   // FC0_INTERVAL: ~1 ms at the ROSC's ceiling
        static constexpr std::uint32_t maxCounts = 256;
    };

    template<typename T, typename... Ts>
    struct XoscStartupConfig {
        using type = XoscMeasured;
    };
}}   // namespace Kvasir::Clocks

namespace Kvasir { namespace DefaultClockSettings {
    // The clocks coreClockInit below programs, as Startup resources (Clocks.hpp). An
    // application's ClockSettings states them (every clock claim needs a provider) with
    //     using Provides = Kvasir::DefaultClockSettings::Provides<ClockSpeed, CrystalSpeed>;
    template<auto ClockSpeed, auto CrystalSpeed>
    using Provides = Clocks::DefaultProvides<ClockSpeed, CrystalSpeed>;

    namespace detail {
        // CLK_x_SELECTED == 1 << src, one decoded bit per CTRL SRC value (RP2040 datasheet Tables 222/225, RP2350
        // Tables 558/561), as a wait condition on the register's one field; may read 0 mid-switch.
        template<auto Selected, unsigned Src>
        inline constexpr auto selectedIs
          = Kvasir::Register::FieldValue<std::remove_cvref_t<decltype(Selected)>,
                                         (std::uint32_t{1} << Src)>{};

        // Every wait of the clock init: bounded and OnTimeout::Report whatever wait policy the image injects - it runs
        // before initMemory(), where an injected Count, Log or Panic would write .bss or a record that the RAM
        // set-up then wipes. Registers only; true if `cond` held within `Bound`.
        template<typename Bound,
                 typename Cond>
        [[nodiscard,
          gnu::always_inline]] inline bool
        waited(Cond const& cond) {
            return static_cast<bool>(
              Kvasir::Register::waitUntil<Bound, Kvasir::Register::OnTimeout::Report>(cond));
        }

        namespace impl {
            template<auto CrystalSpeed,
                     typename Reg>

            static constexpr auto getXoscFreqRange() {
                if constexpr(CrystalSpeed >= 1'000'000 && CrystalSpeed <= 15'000'000) {
                    return Reg::FREQ_RANGEValC::_1_15mhz;
                } else if constexpr(CrystalSpeed >= 10'000'000 && CrystalSpeed <= 30'000'000) {
                    return Reg::FREQ_RANGEValC::_10_30mhz;
                } else if constexpr(CrystalSpeed >= 25'000'000 && CrystalSpeed <= 60'000'000) {
                    return Reg::FREQ_RANGEValC::_25_60mhz;
                } else if constexpr(CrystalSpeed >= 40'000'000 && CrystalSpeed <= 100'000'000) {
                    return Reg::FREQ_RANGEValC::_40_100mhz;
                } else {
                    static_assert(CrystalSpeed >= 1'000'000 && CrystalSpeed <= 100'000'000,
                                  "Crystal frequency must be between 1 MHz and 100 MHz");
                    return Reg::FREQ_RANGEValC::_1_15mhz;
                }
            }
        }   // namespace impl

        template<auto CrystalSpeed>
        static constexpr auto getXoscFreqRange() {
            using Reg = Kvasir::Peripheral::XOSC::Registers<>::CTRL;
            return impl::getXoscFreqRange<CrystalSpeed, Reg>();
        }

        // The image's Clocks::XoscStartupConfig, named late (the hook's specialisation comes after this header).
        template<auto CrystalSpeed>
        using XoscConfig = typename Kvasir::Port::LateT<
          Kvasir::Clocks::XoscStartupConfig<Kvasir::Tag::User>,
          std::integral_constant<decltype(CrystalSpeed), CrystalSpeed>>::type;

        // XOSC.STARTUP.DELAY for Ms: what STATUS.STABLE waits for, in 256 crystal periods (XOSC.STARTUP, RP2040
        // Table 262). pico-sdk's formula (hardware_xosc/xosc.c), its 1 ms times PICO_XOSC_STARTUP_DELAY_MULTIPLIER
        // rounded the same way.
        template<auto          CrystalSpeed,
                 std::uint32_t Ms>
        constexpr std::uint32_t xoscStartupDelay() {
            constexpr std::uint64_t perMs
              = (static_cast<std::uint64_t>(CrystalSpeed) / 1'000 + 128) / 256;
            constexpr std::uint64_t delay = perMs * Ms;
            static_assert(delay >= 1 && delay <= 0x3fff,
                          "XOSC.STARTUP.DELAY holds 14 bits: the startupMs is out of its range for "
                          "this crystal");
            return static_cast<std::uint32_t>(delay);
        }

        // The crystal on, with the startup delay STATUS.STABLE stands for: STABLE is only that counter running out,
        // nothing measures the oscillation. The reset value (0xc4, ~4 ms at 12 MHz) is too short for a crystal that
        // builds up slowly after a power-on or a RUN reset: clk_sys switched to it on STABLE runs on a clock that is
        // not there yet, and the PLL_SYS VCO's power-up next to it ended in garbage executed from the XIP cache (a
        // UsageFault, a lockup or a stalled bus) on a board with such a crystal, every cold boot. A SYSRESETREQ keeps
        // the crystal running, which is why a debugger reset never showed it. DELAY is written while the crystal is
        // off (CTRL.ENABLE is the next write), so the count it starts is the new one. Clocks::XoscStartupConfig picks
        // the delay, and whether crystalSettled() measures after it.
        template<auto CrystalSpeed>
        [[gnu::always_inline]] inline void enableCrystal() {
            using XOSC = Kvasir::Peripheral::XOSC::Registers<>;
            using Kvasir::Register::value;
            constexpr std::uint32_t delay
              = xoscStartupDelay<CrystalSpeed, XoscConfig<CrystalSpeed>::startupMs>();
            apply(write(XOSC::STARTUP::delay, value<delay>()));
            apply(write(XOSC::CTRL::ENABLEValC::en), write(getXoscFreqRange<CrystalSpeed>()));
        }

        // One FC0 count of clk_xosc (XoscMeasured), in 1/32 of what FC0 calls a kHz: FC0_REF_KHZ claims the ROSC's
        // clk_ref ceiling, so the unit is off by clk_ref's real frequency - the same for every count of one wait.
        // 0 for a count that did not finish or a clock that stopped in it. Registers only and always inlined, as
        // FrequencyCounter::measureRaw (ClockOutput.hpp), which is a RAM function and cannot run before initMemory().
        template<typename Config>
        [[gnu::always_inline]] inline std::uint32_t xoscCount() {
            using Clk    = Kvasir::Peripheral::CLOCKS::Registers<>;
            using Status = Clk::FC0_STATUS;
            using Kvasir::Register::writeWhole;
            constexpr std::uint32_t refKhz
              = static_cast<std::uint32_t>(Kvasir::Rp::ChipTraits::clkRefOnRoscHzCeiling / 1'000);
            constexpr std::uint32_t src
              = static_cast<std::uint32_t>(Clk::FC0_SRC::FC0_SRCVal::xosc_clksrc);
            // a count lasts ~1 ms at the ceiling, a few at the ROSC's floor; a poll takes some cycles
            constexpr std::uint32_t pollLimit = 4'000'000;

            std::uint32_t polls = pollLimit;
            while(get<0>(apply(read(Status::running))) != 0) {
                if(--polls == 0) { return 0; }
            }
            writeWhole(Clk::FC0_REF_KHZ::FULLREGISTER, refKhz);
            writeWhole(Clk::FC0_MIN_KHZ::FULLREGISTER, 0U);
            writeWhole(Clk::FC0_MAX_KHZ::FULLREGISTER, 0x1FF'FFFFU);
            writeWhole(Clk::FC0_DELAY::FULLREGISTER, 1U);
            apply(
              write(Clk::FC0_INTERVAL::fc0_interval, Kvasir::Register::value<Config::interval>()));
            writeWhole(Clk::FC0_SRC::FULLREGISTER, src);   // starts the count
            polls = pollLimit;
            while(get<0>(apply(read(Status::done))) == 0) {
                if(--polls == 0) { return 0; }
            }
            if(get<0>(apply(read(Status::died))) != 0) { return 0; }
            auto const r
              = apply(read(Clk::FC0_RESULT::khz), read(Clk::FC0_RESULT::frac));   // one load
            return (static_cast<std::uint32_t>(get<0>(r)) << 5)
                 | static_cast<std::uint32_t>(get<1>(r));
        }

        // After STATUS.STABLE: whether the crystal may clock the chip (Clocks::XoscStartupConfig). XoscDelay: yes,
        // STABLE is the whole check. XoscMeasured: Config::agree counts in a row within 1/2^toleranceShift of the one
        // before, of at most maxCounts; a crystal that does not get there is switched off again (STATUS.STABLE clears,
        // Clocks::initResult() then says xoscNotStable, and crystalStarted() starts it afresh). clk_ref is on the ROSC:
        // what is counted is the crystal against it, before anything runs from the crystal. A crystal clk_ref already
        // runs from (a degraded boot on the crystal, a resus repair's clk_ref, crystalStarted()'s run-time callers) is
        // settled: nothing to count it against, and switching it off would stop clk_ref and clk_sys for good.
        template<auto CrystalSpeed>
        [[nodiscard]] inline bool crystalSettled() {
            using Config = XoscConfig<CrystalSpeed>;
            if constexpr(Config::check == Kvasir::Clocks::XoscCheck::delay) {
                return true;
            } else {
                using RefSelected = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_SELECTED;
                // CLK_REF_SELECTED is one-hot over CTRL.SRC, 2 = xosc_clksrc (RP2040 Table 222, RP2350 Table 558)
                if(Kvasir::Register::fieldEquals(selectedIs<RefSelected::clk_ref_selected, 2>)) {
                    return true;
                }
                static_assert(Config::agree >= 2,
                              "XoscMeasured compares counts: agree is at least 2");
                std::uint32_t previous = 0;
                std::uint32_t run
                  = 0;   // counts in a row, each within the tolerance of the one before
                for(std::uint32_t n = 0; n != Config::maxCounts; ++n) {
                    // FC0 is shared with Clocks::Resus's handler (FrequencyCounter, ClockOutput.hpp): a run-time
                    // caller (DegradedBoot, a resus repair) counts with interrupts masked; at boot they are anyway
                    std::uint32_t const count = [] {
                        Kvasir::Core::IrqGuard const guard{};
                        return xoscCount<Config>();
                    }();
                    std::uint32_t const diff
                      = count > previous ? count - previous : previous - count;
                    bool const agrees
                      = count != 0 && previous != 0 && diff <= (previous >> Config::toleranceShift);
                    run      = agrees ? run + 1 : 1;
                    previous = count;
                    if(count != 0 && run >= Config::agree) { return true; }
                }
                using XOSC = Kvasir::Peripheral::XOSC::Registers<>;
                apply(write(XOSC::CTRL::ENABLEValC::disable));
                return false;
            }
        }
    }   // namespace detail

    namespace detail {
        // CLK_REF_DIV for clk_ref on the ring oscillator (Clocks::onRingOscillator): the ROSC's undivided ceiling
        // (Rp::ChipTraits::roscBaseHzCeiling) over ROSC.DIV's divisor (RP2040 Table 269 md l.10863: 0xaa0 + div;
        // RP2350 Table 611 md l.28310: 0xaa00 + div; 0 and other values per the traits), divided further until clk_ref
        // is within CeilingHz (Rp::ChipTraits::clkRefOnRoscHzCeiling, or a degraded boot's crystal frequency). At the
        // clk_ref ceiling: 1 at the RP2040's default divisor 16 and the RP2350 A2's boot divisor 8; 4 after the A3+
        // boot ROM's divisor 2 (<= 24 MHz). At most CLK_REF_DIV.INT's largest value. Registers only; a loop of at most
        // that many multiplications, no division (the M0+ has no divide instruction).
        template<std::uint64_t CeilingHz = Kvasir::Rp::ChipTraits::clkRefOnRoscHzCeiling>
        [[nodiscard]] std::uint32_t roscRefDivisor() {
            static_assert(CeilingHz >= 1'000'000, "a clk_ref ceiling in whole MHz at least");
            using Div                       = Kvasir::Peripheral::ROSC::Registers<>::DIV;
            using RefDiv                    = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_DIV;
            using Traits                    = Kvasir::Rp::ChipTraits;
            constexpr std::uint32_t pass    = std::to_underlying(Div::DIVVal::pass);
            constexpr std::uint32_t intMask = std::remove_cvref_t<decltype(RefDiv::_int)>::mask;
            constexpr std::uint32_t refDivMax = intMask >> std::countr_zero(intMask);
            constexpr auto baseKhz  = static_cast<std::uint32_t>(Traits::roscBaseHzCeiling / 1000);
            constexpr auto refKhz   = static_cast<std::uint32_t>(CeilingHz / 1000);
            std::uint32_t const raw = std::to_underlying(get<0>(apply(read(Div::div))));
            std::uint32_t       div = Traits::roscDivOfOther;
            if(raw >= pass && raw - pass <= Traits::roscDivMax) {
                div = raw == pass ? Traits::roscDivOfZero : raw - pass;
            }
            std::uint32_t k = 1;
            while(k < refDivMax && k * div * refKhz < baseKhz) { ++k; }
            return k;
        }
    }   // namespace detail
}}   // namespace Kvasir::DefaultClockSettings

namespace Kvasir { namespace Clocks {
    /// clk_ref and clk_sys on the ring oscillator - the tree's safe state, from any state - with clk_ref divided to
    /// CeilingHz at most (DefaultClockSettings::detail::roscRefDivisor; by default Rp::ChipTraits::
    /// clkRefOnRoscHzCeiling): the RP2350 A3+ boot ROM runs the ROSC at divisor 2, 18.4 to 96 MHz (datasheet 8.3.1 md
    /// l.27989), over clk_ref's 25 MHz (8.2.2 md l.27756) at CLK_REF_DIV 1. A divisor that grows goes before the
    /// source, one that shrinks after it (pico-sdk clock_configure, RP2350 datasheet md l.26025, RP2040 md l.8875):
    /// clk_ref never runs faster than either setting allows; on the ROSC already, the divisor changes on the fly
    /// ("All dividers support on-the-fly divisor changes", RP2350 md l.25859, RP2040 md l.8726). Then clk_sys to
    /// clk_ref, undivided. Registers only, each glitchless switch waited for bounded (CLK_x_SELECTED); false if one
    /// does not show. The first switches of coreClockInit() and degradedClockInit(), and what a test that breaks a
    /// source on purpose does first: never break the clock clk_sys runs from.
    template<std::uint64_t CeilingHz = Kvasir::Rp::ChipTraits::clkRefOnRoscHzCeiling>
    [[nodiscard]] bool onRingOscillator() {
        using Clk = Kvasir::Peripheral::CLOCKS::Registers<>;
        using DefaultClockSettings::detail::selectedIs;
        using DefaultClockSettings::detail::waited;
        using Kvasir::Register::value;
        std::uint32_t const want = DefaultClockSettings::detail::roscRefDivisor<CeilingHz>();
        // INT 0 (= its largest + 1) is never written by Kvasir; the A3+ boot ROM leaves 5 (md l.66193)
        std::uint32_t const now = get<0>(apply(read(Clk::CLK_REF_DIV::_int)));
        if(want > now) { apply(write(Clk::CLK_REF_DIV::_int, want)); }
        apply(write(Clk::CLK_REF_CTRL::SRCValC::rosc_clksrc_ph));
        if(!waited<Kvasir::Chip::ClkSelectedBound>(
             selectedIs<Clk::CLK_REF_SELECTED::clk_ref_selected, 0>))
        {
            return false;
        }
        if(want < now) { apply(write(Clk::CLK_REF_DIV::_int, want)); }
        apply(write(Clk::CLK_SYS_CTRL::SRCValC::clk_ref));
        if(!waited<Kvasir::Chip::ClkSelectedBound>(
             selectedIs<Clk::CLK_SYS_SELECTED::clk_sys_selected, 0>))
        {
            return false;
        }
        // a previous image's divider would stay: INT 1, FRAC 0 (RP2040 Table 224, RP2350 Table 560); changing it on
        // the fly is allowed (RP2040 2.15.3.3, RP2350 8.1.3.3)
        apply(write(Clk::CLK_SYS_DIV::_int, value<std::uint32_t{1}>()),
              write(Clk::CLK_SYS_DIV::frac, value<std::uint32_t{0}>()));
        return true;
    }
}}   // namespace Kvasir::Clocks

namespace Kvasir { namespace DefaultClockSettings {

    // Core voltage for the requested clock. Reset value is 1.10 V, the
    // datasheet setting for 150 MHz; above that the regulator must be raised
    // before the PLL switch. 1.15 V covers up to ~266 MHz - extend the range
    // only together with hardware validation.
    //
    // POWMAN writes are ignored unless bits 31:16 carry the 0x5AFE password: the SVD's
    // PASSWORD key field, which every write of VREG names (a compile error otherwise).
    //
    // The sequence is pico-sdk's vreg_set_voltage (hardware_vreg/vreg.c) on the RP2350, and every
    // step of it is load-bearing:
    //   - VREG_CTRL.UNLOCK first: "The regulator can be directly controlled by software, but must
    //     first be unlocked by writing a 1 to the UNLOCK field in the VREG_CTRL register" (RP2350
    //     datasheet 6.3.2). Locked, the VSEL write sets UPDATE_IN_PROGRESS and nothing else: the bit
    //     stays set for good, the voltage stays at 1.10 V and the PLL switch below runs the core at
    //     ClockSpeed on it. Seen at 200 MHz: VREG 0x80C0 -- VSEL 1.15 V, UPDATE_IN_PROGRESS -- for
    //     as long as the image ran. It cannot be relocked; pico-sdk leaves it unlocked too.
    //   - Then time for the output to get there before the PLL switch: UPDATE_IN_PROGRESS covers the
    //     register update only, and VOUT_OK cannot tell 1.10 V from 1.15 V (it asserts at 90 % of the
    //     selected voltage, 6.3.4). pico-sdk waits SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US, 1 ms
    //     (pico_runtime_init/runtime_init_clocks.c); the data sheet gives no settling time.
    //   - SYSCFG.AUXCTRL bit 0 cleared before that (for every image): it forces POWMAN onto the 32 kHz LPOSC, which
    //     the boot ROM's reboot() and Kvasir's own watchdog resets set before a reset (chip/WatchdogDomains.hpp
    //     forcePowmanOnLposc, releasePowmanLposc) and which survives the boot ROM's reboot (found set, 1, in a
    //     running image). On the LPOSC every POWMAN access takes ~100 us: the two bounded polls below were
    //     10 s of every boot. Nothing in this image needs it set; the next reset that needs it sets
    //     it again.
    //
    // False when the regulator's update after the VSEL write does not finish in its bound (Chip::VregUpdateBound):
    // coreClockInit then switches nothing (no PLL on a voltage that may not be there; InitResult::voltageNotSet).
    template<auto ClockSpeed>
    [[nodiscard]] bool vregInit() {
#ifdef KVASIR_CHIP_RP2350
        // every image, whatever its clock: POWMAN (and the AON timer) back on their own clock
        Kvasir::Rp::WatchdogDomains::releasePowmanLposc();
        if constexpr(ClockSpeed > 150'000'000) {
            static_assert(ClockSpeed <= 266'000'000,
                          "no validated core voltage for this clock - extend "
                          "vregInit together with hardware validation");
            using VREG     = Kvasir::Peripheral::POWMAN::Registers<>::VREG;
            using VregCtrl = Kvasir::Peripheral::POWMAN::Registers<>::VREG_CTRL;
            using Kvasir::Register::value;
            constexpr std::uint32_t vsel1v15 = 0b01100;

            apply(write(VregCtrl::PASSWORDValC::password), set(VregCtrl::unlock));

            // The regulator ignores writes while an update is in flight (6.3.2, md l.22847); an update takes
            // microseconds, so the polls are bounded (at clk_ref speed a few ms each).
            bool const idle = detail::waited<Kvasir::Chip::VregUpdateBound>(
              Kvasir::Register::isClear(VREG::update_in_progress));
            // Only a change needs the write and the settling time. The voltage is back at 1.10 V "at
            // initial power-on or after a reset event" (6.3.2), which events the data sheet does not
            // list; a core-0-only reset keeps it, and may even find clk_sys still on the PLL here. A VSEL
            // that reads right proves nothing while an update hangs: one written locked reads 1.15 V for good.
            if(!idle || get<0>(apply(read(VREG::vsel))) != vsel1v15) {
                apply(write(VREG::PASSWORDValC::password),
                      write(VREG::vsel, value<std::uint32_t, vsel1v15>()));
                if(!detail::waited<Kvasir::Chip::VregUpdateBound>(
                     Kvasir::Register::isClear(VREG::update_in_progress)))
                {
                    return false;
                }
                // Still before the PLL switch: clk_sys on the ROSC, which the boot ROM of an A3 runs at
                // up to 96 MHz (8.3, md l.27989). 192 000 cycles are at least 2 ms at that (1 ms is what is
                // needed); on a slow ROSC (4.6 MHz before A3) some tens of ms, once, on the boot that
                // changes the voltage.
                Kvasir::Core::spinCycles<192'000>();
            }
        }
        return true;
#elifdef KVASIR_CHIP_RP2040
        // RP2040. Reset value is 1.10 V (vsel 0b01011), the datasheet setting for 133 MHz.
        // 200 MHz at 1.15 V (0b01100) is the one overclock pico-sdk validates
        // (SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST); nothing above it is.
        if constexpr(ClockSpeed > 133'000'000) {
            static_assert(ClockSpeed <= 200'000'000,
                          "no validated core voltage for this clock on the RP2040 - extend "
                          "vregInit together with hardware validation");
            using VREG = Kvasir::Peripheral::VREG_AND_CHIP_RESET::Registers<>::VREG;
            using Kvasir::Register::value;
            constexpr std::uint32_t vsel1v15 = 0b01100;
            apply(write(VREG::vsel, value<std::uint32_t, vsel1v15>()));
            // The RP2040 regulator has no "update in progress" flag; pico-sdk waits 1 ms.
            // This runs before the crystal is up, on the ring oscillator at up to ~12 MHz,
            // so the wait is counted: 24 000 cycles are at least 2 ms at that (1 ms is what is
            // needed), and only a few ms at the slowest.
            Kvasir::Core::spinCycles<24'000>();
        }
        return true;
#else
        static_assert(ClockSpeed <= 150'000'000,
                      "no POWMAN register description - cannot raise the core "
                      "voltage this clock needs");
        return true;
#endif
    }

    namespace detail {
        // The QMI's M0_TIMING for the flash at the final clock: divider, sample delay,
        // deselect time (the fields: FlashTimingMath.hpp, with flashClkdiv and bootClkdiv). Applied by
        // XipReadMode::apply() in peripheryClockInit(), from RAM, after the PLL switch - see flashInit() for why
        // not earlier.
#ifdef KVASIR_CHIP_RP2350
        template<auto ClockSpeed,
                 auto MaxFlashFreq>
        constexpr std::uint32_t flashTiming() {
            constexpr FlashTimingFields f = flashTimingFields<ClockSpeed, MaxFlashFreq>();
            using Timing                  = Kvasir::Peripheral::QMI::Registers<0>::M0_TIMING;
            using Kvasir::Register::deposit;
            return deposit(Timing::cooldown, f.cooldown)
                 | deposit(Timing::min_deselect, f.minDeselect)
                 | deposit(Timing::rxdelay, f.rxdelay) | deposit(Timing::clkdiv, f.clkdiv);
        }
#endif

        // A degraded boot's M0_TIMING (peripheryClockInit; Clocks::DegradedBoot): the final timing with the boot's
        // divider - or one more where the final sample delay needs it. It reads on the ROSC and the crystal, and at the
        // full clock: QMI samples "simultaneously with the launch of the next SCK edge, plus any delay configured by
        // M0_TIMING.RXDELAY" (RP2350 datasheet 12.14.3, md l.60145), counted in half clk_sys cycles (md l.60488), and the
        // flash launches its bit half an SCK period - CLKDIV half clk_sys cycles - before that edge: an RXDELAY below
        // CLKDIV samples the bit in its window at a slow clock (from 233 MHz on the final RXDELAY is 4: CLKDIV 5), and at
        // the full clock flashInit's argument holds (the round trip is at most half an SCK period at that divider).
        // Every field but CLKDIV is the final one, so the switch-up writes CLKDIV alone - the one field the QMI takes
        // while it fetches (md l.60489), whatever another core runs from flash.
#ifdef KVASIR_CHIP_RP2350
        template<auto ClockSpeed,
                 auto MaxFlashFreq>
        constexpr std::uint32_t degradedFlashTiming() {
            using Timing                    = Kvasir::Peripheral::QMI::Registers<0>::M0_TIMING;
            constexpr std::uint32_t full    = flashTiming<ClockSpeed, MaxFlashFreq>();
            constexpr std::uint32_t rxdelay = Kvasir::Register::extract(Timing::rxdelay, full);
            constexpr std::uint32_t boot    = bootClkdiv<ClockSpeed, MaxFlashFreq>();
            constexpr std::uint32_t div     = rxdelay < boot ? boot : rxdelay + 1;
            return (full & ~Timing::clkdiv.mask) | Kvasir::Register::deposit(Timing::clkdiv, div);
        }
#endif
    }   // namespace detail

    // Before the PLL switch, from flash, on the ring oscillator: only the divider.
    //
    // CLKDIV is the one M0_TIMING field the QMI lets software change while it is fetching;
    // everything else needs the QMI idle, which code running from flash cannot promise.
    // And the sample delay that is right at the final clock is wrong here: with the
    // divider at 2 the next bit lands half an SCK period after the sample edge, which at
    // ROSC speed is long before a 7.5 ns rxdelay samples. Divider 4 with the bootrom's rxdelay 2
    // reads clean at ROSC and at any final clk_sys the divider rule allows, so this is
    // what the code between here and peripheryClockInit() runs on - or the final divider,
    // when a slow flash needs a larger one.
    template<auto ClockSpeed,
             auto MaxFlashFreq = 100'000'000>
    void flashInit() {
#ifdef KVASIR_CHIP_RP2350
        using namespace Kvasir::Peripheral::QMI;
        using QMI = Registers<0>;
        (void)detail::flashTiming<ClockSpeed, MaxFlashFreq>();   // the static_asserts
        constexpr std::uint32_t bootClkdiv = detail::bootClkdiv<ClockSpeed, MaxFlashFreq>();
        // This divider with rxdelay 2 (XipReadMode::TimingAtClkRef, which the resus handler runs
        // coreClockInit() on) samples half an SCK period plus one clk_sys cycle after the edge the
        // flash launches on. The bit is valid from 10 ns after that edge at the latest: 2.5 ns
        // clk_sys to QSPI output + 1.5 ns QSPI input to clk_sys (RP2350 datasheet Table 1292,
        // 3.3 V) + 6 ns tCLQV (W25Q128JV AC table; the W25Q64JV family). Holds up to ~300 MHz.
        constexpr unsigned long long samplePs = (bootClkdiv / 2 + 1) * 1'000'000'000'000ULL
                                              / static_cast<unsigned long long>(ClockSpeed);
        static_assert(samplePs >= 10'000,
                      "clk_sys too fast for the boot/resus flash timing: the flash's data is not "
                      "valid yet when the QMI samples it (see flashInit)");
        apply(write(QMI::M0_TIMING::clkdiv, value<std::uint32_t, bootClkdiv>()));
        // A larger divider is only in effect after the next QMI access, and it has to be before
        // clk_sys goes up (RP2350 datasheet, Table 1297 M0_TIMING.CLKDIV): the next fetch may hit
        // the XIP cache, so read the uncached alias (XIP_NOCACHE_NOALLOC_BASE, 2.2 address map).
        Kvasir::Core::dsb();
        (void)Kvasir::Memory::load<std::uint32_t>(Kvasir::detail::XipReadMode::xipNoCacheBase);
        Kvasir::Core::dsb();
#endif

        // Restore XIP cache to reset state. The debugger may disable it during flash writes;
        // a run_low reset leaves peripheral state as-is, so this ensures the cache is always
        // enabled regardless of reset cause.
        apply(Kvasir::Peripheral::XIP_CTRL::Registers<>::CTRL::overrideDefaults());
    }

    namespace detail {
        // The TIMER's microsecond tick, in every image: the generator divides clk_ref (the crystal, after
        // coreClockInit's switch) by CYCLES. RP2040: WATCHDOG.TICK (4.7.2, Table 550: ENABLE resets to 1, CYCLES to
        // 0 - no tick), which the TIMER needs to count at all (4.6.4: "The Watchdog tick ... must be running for the
        // timer to start counting"). RP2350: TICKS.TIMER0_CTRL / TIMER0_CYCLES (8.5, Tables 624-625, ENABLE resets
        // to 0); "Before changing the cycle count, always stop the tick generator" (8.5.1). And TIMER (TIMER0) out
        // of reset: FirstInitStep holds it there. With it the TIMER counts microseconds in an image that clocks
        // itself by SysTick too, which AnnouncedReset's and uc_log's bounded waits time themselves by
        // (kvasir_announced_reset_time_us); a TimerClock in the list configures the same values again.
        // False if the TIMER's reset does not finish (bounded).
        template<auto CrystalSpeed>
        [[nodiscard]] bool startTimerTick() {
            static_assert(CrystalSpeed % 1'000'000 == 0,
                          "the TIMER tick needs clk_ref in whole MHz");
            static constexpr auto cycles = static_cast<std::uint32_t>(CrystalSpeed / 1'000'000);
            static_assert(cycles > 0 && cycles < (1U << 9U),
                          "the tick's CYCLES field is nine bits");
            using Kvasir::Register::value;
            using RST = Kvasir::Peripheral::RESETS::Registers<0>;
#ifdef KVASIR_CHIP_RP2350
            using Ticks = Kvasir::Peripheral::TICKS::Registers<>;
            apply(clear(Ticks::TIMER0_CTRL::enable));
            apply(write(Ticks::TIMER0_CYCLES::timer0_cycles, value<cycles>()));
            apply(set(Ticks::TIMER0_CTRL::enable));
            apply(clear(RST::RESET::timer0));
            return waited<Kvasir::Chip::ResetDoneBound>(
              Kvasir::Register::isSet(RST::RESET_DONE::timer0));
#else
            using Tick = Kvasir::Peripheral::WATCHDOG::Registers<>::TICK;
            apply(clear(Tick::enable));
            apply(write(Tick::cycles, value<cycles>()), set(Tick::enable));
            apply(clear(RST::RESET::timer));
            return waited<Kvasir::Chip::ResetDoneBound>(
              Kvasir::Register::isSet(RST::RESET_DONE::timer));
#endif
        }

        // One PLL from reset to running at S (RP2040 DS 2.18.3, RP2350 DS 8.6.4 "Configuration": reset, REFDIV/FBDIV, power up
        // the VCO, wait for LOCK, the post dividers, then power them up). Registers only: coreClockInit runs before
        // initMemory(). False, with the post dividers still off, if the reset or the lock does not come in its bound
        // (LockBound: Chip::PllLockBound at the clk_sys the wait runs on).
        template<typename Pll,
                 auto        ResetField,
                 auto        DoneField,
                 PllSettings S,
                 typename LockBound>
        [[nodiscard,
          gnu::always_inline]] inline bool
        pllInit() {
            using Kvasir::Register::value;
            // reset pll
            apply(set(ResetField));
            apply(clear(ResetField));
            if(!waited<Kvasir::Chip::ResetDoneBound>(Kvasir::Register::isSet(DoneField))) {
                return false;
            }

            apply(Pll::CS::overrideDefaults(clear(Pll::CS::bypass),
                                            write(Pll::CS::refdiv, value<S.refdiv>())));

            apply(write(Pll::FBDIV_INT::fbdiv_int, value<S.fbdiv>()));

            apply(Pll::PWR::overrideDefaults(clear(Pll::PWR::vcopd),
                                             clear(Pll::PWR::pd),
                                             set(Pll::PWR::postdivpd)));

            // wait for PLL lock
            if(!waited<LockBound>(Kvasir::Register::isSet(Pll::CS::lock))) { return false; }

            apply(Pll::PRIM::overrideDefaults(write(Pll::PRIM::postdiv1, value<S.pd1>()),
                                              write(Pll::PRIM::postdiv2, value<S.pd2>())));

            apply(Pll::PWR::overrideDefaults(clear(Pll::PWR::vcopd),
                                             clear(Pll::PWR::pd),
                                             clear(Pll::PWR::postdivpd)));
            return true;
        }
    }   // namespace detail

    namespace detail {
        // A generator without a glitchless mux stopped before its AUXSRC changes (RP2040 datasheet 2.15.3.2, RP2350
        // 8.1.3.2). RP2350: CTRL.ENABLED follows ENABLE once it has stopped (Tables 562, 568, 571), waited for bounded;
        // RP2040: no such bit - three cycles of the slowest old source (the ROSC, Rp::ChipTraits::roscHzFloor) at the
        // fastest clk_sys, counted (pico-sdk clock_configure: "Delay for 3 cycles of the target clock", md l.8875).
        // False if it did not stop.
        template<typename Ctrl>
        [[nodiscard,
          gnu::always_inline]] inline bool
        stopGenerator() {
            apply(clear(Ctrl::enable));
#ifdef KVASIR_CHIP_RP2350
            return waited<Kvasir::Chip::ClkSelectedBound>(Kvasir::Register::isClear(Ctrl::enabled));
#else
            constexpr auto floorHz = Kvasir::Rp::ChipTraits::roscHzFloor;
            Kvasir::Core::spinCycles<static_cast<std::uint32_t>(
              3 * ((Kvasir::Rp::ChipTraits::cpuHzCeiling + floorHz - 1) / floorHz))>();
            return true;
#endif
        }

        // A generator without a glitchless mux running from `Src` (one of its AUXSRC values): ENABLE and AUXSRC in one
        // load
        template<typename Ctrl,
                 auto Src>
        [[nodiscard,
          gnu::always_inline]] inline bool
        runsFrom() {
            auto const c = apply(read(Ctrl::enable), read(Ctrl::auxsrc));
            return static_cast<bool>(get<0>(c)) && get<1>(c) == Src;
        }

        // clk_usb and clk_adc to one source: stopped, the source, started (no glitchless mux)
        template<typename UsbSrc,
                 typename AdcSrc>
        [[nodiscard]] bool usbAdcTo(UsbSrc usb,
                                    AdcSrc adc) {
            using Clk = Kvasir::Peripheral::CLOCKS::Registers<>;
            if(!stopGenerator<Clk::CLK_USB_CTRL>() || !stopGenerator<Clk::CLK_ADC_CTRL>()) {
                return false;
            }
            apply(write(usb), write(adc));
            apply(set(Clk::CLK_USB_CTRL::enable), set(Clk::CLK_ADC_CTRL::enable));
            return true;
        }

        // A PLL out of reset and locked (RESETS before its registers, kvasir-chip-rp's rule)
        template<typename Pll,
                 auto ResetField,
                 auto DoneField>
        [[nodiscard,
          gnu::always_inline]] inline bool
        pllUp() {
            return !apply(read(ResetField)) && apply(read(DoneField)) && apply(read(Pll::CS::lock));
        }

        // clk_sys on PLL_SYS: the glitchless mux on its aux input (CLK_SYS_SELECTED one-hot over SRC, RP2040 Table 225,
        // RP2350 Table 561) and the aux mux on PLL_SYS
        [[nodiscard,
          gnu::always_inline]] inline bool
        sysOnPllSys() {
            using Sys = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_CTRL;
            return Kvasir::Register::fieldEquals(
                     selectedIs<
                       Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_SELECTED::clk_sys_selected,
                       1>)
                && get<0>(apply(read(Sys::auxsrc))) == Sys::AUXSRCVal::clksrc_pll_sys;
        }
    }   // namespace detail

    // The boot's clock tree, from whatever the boot ROM, a previous image or a resus left: clk_ref on the crystal,
    // clk_sys = clk_peri on PLL_SYS at ClockSpeed, clk_usb and clk_adc on PLL_USB at 48 MHz. Registers only (it runs
    // before initMemory(), and whatever it wrote to .data or .bss would be wiped). Every wait is bounded
    // (detail::waited, OnTimeout::Report): a step that does not finish stops the init there instead of hanging, and
    // Kvasir::Clocks::initResult() reads from the registers how far it got - peripheryClockInit() asks it at boot, the
    // resus handler after a repair.
    template<auto ClockSpeed,
             auto CrystalSpeed,
             auto MaxFlashFreq = 100'000'000>
    void coreClockInit() {
        using Kvasir::Register::value;

        static constexpr auto pllSettings    = detail::calcPllSettings(ClockSpeed, CrystalSpeed);
        static constexpr auto usbPllSettings = detail::calcPllSettings(48'000'000, CrystalSpeed);

        static_assert(pllSettings.refdiv != 0,
                      "no PLL setting for ClockSpeed: none comes nearer than ClockSpeed itself "
                      "(the PLL gives 750 MHz / 49 at the least, see detail::calcPllSettings)");
        static_assert(usbPllSettings.refdiv != 0, "no USB PLL setting for 48 MHz");
        // exactly, or the message says by how much it is missed
        Prescaler::assertInTolerance<detail::pllOutput(CrystalSpeed, pllSettings),
                                     ClockSpeed,
                                     Prescaler::Tolerance{0, 1},
                                     "clk_sys PLL">();
        Prescaler::assertInTolerance<detail::pllOutput(CrystalSpeed, usbPllSettings),
                                     48'000'000,
                                     Prescaler::Tolerance{0, 1},
                                     "clk_usb PLL">();

        using PeriClock   = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_PERI_CTRL;
        using SysClock    = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_CTRL;
        using RefClock    = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_CTRL;
        using RefClockDiv = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_DIV;
        using UsbClock    = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_USB_CTRL;
        using AdcClock    = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_ADC_CTRL;
        using SysSelected = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_SELECTED;
        using RefSelected = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_SELECTED;
        using XOSC        = Kvasir::Peripheral::XOSC::Registers<>;
        using RST         = Kvasir::Peripheral::RESETS::Registers<0>;
        using PLL         = Kvasir::Peripheral::PLL::Registers<0>;
        using USBPLL      = Kvasir::Peripheral::PLL::Registers<1>;
        using detail::waited;

        // clk_usb / clk_adc off first (their destinations are in reset at boot; a resus repair resets PLL_USB
        // below anyway): only the last step turns them on again, so their enables tell initResult() a finished init
        apply(clear(UsbClock::enable), clear(AdcClock::enable));

        // Voltage first, then flash timing, then the PLL switch - all three
        // still running from ROSC, so the core never executes a cycle at the
        // new frequency on the old voltage. A regulator that does not take the voltage stops the init after the
        // switches to the ROSC below (InitResult::voltageNotSet): no crystal or PLL on it, and clk_sys off a PLL a
        // core reset left it on.
        bool const voltageSet = vregInit<ClockSpeed>();
        flashInit<ClockSpeed, MaxFlashFreq>();

        // After a core-only reset (RP2040 datasheet 2.4.2.9, RP2350 7.3.1) clk_sys may still run on PLL_SYS via its
        // aux mux. So: glitchless muxes to safe sources first (Clocks::onRingOscillator: clk_ref on the ROSC divided
        // to its ceiling, clk_sys on clk_ref, polling CLK_x_SELECTED), aux muxes changed only while deselected,
        // generators without a glitchless mux disabled for two source cycles first (RP2040 2.15.3.2, RP2350
        // 8.1.3.2). Field writes on purpose: overrideDefaults would also reset AUXSRC (RP2350 Table 559).
        if(!Clocks::onRingOscillator()) { return; }

        // clk_peri has no glitchless mux: disable, let it stop, aux to clk_sys; enabled again after the PLL. Its
        // destinations (UART, SPI) are held in reset by FirstInitStep as well.
        apply(clear(PeriClock::enable));
#ifdef KVASIR_CHIP_RP2350
        // CTRL.ENABLED follows ENABLE once the generator has stopped (RP2350 Table 562, 8.1.3.2)
        if(!waited<Kvasir::Chip::ClkSelectedBound>(Kvasir::Register::isClear(PeriClock::enabled))) {
            return;
        }
        // never written before: INT 1 (Table 563, reset 1)
        apply(write(Kvasir::Peripheral::CLOCKS::Registers<>::CLK_PERI_DIV::_int,
                    value<std::uint32_t{1}>()));
#else
        // the RP2040 has no ENABLED bit: two cycles of the old source (clk_sys or slower) pass in these
        Kvasir::Core::spinCycles<64>();
#endif
        apply(write(PeriClock::AUXSRCValC::clk_sys));

        // A clk_sys resus (Clocks::Resus) survives a core reset: the clocks block is not
        // reset by SYSRESETREQ, so after an unhandled event every debugger reset boots with
        // clk_sys forced to clk_ref, the PLL switch below has no effect, and the flash
        // timing peripheryClockInit() applies for ClockSpeed reads garbage at 12 MHz - a
        // HardFault before main, on every reset, until the resus is released.
        // clk_sys is on clk_ref now (CLK_SYS_SELECTED said so), so releasing it changes nothing here.
        {
            using ResusCtrl   = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_RESUS_CTRL;
            using ResusStatus = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_RESUS_STATUS;
            if(apply(read(ResusStatus::resussed))) {
                apply(set(ResusCtrl::clear));
                apply(clear(ResusCtrl::clear));
            }
        }
        if(!voltageSet) { return; }

        // the crystal: "XOSC startup takes several milliseconds" (RP2040 datasheet 2.15.2 md l.8579, RP2350 8.1 md
        // l.25697), on clk_ref on the ROSC (Chip::XoscStableBoundFor counts at its ceiling), then
        // Clocks::XoscStartupConfig's check
        using XoscStable
          = Kvasir::Chip::XoscStableBoundFor<detail::XoscConfig<CrystalSpeed>::startupMs>;
        detail::enableCrystal<CrystalSpeed>();
        if(!waited<XoscStable>(Kvasir::Register::isSet(XOSC::STATUS::stable))
           || !detail::crystalSettled<CrystalSpeed>())
        {
            return;
        }

        // clk_ref to the crystal, then undivided, and the TIMER's microsecond tick from it - before the PLLs, so a PLL
        // that does not lock leaves clk_sys on clk_ref = the crystal (where a resus puts it too) with the TIMER and the
        // frequency counter's reference true. The source before the divisor: on the ROSC clk_ref runs divided
        // (Clocks::onRingOscillator: 4 after the RP2350 A3+ boot ROM's ROSC divisor 2, "Bootrom changes" md l.66193),
        // and the divisor 1 first would run clk_ref - and clk_sys on it - at up to 96 MHz (8.3 md l.27989), over
        // clk_ref's 25 MHz (8.2.2 md l.27756); a divisor that shrinks goes after the source (pico-sdk clock_configure,
        // md l.26025)
        apply(write(RefClock::SRCValC::xosc_clksrc));
        if(!waited<Kvasir::Chip::ClkSelectedBound>(
             detail::selectedIs<RefSelected::clk_ref_selected, 2>))
        {
            return;
        }
        apply(write(RefClockDiv::_int, value<std::uint32_t{1}>()));
        if(!detail::startTimerTick<CrystalSpeed>()) { return; }

        {   //sys pll, waited for with clk_sys on clk_ref = the crystal
            if(!detail::pllInit<PLL,
                                RST::RESET::pll_sys,
                                RST::RESET_DONE::pll_sys,
                                pllSettings,
                                Kvasir::Chip::PllLockBound<CrystalSpeed>>())
            {
                return;
            }

            // clk_sys to the PLL: the aux mux first, while the glitchless mux is still on clk_ref, then the switch
            apply(write(SysClock::AUXSRCValC::clksrc_pll_sys));
            apply(write(SysClock::SRCValC::clksrc_clk_sys_aux));
            if(!waited<Kvasir::Chip::ClkSelectedBound>(
                 detail::selectedIs<SysSelected::clk_sys_selected, 1>))
            {
                return;
            }

            // enable periphery clock (its AUXSRC is clk_sys since above)
            apply(set(PeriClock::enable));
        }

        {   //usb pll, waited for with clk_sys on PLL_SYS
            if(!detail::pllInit<USBPLL,
                                RST::RESET::pll_usb,
                                RST::RESET_DONE::pll_usb,
                                usbPllSettings,
                                Kvasir::Chip::PllLockBound<ClockSpeed>>())
            {
                return;
            }

            // clk_usb / clk_adc: no glitchless mux - the source while disabled, then the enable
            apply(write(UsbClock::AUXSRCValC::clksrc_pll_usb),
                  write(AdcClock::AUXSRCValC::clksrc_pll_usb));
            apply(set(UsbClock::enable), set(AdcClock::enable));
        }
    }

    // The tree Clocks::DegradedBoot (DegradedBoot.hpp) runs a boot on after the clock init failed in too many boots in
    // a row - degraded only as far as the failure needs, from whatever coreClockInit() left:
    //   - the crystal does not run (XOSC.STATUS.STABLE clear): clk_ref and clk_sys on the ring oscillator
    //     (Clocks::onRingOscillator), clk_ref divided to CrystalSpeed at most (the ticks, below);
    //   - the crystal runs: clk_ref on it, undivided, and clk_sys on clk_ref - unless clk_sys runs from a locked
    //     PLL_SYS already (only PLL_USB failed), where it stays at full speed;
    // then clk_peri on clk_sys, the TIMER ticking from clk_ref, and clk_usb and clk_adc on the ROSC or the crystal: no
    // block an image releases from reset is left without a clock (the datasheets do not say whether RESET_DONE of a
    // block whose clock is stopped ever sets: RP2040 2.14 md l.8149, RP2350 7.5 md l.25101). clk_usb's source is what
    // Clocks::degradedTree() reads, so it is the last step: a degraded tree is one whose set-up finished (a TIMER that
    // did not leave reset ends in the clock-init panic, not in a boot DegradedBoot never retries). No PLL is started
    // here - the switch-up tries both (startSources). Registers only, every wait bounded; DegradedTree::none if a
    // switch does not show (the boot then ends in the clock-init panic, as without a DegradedBoot).
    //
    // What it costs until the switch-up: no USB (clk_usb is not 48 MHz) and the ADC at the ROSC's or the crystal's
    // rate; on the ROSC and the crystal every divider a driver computed from ClockSpeed (baud rates, PWM, SPI, I2C,
    // PIO) and SysTick off by ClockSpeed / clk_sys; the flash on the boot's divider (peripheryClockInit); on the ROSC
    // the RP2040's RTC without its clock (Rtc.hpp runs clk_rtc from the crystal) and its stack canary seed from ROSC
    // RANDOMBIT, which is not random while the cores run from the ROSC (Rosc.hpp). The ticks: the TIMER and the
    // watchdog count ticks the tick generators divide from clk_ref ("The tick generators use clk_ref as their reference
    // clock", RP2350 datasheet 8.5 md l.28460, TICKS TIMER0/TIMER1/WATCHDOG; the RP2040's one tick, WATCHDOG.TICK,
    // "clk_tick, is driven from clk_ref", 4.7.2 md l.25462), CYCLES = the crystal's MHz as every Kvasir tick user
    // writes it (a TimerClock, the watchdog driver, coreClockInit). On the crystal they stay exact. On the ROSC clk_ref
    // runs at most at the crystal's frequency (the ROSC itself: RP2040 1.8 to 12 MHz, RP2350 4.6 to 24 MHz at the boot
    // divisor, datasheet 2.17 md l.10643, 8.3.1 md l.27985), so a tick lasts at least its microsecond: the TIMER runs
    // slow (up to the crystal over the ROSC's floor) and an armed watchdog bites late, never early. At the switch-up
    // the ticks become exact: a watchdog count started on slower ticks then ends sooner, but never before its nominal
    // time after the last feed.
    template<auto CrystalSpeed>
    [[nodiscard]] Clocks::DegradedTree degradedClockInit() {
        using Clocks::DegradedTree;
        using Clk  = Kvasir::Peripheral::CLOCKS::Registers<>;
        using XOSC = Kvasir::Peripheral::XOSC::Registers<>;
        using RST  = Kvasir::Peripheral::RESETS::Registers<0>;
        using PLL  = Kvasir::Peripheral::PLL::Registers<0>;
        using detail::selectedIs;
        using detail::waited;
        using Kvasir::Register::value;
        // clk_ref on the ROSC at most at the crystal's frequency, and never over its own ceiling
        constexpr std::uint64_t roscRefHz
          = Kvasir::Rp::ChipTraits::clkRefOnRoscHzCeiling < static_cast<std::uint64_t>(CrystalSpeed)
            ? Kvasir::Rp::ChipTraits::clkRefOnRoscHzCeiling
            : static_cast<std::uint64_t>(CrystalSpeed);
        DegradedTree tree{DegradedTree::none};
        if(!apply(read(XOSC::STATUS::stable))) {
            if(!Clocks::onRingOscillator<roscRefHz>()) { return DegradedTree::none; }
            tree = DegradedTree::ringOscillator;
        } else {
            // CLK_x_SELECTED is one-hot over CTRL.SRC (RP2040 Tables 222/225, RP2350 Tables 558/561); the source before
            // a divisor that shrinks (onRingOscillator)
            if(!Kvasir::Register::fieldEquals(
                 selectedIs<Clk::CLK_REF_SELECTED::clk_ref_selected, 2>))
            {
                apply(write(Clk::CLK_REF_CTRL::SRCValC::xosc_clksrc));
                if(!waited<Kvasir::Chip::ClkSelectedBound>(
                     selectedIs<Clk::CLK_REF_SELECTED::clk_ref_selected, 2>))
                {
                    return DegradedTree::none;
                }
            }
            apply(write(Clk::CLK_REF_DIV::_int, value<std::uint32_t{1}>()));
            bool const onPllSys
              = detail::sysOnPllSys()
             && detail::pllUp<PLL, RST::RESET::pll_sys, RST::RESET_DONE::pll_sys>();
            if(!onPllSys) {
                apply(write(Clk::CLK_SYS_CTRL::SRCValC::clk_ref));
                if(!waited<Kvasir::Chip::ClkSelectedBound>(
                     selectedIs<Clk::CLK_SYS_SELECTED::clk_sys_selected, 0>))
                {
                    return DegradedTree::none;
                }
                // a previous image's divider would stay (onRingOscillator)
                apply(write(Clk::CLK_SYS_DIV::_int, value<std::uint32_t{1}>()),
                      write(Clk::CLK_SYS_DIV::frac, value<std::uint32_t{0}>()));
            }
            tree = onPllSys ? DegradedTree::noPllUsb : DegradedTree::crystal;
        }
        // clk_peri: no glitchless mux - stopped, clk_sys, started
        if(!detail::stopGenerator<Clk::CLK_PERI_CTRL>()) { return DegradedTree::none; }
        apply(write(Clk::CLK_PERI_CTRL::AUXSRCValC::clk_sys));
        apply(set(Clk::CLK_PERI_CTRL::enable));
        if(!detail::startTimerTick<CrystalSpeed>()) { return DegradedTree::none; }
        bool const marked = tree == DegradedTree::ringOscillator
                            ? detail::usbAdcTo(Clk::CLK_USB_CTRL::AUXSRCValC::rosc_clksrc_ph,
                                               Clk::CLK_ADC_CTRL::AUXSRCValC::rosc_clksrc_ph)
                            : detail::usbAdcTo(Clk::CLK_USB_CTRL::AUXSRCValC::xosc_clksrc,
                                               Clk::CLK_ADC_CTRL::AUXSRCValC::xosc_clksrc);
        return marked ? tree : DegradedTree::none;
    }

    // The crystal started without waiting for it, for a caller that looks again on its own schedule (Clocks::
    // DegradedBoot's and Resus's tries: a missing crystal does not hold their loop for XoscStableBoundFor): XOSC.CTRL
    // enabled at CrystalSpeed's range unless STATUS.ENABLED says it is on already (a write while it starts up is not
    // needed; RP2040 datasheet Table 260 md l.10562, RP2350 Table 600 md l.27908). True if it is stable now.
    template<auto CrystalSpeed>
    [[nodiscard]] bool crystalStarted() {
        using XOSC = Kvasir::Peripheral::XOSC::Registers<>;
        auto const st
          = apply(read(XOSC::STATUS::stable), read(XOSC::STATUS::enabled));   // one load
        if(static_cast<bool>(get<0>(st))) { return detail::crystalSettled<CrystalSpeed>(); }
        if(!static_cast<bool>(get<1>(st))) { detail::enableCrystal<CrystalSpeed>(); }
        return false;
    }

    // Run time, from a tree whose clk_sys does not run from the sources the full tree needs (a degraded boot, a resus
    // repair that failed): the voltage, the crystal and the PLLs brought up while interrupts stay on - nothing
    // switched, every wait bounded at the clock it runs on (clk_sys = clk_ref: the ROSC divided to Rp::ChipTraits::
    // clkRefOnRoscHzCeiling, or the crystal; PLL_SYS after a repair or a degraded boot that stopped at PLL_USB). A PLL
    // clk_sys runs from is left alone; clk_usb and clk_adc are stopped first only when they run from PLL_USB (a
    // degraded tree's run from the ROSC or the crystal). Returns the first source that does not come, or ok;
    // switchToSources() then switches, interrupts masked. From flash: the timing it is on reads at every clock involved
    // (flashInit's boot timing, a degraded boot's degradedFlashTiming, the resus's TimingAtClkRef).
    // The caller's loop is held for the bound of a source that does not come: a crystal that does not start
    // XoscStableBoundFor (~0.85 s on an RP2350 A2's ROSC at the default's 100 ms, measured; ~1.6 s at XoscDelay's
    // 192 ms) and then XoscMeasured's counts (Clocks::XoscStartupConfig), a PLL PllLockBound - crystalStarted() first keeps the crystal's out of it.
    template<auto ClockSpeed,
             auto CrystalSpeed>
    [[nodiscard]] Clocks::InitResult startSources() {
        using Clocks::InitResult;
        static constexpr auto pllSettings    = detail::calcPllSettings(ClockSpeed, CrystalSpeed);
        static constexpr auto usbPllSettings = detail::calcPllSettings(48'000'000, CrystalSpeed);
        // clk_sys = clk_ref while a PLL comes up: on the ROSC or the crystal, whichever may be faster
        static constexpr std::uint64_t refHz
          = Kvasir::Rp::ChipTraits::clkRefOnRoscHzCeiling > static_cast<std::uint64_t>(CrystalSpeed)
            ? Kvasir::Rp::ChipTraits::clkRefOnRoscHzCeiling
            : static_cast<std::uint64_t>(CrystalSpeed);
        using Clk    = Kvasir::Peripheral::CLOCKS::Registers<>;
        using XOSC   = Kvasir::Peripheral::XOSC::Registers<>;
        using RST    = Kvasir::Peripheral::RESETS::Registers<0>;
        using PLL    = Kvasir::Peripheral::PLL::Registers<0>;
        using USBPLL = Kvasir::Peripheral::PLL::Registers<1>;
        using detail::waited;

        if(!vregInit<ClockSpeed>()) { return InitResult::voltageNotSet; }
        if(!apply(read(XOSC::STATUS::stable))) {
            using XoscStable
              = Kvasir::Chip::XoscStableBoundFor<detail::XoscConfig<CrystalSpeed>::startupMs>;
            detail::enableCrystal<CrystalSpeed>();
            if(!waited<XoscStable>(Kvasir::Register::isSet(XOSC::STATUS::stable))
               || !detail::crystalSettled<CrystalSpeed>())
            {
                return InitResult::xoscNotStable;
            }
        }
        bool const sysOnPll = detail::sysOnPllSys();
        if(sysOnPll) {
            if(!detail::pllUp<PLL, RST::RESET::pll_sys, RST::RESET_DONE::pll_sys>()) {
                return InitResult::pllSysNotLocked;
            }
        } else if(!detail::pllInit<PLL,
                                   RST::RESET::pll_sys,
                                   RST::RESET_DONE::pll_sys,
                                   pllSettings,
                                   Kvasir::Chip::PllLockBound<refHz>>())
        {
            return InitResult::pllSysNotLocked;
        }
        using Usb                    = Clk::CLK_USB_CTRL;
        using Adc                    = Clk::CLK_ADC_CTRL;
        constexpr auto usbFromPllUsb = Usb::AUXSRCVal::clksrc_pll_usb;
        constexpr auto adcFromPllUsb = Adc::AUXSRCVal::clksrc_pll_usb;
        bool const     usbOn         = detail::runsFrom<Usb, usbFromPllUsb>();
        bool const     adcOn         = detail::runsFrom<Adc, adcFromPllUsb>();
        if(usbOn && adcOn && detail::pllUp<USBPLL, RST::RESET::pll_usb, RST::RESET_DONE::pll_usb>())
        {
            return InitResult::ok;
        }
        // nothing may run from PLL_USB while it is reset
        if(usbOn || adcOn) {
            if(!detail::stopGenerator<Usb>() || !detail::stopGenerator<Adc>()) {
                return InitResult::clockSwitchNotTaken;
            }
        }
        bool const usbPll = sysOnPll ? detail::pllInit<USBPLL,
                                                       RST::RESET::pll_usb,
                                                       RST::RESET_DONE::pll_usb,
                                                       usbPllSettings,
                                                       Kvasir::Chip::PllLockBound<ClockSpeed>>()
                                     : detail::pllInit<USBPLL,
                                                       RST::RESET::pll_usb,
                                                       RST::RESET_DONE::pll_usb,
                                                       usbPllSettings,
                                                       Kvasir::Chip::PllLockBound<refHz>>();
        return usbPll ? InitResult::ok : InitResult::pllUsbNotLocked;
    }

    // The switch after startSources(), interrupts masked by the caller (a resus handler must not run in between): the
    // sources checked again (an interrupt may have reset one meanwhile), clk_ref to the crystal, then undivided (the
    // source before a divisor that shrinks), clk_sys to PLL_SYS through its aux mux (from clk_ref), clk_peri on
    // clk_sys, clk_usb and clk_adc to PLL_USB - each step skipped where the tree is there already. The TIMER's tick
    // keeps the crystal's CYCLES the boot (or degradedClockInit) gave it: it ticks true from the switch on. Registers
    // only, bounded; the flash keeps its timing (the caller puts the full clock's back: peripheryClockInit, or the
    // resus's saved one).
    template<auto ClockSpeed,
             auto CrystalSpeed>
    [[nodiscard]] Clocks::InitResult switchToSources() {
        using Clocks::InitResult;
        using Clk    = Kvasir::Peripheral::CLOCKS::Registers<>;
        using XOSC   = Kvasir::Peripheral::XOSC::Registers<>;
        using RST    = Kvasir::Peripheral::RESETS::Registers<0>;
        using PLL    = Kvasir::Peripheral::PLL::Registers<0>;
        using USBPLL = Kvasir::Peripheral::PLL::Registers<1>;
        using detail::selectedIs;
        using detail::waited;
        using Kvasir::Register::fieldEquals;
        using Kvasir::Register::value;

        if(!apply(read(XOSC::STATUS::stable))) { return InitResult::xoscNotStable; }
        if(!detail::pllUp<PLL, RST::RESET::pll_sys, RST::RESET_DONE::pll_sys>()) {
            return InitResult::pllSysNotLocked;
        }
        if(!detail::pllUp<USBPLL, RST::RESET::pll_usb, RST::RESET_DONE::pll_usb>()) {
            return InitResult::pllUsbNotLocked;
        }
        // CLK_x_SELECTED is one-hot over CTRL.SRC (RP2040 Tables 222/225, RP2350 Tables 558/561)
        if(!fieldEquals(selectedIs<Clk::CLK_REF_SELECTED::clk_ref_selected, 2>)) {
            apply(write(Clk::CLK_REF_CTRL::SRCValC::xosc_clksrc));
            if(!waited<Kvasir::Chip::ClkSelectedBound>(
                 selectedIs<Clk::CLK_REF_SELECTED::clk_ref_selected, 2>))
            {
                return InitResult::clockSwitchNotTaken;
            }
        }
        apply(write(Clk::CLK_REF_DIV::_int, value<std::uint32_t{1}>()));
        if(!detail::sysOnPllSys()) {
            // the aux mux changes only while the glitchless one is on clk_ref
            if(!fieldEquals(selectedIs<Clk::CLK_SYS_SELECTED::clk_sys_selected, 0>)) {
                apply(write(Clk::CLK_SYS_CTRL::SRCValC::clk_ref));
                if(!waited<Kvasir::Chip::ClkSelectedBound>(
                     selectedIs<Clk::CLK_SYS_SELECTED::clk_sys_selected, 0>))
                {
                    return InitResult::clockSwitchNotTaken;
                }
            }
            apply(write(Clk::CLK_SYS_CTRL::AUXSRCValC::clksrc_pll_sys));
            apply(write(Clk::CLK_SYS_CTRL::SRCValC::clksrc_clk_sys_aux));
            if(!waited<Kvasir::Chip::ClkSelectedBound>(
                 selectedIs<Clk::CLK_SYS_SELECTED::clk_sys_selected, 1>))
            {
                return InitResult::clockSwitchNotTaken;
            }
        }
        // clk_peri: a failed init stopped it before the crystal; on clk_sys it follows the switch without a glitch
        if(!apply(read(Clk::CLK_PERI_CTRL::enable))) {
            apply(write(Clk::CLK_PERI_CTRL::AUXSRCValC::clk_sys));
            apply(set(Clk::CLK_PERI_CTRL::enable));
        }
        using Usb = Clk::CLK_USB_CTRL;
        using Adc = Clk::CLK_ADC_CTRL;
        if(!detail::runsFrom<Usb, Usb::AUXSRCVal::clksrc_pll_usb>()
           || !detail::runsFrom<Adc, Adc::AUXSRCVal::clksrc_pll_usb>())
        {
            if(!detail::usbAdcTo(Usb::AUXSRCValC::clksrc_pll_usb, Adc::AUXSRCValC::clksrc_pll_usb))
            {
                return InitResult::clockSwitchNotTaken;
            }
        }
        return InitResult::ok;
    }
}}   // namespace Kvasir::DefaultClockSettings

namespace Kvasir { namespace Clocks {
    /// How far DefaultClockSettings::coreClockInit() got, read back from the registers it leaves: it runs before
    /// initMemory() and keeps no variable. Its last step alone turns clk_usb and clk_adc on from PLL_USB (its first
    /// turned them off), so both running from PLL_USB = every wait held. Otherwise the first fact of the finished tree
    /// that does not hold, in the order coreClockInit establishes them, names where it stopped (InitResult's comments
    /// say the state each leaves); `degraded` for the tree a Clocks::DegradedBoot runs a boot on. A block's registers
    /// are read only after RESETS says it is out of reset. Read right after a clock init (peripheryClockInit at boot,
    /// the resus after a repair): it describes the last one, and an application that turns clk_usb or clk_adc off or
    /// onto another source later reads "not ok".
    [[nodiscard]] inline bool initFinished() {
        using Clk = Kvasir::Peripheral::CLOCKS::Registers<>;
        using Usb = Clk::CLK_USB_CTRL;
        using Adc = Clk::CLK_ADC_CTRL;
        return DefaultClockSettings::detail::runsFrom<Usb, Usb::AUXSRCVal::clksrc_pll_usb>()
            && DefaultClockSettings::detail::runsFrom<Adc, Adc::AUXSRCVal::clksrc_pll_usb>();
    }

    /// Which of DefaultClockSettings::degradedClockInit()'s trees this boot runs on (Clocks::DegradedBoot), from the
    /// registers: clk_usb from the ring oscillator or the crystal, which nothing else in Kvasir sets (one load), and
    /// for the crystal whether clk_sys runs from PLL_SYS. DegradedTree::none for every other tree.
    [[nodiscard]] inline DegradedTree degradedTree() {
        using Usb    = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_USB_CTRL;
        auto const c = apply(read(Usb::enable), read(Usb::auxsrc));
        if(!static_cast<bool>(get<0>(c))) { return DegradedTree::none; }
        if(get<1>(c) == Usb::AUXSRCVal::rosc_clksrc_ph) { return DegradedTree::ringOscillator; }
        if(get<1>(c) != Usb::AUXSRCVal::xosc_clksrc) { return DegradedTree::none; }
        return DefaultClockSettings::detail::sysOnPllSys() ? DegradedTree::noPllUsb
                                                           : DegradedTree::crystal;
    }

    /// This boot runs on one of DefaultClockSettings::degradedClockInit()'s trees (Clocks::DegradedBoot).
    [[nodiscard]] inline bool degraded() { return degradedTree() != DegradedTree::none; }

    /// clk_sys runs from PLL_SYS (its glitchless mux on the aux input, the aux mux on PLL_SYS): what a test that
    /// breaks PLL_SYS on purpose asks first - never break the clock clk_sys runs from.
    [[nodiscard]] inline bool sysOnPllSys() { return DefaultClockSettings::detail::sysOnPllSys(); }

    namespace detail {
        // initResult()'s answer when the init did not finish: cold and out of line, so the boot that works pays two
        // register reads
        [[nodiscard,
          gnu::noinline,
          gnu::cold]] inline InitResult
        whereInitStopped() {
            using Clk = Kvasir::Peripheral::CLOCKS::Registers<>;
            using RST = Kvasir::Peripheral::RESETS::Registers<0>;
            if(degraded()) { return InitResult::degraded; }
#ifdef KVASIR_CHIP_RP2350
            // RP2350 datasheet 6.3.2 md l.22847, VREG bit 15 (md l.23292); never set unless a VSEL write is in flight
            if(apply(read(Kvasir::Peripheral::POWMAN::Registers<>::VREG::update_in_progress))) {
                return InitResult::voltageNotSet;
            }
#endif
            // CLK_x_SELECTED is one-hot over CTRL.SRC (RP2040 datasheet Tables 222/225, RP2350 Tables 558/561)
            if(!apply(read(Kvasir::Peripheral::XOSC::Registers<>::STATUS::stable))) {
                // The crystal step comes after the switches to the ROSC: an init that stopped before it (a power-on
                // boot, where the crystal was never enabled) left them undone
                bool const parked
                  = Register::fieldEquals(DefaultClockSettings::detail::
                                            selectedIs<Clk::CLK_REF_SELECTED::clk_ref_selected, 0>)
                 && Register::fieldEquals(DefaultClockSettings::detail::
                                            selectedIs<Clk::CLK_SYS_SELECTED::clk_sys_selected, 0>);
#ifdef KVASIR_CHIP_RP2350
                // and the RP2350's clk_peri is stopped before the crystal is enabled: a stop that did not show
                // (CTRL.ENABLE clear, ENABLED still set: Table 562) ended the init there, not the crystal
                if(parked) {
                    auto const peri
                      = apply(read(Clk::CLK_PERI_CTRL::enable), read(Clk::CLK_PERI_CTRL::enabled));
                    if(!static_cast<bool>(get<0>(peri)) && static_cast<bool>(get<1>(peri))) {
                        return InitResult::clockSwitchNotTaken;
                    }
                }
#endif
                return parked ? InitResult::xoscNotStable : InitResult::clockSwitchNotTaken;
            }
            if(!Register::fieldEquals(
                 DefaultClockSettings::detail::selectedIs<Clk::CLK_REF_SELECTED::clk_ref_selected,
                                                          2>))
            {
                return InitResult::clockSwitchNotTaken;
            }
#ifdef KVASIR_CHIP_RP2350
            if(apply(read(RST::RESET::timer0)) || !apply(read(RST::RESET_DONE::timer0))) {
                return InitResult::clockSwitchNotTaken;
            }
#else
            if(apply(read(RST::RESET::timer)) || !apply(read(RST::RESET_DONE::timer))) {
                return InitResult::clockSwitchNotTaken;
            }
#endif
            if(apply(read(RST::RESET::pll_sys)) || !apply(read(RST::RESET_DONE::pll_sys))
               || !apply(read(PllSys::CS::lock)))
            {
                return InitResult::pllSysNotLocked;
            }
            if(!Register::fieldEquals(
                 DefaultClockSettings::detail::selectedIs<Clk::CLK_SYS_SELECTED::clk_sys_selected,
                                                          1>))
            {
                return InitResult::clockSwitchNotTaken;
            }
            if(apply(read(RST::RESET::pll_usb)) || !apply(read(RST::RESET_DONE::pll_usb))
               || !apply(read(PllUsb::CS::lock)))
            {
                return InitResult::pllUsbNotLocked;
            }
            // every fact holds but the last step's: a core reset kept the previous tree running and the switch to the
            // ring oscillator (or clk_peri's stop) did not finish
            return InitResult::clockSwitchNotTaken;
        }
    }   // namespace detail

    [[nodiscard]] inline InitResult initResult() {
        return initFinished() ? InitResult::ok : detail::whereInitStopped();
    }
}}   // namespace Kvasir::Clocks

namespace Kvasir { namespace DefaultClockSettings {
    namespace detail {
        // The clocks did not come up at boot: the panic (Cause::clockInit, detail = the InitResult), whose default
        // reaction records it, lets the printer take the log and reboots through the chip's Port::Reboot - the next
        // boot's report names it, and a crystal that was only slow gets a fresh start (every RP image's
        // Clocks::DegradedBoot ends the loop after its limit). Cold, out of line: the boot that works pays a branch.
        //
        // An init that stopped before the TIMER's tick (the voltage, the switches, the crystal, clk_ref's switch) left
        // the TIMER held in reset, and the panic's log line and its reboot's printer waits read TIMERAWL: what a read
        // of a block that RESETS holds returns, the datasheets do not say (RP2040 2.14 md l.8149, RP2350 7.5 md
        // l.25101; the RP2350 read it as standing still). The TIMER is released first, its tick from clk_ref: on the
        // ROSC it counts off by the ROSC's error, but it counts. After the attribution, which reads the TIMER's reset
        // bit.
        template<auto CrystalSpeed>
        [[noreturn,
          gnu::noinline,
          gnu::cold]] void
        clockInitFailed() {
            auto const r = Clocks::initResult();
            using RST    = Kvasir::Peripheral::RESETS::Registers<0>;
#ifdef KVASIR_CHIP_RP2350
            bool const timerHeld = apply(read(RST::RESET::timer0));
#else
            bool const timerHeld = apply(read(RST::RESET::timer));
#endif
            if(timerHeld) { static_cast<void>(startTimerTick<CrystalSpeed>()); }
            KVASIR_DETAIL_FATAL_AT(Kvasir::Panic::Cause::clockInit,
                                   KVASIR_DETAIL_CALLER_PC(),
                                   static_cast<std::uint32_t>(r),
                                   "clock init did not finish: {} ({})",
                                   Clocks::name(r),
                                   static_cast<std::uint32_t>(r));
        }
    }   // namespace detail

    // After initMemory(): the RAM functions exist now, and interrupts are not enabled yet.
    // First coreClockInit's outcome (Clocks::initResult()): a log line and a panic record survive from here on, and
    // the flash timing below is for the full clock. Then the flash's read mode and final timing (see flashInit() and
    // XipReadMode). A project that passes its own MaxFlashFreq must pass it here and to coreClockInit(). A boot a
    // Clocks::DegradedBoot runs on a degraded tree (its postMemoryInit came first) sets the read mode up too (quad
    // enable, continuous EBh: at boot, while no other core runs) but keeps the boot's divider while clk_sys does not
    // run from PLL_SYS (detail::degradedFlashTiming): the switch-up then writes CLKDIV alone (flashAfterSwitchUp), no
    // XIP-off command and no idle QMI needed at run time.
    template<auto ClockSpeed,
             auto CrystalSpeed,
             auto MaxFlashFreq = 100'000'000>
    void peripheryClockInit() {
        bool const finished = Clocks::initFinished();
        if(!finished && !Clocks::degraded()) [[unlikely]] {
            detail::clockInitFailed<CrystalSpeed>();
        }
#ifdef KVASIR_CHIP_RP2350
        Kvasir::detail::enableQuadIo();
        std::uint32_t const timing = finished || detail::sysOnPllSys()
                                     ? detail::flashTiming<ClockSpeed, MaxFlashFreq>()
                                     : detail::degradedFlashTiming<ClockSpeed, MaxFlashFreq>();
        auto const          irqs   = Kvasir::Core::IrqState::saveAndMask();
        Kvasir::detail::XipReadMode::apply(timing);
        irqs.restore();
#endif
    }

    // After a degraded boot's switch-up (Clocks::DegradedBoot: clk_sys at ClockSpeed again), the flash's divider for
    // the full clock. A degraded boot's timing differs from the final one in CLKDIV alone
    // (detail::degradedFlashTiming), the one M0_TIMING field that "can be changed on-the-fly, even when the QMI is
    // currently accessing memory in this address window. All other parameters must only be changed when the QMI is
    // idle." (RP2350 datasheet, M0_TIMING md l.60489): a register write at run time, no XIP-off command (a BBh flash's
    // quad enable reads the JEDEC id in direct mode), no read-mode change and no wait for an idle QMI, whatever another
    // core fetches from flash meanwhile - those ran at boot, single-core (peripheryClockInit). The RP2040's flash runs
    // on boot2's SSI divider, which no clock switch changes: nothing to do there.
    template<auto ClockSpeed,
             auto MaxFlashFreq = 100'000'000>
    void flashAfterSwitchUp() {
#ifdef KVASIR_CHIP_RP2350
        using Timing = Kvasir::Peripheral::QMI::Registers<0>::M0_TIMING;
        using Kvasir::Register::value;
        apply(write(Timing::clkdiv, value<detail::flashClkdiv<ClockSpeed, MaxFlashFreq>()>()));
#endif
    }

    // An application's whole ClockSettings in one line:
    //     using ClockSettings = Kvasir::DefaultClockSettings::Settings<ClockSpeed, CrystalSpeed>;
    // Provides (the clock check), coreClockInit() and peripheryClockInit() with the same three arguments, so the
    // numbers are spelled once - and what Clocks::DegradedBoot (every RP image, DegradedBoot.hpp) and Clocks::Resus
    // need of the tree: how a boot's init is judged (initFinished), the degraded tree (degradedClockInit), and the
    // run-time switch-up (crystalStarted, startSources, switchToSources, flashAfterSwitchUp). A ClockSettings with a
    // coreClockInit of its own derives from Settings when its tree is this one, or names these for its own.
    template<auto ClockSpeed, auto CrystalSpeed, auto MaxFlashFreq = 100'000'000>
    struct Settings {
        using Provides = DefaultClockSettings::Provides<ClockSpeed, CrystalSpeed>;

        static void coreClockInit() {
            DefaultClockSettings::coreClockInit<ClockSpeed, CrystalSpeed, MaxFlashFreq>();
        }

        static void peripheryClockInit() {
            DefaultClockSettings::peripheryClockInit<ClockSpeed, CrystalSpeed, MaxFlashFreq>();
        }

        [[nodiscard]] static bool initFinished() { return Clocks::initFinished(); }

        [[nodiscard]] static Clocks::DegradedTree degradedClockInit() {
            return DefaultClockSettings::degradedClockInit<CrystalSpeed>();
        }

        [[nodiscard]] static bool crystalStarted() {
            return DefaultClockSettings::crystalStarted<CrystalSpeed>();
        }

        [[nodiscard]] static Clocks::InitResult startSources() {
            return DefaultClockSettings::startSources<ClockSpeed, CrystalSpeed>();
        }

        [[nodiscard]] static Clocks::InitResult switchToSources() {
            return DefaultClockSettings::switchToSources<ClockSpeed, CrystalSpeed>();
        }

        static void flashAfterSwitchUp() {
            DefaultClockSettings::flashAfterSwitchUp<ClockSpeed, MaxFlashFreq>();
        }
    };

    /// What Clocks::DegradedBoot and Clocks::Resus need of a ClockSettings (Settings above has it all).
    template<typename CS>
    concept DegradableClockSettings = requires {
        typename CS::Provides;
        { CS::initFinished() } -> std::same_as<bool>;
        { CS::degradedClockInit() } -> std::same_as<Clocks::DegradedTree>;
        { CS::crystalStarted() } -> std::same_as<bool>;
        { CS::startSources() } -> std::same_as<Clocks::InitResult>;
        { CS::switchToSources() } -> std::same_as<Clocks::InitResult>;
        CS::flashAfterSwitchUp();
    };
}}   // namespace Kvasir::DefaultClockSettings

namespace Kvasir { namespace Clocks { namespace detail {
    // Clocks::DegradedBoot's verdict on this boot, for the boot report (Port::StartNotes below): what the boot's own
    // clock init stopped at (ok: this boot is not degraded), the tree it runs on, and in how many boots in a row before
    // it the init had failed. Set after initMemory(), by DegradedBoot's postMemoryInit.
    inline InitResult    degradedBy{InitResult::ok};
    inline DegradedTree  degradedOn{DegradedTree::none};
    inline std::uint32_t failedBootsBefore{0};
}}}   // namespace Kvasir::Clocks::detail

namespace Kvasir { namespace Clocks { namespace detail {
    // What running on a degraded tree costs until the switch-up (degradedClockInit's list), for the boot report
    [[nodiscard]] constexpr std::string_view costOf(DegradedTree t) {
        switch(t) {
        case DegradedTree::noPllUsb:
            return "no USB, the ADC at the crystal's rate, the rest at full speed";
        case DegradedTree::crystal:
            return "no USB; the ADC, SysTick and the dividers computed for the full clock off "
                   "by ClockSpeed / the crystal; the TIMER and the watchdog exact";
        case DegradedTree::ringOscillator:
            return "no USB; the ADC, SysTick and the dividers computed for the full clock off; "
                   "the TIMER and the watchdog slow, never fast";
        case DegradedTree::none: return "";
        }
        return "";
    }
}}}   // namespace Kvasir::Clocks::detail

/// The chip's line in the boot report (kvasir/Port/Hooks.hpp, Boot::logBoot): why this boot runs degraded, on which
/// tree, and what that costs; nothing for a boot whose clocks came up.
template<typename... Ts>
struct Kvasir::Port::StartNotes<Kvasir::Tag::User, Ts...> {
    void operator()() const {
        auto const tree = Kvasir::Clocks::detail::degradedOn;
        if(tree == Kvasir::Clocks::DegradedTree::none) { return; }
        UC_LOG_W(
          "boot: clocks degraded: the clock init failed in {} boots in a row ({}), "
          "this one runs on {} - {}; the crystal and the PLLs are retried from the main loop",
          Kvasir::Clocks::detail::failedBootsBefore + 1,
          Kvasir::Clocks::name(Kvasir::Clocks::detail::degradedBy),
          Kvasir::Clocks::name(tree),
          Kvasir::Clocks::detail::costOf(tree));
    }
};
