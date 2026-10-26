#pragma once
#include "Clocks.hpp"
#include "DegradedBoot.hpp"   // Clocks::RetryPauses
#include "Io.hpp"
#include "PinConfig.hpp"
#include "Rom.hpp"
#include "WaitBounds.hpp"
#include "XipControl.hpp"
#include "chip/Interrupt.hpp"
#include "clock_config.hpp"
#include "core/Nvic.hpp"
#include "core/Special.hpp"
#include "kvasir/Port/Extends.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Runtime/Panic.hpp"
#include "kvasir/StartUp/RamVectorTable.hpp"
#include "kvasir/Util/Backoff.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/attributes.hpp"
#include "peripherals/CLOCKS.hpp"

#include <atomic>
#include <bit>
#include <chrono>
#include <cstdint>
#include <type_traits>
#include <utility>

// Small clock-block peripherals for a Startup list, and the frequency counter.
//
// Clocks::Output<Pin, Source, Div>: one of the four GPOUT generators on its pin, a clock
// source divided by a 16.16 divider. The RP2350A's GPOUT pins are GPIO 13 / 21 (GPOUT0),
// 15 / 23 (GPOUT1), 24 (GPOUT2), 25 (GPOUT3); another pin is a build error.
//
//   using ClockOut = Kvasir::Clocks::Output<HW::Pin::clk_out, Kvasir::Clocks::Source::clkSys, 10>;
//
// Clocks::FrequencyCounter<ClkRefHz>: the FC0 block, any clock source in kHz.
//
// Clocks::Resus<ClockSettings, TimeClock, Config>: the resuscitation of clk_sys. If the system clock
// stops for `timeout` clk_ref cycles, the hardware switches clk_sys to clk_ref and raises
// the `clocks` interrupt; the handler rebuilds the clock tree from RAM and verifies it with
// the frequency counter. Details at the struct.
//
// Clocks::DegradedBoot<ClockSettings> (DegradedBoot.hpp, in every RP image): a clock init that keeps failing at boot
// ends in a boot on a degraded tree instead of an endless reboot loop, and in the full tree once the crystal and the
// PLLs come.
namespace Kvasir { namespace Clocks {

    // What a GPOUT can carry. lposc, clkPeri and clkHstx exist on the RP2350 only.
    enum class Source : std::uint32_t {
        pllSys,
        gpin0,
        gpin1,
        pllUsb,
        rosc,
        xosc,
        lposc,
        clkSys,
        clkUsb,
        clkAdc,
        clkRef,
        clkPeri,
        clkHstx,
    };

    namespace detail {
        using Kvasir::Io::pinNumber;

        constexpr int gpoutOf(int pin) { return PinConfig::gpoutOf(Chip::part, pin); }

        // The chip's AUXSRC value for a Source, by name: the two chips number them
        // differently and the RP2040 lacks the LPOSC, HSTX and OTP sources (it has clk_rtc).
        template<typename Ctrl>
        constexpr auto auxsrcOf(Source s) {
            using V = typename Ctrl::AUXSRCVal;
            switch(s) {
            case Source::pllSys: return V::clksrc_pll_sys;
            case Source::gpin0:  return V::clksrc_gpin0;
            case Source::gpin1:  return V::clksrc_gpin1;
            case Source::pllUsb: return V::clksrc_pll_usb;
            case Source::rosc:   return V::rosc_clksrc;
            case Source::xosc:   return V::xosc_clksrc;
            case Source::clkSys: return V::clk_sys;
            case Source::clkUsb: return V::clk_usb;
            case Source::clkAdc: return V::clk_adc;
            case Source::clkRef: return V::clk_ref;
            default:             return V::clk_sys;   // narrowed below by the static_asserts
            }
        }

        template<typename Ctrl>
        constexpr auto auxsrcOfExtra(Source s) {
#ifdef KVASIR_CHIP_RP2350
            using V = typename Ctrl::AUXSRCVal;
            switch(s) {
            case Source::lposc:   return V::lposc_clksrc;
            case Source::clkPeri: return V::clk_peri;
            case Source::clkHstx: return V::clk_hstx;
            default:              return auxsrcOf<Ctrl>(s);
            }
#else
            return auxsrcOf<Ctrl>(s);
#endif
        }
    }   // namespace detail

    template<typename Pin, Source Src, unsigned DivInt, unsigned DivFrac16 = 0>
    struct Output {
        static constexpr int      pinNumber     = detail::pinNumber(Pin{});
        static constexpr unsigned divider       = DivInt;
        static constexpr unsigned dividerFrac16 = DivFrac16;
        static constexpr int      gpout         = detail::gpoutOf(pinNumber);
        static_assert(gpout >= 0,
                      "not a GPOUT pin: GPIO 21 (GPOUT0), 23 (GPOUT1), 24 (GPOUT2), 25 (GPOUT3); "
                      "on the RP2350 also 13 (GPOUT0) and 15 (GPOUT1)");
        static_assert(!PinConfig::isRp2040(Chip::part)
                        || (Src != Source::lposc && Src != Source::clkPeri
                            && Src != Source::clkHstx),
                      "the RP2040 has no LPOSC, clk_peri or clk_hstx output");

        using Clk  = Kvasir::Peripheral::CLOCKS::Registers<>;
        using Ctrl = brigand::at_c<brigand::list<typename Clk::CLK_GPOUT0_CTRL,
                                                 typename Clk::CLK_GPOUT1_CTRL,
                                                 typename Clk::CLK_GPOUT2_CTRL,
                                                 typename Clk::CLK_GPOUT3_CTRL>,
                                   static_cast<unsigned>(gpout)>;
        using Div  = brigand::at_c<brigand::list<typename Clk::CLK_GPOUT0_DIV,
                                                 typename Clk::CLK_GPOUT1_DIV,
                                                 typename Clk::CLK_GPOUT2_DIV,
                                                 typename Clk::CLK_GPOUT3_DIV>,
                                   static_cast<unsigned>(gpout)>;

        // GPOUT has no glitchless mux: disable, wait two cycles of the old source, change AUXSRC, enable (RP2040
        // datasheet 2.15.3.2, RP2350 8.1.3.2). A previous image may have left it running; the pin step between this
        // disable and the AUXSRC write in PeripheryConfig is the wait.
        static constexpr auto powerClockEnable = list(clear(Ctrl::enable));

        // The clock-output GPIO function: F9 on the RP2350, F8 on the RP2040.
        static constexpr auto initStepPinConfig
          = list(action(Kvasir::Io::Action::PinFunctionDrive<PinConfig::gpoutFunction(Chip::part),
                                                             Io::DriveStrength::mA8,
                                                             true>{},
                        Pin{}));

        // The divider's fields: 16.16 on the RP2350, 24.8 on the RP2040. The fraction is
        // given in 1/65536 and scaled to the field's width.
        static constexpr unsigned fracBits
          = static_cast<unsigned>(std::popcount(std::remove_cvref_t<decltype(Div::frac)>::mask));
        static constexpr unsigned intBits
          = static_cast<unsigned>(std::popcount(std::remove_cvref_t<decltype(Div::_int)>::mask));
        static_assert(DivInt < (1U << intBits),
                      "the GPOUT divider's integer part does not fit the chip's field");
        static_assert(DivFrac16 < 65536,
                      "the fraction is in 1/65536");
        static_assert(DivInt > 0 || DivFrac16 == 0,
                      "an integer part of 0 means the maximum; a fraction needs an integer part");
        static constexpr std::uint32_t divFrac = DivFrac16 >> (16U - fracBits);

        static constexpr auto initStepPeripheryConfig
          = list(write(Div::_int, Register::value<DivInt>()),
                 write(Div::frac, Register::value<divFrac>()),
                 Ctrl::overrideDefaults(write(
                   Ctrl::auxsrc,
                   Register::value<typename Ctrl::AUXSRCVal, detail::auxsrcOfExtra<Ctrl>(Src)>())));

        static constexpr auto initStepPeripheryEnable = list(set(Ctrl::enable));

        static void setEnabled(bool on) {
            if(on) {
                apply(set(Ctrl::enable));
            } else {
                apply(clear(Ctrl::enable));
            }
        }

        /// The output for a source of `sourceHz`.
        static constexpr std::uint32_t outputHz(std::uint32_t sourceHz) {
            return static_cast<std::uint32_t>(
              (static_cast<std::uint64_t>(sourceHz) << 16)
              / ((static_cast<std::uint64_t>(DivInt) << 16) | DivFrac16));
        }
    };

    // -------------------------------------------------------------------------------------
    // The frequency counter FC0
    // -------------------------------------------------------------------------------------
    //
    // Counts the edges of one clock against clk_ref for CountStep * 2^interval (interval 8,
    // the default: 251 us, see countTime()) and returns kHz with a 1/32 kHz fraction; MIN/MAX
    // only feed the PASS / FAIL flags. Register pokes only, so `measureRaw` runs from RAM
    // with interrupts off and inside the resus handler. The reference is clk_ref, i.e. the
    // crystal: a wrong crystal shows as every ratio being right. A stopped source reads 0
    // (DIED is for a clock that stops during the count).
    //
    // Ownership: there is one FC0 and no lock. The Resus handler measures from its
    // interrupt, so a loop that measures while a Resus is armed must use measureGuarded(),
    // which masks interrupts for the count: an interrupted measure() would otherwise see
    // the handler's count and hand back its result, or wait on a counter the handler
    // restarted. The polls on FC0_STATUS are bounded (PollLimit); a count that never
    // finishes comes back with `timedOut` set.
    //
    //     using Fc = Kvasir::Clocks::FrequencyCounter<HW::CrystalSpeed>;
    //     auto const f = Fc::measure(Kvasir::Clocks::Source::clkSys);   // f.khz, f.hz()
    //     bool const ok = Fc::check(Kvasir::Clocks::Source::clkSys, 199'000, 201'000);
    struct Frequency {
        std::uint32_t khz{};
        std::uint32_t frac32{};     // 1/32 kHz
        std::uint32_t status{};     // FC0_STATUS as read when DONE
        bool          timedOut{};   // the poll on FC0_STATUS gave up: nothing above is valid

        [[nodiscard]] constexpr std::uint64_t hz() const {
            return static_cast<std::uint64_t>(khz) * 1000U
                 + (static_cast<std::uint64_t>(frac32) * 1000U) / 32U;
        }

        /// The fraction as hundredths of a kHz, for a "{}.{:02} kHz" log line.
        [[nodiscard]] constexpr std::uint32_t hundredths() const { return (frac32 * 100U) / 32U; }

        [[nodiscard]] constexpr bool pass() const { return (status & Status::pass.mask) != 0; }

        [[nodiscard]] constexpr bool fail() const { return (status & Status::fail.mask) != 0; }

        [[nodiscard]] constexpr bool slow() const { return (status & Status::slow.mask) != 0; }

        [[nodiscard]] constexpr bool fast() const { return (status & Status::fast.mask) != 0; }

        [[nodiscard]] constexpr bool died() const { return (status & Status::died.mask) != 0; }

    private:
        using Status = Kvasir::Peripheral::CLOCKS::Registers<>::FC0_STATUS;
    };

    namespace detail {
        // The chip's FC0_SRC value for a Source, by name (lposc and clk_hstx are RP2350 only).
        template<typename Src>
        constexpr std::uint32_t fcSrcOf(Source s) {
            using V = typename Src::FC0_SRCVal;
            switch(s) {
            case Source::pllSys:  return static_cast<std::uint32_t>(V::pll_sys_clksrc_primary);
            case Source::gpin0:   return static_cast<std::uint32_t>(V::clksrc_gpin0);
            case Source::gpin1:   return static_cast<std::uint32_t>(V::clksrc_gpin1);
            case Source::pllUsb:  return static_cast<std::uint32_t>(V::pll_usb_clksrc_primary);
            case Source::rosc:    return static_cast<std::uint32_t>(V::rosc_clksrc);
            case Source::xosc:    return static_cast<std::uint32_t>(V::xosc_clksrc);
            case Source::clkSys:  return static_cast<std::uint32_t>(V::clk_sys);
            case Source::clkUsb:  return static_cast<std::uint32_t>(V::clk_usb);
            case Source::clkAdc:  return static_cast<std::uint32_t>(V::clk_adc);
            case Source::clkRef:  return static_cast<std::uint32_t>(V::clk_ref);
            case Source::clkPeri: return static_cast<std::uint32_t>(V::clk_peri);
#ifdef KVASIR_CHIP_RP2350
            case Source::lposc:   return static_cast<std::uint32_t>(V::lposc_clksrc);
            case Source::clkHstx: return static_cast<std::uint32_t>(V::clk_hstx);
#endif
            default: return 0;   // null: the counter idles
            }
        }
    }   // namespace detail

    template<auto ClkRefHz>
    struct FrequencyCounter {
        static_assert(ClkRefHz >= 1'000'000 && ClkRefHz <= 100'000'000,
                      "ClkRefHz is the crystal clk_ref runs from");

        using Clk = Kvasir::Peripheral::CLOCKS::Registers<>;

        static constexpr std::uint32_t refKhz = static_cast<std::uint32_t>(ClkRefHz / 1000);
        static constexpr std::uint32_t noMin  = 0;
        static constexpr std::uint32_t noMax  = 0x1FF'FFFFU;

        /// The count lasts CountStep * 2^interval (datasheet, FC0_INTERVAL: "0.98us *
        /// 2**interval"); interval is four bits.
        static constexpr std::chrono::nanoseconds countStep{980};

        /// countTime(8) == 251us, countTime(12) == 4014us.
        [[nodiscard]] static constexpr std::chrono::microseconds countTime(std::uint32_t interval) {
            return std::chrono::round<std::chrono::microseconds>(
              countStep * (std::int64_t{1} << (interval & 0xFU)));
        }

        /// Iterations each FC0_STATUS poll gets before measureRaw() gives up: the longest
        /// count (interval 15, 32 ms) at 200 MHz takes about a million, and at clk_ref
        /// speed after a resus a handful of thousand.
        static constexpr std::uint32_t pollLimit = 4'000'000;

        /// One count of the source `src` (an FC0_SRC value), blocking for countTime(interval).
        /// `timedOut` is set, and the rest of the result meaningless, if the counter did not
        /// go idle or did not finish within PollLimit polls.
        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static Frequency measureRaw(std::uint32_t src,
                                                                   std::uint32_t interval,
                                                                   std::uint32_t minKhz,
                                                                   std::uint32_t maxKhz) {
            KVASIR_RAM_FUNC_MARK();
            // the register layer is always inlined: nothing here calls into flash. The FC0 value registers are
            // written whole (one field each), the status polled by field.
            using namespace Kvasir::Register;
            using Status
              = Clk::FC0_STATUS;   // no lambda for the reads: gcc at -Og leaves one in flash

            // running: it runs even for source null
            std::uint32_t polls = pollLimit;
            while(get<0>(apply(read(Status::running))) != 0) {
                if(--polls == 0) { return Frequency{0, 0, readWhole(Status::FULLREGISTER), true}; }
            }
            writeWhole(Clk::FC0_REF_KHZ::FULLREGISTER, refKhz);
            writeWhole(Clk::FC0_MIN_KHZ::FULLREGISTER, minKhz);
            writeWhole(Clk::FC0_MAX_KHZ::FULLREGISTER, maxKhz);
            writeWhole(Clk::FC0_DELAY::FULLREGISTER, 1U);
            apply(write(Clk::FC0_INTERVAL::fc0_interval, interval));
            writeWhole(Clk::FC0_SRC::FULLREGISTER, src);   // starts the count
            polls = pollLimit;
            while(get<0>(apply(read(Status::done))) == 0) {
                if(--polls == 0) { return Frequency{0, 0, readWhole(Status::FULLREGISTER), true}; }
            }
            auto const r
              = apply(read(Clk::FC0_RESULT::khz), read(Clk::FC0_RESULT::frac));   // one load
            return Frequency{get<0>(r), get<1>(r), readWhole(Status::FULLREGISTER), false};
        }

        static Frequency measure(Source        s,
                                 std::uint32_t interval = 8) {
            return measureRaw(detail::fcSrcOf<typename Clk::FC0_SRC>(s), interval, noMin, noMax);
        }

        /// measure() with interrupts masked for the count: what a loop that measures while
        /// a Resus is armed has to use, since the resus handler shares FC0 (see above).
        static Frequency measureGuarded(Source        s,
                                        std::uint32_t interval = 8) {
            Kvasir::Core::IrqGuard const guard{};
            return measure(s, interval);
        }

        /// PASS of the hardware's window: minKhz <= f <= maxKhz.
        static bool check(Source        s,
                          std::uint32_t minKhz,
                          std::uint32_t maxKhz,
                          std::uint32_t interval = 8) {
            return measureRaw(detail::fcSrcOf<typename Clk::FC0_SRC>(s), interval, minKhz, maxKhz)
              .pass();
        }
    };

    // -------------------------------------------------------------------------------------
    // The clk_sys resus
    // -------------------------------------------------------------------------------------
    //
    // If clk_sys stops (the system PLL powered down by a supply glitch or a stray write, a
    // source switched off) for `timeout` clk_ref cycles, the clocks block forces clk_sys to
    // clk_ref, sets RESUSSED and raises the `clocks` interrupt. The handler here runs from
    // RAM and, in this order:
    //
    //   1. saves the QMI timing and puts the QSPI flash on the boot's pre-PLL timing
    //      (clkdiv 4, rxdelay 2), which reads at clk_ref speed and at the full clock - the
    //      full-clock timing samples too late at 12 MHz and every flash fetch is garbage
    //      until then - and drops the XIP cache: the fetch in flight when the clock
    //      collapsed can complete with shifted data,
    //   2. records what it found (clk_sys by the frequency counter),
    //   3. puts clk_sys explicitly on clk_ref and releases the resus (pico-sdk's order); the
    //      glitchless switch is waited for bounded (Chip::ClkSelectedBound, OnTimeout::Report
    //      whatever the image injects: PRIMASK is set here) - a switch that does not show in
    //      CLK_SYS_SELECTED is counted in `report().selectTimeouts` and the handler goes on,
    //      step 4 switches the clocks again and step 5 judges the result,
    //   4. runs ClockSettings::coreClockInit(), the boot's own PLL bring-up, from flash. Each of its
    //      waits is bounded: a crystal or a PLL that does not come gives up there (a PLL leaves
    //      clk_sys on clk_ref = the crystal) and `report().initResult` says where
    //      (Clocks::initResult()),
    //   5. verifies: step 4's init finished (Clocks::initResult() ok - PLL_USB, clk_usb and
    //      clk_adc too), the clk_sys mux on its aux (PLL) input and the frequency counter within
    //      1 % of the clock ClockSettings provides. Only then the flash timing it found goes
    //      back; otherwise the slow one stays and `report().repaired` is false - a PLL that did
    //      not lock is a failed repair like any other, PLL_USB's as well,
    //   6. counts the event and calls Config::onResus(bool repaired) if there is one - from
    //      the interrupt, which preempts lower-priority ones, so keep it short: a flag, or a
    //      log line (written with interrupts masked, see uc_log's IsrRecordGuard),
    //   7. with Config::panicOnFailure and a failed repair: Panic::raiseAt(clockFailure, 0,
    //      kHz after), so the reset brings the clocks up again. Without it the repair is
    //      retried (below) - the handler only marks it pending and keeps the flash timing it
    //      found for it.
    //
    // A failed repair is no latch: turn(), a Hook::MainLoop entry (in thread mode, so the
    // handler stays small), tries again on a doubling pause (Kvasir::Backoff on TimeClock:
    // 100 ms up to 10 s) until the verify passes; then the flash timing goes back. A try
    // starts a crystal that is not stable without waiting for it (crystalStarted: the next
    // try looks again), brings the PLLs up with interrupts ON (ClockSettings::startSources(),
    // DefaultClockSettings': nothing switched, every wait bounded - a PLL that does not lock holds the
    // loop for its bound, ~40 ms, not the interrupts), then masks interrupts for the switch
    // (switchToSources), the frequency count and the flash timing only (~0.3 ms). Until then
    // clk_sys runs on clk_ref and every peripheral clocked from it at the wrong rate. A
    // TimeClock on the TIMER (clk_ref) keeps the pauses true; a SysTick clock runs ClockSpeed /
    // clk_ref times slow meanwhile, and its pauses with it. `report()` has the tries
    // (`retries`), the repairs they made (`recoveries`) and whether one is still due
    // (`retryPending`). The firmware's loop must run `Startup::run<Kvasir::Hook::MainLoop>()`.
    //
    // Steps 1-5 run with interrupts masked (PRIMASK): until step 1 no handler may run from
    // flash, and SysTick at priority 0 would preempt this one; the QMI timing writes also
    // need the QMI idle.
    //
    // Needs `Kvasir::Startup::RamVectorTable<>` in the Startup list (claimed): with the
    // table in flash the vector fetch itself is the garbage read and the chip locks up.
    // Needs `ClockSettings::Provides` (DefaultClockSettings::Provides) for the clk_sys and
    // clk_ref frequencies. The event costs about a millisecond, half of it the two frequency
    // measurements, and other interrupts wait that long; SysTick counts the slow clock
    // meanwhile, so a SysTick-based clock loses that time, the TIMER (clk_ref) does not.
    //
    // A resus survives a core reset; DefaultClockSettings::coreClockInit releases a pending
    // one at boot. FRCE ("force a resus, for test") is a level CLEAR cannot end: not usable
    // for a self-test, power the PLL down instead.
    //
    // The handler measures with the frequency counter, which has no lock: a loop that
    // measures while the Resus is armed must use FrequencyCounter::measureGuarded().
    //
    //     struct ResusConfig {
    //         static constexpr auto timeout     = 255;   // clk_ref cycles, 21 us at 12 MHz
    //         static constexpr auto isrPriority = 1;
    //         static void onResus(bool repaired) { ... }   // optional
    //         static constexpr bool panicOnFailure = true;  // optional, default false
    //     };
    //     using Resus = Kvasir::Clocks::Resus<HW::ClockSettings, HW::TimerClock, ResusConfig>;
    struct ResusTag {};

    struct DefaultResusConfig {};

    template<typename ClockSettings, typename TimeClock, typename Config_ = DefaultResusConfig>
    struct Resus {
        static_assert(
          DefaultClockSettings::DegradableClockSettings<ClockSettings>,
          "Kvasir::Clocks::Resus: the ClockSettings does not name its tree's run-time pieces "
          "(DefaultClockSettings::DegradableClockSettings: Provides - the clocks to verify -, "
          "crystalStarted(), startSources() and switchToSources() for a retried repair, and the "
          "rest the "
          "chip's DegradedBoot needs); Kvasir::DefaultClockSettings::Settings<ClockSpeed, "
          "CrystalSpeed> "
          "has them (use it, or derive from it)");

        static constexpr unsigned long long clkSysHz
          = Hal::hzOf<ClkSys, typename ClockSettings::Provides>();
        static constexpr unsigned long long clkRefHz
          = Hal::hzOf<ClkRef, typename ClockSettings::Provides>();

        /// CLK_SYS_RESUS_CTRL.TIMEOUT is eight bits: the longest wait, in clk_ref cycles.
        static constexpr std::uint32_t maxTimeout = 255;

        struct Config {
            static constexpr std::uint32_t timeout
              = static_cast<std::uint32_t>(KVASIR_CONFIG_OR(Config_, timeout, 255U));
            static constexpr int isrPriority
              = static_cast<int>(KVASIR_CONFIG_OR(Config_, isrPriority, 2));
            static constexpr bool panicOnFailure
              = static_cast<bool>(KVASIR_CONFIG_OR(Config_, panicOnFailure, false));
        };

        static_assert(Config::timeout <= maxTimeout,
                      "the resus timeout is 8 bits");

        using Clk       = Kvasir::Peripheral::CLOCKS::Registers<>;
        using Fc        = FrequencyCounter<clkRefHz>;
        using NvicIndex = Nvic::Index<decltype(Kvasir::Interrupt::clocks)::value>;

        using Provides = brigand::list<Startup::Resource<ResusTag, 0>>;
        using Claims   = brigand::list<Startup::Resource<Startup::RamVectorTableTag, 0>>;

        static constexpr auto initStepPeripheryConfig
          = list(Clk::CLK_SYS_RESUS_CTRL::overrideDefaults(
                   write(Clk::CLK_SYS_RESUS_CTRL::timeout, Register::value<Config::timeout>())),
                 write(Clk::INTE::clk_sys_resus, Register::value<1>()));

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<Config::isrPriority>(NvicIndex{}),
                 Nvic::makeClearPending(NvicIndex{}));

        static constexpr auto initStepPeripheryEnable
          = list(set(Clk::CLK_SYS_RESUS_CTRL::enable), Nvic::makeEnable(NvicIndex{}));

        struct Report {
            std::uint32_t events{};      // resus events handled since boot
            std::uint32_t failed{};      // of them, repairs the verify refused
            std::uint32_t khzBefore{};   // clk_sys as the last handler found it
            std::uint32_t khzAfter{};    // clk_sys after its repair
            std::uint32_t stage{};       // how far the last handler got (6 = done)
            std::uint32_t
              selectTimeouts{};   // step 3's switch to clk_ref not seen within its bound
            // tries of turn() after a failed repair, since boot (not one a resus event's own repair overtook: that
            // one is the event's)
            std::uint32_t retries{};
            std::uint32_t recoveries{};     // of them, repairs the verify passed
            bool          repaired{};       // the last verify (an event's or a retry's)
            bool          retryPending{};   // a failed repair waits for turn()
            InitResult
              initResult{};   // how far the last repair's clock init got (Clocks::initResult())
        };

        [[nodiscard]] static Report report() {
            return Report{events_.load(std::memory_order_relaxed),
                          failed_.load(std::memory_order_relaxed),
                          khzBefore_.load(std::memory_order_relaxed),
                          khzAfter_.load(std::memory_order_relaxed),
                          stage_.load(std::memory_order_relaxed),
                          selectTimeouts_.load(std::memory_order_relaxed),
                          retries_.load(std::memory_order_relaxed),
                          recoveries_.load(std::memory_order_relaxed),
                          repaired_.load(std::memory_order_relaxed),
                          retryPending_.load(std::memory_order_relaxed),
                          initResult_.load(std::memory_order_relaxed)};
        }

        /// The Hook::MainLoop entry (the comment above the struct): one load while no repair is due.
        static void turn() {
            if(!retryPending_.load(std::memory_order_relaxed)) {
                if(backoff_.failures != 0) {
                    backoff_.succeeded();
                }   // an event's own repair made it good
                return;
            }
            auto const now = TimeClock::now();
            // the failed repair seen for the first time: the first pause starts
            if(backoff_.failures == 0) {
                backoff_.failed(now);
                return;
            }
            if(!backoff_.due(now)) { return; }
            switch(retryRepair_()) {
            case Retry::repaired:
                retries_.fetch_add(1, std::memory_order_relaxed);
                recoveries_.fetch_add(1, std::memory_order_relaxed);
                backoff_.succeeded();
                break;
            case Retry::failed:
                retries_.fetch_add(1, std::memory_order_relaxed);
                backoff_.failed(TimeClock::now());
                break;
            case Retry::byHandler:
                // an event's own repair, counted as the event's (events, report().repaired)
                backoff_.succeeded();
                break;
            }
        }

        // with panicOnFailure a failed repair ends in the panic: nothing to retry
        using Extends = std::conditional_t<Config::panicOnFailure,
                                           brigand::list<>,
                                           Startup::Extend<Kvasir::Hook::MainLoop, &Resus::turn>>;

        /// Whether an event has not been released yet.
        [[nodiscard]] static bool resuscitated() {
            return apply(read(Clk::CLK_SYS_RESUS_STATUS::resussed)) != 0;
        }

        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static void onIsr() {
            // in RAM for steps 1-3 (the flash may not be readable when it is entered); step 4
            // calls coreClockInit() in flash on purpose, after step 1 made it readable again
            KVASIR_RAM_FUNC_MARK_CALLS_FLASH();
            // the register layer is always inlined (fields and readWhole alike): nothing here calls into flash
            // until step 4
            using namespace Kvasir::Register;
            using Selected
              = Clk::CLK_SYS_SELECTED;   // no lambda for the reads: gcc at -Og leaves one in flash
            // CLK_SYS_SELECTED is one-hot over CLK_SYS_CTRL.SRC (RP2040 datasheet Table 225, RP2350 datasheet
            // Table 561): bit n is SRC value n, as clock_config.hpp's selectedIs<> waits read it
            using Src                         = Clk::CLK_SYS_CTRL::SRCVal;
            constexpr std::uint32_t selClkRef = 1U << std::to_underlying(Src::clk_ref);

            if(get<0>(apply(read(Clk::INTS::clk_sys_resus))) == 0) { return; }

            auto const irqs = Kvasir::Core::IrqState::saveAndMask();
            stage_.store(1, std::memory_order_relaxed);

            // 1. flash readable at clk_ref, the XIP cache dropped; the timing to go back to is the one found,
            //    or while a failed repair waits for its retry the one found before it
#ifdef KVASIR_CHIP_RP2350
            auto const timing = retryPending_.load(std::memory_order_relaxed)
                                ? savedTiming_
                                : Kvasir::detail::XipReadMode::currentTiming();
            Kvasir::detail::XipReadMode::apply(Kvasir::detail::XipReadMode::timingAtClkRef);
#endif
            // The fetch that was in flight when the clock collapsed may have completed
            // with shifted data and sit in the cache. Drop the cache before anything
            // runs from flash again; the lookup and the routine are in ROM, always readable.
            if(flushCache_ == nullptr) {
                flushCache_
                  = Kvasir::RomFunctions::getRomFunctionPointerFromRam<'F', 'C', void (*)()>();
            }
            if(flushCache_ != nullptr) { flushCache_(); }
            stage_.store(2, std::memory_order_relaxed);

            // 2. what it found
            constexpr std::uint32_t srcClkSys
              = detail::fcSrcOf<typename Clk::FC0_SRC>(Source::clkSys);
            khzBefore_.store(Fc::measureRaw(srcClkSys, 8, Fc::noMin, Fc::noMax).khz,
                             std::memory_order_relaxed);

            // 3. clk_sys explicitly on clk_ref, then release the resus; the switch bounded, a field compare
            //    (always inlined: no call into flash), PRIMASK set - a timeout is counted and step 4 goes on
            if(!Kvasir::Register::waitUntil<Kvasir::Chip::ClkSelectedBound, OnTimeout::Report>(
                 FieldValue<std::remove_cvref_t<decltype(Selected::clk_sys_selected)>,
                            selClkRef>{}))
            {
                selectTimeouts_.fetch_add(1, std::memory_order_relaxed);
            }
            using ResusCtrl = Clk::CLK_SYS_RESUS_CTRL;
            apply(set(ResusCtrl::clear));
            apply(Kvasir::Register::clear(ResusCtrl::clear));
            stage_.store(3, std::memory_order_relaxed);

            // 4. the boot's clock init, every wait in it bounded
            ClockSettings::coreClockInit();
            auto const init = Clocks::initResult();
            initResult_.store(init, std::memory_order_relaxed);
            stage_.store(4, std::memory_order_relaxed);

            // 5. verify (an init that stopped at PLL_USB leaves clk_sys good: it counts too), then the flash timing
            //    back
            auto const after = Fc::measureRaw(srcClkSys, 8, Fc::noMin, Fc::noMax).khz;
            khzAfter_.store(after, std::memory_order_relaxed);
            bool const ok = init == InitResult::ok && clockGood_(after);
#ifdef KVASIR_CHIP_RP2350
            if(ok) { Kvasir::detail::XipReadMode::apply(timing); }
#endif
            repaired_.store(ok, std::memory_order_relaxed);
            if(!ok) { failed_.fetch_add(1, std::memory_order_relaxed); }
            if constexpr(!Config::panicOnFailure) {
                // a failed repair waits for turn(), with the timing to go back to
#ifdef KVASIR_CHIP_RP2350
                if(!ok) { savedTiming_ = timing; }
#endif
                retryPending_.store(!ok, std::memory_order_relaxed);
            }
            stage_.store(5, std::memory_order_relaxed);
            irqs.restore();

            // 6. count, tell
            events_.fetch_add(1, std::memory_order_relaxed);
            if constexpr(requires { Config_::onResus(true); }) { Config_::onResus(ok); }
            stage_.store(6, std::memory_order_relaxed);

            // 7. a clock that did not come back is not run on (without panicOnFailure: turn() retries)
            if constexpr(Config::panicOnFailure) {
                if(!ok) { Kvasir::Panic::raiseAt(Kvasir::Panic::Cause::clockFailure, 0, after); }
            }
        }

        using Isr = Nvic::Isr<std::addressof(onIsr), NvicIndex>;

    private:
        // Step 5's verdict: the clk_sys mux on its aux (PLL) input and `khz` within 1 % of the clock the ClockSettings
        // provide. Always inlined: the handler runs it from RAM.
        [[KVASIR_RAM_FUNC_INLINE_ATTRIBUTES]] static bool clockGood_(std::uint32_t khz) {
            using namespace Kvasir::Register;
            // CLK_SYS_SELECTED is one-hot over CLK_SYS_CTRL.SRC (the handler's comment)
            constexpr std::uint32_t selAux
              = 1U << std::to_underlying(Clk::CLK_SYS_CTRL::SRCVal::clksrc_clk_sys_aux);
            constexpr std::uint32_t want = static_cast<std::uint32_t>(clkSysHz / 1000);
            return (apply(read(Clk::CLK_SYS_SELECTED::clk_sys_selected)) & selAux) != 0
                && khz > want - want / 100U && khz < want + want / 100U;
        }

        // What a retry came to: its own repair, a failed one, or a resus event's repair while it waited (the
        // handler's).
        enum class Retry : std::uint8_t { repaired, failed, byHandler };

        // Steps 4-5 once more, for turn(), in thread mode: the crystal and the PLLs brought up with interrupts on
        // (ClockSettings::startSources: nothing switched; from flash, on the timing for clk_ref the failed repair
        // left), then masked: the switch (switchToSources, which checks the sources again), the verify, and on a pass
        // the flash timing the handler found. A resus event during the waits is the handler's (its own repair and
        // verdict, counted in `events`): one that repaired leaves nothing to do here, and is no retry of turn()'s.
        static Retry retryRepair_() {
            // a crystal that is not stable is started (again) and looked at by the next retry: no wait for it here
            if(!ClockSettings::crystalStarted()) {
                initResult_.store(InitResult::xoscNotStable, std::memory_order_relaxed);
                repaired_.store(false, std::memory_order_relaxed);
                return Retry::failed;
            }
            auto       init = ClockSettings::startSources();
            auto const irqs = Kvasir::Core::IrqState::saveAndMask();
            if(!retryPending_.load(std::memory_order_relaxed)) {
                irqs.restore();
                return Retry::byHandler;
            }
            if(init == InitResult::ok) { init = ClockSettings::switchToSources(); }
            initResult_.store(init, std::memory_order_relaxed);
            constexpr std::uint32_t srcClkSys
              = detail::fcSrcOf<typename Clk::FC0_SRC>(Source::clkSys);
            auto const after = Fc::measureRaw(srcClkSys, 8, Fc::noMin, Fc::noMax).khz;
            khzAfter_.store(after, std::memory_order_relaxed);
            bool const ok = init == InitResult::ok && clockGood_(after);
            if(ok) {
#ifdef KVASIR_CHIP_RP2350
                Kvasir::detail::XipReadMode::apply(savedTiming_);
#endif
                retryPending_.store(false, std::memory_order_relaxed);
            }
            repaired_.store(ok, std::memory_order_relaxed);
            irqs.restore();
            return ok ? Retry::repaired : Retry::failed;
        }

        inline static std::atomic<std::uint32_t> events_{0};
        inline static std::atomic<std::uint32_t> failed_{0};
        inline static std::atomic<std::uint32_t> khzBefore_{0};
        inline static std::atomic<std::uint32_t> khzAfter_{0};
        inline static std::atomic<std::uint32_t> stage_{0};
        inline static std::atomic<std::uint32_t> selectTimeouts_{0};
        inline static std::atomic<bool>          repaired_{false};
        inline static std::atomic<std::uint32_t> retries_{0};
        inline static std::atomic<std::uint32_t> recoveries_{0};
        inline static std::atomic<bool>          retryPending_{false};
        inline static std::atomic<InitResult>    initResult_{InitResult::ok};
#ifdef KVASIR_CHIP_RP2350
        inline static std::uint32_t
          savedTiming_{};   // the flash timing of the clock before the failed repair
#endif
        inline static Kvasir::Backoff<TimeClock, RetryPauses> backoff_{};
        inline static void (*flushCache_)() = nullptr;
    };

}}   // namespace Kvasir::Clocks
