#pragma once
#include "Clocks.hpp"
#include "Io.hpp"
#include "PinConfig.hpp"
#include "bootrom_functions.hpp"
#include "chip/Interrupt.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/RamVectorTable.hpp"
#include "kvasir/Util/attributes.hpp"
#include "peripherals/CLOCKS.hpp"

#include <atomic>
#include <bit>
#include <chrono>
#include <cstdint>

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
// Clocks::Resus<ClockSettings, Config>: the resuscitation of clk_sys. If the system clock
// stops for `timeout` clk_ref cycles, the hardware switches clk_sys to clk_ref and raises
// the `clocks` interrupt; the handler rebuilds the clock tree from RAM and verifies it with
// the frequency counter. Details at the struct.
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

        constexpr int gpoutOf(int pin) { return PinConfig::gpoutOf(PinConfig::CurrentChip, pin); }

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
#if __has_include("chip/rp2350.hpp")
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
        static constexpr int      PinNumber     = detail::pinNumber(Pin{});
        static constexpr unsigned Divider       = DivInt;
        static constexpr unsigned DividerFrac16 = DivFrac16;
        static constexpr int      Gpout         = detail::gpoutOf(PinNumber);
        static_assert(Gpout >= 0,
                      "not a GPOUT pin: GPIO 21 (GPOUT0), 23 (GPOUT1), 24 (GPOUT2), 25 (GPOUT3); "
                      "on the RP2350 also 13 (GPOUT0) and 15 (GPOUT1)");
        static_assert(!PinConfig::isRp2040(PinConfig::CurrentChip)
                        || (Src != Source::lposc && Src != Source::clkPeri
                            && Src != Source::clkHstx),
                      "the RP2040 has no LPOSC, clk_peri or clk_hstx output");

        using Clk  = Kvasir::Peripheral::CLOCKS::Registers<>;
        using Ctrl = std::conditional_t<
          Gpout == 0,
          typename Clk::CLK_GPOUT0_CTRL,
          std::conditional_t<Gpout == 1,
                             typename Clk::CLK_GPOUT1_CTRL,
                             std::conditional_t<Gpout == 2,
                                                typename Clk::CLK_GPOUT2_CTRL,
                                                typename Clk::CLK_GPOUT3_CTRL>>>;
        using Div = std::conditional_t<
          Gpout == 0,
          typename Clk::CLK_GPOUT0_DIV,
          std::conditional_t<Gpout == 1,
                             typename Clk::CLK_GPOUT1_DIV,
                             std::conditional_t<Gpout == 2,
                                                typename Clk::CLK_GPOUT2_DIV,
                                                typename Clk::CLK_GPOUT3_DIV>>>;

        // The clock-output GPIO function: F9 on the RP2350, F8 on the RP2040.
        static constexpr auto initStepPinConfig = list(action(
          Kvasir::Io::Action::PinFunctionDrive<PinConfig::gpoutFunction(PinConfig::CurrentChip),
                                               Io::DriveStrength::mA_8,
                                               true>{},
          Pin{}));

        // The divider's fields: 16.16 on the RP2350, 24.8 on the RP2040. The fraction is
        // given in 1/65536 and scaled to the field's width.
        static constexpr unsigned FracBits = static_cast<unsigned>(std::popcount(
          Register::Detail::GetMask<std::remove_cvref_t<decltype(Div::frac)>>::value));
        static constexpr unsigned IntBits  = static_cast<unsigned>(std::popcount(
          Register::Detail::GetMask<std::remove_cvref_t<decltype(Div::_int)>>::value));
        static_assert(DivInt < (1U << IntBits),
                      "the GPOUT divider's integer part does not fit the chip's field");
        static_assert(DivFrac16 < 65536,
                      "the fraction is in 1/65536");
        static_assert(DivInt > 0 || DivFrac16 == 0,
                      "an integer part of 0 means the maximum; a fraction needs an integer part");
        static constexpr std::uint32_t DivFrac = DivFrac16 >> (16U - FracBits);

        static constexpr auto initStepPeripheryConfig
          = list(write(Div::_int, Register::value<DivInt>()),
                 write(Div::frac, Register::value<DivFrac>()),
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

        [[nodiscard]] constexpr bool pass() const { return (status & 1U) != 0; }

        [[nodiscard]] constexpr bool fail() const { return (status & (1U << 16)) != 0; }

        [[nodiscard]] constexpr bool slow() const { return (status & (1U << 20)) != 0; }

        [[nodiscard]] constexpr bool fast() const { return (status & (1U << 24)) != 0; }

        [[nodiscard]] constexpr bool died() const { return (status & (1U << 28)) != 0; }
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
#if __has_include("chip/rp2350.hpp")
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

        static constexpr std::uint32_t RefKhz = static_cast<std::uint32_t>(ClkRefHz / 1000);
        static constexpr std::uint32_t NoMin  = 0;
        static constexpr std::uint32_t NoMax  = 0x1FF'FFFFU;

        /// The count lasts CountStep * 2^interval (datasheet, FC0_INTERVAL: "0.98us *
        /// 2**interval"); interval is four bits.
        static constexpr std::chrono::nanoseconds CountStep{980};

        /// countTime(8) == 251us, countTime(12) == 4014us.
        [[nodiscard]] static constexpr std::chrono::microseconds countTime(std::uint32_t interval) {
            return std::chrono::round<std::chrono::microseconds>(
              CountStep * (std::int64_t{1} << (interval & 0xFU)));
        }

        /// Iterations each FC0_STATUS poll gets before measureRaw() gives up: the longest
        /// count (interval 15, 32 ms) at 200 MHz takes about a million, and at clk_ref
        /// speed after a resus a handful of thousand.
        static constexpr std::uint32_t PollLimit = 4'000'000;

        /// One count of the source `src` (an FC0_SRC value), blocking for countTime(interval).
        /// `timedOut` is set, and the rest of the result meaningless, if the counter did not
        /// go idle or did not finish within PollLimit polls.
        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static Frequency measureRaw(std::uint32_t src,
                                                                   std::uint32_t interval,
                                                                   std::uint32_t minKhz,
                                                                   std::uint32_t maxKhz) {
            KVASIR_RAM_FUNC_MARK();
            auto* const ref
              = reinterpret_cast<std::uint32_t volatile*>(Clk::FC0_REF_KHZ::Addr::value);
            auto* const min
              = reinterpret_cast<std::uint32_t volatile*>(Clk::FC0_MIN_KHZ::Addr::value);
            auto* const max
              = reinterpret_cast<std::uint32_t volatile*>(Clk::FC0_MAX_KHZ::Addr::value);
            auto* const delay
              = reinterpret_cast<std::uint32_t volatile*>(Clk::FC0_DELAY::Addr::value);
            auto* const ival
              = reinterpret_cast<std::uint32_t volatile*>(Clk::FC0_INTERVAL::Addr::value);
            auto* const srcR = reinterpret_cast<std::uint32_t volatile*>(Clk::FC0_SRC::Addr::value);
            auto* const stat
              = reinterpret_cast<std::uint32_t const volatile*>(Clk::FC0_STATUS::Addr::value);
            auto* const res
              = reinterpret_cast<std::uint32_t const volatile*>(Clk::FC0_RESULT::Addr::value);

            // running: it runs even for source null
            std::uint32_t polls = PollLimit;
            while((*stat & (1U << 8)) != 0) {
                if(--polls == 0) { return Frequency{0, 0, *stat, true}; }
            }
            *ref   = RefKhz;
            *min   = minKhz;
            *max   = maxKhz;
            *delay = 1;
            *ival  = interval & 0xFU;
            *srcR  = src;   // starts the count
            polls  = PollLimit;
            while((*stat & (1U << 4)) == 0) {   // done
                if(--polls == 0) { return Frequency{0, 0, *stat, true}; }
            }
            auto const r = *res;
            return Frequency{r >> 5, r & 31U, *stat, false};
        }

        static Frequency measure(Source        s,
                                 std::uint32_t interval = 8) {
            return measureRaw(detail::fcSrcOf<typename Clk::FC0_SRC>(s), interval, NoMin, NoMax);
        }

        /// measure() with interrupts masked for the count: what a loop that measures while
        /// a Resus is armed has to use, since the resus handler shares FC0 (see above).
        static Frequency measureGuarded(Source        s,
                                        std::uint32_t interval = 8) {
            Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> const guard{};
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
    //   3. puts clk_sys explicitly on clk_ref and releases the resus (pico-sdk's order),
    //   4. runs ClockSettings::coreClockInit(), the boot's own PLL bring-up, from flash,
    //   5. verifies: the clk_sys mux on its aux (PLL) input and the frequency counter within
    //      1 % of the clock ClockSettings provides. Only then the flash timing it found goes
    //      back; otherwise the slow one stays and `report().repaired` is false,
    //   6. counts the event and calls Config::onResus(bool repaired) if there is one - from
    //      the interrupt, which preempts lower-priority ones, so keep it short: a flag, or a
    //      log line (written with interrupts masked, see uc_log's IsrRecordGuard).
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
    //     };
    //     using Resus = Kvasir::Clocks::Resus<HW::ClockSettings, ResusConfig>;
    struct ResusTag {};

    struct DefaultResusConfig {};

    template<typename ClockSettings, typename Config_ = DefaultResusConfig>
    struct Resus {
        static_assert(
          requires { typename ClockSettings::Provides; },
          "Resus needs the clock frequencies: give the ClockSettings a Provides "
          "(using Provides = Kvasir::DefaultClockSettings::Provides<ClockSpeed, "
          "CrystalSpeed>;)");

        static constexpr unsigned long long ClkSysHz
          = hzOf<ClkSys, typename ClockSettings::Provides>();
        static constexpr unsigned long long ClkRefHz
          = hzOf<ClkRef, typename ClockSettings::Provides>();

        /// CLK_SYS_RESUS_CTRL.TIMEOUT is eight bits: the longest wait, in clk_ref cycles.
        static constexpr std::uint32_t MaxTimeout = 255;

        struct Config {
            static constexpr std::uint32_t timeout = [] {
                if constexpr(requires { Config_::timeout; }) {
                    return static_cast<std::uint32_t>(Config_::timeout);
                } else {
                    return 255U;
                }
            }();
            static constexpr int isrPriority = [] {
                if constexpr(requires { Config_::isrPriority; }) {
                    return static_cast<int>(Config_::isrPriority);
                } else {
                    return 2;
                }
            }();
        };

        static_assert(Config::timeout <= MaxTimeout,
                      "the resus timeout is 8 bits");

        using Clk       = Kvasir::Peripheral::CLOCKS::Registers<>;
        using Fc        = FrequencyCounter<ClkRefHz>;
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
            bool          repaired{};    // the last event's verify
        };

        [[nodiscard]] static Report report() {
            return Report{events_.load(std::memory_order_relaxed),
                          failed_.load(std::memory_order_relaxed),
                          khzBefore_.load(std::memory_order_relaxed),
                          khzAfter_.load(std::memory_order_relaxed),
                          stage_.load(std::memory_order_relaxed),
                          repaired_.load(std::memory_order_relaxed)};
        }

        /// Whether an event has not been released yet.
        [[nodiscard]] static bool resuscitated() {
            return apply(read(Clk::CLK_SYS_RESUS_STATUS::resussed));
        }

        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static void onIsr() {
            // in RAM for steps 1-3 (the flash may not be readable when it is entered); step 4
            // calls coreClockInit() in flash on purpose, after step 1 made it readable again
            KVASIR_RAM_FUNC_MARK_CALLS_FLASH();
            auto* const ints
              = reinterpret_cast<std::uint32_t const volatile*>(Clk::INTS::Addr::value);
            auto* const resusCtrl
              = reinterpret_cast<std::uint32_t volatile*>(Clk::CLK_SYS_RESUS_CTRL::Addr::value);
            auto* const sysCtrl
              = reinterpret_cast<std::uint32_t volatile*>(Clk::CLK_SYS_CTRL::Addr::value);
            auto* const sysSelected
              = reinterpret_cast<std::uint32_t const volatile*>(Clk::CLK_SYS_SELECTED::Addr::value);
            constexpr std::uint32_t ClearBit  = 1U << 16;
            constexpr std::uint32_t SrcMask   = 0x3U;   // CLK_SYS_CTRL.SRC: 0 clk_ref, 1 aux
            constexpr std::uint32_t SelClkRef = 1U << 0;
            constexpr std::uint32_t SelAux    = 1U << 1;

            if((*ints & 1U) == 0) { return; }

            std::uint32_t primask{};
            asm volatile("mrs %0, primask\n cpsid i" : "=r"(primask)::"memory");
            stage_.store(1, std::memory_order_relaxed);

            // 1. flash readable at clk_ref, the XIP cache dropped
#if __has_include("peripherals/QMI.hpp")
            auto const timing = Kvasir::detail::XipReadMode::currentTiming();
            Kvasir::detail::XipReadMode::apply(Kvasir::detail::XipReadMode::TimingAtClkRef);
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
            khzBefore_.store(Fc::measureRaw(srcClkSys, 8, Fc::NoMin, Fc::NoMax).khz,
                             std::memory_order_relaxed);

            // 3. clk_sys explicitly on clk_ref, then release the resus
            *sysCtrl = *sysCtrl & ~SrcMask;
            while((*sysSelected & SelClkRef) == 0) {}
            *resusCtrl = *resusCtrl | ClearBit;
            *resusCtrl = *resusCtrl & ~ClearBit;
            stage_.store(3, std::memory_order_relaxed);

            // 4. the boot's clock init
            ClockSettings::coreClockInit();
            stage_.store(4, std::memory_order_relaxed);

            // 5. verify, then the flash timing back
            auto const after = Fc::measureRaw(srcClkSys, 8, Fc::NoMin, Fc::NoMax).khz;
            khzAfter_.store(after, std::memory_order_relaxed);
            constexpr std::uint32_t want = static_cast<std::uint32_t>(ClkSysHz / 1000);
            bool const              ok = (*sysSelected & SelAux) != 0 && after > want - want / 100U
                                      && after < want + want / 100U;
#if __has_include("peripherals/QMI.hpp")
            if(ok) { Kvasir::detail::XipReadMode::apply(timing); }
#endif
            repaired_.store(ok, std::memory_order_relaxed);
            if(!ok) { failed_.fetch_add(1, std::memory_order_relaxed); }
            stage_.store(5, std::memory_order_relaxed);
            asm volatile("msr primask, %0" ::"r"(primask) : "memory");

            // 6. count, tell
            events_.fetch_add(1, std::memory_order_relaxed);
            if constexpr(requires { Config_::onResus(true); }) { Config_::onResus(ok); }
            stage_.store(6, std::memory_order_relaxed);
        }

        static constexpr Nvic::Isr<std::addressof(onIsr), NvicIndex> isr{};

    private:
        inline static std::atomic<std::uint32_t> events_{0};
        inline static std::atomic<std::uint32_t> failed_{0};
        inline static std::atomic<std::uint32_t> khzBefore_{0};
        inline static std::atomic<std::uint32_t> khzAfter_{0};
        inline static std::atomic<std::uint32_t> stage_{0};
        inline static std::atomic<bool>          repaired_{false};
        inline static void (*flushCache_)() = nullptr;
    };

}}   // namespace Kvasir::Clocks
