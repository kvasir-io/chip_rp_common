#pragma once
#ifndef KVASIR_CHIP_RP2040
    #error "the RTC exists on the RP2040 only; the RP2350 has the always-on timer (Powman.hpp)"
#endif
#include "Clocks.hpp"
#include "WaitBounds.hpp"
#include "chip/Blocks.hpp"   // the chip's block table (Rp::Block)
#include "chip/Interrupt.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "peripherals/CLOCKS.hpp"
#include "peripherals/RESETS.hpp"
#include "peripherals/RTC.hpp"

#include <chrono>
#include <cstdint>

// The RP2040's real-time clock: a calendar counter (year, month, day, day of week, hour,
// minute, second) ticked by clk_rtc, with one alarm that matches on any subset of those
// fields. The counter is not retained across a reset of the RTC block, and the chip has no
// battery domain: this is the RP2040 counterpart of the RP2350's POWMAN always-on timer for
// the examples, not a replacement for an external RTC.
//
//   using Rtc   = Kvasir::Rtc::Rtc<HW::CrystalSpeed>;
//   using Alarm = Kvasir::Rtc::Alarm<Rtc, &onAlarm>;
//   Startup<..., Rtc, Alarm>;
//   if(!Rtc::set(Kvasir::Rtc::DateTime::fromUnixSeconds(BuildEpoch))) { /* clk_rtc stopped: try later */ }
//   Alarm::armAt(Rtc::now().plus(std::chrono::seconds{2}));
//
// set() waits for the block to stop and to start again, bounded: a write takes 2 clk_rtc periods to arrive (RP2040
// datasheet 4.8.4 note, md l.25727), and a clk_rtc that does not run (stopped by another owner of CLOCKS, or a dead
// crystal) never lets RTC_ACTIVE change. On a timeout it sets clk_rtc up again (clockSetup, the clock half of
// powerClockEnable) and tries once more; false if that did not help either - nothing waits for ever, and the next
// set() tries again. clockRestarts() counts the set-ups it needed.
namespace Kvasir { namespace Rtc {

    struct DateTime {
        std::uint16_t year{1970};   // 0..4095
        std::uint8_t  month{1};     // 1..12
        std::uint8_t  day{1};       // 1..31
        std::uint8_t  dotw{4};      // 0 (Sunday) .. 6
        std::uint8_t  hour{0};      // 0..23
        std::uint8_t  minute{0};    // 0..59
        std::uint8_t  second{0};    // 0..59

        friend constexpr bool operator==(DateTime const&,
                                         DateTime const&) = default;

        // Howard Hinnant's days_from_civil / civil_from_days, for the conversions the RTC
        // does not do itself.
        static constexpr std::int64_t daysFromCivil(std::int64_t y,
                                                    std::int64_t m,
                                                    std::int64_t d) {
            y -= m <= 2 ? 1 : 0;
            auto const era = (y >= 0 ? y : y - 399) / 400;
            auto const yoe = y - era * 400;
            auto const doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
            auto const doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
            return era * 146097 + doe - 719468;
        }

        static constexpr DateTime fromUnixSeconds(std::int64_t s) {
            auto const days = s >= 0 ? s / 86400 : (s - 86399) / 86400;
            auto const rem  = s - days * 86400;
            auto const z    = days + 719468;
            auto const era  = (z >= 0 ? z : z - 146096) / 146097;
            auto const doe  = z - era * 146097;
            auto const yoe  = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
            auto const y    = yoe + era * 400;
            auto const doy  = doe - (365 * yoe + yoe / 4 - yoe / 100);
            auto const mp   = (5 * doy + 2) / 153;
            auto const d    = doy - (153 * mp + 2) / 5 + 1;
            auto const m    = mp < 10 ? mp + 3 : mp - 9;
            // 1970-01-01 was a Thursday (4).
            auto const dotw = ((days % 7) + 7 + 4) % 7;
            return {static_cast<std::uint16_t>(y + (m <= 2 ? 1 : 0)),
                    static_cast<std::uint8_t>(m),
                    static_cast<std::uint8_t>(d),
                    static_cast<std::uint8_t>(dotw),
                    static_cast<std::uint8_t>(rem / 3600),
                    static_cast<std::uint8_t>((rem / 60) % 60),
                    static_cast<std::uint8_t>(rem % 60)};
        }

        [[nodiscard]] constexpr std::int64_t toUnixSeconds() const {
            return daysFromCivil(year, month, day) * 86400
                 + static_cast<std::int64_t>(hour * 3600 + minute * 60 + second);
        }

        [[nodiscard]] constexpr DateTime plus(std::chrono::seconds s) const {
            return fromUnixSeconds(toUnixSeconds() + s.count());
        }
    };

    struct TimerTag {};

    struct AlarmTag {};

    /// The RTC block, clocked from the crystal: clk_rtc = XOSC / 256, and the block's own
    /// divider brings that down to the 1 Hz the calendar counts.
    template<auto CrystalHz>
    struct Rtc {
        using Regs = Kvasir::Peripheral::RTC::Registers<>;
        using Clk  = Kvasir::Peripheral::CLOCKS::Registers<>;

        static constexpr std::uint32_t clkRtcDiv = 256;
        static_assert(CrystalHz % clkRtcDiv == 0,
                      "the crystal must divide by 256");
        static constexpr std::uint32_t clkRtcHz = static_cast<std::uint32_t>(CrystalHz / clkRtcDiv);
        static_assert(clkRtcHz - 1 < 65536,
                      "CLKDIV_M1 is 16 bits: the crystal is too fast");

        using Provides = brigand::list<Startup::Resource<TimerTag, 0>>;
        using Claims   = Hal::Claim<Clocks::ClkRef, CrystalHz>;   // clk_ref is the crystal

        // clk_rtc from the crystal through /256. No glitchless mux: disable, source, enable as separate writes
        // (RP2040 datasheet 2.15.3.2, Table 234); divider before the enable, as the datasheet asks when the ratio
        // grows. A previous image may have left clk_rtc running. set() runs it again for a clk_rtc that stopped.
        static constexpr auto clockSetup = list(
          clear(Clk::CLK_RTC_CTRL::enable),
          Register::sequencePoint,
          write(Clk::CLK_RTC_DIV::_int, Register::value<clkRtcDiv>()),
          write(Clk::CLK_RTC_DIV::frac, Register::value<0>()),
          Clk::CLK_RTC_CTRL::overrideDefaults(write(Clk::CLK_RTC_CTRL::AUXSRCValC::xosc_clksrc)),
          Register::sequencePoint,
          set(Clk::CLK_RTC_CTRL::enable));

        // clk_rtc set up while the RTC is still in reset, then the block out of reset
        static constexpr auto powerClockEnable
          = list(clockSetup, Register::sequencePoint, clear(Rp::Block<Regs>::reset));

        // RTC_ACTIVE after an RTC_ENABLE write: the write "will take 2 clk_rtc clock periods to arrive, additional
        // to the clk_sys domain" (RP2040 datasheet 4.8.4 note, md l.25727); the way back is not given - as long
        // again (unchecked), x2: 8 clk_rtc periods.
        using SyncBound = Register::MicrosecondsBound<8ULL * 1'000'000 / clkRtcHz + 1>;

        // the block's 1 Hz divider
        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::CLKDIV_M1::clkdiv_m1, Register::value<clkRtcHz - 1>()));

        [[nodiscard]] static bool running() { return apply(read(Regs::CTRL::active)) != 0; }

        /// Load a calendar time and start counting. Blocks for a few clk_rtc cycles while the block stops and
        /// restarts; false if it did not answer within SyncBound, also after clk_rtc was set up again (header
        /// comment). The RTC is stopped then: call it again later.
        [[nodiscard]] static bool set(DateTime const& t) {
            if(load(t)) { return true; }
            apply(clockSetup);
            clockRestarts_ = clockRestarts_ + 1;
            return load(t);
        }

        /// The clk_rtc set-ups set() needed since boot (its RTC_ACTIVE did not change in time).
        [[nodiscard]] static std::uint32_t clockRestarts() { return clockRestarts_; }

    private:
        inline static std::uint32_t volatile clockRestarts_{};

        // stop, load, start: false where RTC_ACTIVE did not follow
        static bool load(DateTime const& t) {
            using Register::OnTimeout::Report;
            apply(write(Regs::CTRL::enable, Register::value<0>()));
            if(!Register::waitUntil<SyncBound, Report>(Register::isClear(Regs::CTRL::active))) {
                return false;
            }
            apply(write(Regs::SETUP_0::year, static_cast<std::uint32_t>(t.year)),
                  write(Regs::SETUP_0::month, static_cast<std::uint32_t>(t.month)),
                  write(Regs::SETUP_0::day, static_cast<std::uint32_t>(t.day)));
            apply(write(Regs::SETUP_1::dotw, static_cast<std::uint32_t>(t.dotw)),
                  write(Regs::SETUP_1::hour, static_cast<std::uint32_t>(t.hour)),
                  write(Regs::SETUP_1::min, static_cast<std::uint32_t>(t.minute)),
                  write(Regs::SETUP_1::sec, static_cast<std::uint32_t>(t.second)));
            apply(write(Regs::CTRL::load, Register::value<1>()));
            apply(write(Regs::CTRL::enable, Register::value<1>()));
            return static_cast<bool>(
              Register::waitUntil<SyncBound, Report>(Register::isSet(Regs::CTRL::active)));
        }

    public:
        /// The calendar now. RTC_0 has to be read first: it latches RTC_1.
        [[nodiscard]] static DateTime now() {
            auto const r0 = apply(read(Regs::_0::dotw),
                                  read(Regs::_0::hour),
                                  read(Regs::_0::min),
                                  read(Regs::_0::sec));
            auto const r1 = apply(read(Regs::_1::year), read(Regs::_1::month), read(Regs::_1::day));
            return {static_cast<std::uint16_t>(get<0>(r1)),
                    static_cast<std::uint8_t>(get<1>(r1)),
                    static_cast<std::uint8_t>(get<2>(r1)),
                    static_cast<std::uint8_t>(get<0>(r0)),
                    static_cast<std::uint8_t>(get<1>(r0)),
                    static_cast<std::uint8_t>(get<2>(r0)),
                    static_cast<std::uint8_t>(get<3>(r0))};
        }

        // -- the alarm, used by Alarm<> below -------------------------------------------
        static void setMatch(DateTime const& t) {
            // Match on the calendar fields, not the day of the week (it follows from them).
            apply(write(Regs::IRQ_SETUP_0::year, static_cast<std::uint32_t>(t.year)),
                  write(Regs::IRQ_SETUP_0::month, static_cast<std::uint32_t>(t.month)),
                  write(Regs::IRQ_SETUP_0::day, static_cast<std::uint32_t>(t.day)),
                  write(Regs::IRQ_SETUP_0::year_ena, 1U),
                  write(Regs::IRQ_SETUP_0::month_ena, 1U),
                  write(Regs::IRQ_SETUP_0::day_ena, 1U),
                  write(Regs::IRQ_SETUP_0::match_ena, 0U));
            apply(write(Regs::IRQ_SETUP_1::hour, static_cast<std::uint32_t>(t.hour)),
                  write(Regs::IRQ_SETUP_1::min, static_cast<std::uint32_t>(t.minute)),
                  write(Regs::IRQ_SETUP_1::sec, static_cast<std::uint32_t>(t.second)),
                  write(Regs::IRQ_SETUP_1::hour_ena, 1U),
                  write(Regs::IRQ_SETUP_1::min_ena, 1U),
                  write(Regs::IRQ_SETUP_1::sec_ena, 1U),
                  write(Regs::IRQ_SETUP_1::dotw_ena, 0U));
        }

        static void matchEnable(bool on) {
            apply(write(Regs::IRQ_SETUP_0::match_ena, on ? 1U : 0U));
        }

        [[nodiscard]] static bool matchEnabled() {
            return apply(read(Regs::IRQ_SETUP_0::match_ena)) != 0;
        }

        [[nodiscard]] static bool matching() {
            return apply(read(Regs::IRQ_SETUP_0::match_active)) != 0;
        }
    };

    /// The RTC's alarm: `Callback` runs from the rtc interrupt in the second the counter
    /// matches the armed calendar time. The match is disabled before the callback (the
    /// interrupt stays asserted for the whole matching second otherwise), so a re-arm inside
    /// the callback wins.
    template<typename Timer, void (*Callback)(), int Priority = 3>
    struct Alarm {
        using NvicIndex = Nvic::Index<decltype(Kvasir::Interrupt::rtc)::value>;
        using Regs      = typename Timer::Regs;

        using Provides = brigand::list<Startup::Resource<AlarmTag, 0>>;
        using Claims   = brigand::list<Startup::Resource<TimerTag, 0>>;

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<Priority>(NvicIndex{}), Nvic::makeClearPending(NvicIndex{}));

        static constexpr auto initStepPeripheryEnable = list(Nvic::makeEnable(NvicIndex{}));

        static void runtimeInit() { apply(write(Regs::INTE::rtc, Register::value<1>())); }

        static void armAt(DateTime const& when) {
            Timer::matchEnable(false);
            Timer::setMatch(when);
            Timer::matchEnable(true);
        }

        static void armIn(std::chrono::seconds in) { armAt(Timer::now().plus(in)); }

        static void disarm() { Timer::matchEnable(false); }

        [[nodiscard]] static bool armed() { return Timer::matchEnabled(); }

        static void onIsr() {
            Timer::matchEnable(false);
            Callback();
        }

        using Isr = Nvic::Isr<std::addressof(onIsr), NvicIndex>;
    };

}}   // namespace Kvasir::Rtc
