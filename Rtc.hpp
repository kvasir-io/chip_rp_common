#pragma once
#if !__has_include("chip/rp2040.hpp")
    #error "the RTC exists on the RP2040 only; the RP2350 has the always-on timer (Powman.hpp)"
#endif
#include "Clocks.hpp"
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
//   Rtc::set(Kvasir::Rtc::DateTime::fromUnixSeconds(BuildEpoch));
//   Alarm::armAt(Rtc::now().plus(std::chrono::seconds{2}));
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
            return daysFromCivil(year, month, day) * 86400 + hour * 3600 + minute * 60 + second;
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
        using Regs   = Kvasir::Peripheral::RTC::Registers<>;
        using Clk    = Kvasir::Peripheral::CLOCKS::Registers<>;
        using Resets = Kvasir::Peripheral::RESETS::Registers<>;

        static constexpr std::uint32_t ClkRtcDiv = 256;
        static_assert(CrystalHz % ClkRtcDiv == 0,
                      "the crystal must divide by 256");
        static constexpr std::uint32_t ClkRtcHz = static_cast<std::uint32_t>(CrystalHz / ClkRtcDiv);
        static_assert(ClkRtcHz - 1 < 65536,
                      "CLKDIV_M1 is 16 bits: the crystal is too fast");

        using Provides = brigand::list<Startup::Resource<TimerTag, 0>>;
        using Claims   = Clocks::Claim<Clocks::ClkRef, CrystalHz>;   // clk_ref is the crystal

        static constexpr auto powerClockEnable = list(clear(Resets::RESET::rtc));

        // clk_rtc from the crystal through /256 (the divider first, as the datasheet asks
        // when the ratio grows), then the block's 1 Hz divider.
        static constexpr auto initStepPeripheryConfig
          = list(write(Clk::CLK_RTC_DIV::_int, Register::value<ClkRtcDiv>()),
                 write(Clk::CLK_RTC_DIV::frac, Register::value<0>()),
                 Clk::CLK_RTC_CTRL::overrideDefaults(
                   write(Clk::CLK_RTC_CTRL::AUXSRCValC::xosc_clksrc),
                   write(Clk::CLK_RTC_CTRL::enable, Register::value<1>())),
                 write(Regs::CLKDIV_M1::clkdiv_m1, Register::value<ClkRtcHz - 1>()));

        [[nodiscard]] static bool running() { return apply(read(Regs::CTRL::active)); }

        /// Load a calendar time and start counting. Blocks for a few clk_rtc cycles while the
        /// block stops and restarts.
        static void set(DateTime const& t) {
            apply(write(Regs::CTRL::enable, Register::value<0>()));
            while(running()) {}
            apply(write(Regs::SETUP_0::year, static_cast<std::uint32_t>(t.year)),
                  write(Regs::SETUP_0::month, static_cast<std::uint32_t>(t.month)),
                  write(Regs::SETUP_0::day, static_cast<std::uint32_t>(t.day)));
            apply(write(Regs::SETUP_1::dotw, static_cast<std::uint32_t>(t.dotw)),
                  write(Regs::SETUP_1::hour, static_cast<std::uint32_t>(t.hour)),
                  write(Regs::SETUP_1::min, static_cast<std::uint32_t>(t.minute)),
                  write(Regs::SETUP_1::sec, static_cast<std::uint32_t>(t.second)));
            apply(write(Regs::CTRL::load, Register::value<1>()));
            apply(write(Regs::CTRL::enable, Register::value<1>()));
            while(!running()) {}
        }

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
            return apply(read(Regs::IRQ_SETUP_0::match_ena));
        }

        [[nodiscard]] static bool matching() {
            return apply(read(Regs::IRQ_SETUP_0::match_active));
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

        static constexpr Nvic::Isr<std::addressof(onIsr), NvicIndex> isr{};
    };

}}   // namespace Kvasir::Rtc
