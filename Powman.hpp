#pragma once
#if !__has_include("chip/rp2350.hpp")
    #error "the always-on timer (POWMAN) exists on the RP2350 only; the RP2040 has an RTC (Rtc.hpp)"
#endif
#include "Clocks.hpp"
#include "chip/Interrupt.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/POWMAN.hpp"

#include <chrono>
#include <cstdint>

// The RP2350's always-on timer (POWMAN.TIMER): a 64-bit millisecond counter in the
// always-on domain, the chip's replacement for the RP2040's RTC. It ticks from the crystal
// (precise) or the ~32 kHz LPOSC (imprecise, alive in the low-power states) and has one
// alarm on the powman_timer interrupt, which can also power the chip up. Every register
// write carries the password 0x5AFE in its upper half; the driver supplies it.
//
//   using AonTimer = Kvasir::Powman::AonTimer<HW::CrystalSpeed>;      // in the Startup list
//   AonTimer::set(std::chrono::milliseconds{...});                    // an epoch of your choosing
//   auto const ms = AonTimer::now();                                  // std::chrono::milliseconds
//
//   void onAlarm();
//   using Alarm = Kvasir::Powman::Alarm<AonTimer, &onAlarm>;          // in the Startup list too
//   Alarm::armAt(AonTimer::now() + 1500ms); Alarm::disarm();
namespace Kvasir { namespace Powman {

    namespace detail {
        constexpr std::uint32_t Password = 0x5AFE'0000U;

        template<typename Reg>
        inline void pw(std::uint32_t value16) {
            apply(write(Reg::FULLREGISTER, Password | (value16 & 0xFFFFU)));
        }
    }   // namespace detail

    struct TimerTag {};

    template<auto CrystalHz, bool UseCrystal = true>
    struct AonTimer {
        using Regs = Kvasir::Peripheral::POWMAN::Registers<>;

        static constexpr std::uint32_t KhzInt    = static_cast<std::uint32_t>(CrystalHz / 1000);
        static constexpr std::uint32_t KhzFrac16 = static_cast<std::uint32_t>(
          (static_cast<std::uint64_t>(CrystalHz % 1000) * 65536U) / 1000U);

        using Provides = brigand::list<Startup::Resource<TimerTag, 0>>;

        // The TIMER register bits (the password goes in the upper half of every write).
        static constexpr std::uint32_t Run            = 1U << 1;
        static constexpr std::uint32_t Clear          = 1U << 2;
        static constexpr std::uint32_t AlarmEnable    = 1U << 4;
        static constexpr std::uint32_t PowerUpOnAlarm = 1U << 5;
        static constexpr std::uint32_t AlarmFlag      = 1U << 6;
        static constexpr std::uint32_t UseLposc       = 1U << 8;
        static constexpr std::uint32_t UseXosc        = 1U << 9;
        static constexpr std::uint32_t UsingXosc      = 1U << 16;
        static constexpr std::uint32_t UsingLposc     = 1U << 17;

        // Stop, pick the tick source, start. The 1 kHz tick from the crystal needs its
        // frequency in kHz (16.16); the LPOSC's is whatever the oscillator does.
        static void runtimeInit() {
            detail::pw<typename Regs::TIMER>(0);   // stop, so the source can change
            if constexpr(UseCrystal) {
                detail::pw<typename Regs::XOSC_FREQ_KHZ_INT>(KhzInt);
                detail::pw<typename Regs::XOSC_FREQ_KHZ_FRAC>(KhzFrac16);
                detail::pw<typename Regs::TIMER>(UseXosc);
            } else {
                detail::pw<typename Regs::TIMER>(UseLposc);
            }
            detail::pw<typename Regs::TIMER>(Run | (UseCrystal ? UseXosc : UseLposc));
            if constexpr(UseCrystal) {
                while((get<0>(apply(read(Regs::TIMER::FULLREGISTER))) & UsingXosc) == 0) {}
            }
        }

        [[nodiscard]] static bool running() {
            return (get<0>(apply(read(Regs::TIMER::FULLREGISTER))) & Run) != 0;
        }

        /// The counter, in milliseconds since whatever set() made zero.
        [[nodiscard]] static std::chrono::milliseconds now() {
            return std::chrono::milliseconds{raw()};
        }

        [[nodiscard]] static std::uint64_t raw() {
            std::uint32_t hi = get<0>(apply(read(Regs::READ_TIME_UPPER::read_time_upper)));
            while(true) {
                auto const lo   = get<0>(apply(read(Regs::READ_TIME_LOWER::read_time_lower)));
                auto const next = get<0>(apply(read(Regs::READ_TIME_UPPER::read_time_upper)));
                if(next == hi) { return (static_cast<std::uint64_t>(hi) << 32) | lo; }
                hi = next;
            }
        }

        /// Set the counter (stopped for the write, then running again).
        static void set(std::chrono::milliseconds t) {
            auto const v      = static_cast<std::uint64_t>(t.count());
            auto const source = UseCrystal ? UseXosc : UseLposc;
            detail::pw<typename Regs::TIMER>(source);   // Run clear: stopped
            detail::pw<typename Regs::SET_TIME_15TO0>(static_cast<std::uint32_t>(v));
            detail::pw<typename Regs::SET_TIME_31TO16>(static_cast<std::uint32_t>(v >> 16));
            detail::pw<typename Regs::SET_TIME_47TO32>(static_cast<std::uint32_t>(v >> 32));
            detail::pw<typename Regs::SET_TIME_63TO48>(static_cast<std::uint32_t>(v >> 48));
            detail::pw<typename Regs::TIMER>(Run | source);
        }

        // -- the alarm, used by Alarm<> below -------------------------------------------
        static void setAlarmTime(std::uint64_t ms) {
            detail::pw<typename Regs::ALARM_TIME_15TO0>(static_cast<std::uint32_t>(ms));
            detail::pw<typename Regs::ALARM_TIME_31TO16>(static_cast<std::uint32_t>(ms >> 16));
            detail::pw<typename Regs::ALARM_TIME_47TO32>(static_cast<std::uint32_t>(ms >> 32));
            detail::pw<typename Regs::ALARM_TIME_63TO48>(static_cast<std::uint32_t>(ms >> 48));
        }

        /// TIMER bits set / cleared with the password (the atomic aliases carry it too).
        static void timerSet(std::uint32_t bits) {
            apply(write(Regs::TIMER::FULLREGISTER,
                        (get<0>(apply(read(Regs::TIMER::FULLREGISTER))) & 0xFFFFU & ~AlarmFlag)
                          | bits | detail::Password));
        }

        static void timerClear(std::uint32_t bits) {
            apply(write(
              Regs::TIMER::FULLREGISTER,
              ((get<0>(apply(read(Regs::TIMER::FULLREGISTER))) & 0xFFFFU & ~AlarmFlag) & ~bits)
                | detail::Password));
        }

        static void clearAlarmFlag() {
            apply(write(Regs::TIMER::FULLREGISTER,
                        (get<0>(apply(read(Regs::TIMER::FULLREGISTER))) & 0xFFFFU) | AlarmFlag
                          | detail::Password));
        }

        [[nodiscard]] static bool alarmFlag() {
            return (get<0>(apply(read(Regs::TIMER::FULLREGISTER))) & AlarmFlag) != 0;
        }
    };

    struct AlarmTag {};

    /// The always-on timer's one alarm: `Callback` runs from the powman_timer interrupt when
    /// the counter reaches the armed time. PowerUp: the alarm also powers the chip up from
    /// the low-power states.
    template<typename Timer, void (*Callback)(), bool PowerUp = false, int Priority = 3>
    struct Alarm {
        using Regs      = typename Timer::Regs;
        using NvicIndex = Nvic::Index<decltype(Kvasir::Interrupt::powman_timer)::value>;

        using Provides = brigand::list<Startup::Resource<AlarmTag, 0>>;
        using Claims   = brigand::list<Startup::Resource<TimerTag, 0>>;

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<Priority>(NvicIndex{}), Nvic::makeClearPending(NvicIndex{}));

        static constexpr auto initStepPeripheryEnable = list(Nvic::makeEnable(NvicIndex{}));

        static void runtimeInit() {
            // INTE.TIMER: the alarm's interrupt; the flag itself is enabled per arm.
            apply(write(Regs::INTE::FULLREGISTER, detail::Password | (1U << 1)));
        }

        static void armAt(std::chrono::milliseconds when) {
            Timer::timerClear(Timer::AlarmEnable);
            Timer::clearAlarmFlag();
            Timer::setAlarmTime(static_cast<std::uint64_t>(when.count()));
            Timer::timerSet(Timer::AlarmEnable | (PowerUp ? Timer::PowerUpOnAlarm : 0U));
        }

        static void armIn(std::chrono::milliseconds in) { armAt(Timer::now() + in); }

        static void disarm() {
            Timer::timerClear(Timer::AlarmEnable | Timer::PowerUpOnAlarm);
            Timer::clearAlarmFlag();
        }

        [[nodiscard]] static bool armed() {
            return (get<0>(apply(read(Regs::TIMER::FULLREGISTER))) & Timer::AlarmEnable) != 0;
        }

        static void onIsr() {
            // The alarm bit stays set while the time matches; disarm before the callback
            // so a re-arm inside it wins. PWRUP_ON_ALARM goes with it: a fired alarm must
            // not stay armed as a power-up source for a wake nobody asked for (pico-sdk
            // 2.3.1 made aon_timer_disable_alarm clear it for the same reason).
            Timer::timerClear(Timer::AlarmEnable | Timer::PowerUpOnAlarm);
            Timer::clearAlarmFlag();
            Callback();
        }

        static constexpr Nvic::Isr<std::addressof(onIsr), NvicIndex> isr{};
    };

}}   // namespace Kvasir::Powman
