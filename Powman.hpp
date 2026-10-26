#pragma once
#ifndef KVASIR_CHIP_RP2350
    #error "the always-on timer (POWMAN) exists on the RP2350 only; the RP2040 has an RTC (Rtc.hpp)"
#endif
#include "Clocks.hpp"
#include "WaitBounds.hpp"
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
        // A 16-bit POWMAN register (XOSC_FREQ_KHZ_*, SET_TIME_*, ALARM_TIME_*): the password key field and the
        // value's field cover the word, so it is one store with no read (RP2350 datasheet 6.4, md l.23076: every
        // protected write carries 0x5AFE in bits 31:16).
        template<typename Reg,
                 typename Field>
        [[gnu::always_inline]] inline void write16(Field         field,
                                                   std::uint32_t value) {
            apply(write(Reg::PASSWORDValC::password), write(field, value));
        }
    }   // namespace detail

    struct TimerTag {};

    template<auto CrystalHz, bool UseCrystal = true>
    struct AonTimer {
        using Regs = Kvasir::Peripheral::POWMAN::Registers<>;

        static constexpr std::uint32_t khzInt    = static_cast<std::uint32_t>(CrystalHz / 1000);
        static constexpr std::uint32_t khzFrac16 = static_cast<std::uint32_t>(
          (static_cast<std::uint64_t>(CrystalHz % 1000) * 65536U) / 1000U);

        using Provides = brigand::list<Startup::Resource<TimerTag, 0>>;

        // TIMER as one store: the key field `password` (svd_converter adds it from chip.svd's write key marker: the
        // SVD cannot have it as a field, USING_* are read-only up there), every read-write field named - nonsec_write,
        // run, alarm_enab, pwrup_on_alarm, use_gpio_1hz: all 0 but run - and the source triggers given. What is left
        // of the register (the clear/use_* triggers, the W1C alarm flag, read-only and reserved bits) needs no read
        // and goes out 0: the word of a whole-register write, 0x5AFE0000 | run | trigger.
        template<bool Run,
                 typename... Triggers>
        [[gnu::always_inline]] static void timerWrite(Triggers... triggers) {
            using T = typename Regs::TIMER;
            apply(write(T::PASSWORDValC::password),
                  Kvasir::Register::clear(T::nonsec_write),
                  write(T::run, Kvasir::Register::value<std::uint32_t, Run ? 1U : 0U>()),
                  Kvasir::Register::clear(T::alarm_enab),
                  Kvasir::Register::clear(T::pwrup_on_alarm),
                  Kvasir::Register::clear(T::use_gpio_1hz),
                  Kvasir::Register::set(triggers)...);
        }

        // the tick source's one-to-set trigger: USE_XOSC or USE_LPOSC
        static consteval auto sourceTrigger() {
            if constexpr(UseCrystal) {
                return Regs::TIMER::use_xosc;
            } else {
                return Regs::TIMER::use_lposc;
            }
        }

        // Stop, pick the tick source, start. The 1 kHz tick from the crystal needs its
        // frequency in kHz (16.16); the LPOSC's is whatever the oscillator does.
        static void runtimeInit() {
            timerWrite<false>();   // stop, so the source can change
            if constexpr(UseCrystal) {
                detail::write16<typename Regs::XOSC_FREQ_KHZ_INT>(
                  Regs::XOSC_FREQ_KHZ_INT::xosc_freq_khz_int,
                  khzInt);
                detail::write16<typename Regs::XOSC_FREQ_KHZ_FRAC>(
                  Regs::XOSC_FREQ_KHZ_FRAC::xosc_freq_khz_frac,
                  khzFrac16);
            }
            timerWrite<false>(sourceTrigger());
            timerWrite<true>(sourceTrigger());
            if constexpr(UseCrystal) {
                // "may take up to 1 tick cycle (1ms in normal operation)" (RP2350 datasheet 12.10.5.3, md l.58487)
                Kvasir::Register::waitUntil<Kvasir::Register::MicrosecondsBound<3'000>>(
                  [] { return get<0>(apply(read(Regs::TIMER::using_xosc))) != 0; });
            }
        }

        [[nodiscard]] static bool running() { return get<0>(apply(read(Regs::TIMER::run))) != 0; }

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
            auto const v = static_cast<std::uint64_t>(t.count());
            timerWrite<false>(sourceTrigger());   // run clear: stopped
            detail::write16<typename Regs::SET_TIME_15TO0>(Regs::SET_TIME_15TO0::set_time_15to0,
                                                           static_cast<std::uint32_t>(v));
            detail::write16<typename Regs::SET_TIME_31TO16>(Regs::SET_TIME_31TO16::set_time_31to16,
                                                            static_cast<std::uint32_t>(v >> 16));
            detail::write16<typename Regs::SET_TIME_47TO32>(Regs::SET_TIME_47TO32::set_time_47to32,
                                                            static_cast<std::uint32_t>(v >> 32));
            detail::write16<typename Regs::SET_TIME_63TO48>(Regs::SET_TIME_63TO48::set_time_63to48,
                                                            static_cast<std::uint32_t>(v >> 48));
            timerWrite<true>(sourceTrigger());
        }

        // -- the alarm, used by Alarm<> below -------------------------------------------
        static void setAlarmTime(std::uint64_t ms) {
            detail::write16<typename Regs::ALARM_TIME_15TO0>(
              Regs::ALARM_TIME_15TO0::alarm_time_15to0,
              static_cast<std::uint32_t>(ms));
            detail::write16<typename Regs::ALARM_TIME_31TO16>(
              Regs::ALARM_TIME_31TO16::alarm_time_31to16,
              static_cast<std::uint32_t>(ms >> 16));
            detail::write16<typename Regs::ALARM_TIME_47TO32>(
              Regs::ALARM_TIME_47TO32::alarm_time_47to32,
              static_cast<std::uint32_t>(ms >> 32));
            detail::write16<typename Regs::ALARM_TIME_63TO48>(
              Regs::ALARM_TIME_63TO48::alarm_time_63to48,
              static_cast<std::uint32_t>(ms >> 48));
        }

        /// TIMER fields set / cleared: one read-modify-write that carries the password (the key
        /// field; without it the write does not compile) and writes the ALARM flag (W1C) and the
        /// one-to-set triggers as 0.
        template<typename... Fields>
        static void timerSet(Fields... fields) {
            apply(write(Regs::TIMER::PASSWORDValC::password), Kvasir::Register::set(fields)...);
        }

        template<typename... Fields>
        static void timerClear(Fields... fields) {
            apply(write(Regs::TIMER::PASSWORDValC::password), Kvasir::Register::clear(fields)...);
        }

        static void clearAlarmFlag() {
            apply(write(Regs::TIMER::PASSWORDValC::password),
                  Kvasir::Register::reset(Regs::TIMER::alarm));
        }

        [[nodiscard]] static bool alarmFlag() {
            return get<0>(apply(read(Regs::TIMER::alarm))) != 0;
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

        // Only POR, BOR, the RUN pin and a few debugger/watchdog sources reset POWMAN (RP2350
        // datasheet 7.3.1 Table 528); a system reset - SYSRESETREQ, as after a flash - keeps the
        // timer, its alarm and INTE as the image before left them. That alarm may still be
        // armed, or have fired, with INTE.TIMER on: the interrupt would call Callback the moment
        // the NVIC enables it, before this image armed anything (a periodic user re-arming from
        // its own deadline then counts the whole wall time as missed periods). Disarmed, flag
        // and latched NVIC pending bit dropped, before the enable.
        static void preEnableRuntimeInit() {
            disarm();
            apply(Nvic::makeClearPending(NvicIndex{}));
        }

        static void runtimeInit() {
            // INTE.TIMER: the alarm's interrupt; the flag itself is enabled per arm. INTE (0xE4)
            // is outside the password-protected registers (offset <= 0xAC, RP2350 datasheet 6.4).
            apply(set(Regs::INTE::timer));
        }

        static void armAt(std::chrono::milliseconds when) {
            Timer::timerClear(Regs::TIMER::alarm_enab);
            Timer::clearAlarmFlag();
            Timer::setAlarmTime(static_cast<std::uint64_t>(when.count()));
            if constexpr(PowerUp) {
                Timer::timerSet(Regs::TIMER::alarm_enab, Regs::TIMER::pwrup_on_alarm);
            } else {
                Timer::timerSet(Regs::TIMER::alarm_enab);
            }
        }

        static void armIn(std::chrono::milliseconds in) { armAt(Timer::now() + in); }

        static void disarm() {
            Timer::timerClear(Regs::TIMER::alarm_enab, Regs::TIMER::pwrup_on_alarm);
            Timer::clearAlarmFlag();
        }

        [[nodiscard]] static bool armed() {
            return get<0>(apply(read(Regs::TIMER::alarm_enab))) != 0;
        }

        static void onIsr() {
            // The alarm bit stays set while the time matches; disarm before the callback
            // so a re-arm inside it wins. PWRUP_ON_ALARM goes with it: a fired alarm must
            // not stay armed as a power-up source for a wake nobody asked for (pico-sdk
            // 2.3.1 made aon_timer_disable_alarm clear it for the same reason).
            Timer::timerClear(Regs::TIMER::alarm_enab, Regs::TIMER::pwrup_on_alarm);
            Timer::clearAlarmFlag();
            Callback();
        }

        using Isr = Nvic::Isr<std::addressof(onIsr), NvicIndex>;
    };

}}   // namespace Kvasir::Powman
