#pragma once
#include "Clocks.hpp"
#include "WaitBounds.hpp"
#include "chip/Blocks.hpp"   // the chip's block table (Rp::Block)
#include "chip/Interrupt.hpp"
#include "core/Nvic.hpp"
#include "core/core.hpp"
#include "kvasir/Atomic/Atomic.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "kvasir/Util/Rollover.hpp"
#include "kvasir/Util/SteadyClock.hpp"
#include "kvasir/Util/attributes.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <peripherals/RESETS.hpp>
#include <peripherals/TIMER.hpp>
#ifdef KVASIR_CHIP_RP2350
    #include <peripherals/TICKS.hpp>
#else
    #include <peripherals/WATCHDOG.hpp>   // the RP2040 ticks its timer from WATCHDOG.TICK
#endif

// A std::chrono clock on the RP2350's TIMER block: 64-bit, one microsecond, free-running
// from the tick generator, and one counter for the whole chip. That last property is why it
// exists next to Systick::SystickClockBase: SysTick is a per-core peripheral whose overrun
// counter lives in one static bumped by one core's interrupt, so its reading is only
// meaningful on the core that owns it. This clock reads the same on both cores, needs no
// interrupt, and a single Startup-list entry (on core 0) brings it up for everyone.
//
// Config:
//   static constexpr unsigned instance    = 0;          // TIMER0 or TIMER1
//   static constexpr auto     tickClockHz = 12'000'000; // clk_ref, which feeds TICKS
namespace Kvasir::Timer {

// Startup resource: one TIMER block and its TICKS generator (kvasir/StartUp/Resources.hpp).
// Claimed by every Alarm on the block, so the claim is shared.
struct InstanceTag {
    static constexpr bool sharedClaim = true;
};

template<typename TConfig>
struct TimerClockBase;

namespace Detail {
    // The std::chrono shell of TimerClockBase (Kvasir::Chrono::SteadyClock): one microsecond a tick, the same shape
    // as Systick::SystickClockBase's, so the two are interchangeable as a `Clock` template argument.
    template<typename TConfig>
    using TimerShell = Chrono::SteadyClock<TimerClockBase<TConfig>,
                                           std::chrono::duration<std::int64_t, std::micro>>;
}   // namespace Detail

template<typename TConfig>
struct TimerClockBase : Detail::TimerShell<TConfig> {
private:
    using Config = TConfig;

    static constexpr bool rp2040 = PinConfig::isRp2040(Chip::part);

    static constexpr unsigned instanceValue = Config::instance;
    static_assert(instanceValue < PinConfig::timerCount(Chip::part),
                  "the RP2350 has TIMER0 and TIMER1, the RP2040 one TIMER (instance 0)");

    using Regs = Kvasir::Peripheral::TIMER::Registers<instanceValue>;

    // The tick generator (TICKS on the RP2350, WATCHDOG.TICK on the RP2040) divides clk_ref
    // down to the 1 MHz the timer counts.
    static constexpr std::uint64_t tickClockHz = Config::tickClockHz;
    static_assert(tickClockHz % 1'000'000 == 0,
                  "clk_ref must be a whole number of MHz");
    static constexpr std::uint32_t cycles = static_cast<std::uint32_t>(tickClockHz / 1'000'000);
    static_assert(cycles > 0 && cycles < (1U << 9U),
                  "TIMERx_CYCLES is nine bits");

#ifdef KVASIR_CHIP_RP2350
    static constexpr auto tickConfig() {
        using Ticks = Kvasir::Peripheral::TICKS::Registers<>;
        if constexpr(instanceValue == 0) {
            return list(write(Ticks::TIMER0_CYCLES::timer0_cycles, Register::value<cycles>()),
                        write(Ticks::TIMER0_CTRL::enable, Register::value<1>()));
        } else {
            return list(write(Ticks::TIMER1_CYCLES::timer1_cycles, Register::value<cycles>()),
                        write(Ticks::TIMER1_CTRL::enable, Register::value<1>()));
        }
    }
#else
    static constexpr auto tickConfig() {
        using Tick = Kvasir::Peripheral::WATCHDOG::Registers<>::TICK;
        return list(write(Tick::cycles, Register::value<cycles>()),
                    write(Tick::enable, Register::value<1>()));
    }
#endif

    [[nodiscard]] static std::uint32_t rawHigh() {
        return get<0>(apply(read(Regs::TIMERAWH::FULLREGISTER)));
    }

    [[nodiscard]] static std::uint32_t rawLow() {
        return get<0>(apply(read(Regs::TIMERAWL::FULLREGISTER)));
    }

public:
    /// Which TIMER block this is, for the alarms below.
    static constexpr unsigned instance = instanceValue;

    // Startup: this timer, and the clock its TICKS generator divides (clk_ref).
    using Provides = brigand::list<Startup::Resource<InstanceTag, instanceValue>>;
    using Claims   = Hal::Claim<Clocks::ClkRef, tickClockHz>;

    using Shell = Detail::TimerShell<TConfig>;
    using typename Shell::duration;
    using typename Shell::time_point;

    // Counts from the tick generator, not from the core's clock: it runs while the core sleeps in WFE/WFI (SLEEP
    // gates clocks by SLEEP_EN0/1, all on by default - a firmware that prunes them must keep clk_ref, TICKS and the
    // TIMER; RP2040 2.11.2). Kvasir::Executor's sleeping loop requires it.
    static constexpr bool runsAsleep = true;

    // The raw pair, not the latched TIMELR/TIMEHR: reading TIMELR latches TIMEHR in one
    // register shared by both cores, so two readers would hand each other stale halves.
    // High-low-high catches the carry instead, and is safe from any core and any ISR.
    [[nodiscard]] static time_point now() {
        std::uint32_t hi = rawHigh();
        while(true) {
            std::uint32_t const lo  = rawLow();
            std::uint32_t const hi2 = rawHigh();
            if(hi2 == hi) {
                return time_point{duration{static_cast<std::int64_t>(
                  (static_cast<std::uint64_t>(hi) << 32U) | static_cast<std::uint64_t>(lo))}};
            }
            hi = hi2;
        }
    }

    template<typename Duration,
             typename duration::rep Value>
    static void delay() {
        static constexpr auto wait = std::chrono::duration_cast<duration>(Duration{Value});
        auto const            end  = now() + wait;
        while(now() < end) {}
    }

    /// Blocking delay of a runtime duration. Negative or zero returns at once.
    template<typename Rep,
             typename Period>
    static void delay(std::chrono::duration<Rep,
                                            Period> d) {
        auto const wait = std::chrono::duration_cast<duration>(d);
        if(wait <= duration::zero()) { return; }
        auto const end = now() + wait;
        while(now() < end) {}
    }

    /// Block until this clock reads `t`; returns at once if `t` is already past.
    static void delayUntil(time_point t) {
        while(now() < t) {}
    }

    // kvasir init: once, on the core whose Startup list carries this type. TICKS is not
    // behind a reset; the timer itself is, hence the reset-done poll before anyone can
    // call now().
    static constexpr auto powerClockEnable        = list(clear(Rp::Block<Regs>::reset));
    static constexpr auto initStepPeripheryConfig = tickConfig();

    static void preEnableRuntimeInit() { Rp::Block<Regs>::waitResetDone(); }
};

// One of a TIMER block's four alarm comparators as a Startup-list peripheral: `F()` runs
// from the alarm's interrupt when the counter's low 32 bits reach the armed value.
//
//   void onTick();
//   using Tick = Kvasir::Timer::Alarm<HW::TimerClock, 0, &onTick>;   // TIMER0 alarm 0
//   ...
//   Tick::armIn(std::chrono::milliseconds{1});     // one shot; re-arm from F for a period
//   Tick::arm(HW::TimerClock::now() + 500us);      // at a time_point of the clock
//
// Each alarm has its own interrupt line, so four alarms on one block are four independent
// peripherals; two on one comparator is a build error, and the clock has to be in the same
// core's list. An alarm fires once and disarms; a time already passed fires it at once
// through INTF. Arm from the core that owns the alarm's interrupt: armRaw masks this
// core's NVIC line while it decides between "already fired" and "must force".
struct AlarmTag {
    static constexpr unsigned keyArity = 2;   // (timer instance, alarm number)
};

template<typename TimerClock, unsigned N, void (*F)(), int Priority = 3>
struct Alarm {
    static_assert(N < 4,
                  "a TIMER block has four alarms");

    static constexpr unsigned instanceValue = TimerClock::instance;
    static_assert(instanceValue < PinConfig::timerCount(Chip::part),
                  "the RP2350 has TIMER0 and TIMER1, the RP2040 one TIMER");

    using Regs = Kvasir::Peripheral::TIMER::Registers<instanceValue>;
    // TIMERn_IRQ_m: 0..3 for TIMER0 (and the RP2040's timer), 4..7 for TIMER1.
    using Irq = Kvasir::Nvic::Index<static_cast<int>(instanceValue * 4 + N)>;

    using duration   = typename TimerClock::duration;
    using time_point = typename TimerClock::time_point;

    // The comparator matches the counter's low 32 bits (RP2040 4.6, md l.25040; RP2350 12.8, md l.57654) and armRaw's
    // "already behind" test covers half the range: a wake further ahead than this is clamped (Kvasir::Executor).
    static constexpr auto maxAhead = std::chrono::microseconds{(std::int64_t{1} << 31) - 1'000'000};

    using Provides = brigand::list<Startup::Resource<AlarmTag, instanceValue, N>>;
    using Claims   = brigand::list<Startup::Resource<InstanceTag, instanceValue>>;

    static constexpr std::uint32_t bit = 1U << N;

private:
    // ALARMn, and alarm n's bit of INTE / INTR / INTF: bit n (RP2040 datasheet Table 542, md l.25389; RP2350
    // datasheet Table 1244, md l.58055)
    static constexpr auto alarmField() {
        using Alarms = brigand::list<typename Regs::ALARM0,
                                     typename Regs::ALARM1,
                                     typename Regs::ALARM2,
                                     typename Regs::ALARM3>;
        return brigand::at_c<Alarms, N>::FULLREGISTER;
    }

    static constexpr auto inteField() { return Register::fieldAt<N>(typename Regs::INTE{}); }

    static constexpr auto intrField() { return Register::fieldAt<N>(typename Regs::INTR{}); }

    static constexpr auto intfField() { return Register::fieldAt<N>(typename Regs::INTF{}); }

    [[nodiscard]] static std::uint32_t lowNow() {
        return static_cast<std::uint32_t>(TimerClock::now().time_since_epoch().count());
    }

public:
    // Startup: enable this alarm's interrupt in the block and in the NVIC. INTR is
    // write-one-to-clear: the reset() writes this alarm's bit alone, one store.
    static constexpr auto initStepPeripheryConfig
      = list(Register::reset(intrField()), set(inteField()));

    static constexpr auto initStepInterruptConfig
      = list(Nvic::makeSetPriority<Priority>(Irq{}), Nvic::makeClearPending(Irq{}));
    static constexpr auto initStepPeripheryEnable = list(Nvic::makeEnable(Irq{}));

    /// Arm for the counter's low word reaching `low`. The comparator fires on equality
    /// only, so a target the counter passed between the write and the check would wait a
    /// whole 2^32 us wrap: a target behind the counter (by less than half a wrap) is forced
    /// through INTF. The check races the hardware, so the line is masked meanwhile and
    /// ARMED decides: still armed with the target behind means the comparator missed it
    /// (disarm and force); ARMED clear means it fired and the interrupt is pending. Either
    /// way onIsr sees at most one of INTR and INTF and runs F once. (pico-sdk's
    /// hardware_alarm_set_target makes the same ARMED check under its spin lock.)
    static void armRaw(std::uint32_t low) {
        Nvic::InterruptGuard<Irq> const guard;
        apply(write(alarmField(), low));
        using Low = Rollover<std::uint32_t>;
        if(!after(Low{low}, Low{lowNow()}) && armed()) {
            disarm();
            apply(set(intfField()));
        }
    }

    static void arm(time_point at) {
        armRaw(static_cast<std::uint32_t>(at.time_since_epoch().count()));
    }

    template<typename Rep,
             typename Period>
    static void armIn(std::chrono::duration<Rep,
                                            Period> d) {
        arm(TimerClock::now() + std::chrono::duration_cast<duration>(d));
    }

    /// Cancel a pending alarm (ARMED is write-one-to-clear).
    static void disarm() {
        apply(Register::reset(
          Register::bit<N>(Regs::ARMED::armed)));   // this alarm's bit alone: one store
    }

    [[nodiscard]] static bool armed() {
        return get<0>(apply(read(Register::bit<N>(Regs::ARMED::armed)))) != 0;
    }

    static void onIsr() {
        apply(clear(intfField()));
        apply(Register::reset(intrField()));
        F();
    }

    using Isr = Nvic::Isr<std::addressof(onIsr), Irq>;
};

}   // namespace Kvasir::Timer
