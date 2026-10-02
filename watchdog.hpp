#pragma once

#if __has_include("peripherals/TICKS.hpp")
    #include "peripherals/SYSCFG.hpp"
    #include "peripherals/TICKS.hpp"
#endif

#include "Clocks.hpp"
#include "peripherals/WATCHDOG.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>

// The RP2040 / RP2350 watchdog (RP2040 datasheet 4.7, RP2350 datasheet 12.9): a 24-bit counter
// loaded by LOAD (and by every feed), counting down once per watchdog tick; at zero it resets what
// PSM.WDSEL / RESETS.WDSEL select - chip/StartUp.hpp's FirstInitStep selects everything.
//
//   struct WatchdogConfig {
//       static constexpr auto clockSpeed   = HW::CrystalSpeed;   // clk_ref, which the tick divides
//       static constexpr auto overrunTime  = 1s;
//       static constexpr bool pauseOnDebug = true;               // optional, default true
//   };
//   using Watchdog = Kvasir::Watchdog<WatchdogConfig>;   // in the Startup list (or arm() it)
//   Watchdog::feed();
//
// The tick:
//   - RP2350: the TICKS block's WATCHDOG generator (8.5, Table 617), its own divider of clk_ref,
//     nine bits (Table 631). A 1 us tick (CYCLES = clk_ref in MHz) as long as the timeout fits in
//     the 24-bit LOAD (0xffffff us = 16.78 s, Table 1249) - then CTRL.TIME is in microseconds as
//     Table 1248 says, and the boot ROM's reboot(), which loads `delay_ms * 1000` and keeps a
//     running tick as it finds it (pico-bootrom-rp2350 varm_apis.c), waits what it was asked
//     to. Longer timeouts take a coarser tick, up to 511 cycles: 714 s at 12 MHz.
//   - RP2040: WATCHDOG.TICK (4.7.2, Table 550), which is ALSO the TIMER's tick (4.7.2, Note) -
//     so it stays at 1 us (what Timer.hpp writes), and the counter is decremented twice per tick
//     (erratum RP2040-E1, 4.7.3 / Table 547): LOAD is twice the ticks, 8.39 s at most.
namespace Kvasir {

// Startup resource: the one watchdog (kvasir/StartUp/Resources.hpp).
struct WatchdogTag {};

template<typename Config>
struct Watchdog {
    using Regs = Kvasir::Peripheral::WATCHDOG::Registers<>;

#if __has_include("peripherals/TICKS.hpp")
    static constexpr bool Rp2040 = false;
#else
    static constexpr bool Rp2040 = true;
#endif

    // Startup: the watchdog, and the clock its tick generator counts. That is clk_ref (the
    // crystal), not the core clock: a Config::clockSpeed of the core's speed gives a period
    // sixteen times too short, and with the clocks declared that is a build error.
    using Provides = brigand::list<Startup::Resource<WatchdogTag, 0>>;
    using Claims   = Clocks::Claim<Clocks::ClkRef, Config::clockSpeed>;

    static constexpr std::uint64_t ClockHz = static_cast<std::uint64_t>(Config::clockSpeed);

    /// PAUSE_DBG0/DBG1/JTAG (Table 1248 / RP2040 Table 546): the counter stands while a core is
    /// halted by the debugger or the debugger accesses the bus. Default on (the reset value).
    static constexpr bool PauseOnDebug = [] {
        if constexpr(requires { Config::pauseOnDebug; }) {
            return static_cast<bool>(Config::pauseOnDebug);
        } else {
            return true;
        }
    }();

    static constexpr std::uint64_t MaxLoad = 0xffffff;   // LOAD[23:0]

    /// RP2040-E1: the counter goes down by two per tick (RP2040 only; the RP2350 data sheet
    /// 12.9 has no such note and its TIME field counts microseconds, Table 1248).
    static constexpr std::uint64_t DecrementsPerTick = Rp2040 ? 2 : 1;

    static constexpr std::uint64_t TotalCycles{
      static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(Config::overrunTime).count())
      * ClockHz / 1'000'000};

    // clk_ref cycles of a 1 us tick, 0 when clk_ref is not a whole number of MHz
    static constexpr std::uint64_t MicrosecondCycles
      = ClockHz % 1'000'000 == 0 ? ClockHz / 1'000'000 : 0;

    static_assert(!Rp2040 || MicrosecondCycles != 0,
                  "RP2040: the watchdog tick is the TIMER's 1 us tick (datasheet 4.7.2), so "
                  "clk_ref must be a whole number of MHz");

    static constexpr std::uint64_t MaxTicks = MaxLoad / DecrementsPerTick;

    static constexpr std::uint32_t CyclesPerTick = static_cast<std::uint32_t>(
      Rp2040 ? MicrosecondCycles
             : std::max<std::uint64_t>(MicrosecondCycles == 0 ? 1 : MicrosecondCycles,
                                       (TotalCycles + MaxTicks - 1) / MaxTicks));

    static_assert(
      CyclesPerTick <= 511,
      "Watchdog timeout too long: CyclesPerTick exceeds the 9-bit tick divider (max 511, RP2350 "
      "datasheet Table 631; ~714 s at 12 MHz)");

    /// Ticks from a feed to the reset, and what LOAD is written with.
    static constexpr std::uint64_t Ticks = (TotalCycles + CyclesPerTick - 1) / CyclesPerTick;

    static_assert(Ticks <= MaxTicks,
                  "Watchdog timeout too long: more ticks than the 24-bit LOAD holds (RP2040: "
                  "0xffffff / 2 us = 8.39 s because of erratum RP2040-E1)");
    static_assert(Ticks > 0,
                  "Watchdog timeout too short: less than one tick");

    static constexpr std::uint32_t ReloadValue
      = static_cast<std::uint32_t>(Ticks * DecrementsPerTick);

    /// The timeout the registers really give (rounded up to whole ticks).
    static constexpr std::chrono::nanoseconds Timeout{
      static_cast<std::int64_t>(Ticks * CyclesPerTick * 1'000'000'000ULL / ClockHz)};

    /// One tick, the resolution of the timeout.
    static constexpr std::chrono::nanoseconds TickPeriod{
      static_cast<std::int64_t>(CyclesPerTick * 1'000'000'000ULL / ClockHz)};

private:
    static constexpr auto tickConfig() {
#if __has_include("peripherals/TICKS.hpp")
        // "Before changing the cycle count, always stop the tick generator" (RP2350 8.5.1).
        using Ticks_ = Kvasir::Peripheral::TICKS::Registers<>;
        return list(
          Ticks_::WATCHDOG_CTRL::overrideDefaults(clear(Ticks_::WATCHDOG_CTRL::enable)),
          Kvasir::Register::sequencePoint,
          write(Ticks_::WATCHDOG_CYCLES::watchdog_cycles, Kvasir::Register::value<CyclesPerTick>()),
          Kvasir::Register::sequencePoint,
          Ticks_::WATCHDOG_CTRL::overrideDefaults(set(Ticks_::WATCHDOG_CTRL::enable)));
#else
        // The TIMER's tick too: the same two writes Timer.hpp makes, never stopped (a stop
        // would stop the TIMER).
        using Tick = Kvasir::Peripheral::WATCHDOG::Registers<>::TICK;
        return list(write(Tick::cycles, Kvasir::Register::value<CyclesPerTick>()),
                    write(Tick::enable, Kvasir::Register::value<1>()));
#endif
    }

public:
    static constexpr auto initStepPeripheryConfig
      = list(Regs::CTRL::overrideDefaults(clear(Regs::CTRL::enable)),
             Kvasir::Register::sequencePoint,
             tickConfig(),
             write(Regs::LOAD::load, Kvasir::Register::value<ReloadValue>()));

#if __has_include("peripherals/TICKS.hpp")
    // SYSCFG.AUXCTRL bit 0 (RP2350 datasheet Table 1317): "Force POWMAN clock to switch to
    // LPOSC ... This must be set before initiating a watchdog reset of the RSM from a stage that
    // includes CLOCKS, if POWMAN is running from clk_ref" - which it does by default
    // (SEQ_CFG.USING_FAST_POWCK, Table 491), and PSM.WDSEL includes CLOCKS. Set only right before
    // a reset this code makes itself, as the boot ROM's reboot() does: while the bit is set POWMAN
    // runs from the 32 kHz LPOSC and the AON timer's tick is far off. The watchdog reset clears it.
    static void powmanOffClkRef() {
        using AuxCtrl    = Kvasir::Peripheral::SYSCFG::Registers<>::AUXCTRL;
        auto const value = get<0>(apply(read(AuxCtrl::auxctrl)));
        apply(write(AuxCtrl::auxctrl, value | 1U));
    }
#endif

    // CTRL.TIME is read-only (the live countdown on the RP2350, not on the RP2040: see
    // remainingTicks()); the counter is set through LOAD above and by feed(). The pause bits keep the watchdog from firing
    // while a debugger holds a core (Config::pauseOnDebug).
    static constexpr auto initStepPeripheryEnable = list(Regs::CTRL::overrideDefaults(
      set(Regs::CTRL::enable),
      write(Regs::CTRL::pause_dbg1, Kvasir::Register::value<PauseOnDebug ? 1U : 0U>()),
      write(Regs::CTRL::pause_dbg0, Kvasir::Register::value<PauseOnDebug ? 1U : 0U>()),
      write(Regs::CTRL::pause_jtag, Kvasir::Register::value<PauseOnDebug ? 1U : 0U>())));

    /// Reload the counter to the full timeout (LOAD, Table 1249: write-only, takes effect at once).
    static void feed() { apply(write(Regs::LOAD::load, Kvasir::Register::value<ReloadValue>())); }

    /// Arm at run time, as the Startup list would: for a watchdog that is not in the list, or one
    /// that was disarmed.
    static void arm() {
        apply(initStepPeripheryConfig);
        apply(initStepPeripheryEnable);
    }

    /// Stop the counter (CTRL.ENABLE = 0: "When not enabled the watchdog timer is paused").
    static void disarm() { apply(clear(Regs::CTRL::enable)); }

    /// Ticks left before the reset (CTRL.TIME). RP2350 only: on the RP2040 CTRL.TIME does not
    /// follow the counter (a hardware bug the pico-sdk documents at
    /// watchdog_get_time_remaining_us).
    [[nodiscard]] static std::uint32_t remainingTicks()
        requires(!Rp2040)
    {
        return static_cast<std::uint32_t>(get<0>(apply(read(Regs::CTRL::time))));
    }

    /// Reset now through CTRL.TRIGGER; REASON then says FORCE. On the RP2350 POWMAN is moved
    /// off clk_ref first and given a few clk_ref cycles for the switch (the boot ROM's reboot()
    /// counts on its 1 ms timer for that; a trigger has no such delay).
    [[noreturn]] static void trigger() {
#if __has_include("peripherals/TICKS.hpp")
        powmanOffClkRef();
        for(std::uint32_t volatile i = 0; i != 1000; i = i + 1) {}
#endif
        apply(set(Regs::CTRL::trigger));
        while(true) { asm volatile("" ::: "memory"); }
    }

    /// Whether the last reset was the watchdog's (REASON: timer or force). Both bits are zero after
    /// a hardware reset, and on the RP2350 also after a debugger's warm reset of a core (Table 1250).
    static bool causedReset() {
        auto const reasons = apply(read(Regs::REASON::force), read(Regs::REASON::timer));
        return get<0>(reasons) || get<1>(reasons);
    }
};

}   // namespace Kvasir
