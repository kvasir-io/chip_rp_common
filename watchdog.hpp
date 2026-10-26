#pragma once

#ifdef KVASIR_CHIP_RP2350
    #include "peripherals/TICKS.hpp"
#endif

#include "Clocks.hpp"
#include "PinConfig.hpp"
#include "WatchdogMath.hpp"
#include "WatchdogReset.hpp"
#include "chip_reset.hpp"
#include "kvasir/Health/WatchdogFront.hpp"
#include "kvasir/Util/Config.hpp"
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

// arm(), feed() and disarm(), gated or not: the SDK's one front (kvasir/Health/WatchdogFront.hpp).
template<typename Config>
struct Watchdog : Health::WatchdogFront<Watchdog<Config>, Health::gatedFeedOf<Config>()> {
    using Regs = Kvasir::Peripheral::WATCHDOG::Registers<>;

    static constexpr bool rp2040 = Chip::part == Chip::Part::rp2040;

    // Startup: the watchdog, and the clock its tick generator counts. That is clk_ref (the
    // crystal), not the core clock: a Config::clockSpeed of the core's speed gives a period
    // sixteen times too short, and with the clocks declared that is a build error.
    using Provides = brigand::list<Startup::Resource<WatchdogTag, 0>>;
    using Claims   = Hal::Claim<Clocks::ClkRef, Config::clockSpeed>;

    static constexpr std::uint64_t clockHz = static_cast<std::uint64_t>(Config::clockSpeed);

    /// PAUSE_DBG0/DBG1/JTAG (Table 1248 / RP2040 Table 546): the counter stands while a core is
    /// halted by the debugger or the debugger accesses the bus. Default on (the reset value).
    /// Keep it on in a product: a panic's or fault's halt under the log printer then stays
    /// halted until it is reset from there, state intact. Off, the watchdog resets the halted
    /// core with the probe attached - a reset nobody can announce, which parks an RP2040 in
    /// the boot ROM (the Rescue-DP park; Kvasir_SDK Runtime/AnnouncedReset.hpp). Off is for a test
    /// that lets the watchdog bite under a printer and announces it first.
    static constexpr bool pauseOnDebug
      = static_cast<bool>(KVASIR_CONFIG_OR(Config, pauseOnDebug, true));

    /// Only Kvasir::Health::Supervisor feeds, arms and disarms (Health/HealthKey.hpp): feed(), arm() and disarm()
    /// take its FeedKey, the plain ones do not compile - arm() reloads the counter and disarm() stops it, so
    /// either would be a way round a gate on feed() alone. Default off.
    static constexpr bool gatedFeed = Health::gatedFeedOf<Config>();

    // The tick and LOAD (WatchdogMath.hpp: the datasheet facts, the refusals).
    using Timing = Detail::WatchdogTiming<
      clockHz,
      static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(Config::overrunTime).count()),
      rp2040>;

    static constexpr std::uint64_t maxLoad           = Timing::maxLoad;
    static constexpr std::uint64_t decrementsPerTick = Timing::decrementsPerTick;
    static constexpr std::uint64_t maxTicks          = Timing::maxTicks;
    static constexpr std::uint32_t cyclesPerTick     = Timing::cyclesPerTick;
    /// Ticks from a feed to the reset, and what LOAD is written with.
    static constexpr std::uint64_t ticks       = Timing::ticks;
    static constexpr std::uint32_t reloadValue = Timing::reloadValue;
    /// The timeout the registers really give (rounded up to whole ticks).
    static constexpr std::chrono::nanoseconds timeout = Timing::timeout;
    /// One tick, the resolution of the timeout.
    static constexpr std::chrono::nanoseconds tickPeriod = Timing::tickPeriod;

private:
    static constexpr auto tickConfig() {
#ifdef KVASIR_CHIP_RP2350
        // "Before changing the cycle count, always stop the tick generator" (RP2350 8.5.1).
        using Ticks_ = Kvasir::Peripheral::TICKS::Registers<>;
        return list(
          Ticks_::WATCHDOG_CTRL::overrideDefaults(clear(Ticks_::WATCHDOG_CTRL::enable)),
          Kvasir::Register::sequencePoint,
          write(Ticks_::WATCHDOG_CYCLES::watchdog_cycles, Kvasir::Register::value<cyclesPerTick>()),
          Kvasir::Register::sequencePoint,
          Ticks_::WATCHDOG_CTRL::overrideDefaults(set(Ticks_::WATCHDOG_CTRL::enable)));
#else
        // The TIMER's tick too: the same two writes Timer.hpp makes, never stopped (a stop
        // would stop the TIMER).
        using Tick = Kvasir::Peripheral::WATCHDOG::Registers<>::TICK;
        return list(write(Tick::cycles, Kvasir::Register::value<cyclesPerTick>()),
                    write(Tick::enable, Kvasir::Register::value<1>()));
#endif
    }

public:
    static constexpr auto initStepPeripheryConfig
      = list(Regs::CTRL::overrideDefaults(clear(Regs::CTRL::enable)),
             Kvasir::Register::sequencePoint,
             tickConfig(),
             write(Regs::LOAD::load, Kvasir::Register::value<reloadValue>()));

    // CTRL.TIME is read-only (the live countdown on the RP2350, not on the RP2040: see
    // remainingTicks()); the counter is set through LOAD above and by feed(). The pause bits keep the watchdog from firing
    // while a debugger holds a core (Config::pauseOnDebug).
    static constexpr auto initStepPeripheryEnable = list(Regs::CTRL::overrideDefaults(
      set(Regs::CTRL::enable),
      write(Regs::CTRL::pause_dbg1, Kvasir::Register::value<pauseOnDebug ? 1U : 0U>()),
      write(Regs::CTRL::pause_dbg0, Kvasir::Register::value<pauseOnDebug ? 1U : 0U>()),
      write(Regs::CTRL::pause_jtag, Kvasir::Register::value<pauseOnDebug ? 1U : 0U>())));

private:
    friend struct Health::WatchdogFront<Watchdog, gatedFeed>;

    /// feed(): reload the counter to the full timeout (LOAD, Table 1249: write-only, takes effect at once).
    static void feedNow() {
        apply(write(Regs::LOAD::load, Kvasir::Register::value<reloadValue>()));
    }

    /// arm(): as the Startup list would.
    static void armNow() {
        apply(initStepPeripheryConfig);
        apply(initStepPeripheryEnable);
    }

    /// disarm(): stop the counter (CTRL.ENABLE = 0: "When not enabled the watchdog timer is paused").
    static void disarmNow() { apply(clear(Regs::CTRL::enable)); }

public:
    /// Ticks left before the reset (CTRL.TIME). RP2350 only: on the RP2040 CTRL.TIME does not
    /// follow the counter (a hardware bug the pico-sdk documents at
    /// watchdog_get_time_remaining_us).
    [[nodiscard]] static std::uint32_t remainingTicks()
        requires(!rp2040)
    {
        return static_cast<std::uint32_t>(get<0>(apply(read(Regs::CTRL::time))));
    }

    /// Reset now through CTRL.TRIGGER; REASON then says FORCE. PSM.WDSEL stays as the boot set it (everything,
    /// WatchdogDomains::onTimeout). The chip's steps before the trigger (RP2350: the tick, POWMAN onto the LPOSC)
    /// are the one reset owner's (WatchdogReset.hpp), as for Rp::watchdogReboot().
    [[noreturn]] static void trigger() { Rp::Detail::watchdogResetNow(); }

    /// Whether the last reset was the watchdog's (REASON: timer or force). Both bits are zero after
    /// a hardware reset, and on the RP2350 also after a debugger's warm reset of a core (Table 1250).
    /// The RP2040 keeps REASON across a reset of the cores alone (Table 548): there the reset cause decides,
    /// which tells that reset apart (chip_reset.hpp, ResetCauseRules.hpp).
    static bool causedReset() {
#ifdef KVASIR_CHIP_RP2350
        auto const reasons = apply(read(Regs::REASON::force), read(Regs::REASON::timer));
        return get<0>(reasons) != 0 || get<1>(reasons) != 0;
#else
        auto const cause = PM::resetCause();
        return cause == PM::ResetCause::watchdogForce || cause == PM::ResetCause::watchdogTimer;
#endif
    }
};

}   // namespace Kvasir
