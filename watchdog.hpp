#pragma once

#if __has_include("peripherals/TICKS.hpp")
    #include "peripherals/TICKS.hpp"
#endif

#include "Clocks.hpp"
#include "peripherals/WATCHDOG.hpp"

#include <chrono>

namespace Kvasir {

namespace Detail {

    template<typename Reg>
    constexpr auto getDisableTick() {
        if constexpr(requires { Reg::overrideDefaults(clear(Reg::enable)); }) {
            return Reg::overrideDefaults(clear(Reg::enable));
        } else {
            return Reg::WATCHDOG_CTRL::overrideDefaults(clear(Reg::WATCHDOG_CTRL::enable));
        }
    }

    template<typename Reg>
    constexpr auto getEnableTick() {
        if constexpr(requires { set(Reg::enable); }) {
            return set(Reg::enable);
        } else {
            return Reg::WATCHDOG_CTRL::overrideDefaults(set(Reg::WATCHDOG_CTRL::enable));
        }
    }

    template<typename Reg,
             auto CyclesPerTick>
    constexpr auto setTick() {
        if constexpr(requires { write(Reg::cycles, Kvasir::Register::value<CyclesPerTick>()); }) {
            return write(Reg::cycles, Kvasir::Register::value<CyclesPerTick>());
        } else {
            return write(Reg::WATCHDOG_CYCLES::watchdog_cycles,
                         Kvasir::Register::value<CyclesPerTick>());
        }
    }

    constexpr auto getTicksReg() {
#if __has_include("peripherals/TICKS.hpp")
        return Kvasir::Peripheral::TICKS::Registers<>{};
#else
        return Kvasir::Peripheral::WATCHDOG::Registers<>::TICK{};
#endif
    }
}   // namespace Detail

// Startup resource: the one watchdog (kvasir/StartUp/Resources.hpp).
struct WatchdogTag {};

template<typename Config>
struct Watchdog {
    using Regs      = Kvasir::Peripheral::WATCHDOG::Registers<>;
    using TicksRegs = decltype(Detail::getTicksReg());

    // Startup: the watchdog, and the clock its tick generator counts. That is clk_ref (the
    // crystal), not the core clock: a Config::clockSpeed of the core's speed gives a period
    // sixteen times too short, and with the clocks declared that is a build error.
    using Provides = brigand::list<Startup::Resource<WatchdogTag, 0>>;
    using Claims   = Clocks::Claim<Clocks::ClkRef, Config::clockSpeed>;

    static constexpr std::uint64_t TotalCycles{
      static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(Config::overrunTime).count())
      * Config::clockSpeed / 1'000'000};

    static constexpr std::uint32_t CyclesPerTick{
      TotalCycles > 0xffffff ? ((TotalCycles + 0xffffff - 1) / 0xffffff) : 1};

    static constexpr std::uint32_t ReadloadValue{
      static_cast<std::uint32_t>((TotalCycles + CyclesPerTick - 1) / CyclesPerTick)};

    static_assert(
      CyclesPerTick <= 511,
      "Watchdog timeout too long: CyclesPerTick exceeds 9-bit register limit (max 511)");
    static_assert(
      ReadloadValue <= 0xffffff,
      "Watchdog timeout too long: ReadloadValue exceeds 24-bit register limit (max 0xffffff)");
    static_assert(ReadloadValue > 0,
                  "Watchdog timeout too short: ReadloadValue must be greater than 0");

    static constexpr auto initStepPeripheryConfig
      = list(Detail::getDisableTick<TicksRegs>(),
             Regs::CTRL::overrideDefaults(clear(Regs::CTRL::enable)),

             Kvasir::Register::sequencePoint,
             Detail::setTick<TicksRegs, CyclesPerTick>(),
             write(Regs::LOAD::load, Kvasir::Register::value<ReadloadValue>()),

             Kvasir::Register::sequencePoint,
             Detail::getEnableTick<TicksRegs>());

    // CTRL.TIME is read-only (the live countdown, on the RP2040 as on the RP2350); the counter
    // is set through LOAD above and by feed(). The pause bits keep the watchdog from firing
    // while a debugger holds a core.
    static constexpr auto initStepPeripheryEnable
      = list(Regs::CTRL::overrideDefaults(set(Regs::CTRL::enable),
                                          set(Regs::CTRL::pause_dbg0),
                                          set(Regs::CTRL::pause_dbg1),
                                          set(Regs::CTRL::pause_jtag)));

    static void feed() { apply(write(Regs::LOAD::load, Kvasir::Register::value<ReadloadValue>())); }

    static bool causedReset() {
        auto const reasons = apply(read(Regs::REASON::force), read(Regs::REASON::timer));
        return get<0>(reasons) || get<1>(reasons);
    }
};

}   // namespace Kvasir
