#pragma once

#include "peripherals/TICKS.hpp"
#include "peripherals/WATCHDOG.hpp"

#include <chrono>

namespace Kvasir {

template<typename Config>
struct Watchdog {
    using Regs      = Kvasir::Peripheral::WATCHDOG::Registers<>;
    using TicksRegs = Kvasir::Peripheral::TICKS::Registers<>;

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

    static constexpr auto initStepPeripheryConfig = list(
      TicksRegs::WATCHDOG_CTRL::overrideDefaults(clear(TicksRegs::WATCHDOG_CTRL::enable)),
      Regs::CTRL::overrideDefaults(clear(Regs::CTRL::enable)),

      Kvasir::Register::sequencePoint,
      write(TicksRegs::WATCHDOG_CYCLES::watchdog_cycles, Kvasir::Register::value<CyclesPerTick>()),
      write(Regs::LOAD::load, Kvasir::Register::value<ReadloadValue>()),

      Kvasir::Register::sequencePoint,
      TicksRegs::WATCHDOG_CTRL::overrideDefaults(set(TicksRegs::WATCHDOG_CTRL::enable)));

    static constexpr auto initStepPeripheryEnable = list(Regs::CTRL::overrideDefaults(
      set(Regs::CTRL::enable),
      set(Regs::CTRL::pause_dbg0),
      set(Regs::CTRL::pause_dbg1),
      set(Regs::CTRL::pause_jtag),
      write(Regs::CTRL::time, Kvasir::Register::value<ReadloadValue>())));

    static void feed() { apply(write(Regs::LOAD::load, Kvasir::Register::value<ReadloadValue>())); }

    static bool causedReset() {
        auto const reasons = apply(read(Regs::REASON::force), read(Regs::REASON::timer));
        return get<0>(reasons) || get<1>(reasons);
    }
};

}   // namespace Kvasir
