#pragma once

// RP2350: a watchdog reset keeps SRAM only while the watchdog tick generator in TICKS runs (8.5, 12.9.2);
// with it stopped, SRAM comes back with its power-up contents. Not in the datasheet; measured. The generator
// resets to stopped (WATCHDOG_CTRL reset value 0), so keepWatchdogTickRunning() starts it before any watchdog
// reset - or .noInit, with the panic and fault records, is lost. Called by chip_rp2350's WatchdogDomains::beforeReset()
// (the step of the one reset owner, WatchdogReset.hpp, before every CTRL.TRIGGER) and by its FirstInitStep
// (BootTraits).

#ifdef KVASIR_CHIP_RP2350
    #include "peripherals/TICKS.hpp"

    #include <cstdint>
    #include <kvasir/Register/Register.hpp>

namespace Kvasir { namespace Rp {
    // Starts the watchdog tick if it is stopped; a running one (the Watchdog driver's, the boot ROM's) is left
    // as it is - "Before changing the cycle count, always stop the tick generator" (8.5.1). 12 cycles: a 1 us
    // tick from a 12 MHz clk_ref; the Watchdog driver sets its own. Waits until RUNNING (bounded).
    inline void keepWatchdogTickRunning() {
        using Ticks = Kvasir::Peripheral::TICKS::Registers<>;
        if(get<0>(apply(read(Ticks::WATCHDOG_CTRL::enable))) != 0U) { return; }
        apply(write(Ticks::WATCHDOG_CYCLES::watchdog_cycles, Kvasir::Register::value<12U>()));
        apply(Ticks::WATCHDOG_CTRL::overrideDefaults(set(Ticks::WATCHDOG_CTRL::enable)));
        for(std::uint32_t i = 0; i != 1'000U; ++i) {
            if(get<0>(apply(read(Ticks::WATCHDOG_CTRL::running))) != 0U) { return; }
        }
    }
}}   // namespace Kvasir::Rp
#endif
