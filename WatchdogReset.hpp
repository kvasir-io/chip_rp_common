#pragma once
// The one place an RP image resets the chip through WATCHDOG CTRL.TRIGGER. Both reset paths end here:
// Kvasir::Watchdog<>::trigger() (watchdog.hpp; PSM.WDSEL as the boot set it, WatchdogDomains::onTimeout, REASON
// force) and Rp::watchdogReboot() (BootromReboot.hpp; WatchdogDomains::onReboot first). What a chip needs right
// before the trigger is its WatchdogDomains::beforeReset() (chip/WatchdogDomains.hpp): on the RP2350 the watchdog
// tick (SRAM retention, WatchdogTick.hpp) and POWMAN forced onto the LPOSC, waited for; nothing on the RP2040.
#include "chip/WatchdogDomains.hpp"
#include "peripherals/WATCHDOG.hpp"

#include <kvasir/Register/Register.hpp>

namespace Kvasir::Rp::Detail {
[[noreturn,
  gnu::always_inline]] inline void
watchdogResetNow() {
    using WD = Kvasir::Peripheral::WATCHDOG::Registers<>;
    Rp::WatchdogDomains::beforeReset();
    apply(set(WD::CTRL::trigger));
    while(true) { asm volatile("wfi"); }
}
}   // namespace Kvasir::Rp::Detail
