#pragma once
// Kvasir::Chip::reboot() on the RP2040 and RP2350: Bootrom::reboot() (BootromReboot.hpp) - both cores and every
// peripheral restart from the boot ROM, announced to the log printer first; the next boot reads watchdog_timer
// (RP2350) or watchdog_force (RP2040) as its PM::resetCause(). Not SYSRESETREQ, which resets one core only (the
// comment at Bootrom::reboot()). A BootGuard's plannedReset() stays the caller's: to the guard a reboot is a watchdog
// reset.
#include "BootromReboot.hpp"
#include "FreeRunningUs.hpp"   // the fatal-path port hooks: the panic record's time
#include "StopOtherCore.hpp"   // and stopping the other core
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Port/Hooks.hpp"

namespace Kvasir::Chip {
[[noreturn]] inline void reboot() { Bootrom::reboot(); }
}   // namespace Kvasir::Chip

/// reboot() as the port's hook (kvasir/Port/Hooks.hpp), for what the SDK ends in: Port::LateT<Port::Reboot<Tag::User>,
/// Dep>{}().
template<typename... Ts>
struct Kvasir::Port::Reboot<Kvasir::Tag::User, Ts...> {
    [[noreturn]] void operator()() const { Kvasir::Chip::reboot(); }
};
