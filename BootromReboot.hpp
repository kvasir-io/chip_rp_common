#pragma once
// Reboots of the RP2040 and RP2350: the watchdog's own reboot and the boot ROM's reboot() / reset to USB boot.
#include "Rom.hpp"
#include "WaitBounds.hpp"   // Register::CpuHzCeiling<Tag::User>: AnnouncedReset::announce()'s polls
#include "WatchdogBoot.hpp"
#include "WatchdogReset.hpp"   // the one trigger; the chip's PSM.WDSEL lists (chip/WatchdogDomains.hpp)
#include "peripherals/WATCHDOG.hpp"

#include <cstdint>
#include <kvasir/Register/Register.hpp>
#include <kvasir/Runtime/AnnouncedReset.hpp>
#include <uc_log/uc_log.hpp>

namespace Kvasir {
namespace Rp {
    // The watchdog's own reboot: what the pico-sdk's watchdog_reboot(0, 0, ..) and the RP2350
    // bootrom's reboot() do underneath, without the timer. CTRL is written as a whole so the
    // PAUSE_DBG0/1/JTAG bits go too: with them set a watchdog does nothing while a probe is
    // attached, and "does not reboot under the debugger" is not a reboot. The watchdog boot
    // vector is disarmed so the bootrom takes no stale one (WatchdogBoot.hpp). PSM.WDSEL names
    // the stages the reset runs through, the chip's reboot list (chip/WatchdogDomains.hpp
    // onReboot): narrower than what a watchdog timeout resets (onTimeout, set by FirstInitStep). The trigger is
    // the one reset owner's (WatchdogReset.hpp), as for Watchdog<>::trigger().
    [[noreturn]] inline void watchdogReboot() {
        using WD = Kvasir::Peripheral::WATCHDOG::Registers<>;

        apply(WD::CTRL::overrideDefaults(write(WD::CTRL::pause_dbg1, Register::value<0>()),
                                         write(WD::CTRL::pause_dbg0, Register::value<0>()),
                                         write(WD::CTRL::pause_jtag, Register::value<0>())));
        Rp::WatchdogBoot::disarm();
        Rp::WatchdogDomains::onReboot();
        Rp::Detail::watchdogResetNow();
    }
}   // namespace Rp

// The bootrom's reboot paths, in a namespace of their own for their log module ("bootrom", derived
// from the scope).
namespace Bootrom {
    // reboot() flags (RP2350 datasheet 5.4.8.24): the reboot type in the low byte, NO_RETURN_ON_SUCCESS parks the
    // calling core in the ROM until the watchdog fires.
    namespace RebootFlags {
        constexpr std::uint32_t rebootTypeNormal  = 0x0000;
        constexpr std::uint32_t rebootTypeBootsel = 0x0002;
        constexpr std::uint32_t noReturnOnSuccess = 0x0100;
    }   // namespace RebootFlags

    // Both reboots announce themselves first (Kvasir_SDK Runtime/AnnouncedReset.hpp): a J-Link that
    // touches the chip while it resets makes SEGGER's DLL "recover" the RP2040 through the Rescue
    // DP, and the boot ROM then parks the core instead of booting. With a log printer attached
    // this waits until the printer is off the chip (or ~0.5 s); without one it returns at once.
    [[noreturn]] inline void resetToUsbBoot() {
        (void)AnnouncedReset::announce();
        if constexpr(Chip::part == Kvasir::Chip::Part::rp2040) {
            using RomResetToUsbBoot
              = void (*)(std::uint32_t gpioActivityPinMask, std::uint32_t disableInterfaceMask);

            RomFunctions::call<'U', 'B', RomResetToUsbBoot>(0, 0);
        } else {
            [[maybe_unused]] auto const ret = RomFunctions::reboot(
              RebootFlags::rebootTypeBootsel | RebootFlags::noReturnOnSuccess,
              1,
              0,
              0);
            UC_LOG_E("reboot ret {}", ret);
        }

        // the watchdog reboot below still resets
        UC_LOG_E("This should not happen reboot returned");
        Rp::watchdogReboot();   // a chip reset at least, see reboot() below
    }

    // Reboot the chip: both cores and every peripheral restart from the bootrom, as after a
    // power-on. PM::resetCause() reports watchdog_timer on the RP2350 (the bootrom arms the
    // watchdog's timer, 1 ms) and watchdog_force on the raw-trigger path.
    //
    // Not SystemControl::SystemReset: SYSRESETREQ is a warm reset of the core that asserts it
    // and of nothing else (RP2040 datasheet 2.4.2.9, RP2350 datasheet 12.9, pico-feedback #329).
    // Issued from core 1 it parks core 1 in the bootrom and leaves core 0 running; from core 0
    // it leaves core 1 running and the peripherals configured, and only looks like a reboot
    // because FirstInitStep puts the peripherals back into reset.
    //
    // The RP2350 goes through the bootrom's reboot() (datasheet 5.4.8.24), the pico-sdk's and
    // picotool's path: it switches POWMAN off clk_ref before the clock generators reset (a
    // clk_pow glitch otherwise) and keeps the boot diagnostics. NO_RETURN_ON_SUCCESS parks this
    // core in the ROM until the watchdog fires, 1 ms later. It writes p0/p1 (0, 0) into watchdog
    // SCRATCH2/3 (5.4.8.24, md l.20618): keep nothing there across it.
    // The raw watchdog is the RP2040's path and the fallback should the ROM call refuse.
    [[noreturn]] inline void reboot() {
        (void)AnnouncedReset::announce();
        if constexpr(Chip::part != Kvasir::Chip::Part::rp2040) {
            [[maybe_unused]] auto const ret
              = RomFunctions::reboot(RebootFlags::rebootTypeNormal | RebootFlags::noReturnOnSuccess,
                                     1,
                                     0,
                                     0);
            UC_LOG_E("bootrom reboot returned {}", ret);   // the watchdog reboot below still resets
        }
        Rp::watchdogReboot();
    }
}   // namespace Bootrom
}   // namespace Kvasir
