#pragma once
// Reboots of the RP2040 and RP2350: the watchdog's own reboot and the boot ROM's reboot() / reset to USB boot.
#include "Rom.hpp"
#include "StopOtherCore.hpp"   // Port::StopOtherCore: every boot ROM reboot stops the other core
#include "WaitBounds.hpp"   // Register::CpuHzCeiling<Tag::User>: AnnouncedReset::announce()'s polls
#include "WatchdogBoot.hpp"
#include "WatchdogReset.hpp"   // the one trigger; the chip's PSM.WDSEL lists (chip/WatchdogDomains.hpp)
#include "core/InterruptMask.hpp"
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

    // Every boot ROM reboot's step between its announcement and the ROM call (Bootrom below: why): this core's
    // interrupts masked for good, then the other core stopped by the chip's Port::StopOtherCore (StopOtherCore.hpp), the
    // fatal path's step - on core 0 core 1 is reset into the boot ROM, on core 1 core 0 is parked in its NMI. An image
    // without code on core 1 (no KVASIR_MULTICORE) has core 1 asleep in the boot ROM: the mask alone.
    inline void stopForRomReboot() {
        Core::maskIrqs();
#ifdef KVASIR_MULTICORE
        Kvasir::Port::StopOtherCore<Kvasir::Tag::User>{}();
#endif
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
        // p0: the XIP address of the first byte of the flash just written (5.4.8.24; the boot ROM takes p0 - XIP_BASE
        // as the flash offset, pico-bootrom varm_boot_path.c)
        constexpr std::uint32_t rebootTypeFlashUpdate = 0x0004;
        constexpr std::uint32_t noReturnOnSuccess     = 0x0100;
    }   // namespace RebootFlags

    // Every reboot announces itself first (Kvasir_SDK Runtime/AnnouncedReset.hpp): a J-Link that
    // touches the chip while it resets makes SEGGER's DLL "recover" the RP2040 through the Rescue
    // DP, and the boot ROM then parks the core instead of booting. With a log printer attached
    // this waits until the printer is off the chip (or ~0.5 s); without one it returns at once.
    //
    // Then nothing else may run (Rp::stopForRomReboot): this core's interrupts are masked (PRIMASK) for good, and the
    // other core is stopped. The RP2350 boot ROM's reboot() writes the next boot's type and p0/p1 into watchdog
    // SCRATCH2..7, loads the watchdog with the delay and, with NO_RETURN_ON_SUCCESS, waits in a WFI loop with interrupts
    // as the caller left them (pico-bootrom varm_apis.c, s_varm_hx_reboot). A handler on either core that runs in that
    // window can undo the reboot: a watchdog feed (a health supervisor on a timer alarm) reloads the counter and the
    // reset waits for that timeout. With the supervisor on the calling core, that core's main loop, parked in the ROM,
    // starves meanwhile - the supervisor's panic then reboots on its own, a NORMAL reboot that rewrites SCRATCH2..7.
    // After a FLASH_UPDATE reboot that is a regular boot, and the boot ROM does not launch an unbought TBYB image on one
    // (varm_launch_image.c: "NOT booting TBYB flagged image which isn't the flash update"): the board comes back on the
    // other partition of its A/B pair. With the supervisor on the other core, whose loop still beats, nothing starves
    // and the reboot never comes. With PRIMASK set no handler runs on this core; WFI still returns on a pending
    // interrupt (DDI0553B.y B2.1.2 md l.3270: a WFI wakeup event ignores PRIMASK) and the ROM's loop waits on, so the
    // watchdog fires on time. The RP2040's paths take the same step: reboot() is the watchdog's immediate trigger there,
    // and resetToUsbBoot() the RP2040 boot ROM's own reset.
    [[noreturn]] inline void resetToUsbBoot() {
        (void)AnnouncedReset::announce();
        Rp::stopForRomReboot();   // for good: nothing on either core may undo the reboot (above)
        if constexpr(Chip::part == Kvasir::Chip::Part::rp2040) {
            using RomResetToUsbBoot
              = void (*)(std::uint32_t gpioActivityPinMask, std::uint32_t disableInterfaceMask);

            RomFunctions::call<'U', 'B', RomResetToUsbBoot>(0U, 0U);
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

    // RP2350: reboot as a flash update boot (datasheet 5.1.16, 5.4.8.24) for the flash written from `windowBase` on,
    // the XIP address of a partition's first byte (Flash::xipBase + its offset): the boot ROM then takes that
    // partition's IMAGE_DEF whatever its version, and it is the only boot that launches a TBYB image not bought yet
    // over the other partition of its A/B pair (5.1.17) - under the boot ROM's 16.7 s watchdog, which the image
    // ends with Bootrom::explicitBuy() (BootInfo.hpp). The boot ROM erases the other partition's first sector when it
    // launches an image without TBYB that way (5.1.16 step 2). Announced like reboot() below; the watchdog's chip reset
    // if the ROM call returns. The boot type and p0 travel in watchdog SCRATCH2..7: nothing kept there survives it.
#ifdef KVASIR_CHIP_RP2350
    [[noreturn]] inline void rebootFlashUpdate(std::uint32_t windowBase) {
        (void)AnnouncedReset::announce();
        Rp::stopForRomReboot();   // for good: nothing on either core may undo the reboot (above)
        [[maybe_unused]] auto const ret = RomFunctions::reboot(RebootFlags::rebootTypeFlashUpdate
                                                                 | RebootFlags::noReturnOnSuccess,
                                                               1,
                                                               windowBase,
                                                               0);
        UC_LOG_E("bootrom flash update reboot returned {}", ret);
        Rp::watchdogReboot();
    }

    // The same, with the boot ROM's diagnostics pointed at `diagnosticPartition` for the boot it starts
    // (DiagnosticPartition in BootInfo.hpp, or a partition number: of an A/B pair its A, by which the boot ROM keys the
    // pair's diagnostics). The boot after it - the new image, or the old one if the boot ROM refused the new - reads in
    // BootInfo::diagnostic how far the scan got with each half: block loop, IMAGE_DEF, hash or signature, chosen,
    // launched (5.4.8.17). Only a regular reboot sets the diagnostic partition, from its p0 and during the call
    // (pico-bootrom varm_apis.c: bootram always.diagnostic_partition_index, kept by every reset but a cold one). So a
    // regular reboot() that returns - no NO_RETURN_ON_SUCCESS; its 1 s watchdog never runs out - and at once the flash
    // update reboot, which writes SCRATCH2..7 and the watchdog anew. One reset, as without.
    //
    // A chain_image() resets the diagnostics (to DiagnosticPartition::image): after an image whose first stage chains
    // into another (a decryption stage), the record describes what that stage chained into, not the pair.
    [[noreturn]] inline void rebootFlashUpdate(std::uint32_t windowBase,
                                               std::int8_t   diagnosticPartition) {
        (void)AnnouncedReset::announce();
        Rp::stopForRomReboot();   // for good: nothing on either core may undo the reboot (above)
        [[maybe_unused]] auto const set = RomFunctions::reboot(
          RebootFlags::rebootTypeNormal,
          1000,
          static_cast<std::uint32_t>(static_cast<std::int32_t>(diagnosticPartition)),
          0);
        [[maybe_unused]] auto const ret = RomFunctions::reboot(RebootFlags::rebootTypeFlashUpdate
                                                                 | RebootFlags::noReturnOnSuccess,
                                                               1,
                                                               windowBase,
                                                               0);
        UC_LOG_E("bootrom flash update reboot returned {} (setting the diagnostic partition: {})",
                 ret,
                 set);
        Rp::watchdogReboot();
    }
#endif

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
    // core in the ROM, interrupts masked, until the watchdog fires, 1 ms later. It writes p0/p1 (0, 0) into watchdog
    // SCRATCH2/3 (5.4.8.24, md l.20618): keep nothing there across it.
    // The raw watchdog is the RP2040's path and the fallback should the ROM call refuse.
    [[noreturn]] inline void reboot() {
        (void)AnnouncedReset::announce();
        Rp::stopForRomReboot();   // for good: nothing on either core may undo the reboot (above)
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
