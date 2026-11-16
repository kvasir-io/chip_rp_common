#pragma once
// The boot ROM's chain_image (RP2350 datasheet 5.4.8.2; pico-bootrom-rp2350 varm_apis.c s_varm_api_chain_image), from a
// running image as its last act: the ROM searches a window of flash (or RAM) for a block loop, takes the IMAGE_DEF it
// would launch, COPIES its load map over RAM, then hashes what it copied and checks the signature a secure chip needs
// (varm_blocks.c s_varm_crit_ram_trash_verify_block: the copy comes first, it is in the hash), and enters it with the
// boot type's CHAINED flag set (varm_launch_image.c) - or returns an error into whatever the copy left of its caller.
// A flash window is rolled as a partition is for a flash boot (ATRANS, reset again should the launch fail), so a
// packaged RAM image (picotool seal, relative load map) runs from any partition.
//
// chainImageOrReboot():
// - stops what could undo or disturb it, for good: interrupts masked and the other core stopped
//   (Rp::stopForRomReboot), the DMA and the PIO blocks reset (a running channel would write into what the ROM copies),
//   the MPU off (the ROM programs regions of its own, and an ARMv8-M address in two enabled regions faults; an image's
//   read-only code is what the copy overwrites); the SHA-256 block out of reset (Kvasir holds it there; the ROM hashes on
//   it) and the SHA-256, flash-op and OTP boot locks claimed (where a stage before the image turned lock checking on,
//   the ROM refuses otherwise). The watchdog keeps running: should nothing below come back, it reboots the chip;
// - calls the ROM from a RAM function (ChainDetail::chain) on the caller's stack, with the work area the caller names
//   (partitionWorkareaBytes, word aligned) - neither may lie where the image's load map writes, nor the RAM function.
//   That is the caller's to see to (it knows where the image goes: PicobinReader.hpp Walker::loadTargets()); USB DPRAM,
//   once the device is off the bus, is a work area no image's load map writes (the ROM's entry into the image clears it
//   on a secure chip anyway: arm8_bootrom_rt0.S s_native_crit_init_default_xip_setup_and_enter_image_thunk);
// - should the ROM return (an image it refused after the copy), reboots the chip normally through the ROM's reboot,
//   fetched before: the caller is not run on again. Nothing else of the caller runs after the copy.
#include "BootInfo.hpp"   // Bootrom::BootLockGuard, partitionWorkareaBytes, Detail::sha256OutOfReset
#include "BootromReboot.hpp"   // Rp::stopForRomReboot, RebootFlags
#include "Rom.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/attributes.hpp"

#include <cstddef>
#include <cstdint>
#include <kvasir/Runtime/AnnouncedReset.hpp>
#ifdef KVASIR_CHIP_RP2350
    #include "core/Mpu.hpp"
    #include "peripherals/RESETS.hpp"
#endif

namespace Kvasir { namespace Bootrom {
#ifdef KVASIR_CHIP_RP2350
    /// chain_image (ROM code 'C', 'I'): 0 never comes back - the image runs -, else a BOOTROM_ERROR_*.
    using ChainImageFunction = int (*)(std::uint8_t* workarea,
                                       std::uint32_t workareaBytes,
                                       std::uint32_t windowBase,
                                       std::uint32_t windowBytes);
    /// reboot (ROM code 'R', 'B'), as RomFunctions::reboot calls it.
    using RebootFunction = int (*)(std::uint32_t flags,
                                   std::uint32_t delayMs,
                                   std::uint32_t p0,
                                   std::uint32_t p1);

    namespace ChainDetail {
        /// The ROM call, and the reboot should it come back: from RAM, as what the chain copies may have replaced
        /// the caller's code, with nothing but the ROM's two functions and the arguments in registers and on the
        /// stack. Never returns; the loop at the end only waits for the watchdog should even the reboot fail.
        [[KVASIR_RAM_FUNC_ATTRIBUTES,
          noreturn]] inline void
        chain(ChainImageFunction chainImage,
              RebootFunction     reboot,
              std::uintptr_t     workarea,
              std::uint32_t      windowBase,
              std::uint32_t      windowBytes) {
            KVASIR_RAM_FUNC_MARK();
            static_cast<void>(chainImage(reinterpret_cast<std::uint8_t*>(workarea),
                                         static_cast<std::uint32_t>(partitionWorkareaBytes),
                                         windowBase,
                                         windowBytes));
            static_cast<void>(
              reboot(RebootFlags::rebootTypeNormal | RebootFlags::noReturnOnSuccess, 1, 0, 0));
            while(true) {
                asm volatile("" ::: "memory");   // the watchdog's
            }
        }
    }   // namespace ChainDetail

    /// Starts the image the boot ROM finds in [@p windowBase, @p windowBase + @p windowBytes) - an XIP address and
    /// size for a flash window (a partition: Flash::xipBase + its offset, its size) - through chain_image (see the
    /// header), with @p workarea (the address of partitionWorkareaBytes, word aligned) as the ROM's scan work area;
    /// reboots normally should the ROM refuse it. The caller's last act: it announces the reset like every reboot
    /// (AnnouncedReset), and nothing of the caller runs again.
    [[noreturn]] inline void chainImageOrReboot(std::uint32_t  windowBase,
                                                std::uint32_t  windowBytes,
                                                std::uintptr_t workarea) {
        (void)AnnouncedReset::announce();
        Rp::stopForRomReboot();   // for good, as for every boot ROM reboot (BootromReboot.hpp)
        using Reset = Kvasir::Peripheral::RESETS::Registers<>::RESET;
        apply(set(Reset::dma), set(Reset::pio0), set(Reset::pio1), set(Reset::pio2));
        Kvasir::Core::Mpu::disable();
        Kvasir::Detail::sha256OutOfReset();
        // Not released: the chain does not come back, and a reboot starts the boot ROM's own locks afresh.
        BootLockGuard<BootLock::sha256> const  shaLock{};
        BootLockGuard<BootLock::flashOp> const flashLock{};
        BootLockGuard<BootLock::otp> const     otpLock{};
        auto const chainImage = RomFunctions::getRomFunctionPointer<'C', 'I', ChainImageFunction>();
        auto const reboot     = RomFunctions::getRomFunctionPointer<'R', 'B', RebootFunction>();
        ChainDetail::chain(chainImage, reboot, workarea, windowBase, windowBytes);
    }
#endif
}}   // namespace Kvasir::Bootrom
