#pragma once
// A reset while the flash erases or programs, and a boot ROM that reads the flash before it is done.
//
// The boot ROM tries the flash once per boot, right after the reset, and never asks the flash whether it is ready: it
// sends its XIP exit sequence and reads (pico-bootrom varm_generic_flash.c s_varm_api_crit_flash_exit_xip,
// varm_boot_path.c: flash boot is tried once, then nsboot, which has no timeout). A sector erase goes on inside the
// flash after the chip that started it has been reset (about 30 ms measured for a written sector on a W25Q-class
// flash, 400 ms the usual datasheet maximum), and a busy flash answers no read: the boot ROM finds no partition table
// and no image and stays in BOOTSEL, with an intact flash that any later reset boots again. Seen both after a watchdog
// trigger in the middle of a sector erase and after a RUN-pin reset in the middle of explicit_buy's.
//
// For the resets that keep the WATCHDOG scratch registers - the watchdog's (a bite, CTRL.TRIGGER) and a reset of the
// cores alone (SYSRESETREQ), not a RUN-pin, power-on or brown-out reset - the boot ROM calls the watchdog boot vector
// (WatchdogBoot.hpp) before it touches the flash (varm_boot_path.c step 8; the QSPI pads come out of reset at step 11),
// and goes on with the flash boot when the vector returns. Guard arms it for the length of one operation, inside the
// RAM function that runs it, once XIP is off. Detail::entry():
//   1. does CoreResetGuard's work in an image that arms that guard (DMA aborted, core 1 back in the ROM's pen);
//   2. connects the flash as the ROM's own connect_internal_flash does - IO_QSPI and PADS_QSPI out of reset, the pins'
//      FUNCSEL to XIP, the pads' isolation off: at the vector none of that is done yet, and an unconnected status read
//      comes back 0, "not busy" - and takes SYSCFG out of reset for step 5;
//   3. puts the QMI's direct mode into a clean state (below);
//   4. reads the flash's status register (05h) until its busy bit reads clear twice in a row - at most maxPolls reads;
//      2000-6600 after a reset 3-20 ms into a sector erase, at the boot ROM's clocks;
//   5. reboots the chip through the watchdog, the vector disarmed and POWMAN on the LPOSC first, so the next boot is a
//      plain flash boot of an idle flash.
// Why 5 and not a return into the boot ROM: after a reset of the cores alone (a debugger's SYSRESETREQ) the QMI/XIP stay
// as the interrupted operation set them - only a chip-level reset puts them back - and the ROM's flash boot found
// nothing; its nsboot, disabled once a vector was taken, then rebooted into BOOTSEL with boot type BOOTSEL, which reads
// as asked for. Its own reset would leave REASON = FORCE for the reset that started the routine (a watchdog timeout: a
// hang); the routine carries REASON as it found it across (PM::carryReasonAcrossReset, chip_reset.hpp), so the next
// boot's PM::resetCause() is that reset's. Detail::record keeps what the routine saw: for a debugger, for a board that
// ends up in BOOTSEL anyway, and for the next boot's report (atBoot(); the chip's boot report line, clock_config.hpp).
//
// A RUN-pin, power-on or brown-out reset clears the scratch registers, so the vector is not taken: such a reset in the
// middle of an erase still ends in BOOTSEL, and only another reset - from whatever watches the board - gets it out.
//
// The vector the guard replaces is put back when the operation ends (CoreResetGuard's, in a multicore flash image), and
// entry() does that guard's work first, so a reset of core 0 alone during the operation is covered as before. The boot
// after entry()'s reboot is a normal one: a FLASH_UPDATE trial interrupted so boots the other partition, as any reset
// before the buy does. RP2040: Guard is empty (its flash is reached through the SSI, not the QMI).
#include "BootSequence.hpp"   // KVASIR_RP_CORE_RESET_GUARDED
#include "WatchdogBoot.hpp"
#include "WatchdogReset.hpp"
#include "XipControl.hpp"
#include "chip_reset.hpp"
#include "kvasir/Port/LinkerSymbols.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/attributes.hpp"

#include <cstdint>
#include <optional>
#ifdef KVASIR_CHIP_RP2350
    #include "peripherals/IO_QSPI.hpp"
    #include "peripherals/PADS_QSPI.hpp"
    #include "peripherals/QMI.hpp"
    #include "peripherals/RESETS.hpp"
    #include "peripherals/WATCHDOG.hpp"
    #if KVASIR_RP_CORE_RESET_GUARDED
        #include "chip/CoreResetGuard.hpp"
    #endif
#endif

namespace Kvasir::Flash::BusyBoot {
/// What entry() found after the reset that stopped a flash operation (atBoot()).
struct Report {
    std::uint32_t polls;           ///< status reads it made
    std::uint8_t  status;          ///< the last status byte read
    bool          ready;           ///< the flash read ready (two reads in a row) before maxPolls
    bool          powmanOnLposc;   ///< POWMAN read on the LPOSC before entry()'s reset
};

#ifdef KVASIR_CHIP_RP2350
/// The status reads entry() makes at most before it reboots anyway. One read is 16 SCK cycles at clk_sys / 6 plus the
/// FIFO work: a few us on the ROSC clk_sys after a watchdog reset, so this is a few hundred ms to a few s - past any
/// sector erase or page program, short of a board that waits forever on a flash that never reads ready (none fitted,
/// a wedged bus).
inline constexpr std::uint32_t maxPolls
  = 600'000U;   // ~3 s at ~5 us a read: past a 64 KiB block erase's 2 s too

namespace Detail {
    /// What the last run of entry() did, in .noInit: the next boot of the image reads it (atBoot()), and a board that
    /// ends up in BOOTSEL anyway keeps it for a debugger.
    struct Record {
        std::uint32_t magic;    // recordMagic once entry() ran
        std::uint32_t resets;   // RESETS.RESET as entry() found it
        std::uint32_t polls;    // status reads made
        std::uint32_t status;   // the last status byte read (0x100: none yet)
        std::uint32_t ready;    // 1: two reads in a row said ready
        std::uint32_t lposc;    // 1: POWMAN read on the LPOSC before the reset
    };

    inline constexpr std::uint32_t  recordMagic = 0x4255'5359U;   // "BUSY"
    [[KVASIR_NOINIT]] inline Record record;

    // The vector's routine: called by the boot ROM as a function, on the image's initial stack (SCRATCH6), with the
    // QMI and the QSPI pads as the reset left them; RAM code that calls only RAM code (check_ram_funcs.py). Only Guard
    // refers to it: an image without a flash operation has none.
    [[noreturn,
      KVASIR_RAM_FUNC_ATTRIBUTES]] inline void
    entry() {
        KVASIR_RAM_FUNC_MARK();
        using Pads   = Kvasir::Peripheral::PADS_QSPI::Registers<>;
        using Io     = Kvasir::Peripheral::IO_QSPI::Registers<>;
        using Resets = Kvasir::Peripheral::RESETS::Registers<>;
        using QMI    = Kvasir::Peripheral::QMI::Registers<0>;
        using Csr    = QMI::DIRECT_CSR;
        using WD     = Kvasir::Peripheral::WATCHDOG::Registers<>;
        using namespace Kvasir::Register;
        auto volatile& rec = record;
        rec.magic          = recordMagic;
        rec.resets
          = get<0>(apply(read(Resets::RESET::FULLREGISTER)));   // a snapshot for a debugger
        rec.polls  = 0;
        rec.status = 0x100U;
        rec.ready  = 0;
        rec.lposc  = 0;
    #if KVASIR_RP_CORE_RESET_GUARDED
        Kvasir::CoreResetGuard::entry();   // DMA aborted, core 1 back in the ROM's pen
    #endif
        // 2. At the vector the QSPI pins are not the QMI's yet: the reset left IO_QSPI and PADS_QSPI held in reset, and
        // the ROM connects the flash only later (s_varm_api_crit_connect_internal_flash, from its step 11 on).
        // Unconnected, every status read comes back 0, "not busy" (seen: RESETS.RESET 0x1dddffff at the vector, one
        // poll, status 0, the flash still erasing). Both blocks out of reset, each pin's FUNCSEL to XIP (CTRL 0), the
        // pads' isolation off; the ROM resets the blocks again before it uses them itself.
        // SYSCFG too (RESETS bit 20, RP2350 datasheet Table 535): the reset holds it as well, and neither the ROM
        // before the vector (pico-bootrom varm_boot_path.c releases TRNG, SHA256 and SYSINFO only) nor anything of
        // the image's own boot (chip/BootTraits.hpp holdBlocksInReset, later) has released it yet. Step 5 writes its
        // AUXCTRL bit 0 (WatchdogDomains::beforeReset): POWMAN onto the LPOSC before a watchdog reset whose stages
        // include CLOCKS, or "the short pulse generated on clk_ref by the reset of the CLOCKS block may affect POWMAN
        // register state" (Table 1317) - the glitch the ROM's own reboot avoids the same way. What a write to a block
        // held in reset does, the datasheet does not say; measured: without this release the bit has no effect, and
        // POWMAN never reads on the LPOSC (rec.lposc 0). After a reset of the cores alone SYSCFG is out of reset
        // already; the reset of step 5 holds it again.
        apply(clear(Resets::RESET::io_qspi),
              clear(Resets::RESET::pads_qspi),
              clear(Resets::RESET::syscfg));
        for(std::uint32_t i = 0; i < 100'000U; ++i) {
            auto const done = apply(read(Resets::RESET_DONE::io_qspi),
                                    read(Resets::RESET_DONE::pads_qspi),
                                    read(Resets::RESET_DONE::syscfg));
            if(get<0>(done) != 0U && get<1>(done) != 0U && get<2>(done) != 0U) { break; }
        }
        apply(write(Io::GPIO_QSPI_SCLK_CTRL::FULLREGISTER, value<std::uint32_t, 0U>()),
              write(Io::GPIO_QSPI_SS_CTRL::FULLREGISTER, value<std::uint32_t, 0U>()),
              write(Io::GPIO_QSPI_SD0_CTRL::FULLREGISTER, value<std::uint32_t, 0U>()),
              write(Io::GPIO_QSPI_SD1_CTRL::FULLREGISTER, value<std::uint32_t, 0U>()),
              write(Io::GPIO_QSPI_SD2_CTRL::FULLREGISTER, value<std::uint32_t, 0U>()),
              write(Io::GPIO_QSPI_SD3_CTRL::FULLREGISTER, value<std::uint32_t, 0U>()));
        apply(clear(Pads::GPIO_QSPI_SCLK::iso),
              clear(Pads::GPIO_QSPI_SD0::iso),
              clear(Pads::GPIO_QSPI_SD1::iso),
              clear(Pads::GPIO_QSPI_SD2::iso),
              clear(Pads::GPIO_QSPI_SD3::iso),
              clear(Pads::GPIO_QSPI_SS::iso));
        // 3. A reset of the cores alone keeps the QMI as the interrupted operation left it: possibly in direct mode
        // with CS held low and the reply of the ROM's own status read still in the RX FIFO, which the first poll then
        // took for its answer - "not busy" at once, a reboot into a flash still erasing, BOOTSEL (seen after a
        // SYSRESETREQ 16-21 ms into an erase). So: CS up, the shifter finished, the RX FIFO drained, direct mode off -
        // and the flash counts as ready only after two reads in a row say so.
        apply(clear(Csr::assert_cs0n), clear(Csr::assert_cs1n));
        for(std::uint32_t i = 0; i < 10'000U; ++i) {
            if(std::uint32_t{apply(read(Csr::busy))} == 0U) { break; }
        }
        for(std::uint32_t i = 0; i < 64U; ++i) {
            if(std::uint32_t{apply(read(Csr::rxempty))} != 0U) { break; }
            (void)readWhole(QMI::DIRECT_RX::FULLREGISTER);   // the FIFO's data word
        }
        Kvasir::Detail::directModeOff();
        // 4. 05h and one byte clocked back: the status register, bit 0 busy (WIP)
        std::uint32_t readyInARow = 0;
        for(std::uint32_t i = 0; i < maxPolls && readyInARow < 2U; ++i) {
            auto const rx
              = Kvasir::Detail::directCommand<2>(0x05U, Kvasir::Detail::directCommandClkdiv);
            rec.polls   = i + 1U;
            rec.status  = (rx >> 8U) & 0xFFU;
            readyInARow = ((rx >> 8U) & 1U) == 0U ? readyInARow + 1U : 0U;
        }
        rec.ready = readyInARow >= 2U ? 1U : 0U;
        // 5. A chip reset, not a return into the ROM (the header). As Rp::watchdogReboot(), in RAM: the debug pause bits
        // off (this reset may well come from a debugger, and a paused watchdog never fires), the vector disarmed, REASON
        // carried, then the trigger - with PROC_COLD among the stages (WatchdogDomains::onTimeout, not onReboot): it
        // holds the debug state, and a reset of the cores alone is a debugger's, whose DHCSR.C_DEBUGEN would otherwise
        // outlive its session and keep the next boot's MPU off (Mpu.hpp). A flash that never read ready gets the same
        // reboot: the next boot finds it busy and stays in BOOTSEL as an unasked one, where a return would have left a
        // BOOTSEL boot that reads as asked for.
        apply(WD::CTRL::overrideDefaults(write(WD::CTRL::pause_dbg1, value<0>()),
                                         write(WD::CTRL::pause_dbg0, value<0>()),
                                         write(WD::CTRL::pause_jtag, value<0>())));
        Kvasir::Rp::WatchdogBoot::disarm();
        Kvasir::PM::carryReasonAcrossReset();
        Kvasir::Rp::WatchdogDomains::onTimeout();
        // POWMAN onto the LPOSC here already, for the record (step 2); beforeReset() in watchdogResetNow() sets the
        // bit again and finds POWMAN there at its first read
        rec.lposc = Kvasir::Rp::WatchdogDomains::forcePowmanOnLposc() ? 1U : 0U;
        Kvasir::Rp::Detail::watchdogResetNow();
    }
}   // namespace Detail

/// For one flash operation: the watchdog boot vector at Detail::entry(), and what it replaced back afterwards.
/// Constructed after the XipGuard (XIP off, the flash in serial command mode, where 05h reads the status) and so
/// destroyed before it. Only register stores: it lives inside the RAM functions.
struct Guard {
    using WD = Kvasir::Peripheral::WATCHDOG::Registers<>;

    // the vector before (CoreResetGuard's, or none), read in this order
    std::uint32_t scratch4;
    std::uint32_t scratch5;
    std::uint32_t scratch6;
    std::uint32_t scratch7;

    [[gnu::always_inline]] Guard()
      : scratch4{get<0>(apply(read(WD::SCRATCH4::FULLREGISTER)))}
      , scratch5{get<0>(apply(read(WD::SCRATCH5::FULLREGISTER)))}
      , scratch6{get<0>(apply(read(WD::SCRATCH6::FULLREGISTER)))}
      , scratch7{get<0>(apply(read(WD::SCRATCH7::FULLREGISTER)))} {
        Rp::WatchdogBoot::arm(
          static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&Detail::entry)) | 1U,
          static_cast<std::uint32_t>(Kvasir::Startup::PrimaryStackTop::value()));
    }

    [[gnu::always_inline]] ~Guard() {
        if(scratch4 == Rp::WatchdogBoot::magic) {
            // the vector that was armed before, SCRATCH4 last (as WatchdogBoot::arm)
            apply(write(WD::SCRATCH7::FULLREGISTER, scratch7));
            apply(write(WD::SCRATCH6::FULLREGISTER, scratch6));
            apply(write(WD::SCRATCH5::FULLREGISTER, scratch5));
            apply(write(WD::SCRATCH4::FULLREGISTER, scratch4));
        } else {
            Rp::WatchdogBoot::disarm();
        }
    }

    Guard(Guard const&)            = delete;
    Guard& operator=(Guard const&) = delete;
};

/// Whether this boot follows entry()'s reboot - a reset stopped a flash operation and found the flash busy - and what
/// entry() saw. The same answer for every call of a boot (PM::takeCarriedReason, the boot's first step).
[[nodiscard]] inline std::optional<Report> atBoot() {
    Detail::Record const r = Detail::record;
    if(!PM::followsCarriedReset() || r.magic != Detail::recordMagic) { return std::nullopt; }
    return Report{.polls         = r.polls,
                  .status        = static_cast<std::uint8_t>(r.status),
                  .ready         = r.ready != 0U,
                  .powmanOnLposc = r.lposc != 0U};
}
#else
/// The RP2040: nothing armed (its users mark the object [[maybe_unused]]).
struct Guard {};

[[nodiscard]] inline std::optional<Report> atBoot() { return std::nullopt; }
#endif
}   // namespace Kvasir::Flash::BusyBoot
