#pragma once
// The RP reset-cause decision without the registers (chip_reset.hpp reads them): standard library only, so the host
// tests (tests/reset_cause_test.cpp) check it for both chips.
//
// What the registers can and cannot tell:
//   - CHIP_RESET (RP2350: POWMAN, datasheet 6.4 Table 489; RP2040: VREG_AND_CHIP_RESET, Table 191) records the LAST
//     CHIP-LEVEL reset. Its HAD_* bits are read-only: no write clears them (with or without POWMAN's password); only
//     the next chip-level reset replaces them. A reset of the cores alone is not chip-level - AIRCR.SYSRESETREQ, from
//     the firmware or a debugger (RP2350 7.3.3 lists the chip-level sources; SYSRESETREQ is a "warm reset" of a core,
//     Table 1250's description; RP2040: core 0 only) - so after one, CHIP_RESET still names the reset before it:
//     a debugger reset after a RUN-pin reset read as "RUN pin" again.
//   - WATCHDOG REASON: the RP2350 clears it on a debugger warm reset ("a debugger warm reset of either core
//     (SYSRESETREQ or hartreset) will also clear the watchdog reason register", Table 1250, md l.58302); the RP2040
//     keeps it (Table 548, md l.25601: "Logs the reason for the last reset", RO): there a reset of core 0 after a
//     watchdog reset would read as that watchdog reset again. The RP2040 tells the two apart by the CLOCKS block
//     instead (below, `clocksKeptImageSources`).
//   - The mark below tells the two apart: WATCHDOG SCRATCH1, which every chip-level reset clears on the RP2350 ("All
//     chip-level reset sources ... also reset the system watchdog peripheral. This includes watchdog scratch registers
//     SCRATCH0 -> SCRATCH7", 7.3.1) and a RUN-pin, power-on or brown-out reset clears on the RP2040 (rst_n_run, 4.7.4 /
//     2.13.1; a rescue, a PSM restart, keeps them). Neither a SYSRESETREQ nor a watchdog reset of the PSM stages
//     touches it. Each boot's first step writes it (Rp::firstInit, BootSequence.hpp): `Fresh` when it found no mark -
//     CHIP_RESET is this reset's - else `Earlier` - CHIP_RESET is an earlier boot's -, or on the RP2040 `CoresOnly`:
//     as Earlier, and this reset was of the cores alone.
//   - RP2040, a reset of the cores alone: AIRCR.SYSRESETREQ "only resets the Cortex-M0+ processor core (not the Debug
//     or PMU)" (2.4.2.9, md l.3388), so the CLOCKS block keeps the sources the image set. Every watchdog reset Kvasir
//     arms resets the CLOCKS stage of the power-on state machine (PSM.WDSEL bit 2, Table 199 md l.8108; set by both
//     intents, chip_rp2040 WatchdogDomains.hpp, and by the probe's stall recovery 0x0001fffc), and so does every
//     chip-level reset and the rescue (a PSM restart, 2.13): the clock generators come out of reset with "clk_ref
//     ... running from the ring oscillator with no divider. clk_sys is running from clk_ref" (2.13.2, md l.7945),
//     CLK_REF_SELECTED / CLK_SYS_SELECTED 0x1 (Tables 222/225, md l.9470/9538). Measured on the chip: a watchdog
//     timeout and a forced watchdog reboot find them so, a debugger reset finds clk_ref on the XOSC and clk_sys on
//     PLL_SYS. (2.13.1, md l.7918, keeps "clock dividers ... (clk_ref and clk_sys)" running through a PSM restart:
//     the dividers, not the source selection.) Not told apart: a reset of the cores of an
//     image that never left those reset sources (a reset before its clock init, a degraded boot on the ROSC) - it
//     reads as REASON says; and a reset of the cores that finds no mark (the image before wrote none) is judged
//     `Fresh`. A RAM_ONLY image started without a reset over a running image (its J-Link script) finds that image's
//     clocks and reads as core_reset.
#include <cstdint>

namespace Kvasir { namespace PM {
    enum class ResetCause : std::uint8_t {
        // Watchdog resets
        watchdogForce,   // Software triggered watchdog reset
        watchdogTimer,   // Watchdog timeout

        // Debug/rescue resets
        rescue,       // Rescue reset from debugger
        dpResetReq,   // Reset request from ARM debugger (RP2350 HAD_DP_RESET_REQ: a chip-level reset)
        coreReset,   // a reset of the cores alone - AIRCR.SYSRESETREQ from the firmware or from a debugger (J-Link's
        // ordinary reset), RP2350 also a RISC-V hartreset: no chip-level reset and no watchdog since the
        // boot before (the mark above; RP2040 also chip_rp2040 CoreResetGuard.hpp's carried word)

        // Power-related resets
        glitchDetect,   // Power supply glitch
        bor,            // Brown-out detection
        por,            // Power-on reset
        runLow,         // RUN pin reset
        swcorePd,       // Switched core powerdown

        // Unknown/default
        unknown
    };

    // WATCHDOG SCRATCH1's word (RP2350 datasheet 12.9 Table 1251, RP2040 4.7.6 Table 549; owners: WatchdogBoot.hpp).
    namespace ChipResetMark {
        inline constexpr std::uint32_t fresh
          = 0x4B52'5346U;   // "KRSF": CHIP_RESET is this boot's reset
        inline constexpr std::uint32_t earlier
          = 0x4B52'5345U;   // "KRSE": CHIP_RESET is an earlier boot's
        // "KRSC", RP2040 only: CHIP_RESET is an earlier boot's, and this reset was of the cores alone (the clocks)
        inline constexpr std::uint32_t coresOnly = 0x4B52'5343U;

        // What a boot writes over `found`: a mark that survived means no chip-level reset since it was written;
        // `coresOnlyReset` says whether this boot's reset was of the cores alone (always false on the RP2350).
        constexpr std::uint32_t next(std::uint32_t found,
                                     bool          coresOnlyReset) {
            bool const marked = found == fresh || found == earlier || found == coresOnly;
            if(!marked) { return fresh; }
            return coresOnlyReset ? coresOnly : earlier;
        }

        constexpr bool chipResetIsEarlier(std::uint32_t mark) {
            return mark == earlier || mark == coresOnly;
        }

        constexpr bool isCoresOnly(std::uint32_t mark) { return mark == coresOnly; }
    }   // namespace ChipResetMark

    namespace Detail {
        // RP2350: REASON (Table 1250) and POWMAN CHIP_RESET (Table 489) as read, and the mark.
        struct Rp2350Reset {
            bool wdForce;
            bool wdTimer;
            bool chipResetEarlier;
            bool hadRescue;
            bool hadDpResetReq;
            bool hadGlitchDetect;
            bool hadBor;
            bool hadPor;
            bool hadRunLow;
            bool hadSwcorePd;
        };

        constexpr ResetCause classify(Rp2350Reset const& r) {
            // REASON first: a watchdog reset of the PSM stages is not chip-level (CHIP_RESET keeps the one before),
            // and a SYSRESETREQ clears REASON - so a set bit is this boot's
            if(r.wdForce) { return ResetCause::watchdogForce; }
            if(r.wdTimer) { return ResetCause::watchdogTimer; }
            // every chip-level reset clears the mark: with it, all of CHIP_RESET is an earlier boot's
            if(r.chipResetEarlier) { return ResetCause::coreReset; }
            if(r.hadRescue) { return ResetCause::rescue; }
            if(r.hadDpResetReq) { return ResetCause::dpResetReq; }
            if(r.hadGlitchDetect) { return ResetCause::glitchDetect; }
            if(r.hadBor) { return ResetCause::bor; }
            if(r.hadPor) { return ResetCause::por; }
            if(r.hadRunLow) { return ResetCause::runLow; }
            if(r.hadSwcorePd) { return ResetCause::swcorePd; }
            return ResetCause::unknown;
        }

        // RP2040: whether the CLOCKS block kept an image's sources at the first init step - the one-hot
        // CLK_REF_SELECTED / CLK_SYS_SELECTED (Tables 222/225) anything but their reset value 0x1 (clk_ref on the
        // ROSC, clk_sys on clk_ref; 2.13.2): the reset that started this boot was of the cores alone.
        constexpr bool clocksKeptImageSources(std::uint32_t refSelected,
                                              std::uint32_t sysSelected) {
            return refSelected != 1U || sysSelected != 1U;
        }

        // RP2040: CoreResetGuard's carried word, the mark's cores-only verdict, REASON (Table 548), CHIP_RESET
        // (Table 191) and the mark.
        struct Rp2040Reset {
            bool coreResetCarried;
            bool coresOnly;
            bool wdForce;
            bool wdTimer;
            bool chipResetEarlier;
            bool hadPsmRestart;
            bool hadPor;
            bool hadRun;
        };

        constexpr ResetCause classify(Rp2040Reset const& r) {
            if(r.coreResetCarried) { return ResetCause::coreReset; }
            // before REASON, which survives a reset of core 0 (Table 548): a watchdog reset followed by a debugger
            // reset is the debugger's
            if(r.coresOnly) { return ResetCause::coreReset; }
            if(r.wdForce) { return ResetCause::watchdogForce; }
            if(r.wdTimer) { return ResetCause::watchdogTimer; }
            // a rescue (PSM restart) keeps the watchdog scratch words, so the mark cannot judge HAD_PSM_RESTART: a
            // debugger's reset either way
            if(r.hadPsmRestart) { return ResetCause::rescue; }
            // the RUN pin, power-on and brown-out clear the mark: with it, HAD_POR / HAD_RUN are an earlier boot's
            if(r.chipResetEarlier) { return ResetCause::coreReset; }
            if(r.hadPor) { return ResetCause::por; }
            if(r.hadRun) { return ResetCause::runLow; }
            return ResetCause::unknown;
        }
    }   // namespace Detail
}}   // namespace Kvasir::PM
