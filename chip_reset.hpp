#pragma once

#include "kvasir/Port/Hooks.hpp"
#include "kvasir/Port/ResetKind.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"

#ifdef KVASIR_CHIP_RP2350
    #include "peripherals/POWMAN.hpp"
#else
    #include "peripherals/CLOCKS.hpp"
    #include "peripherals/VREG_AND_CHIP_RESET.hpp"
#endif
#include "ResetCauseRules.hpp"   // ResetCause, the decision, the chip-reset mark
#include "WatchdogBoot.hpp"      // RP2040: PM::isCoreResetCarried()
#include "peripherals/WATCHDOG.hpp"

#include <cstdint>
#include <string_view>

namespace Kvasir { namespace PM {
    // All reset bits, not just the summary of `resetCause()`. CHIP_RESET names the last CHIP-LEVEL reset and is
    // read-only (ResetCauseRules.hpp): after a reset of the cores alone its bits are an earlier boot's, which
    // `chip_reset_earlier` says (bit 0, unused by CHIP_RESET); on the RP2040 `cores_only` (bit 1, unused too) says
    // that the reset was of the cores alone (the mark's CoresOnly, ResetCauseRules.hpp).
    // Values are the RP2350 CHIP_RESET bit positions (POWMAN, datasheet Table 489); wd_timer/wd_force
    // come from WATCHDOG::REASON (Table 1250) and use bits 2 and 3, which CHIP_RESET leaves unused.
    // had_watchdog_reset_rsm is the datasheet's HAD_WATCHDOG_RESET_PSM (bit 28; the SVD says RSM).
    // On the RP2040 the three bits of VREG_AND_CHIP_RESET.CHIP_RESET (RP2040 datasheet Table 191)
    // map onto had_por (HAD_POR: power-on or brown-out), had_run_low (HAD_RUN) and had_rescue
    // (HAD_PSM_RESTART: the debugger's rescue restart), REASON is the same register (Table 548).
    // A set of bits: flag_enum tells the analyzer that an OR of enumerators is a value of the type.
    enum class [[clang::flag_enum]] ResetSource : std::uint32_t {
        none                        = 0,
        chipResetEarlier            = 1U << 0U,
        coresOnly                   = 1U << 1U,
        wdTimer                     = 1U << 2U,
        wdForce                     = 1U << 3U,
        hadPor                      = 1U << 16U,
        hadBor                      = 1U << 17U,
        hadRunLow                   = 1U << 18U,
        hadDpResetReq               = 1U << 19U,
        hadRescue                   = 1U << 21U,
        hadWatchdogResetPowmanAsync = 1U << 22U,
        hadWatchdogResetPowman      = 1U << 23U,
        hadWatchdogResetSwcore      = 1U << 24U,
        hadSwcorePd                 = 1U << 25U,
        hadGlitchDetect             = 1U << 26U,
        hadHzdSysResetReq           = 1U << 27U,
        hadWatchdogResetRsm         = 1U << 28U
    };

    // enchantum recognizes a bitflag enum by exactly these operators (no trait to specialize), which
    // is what makes remote_fmt print enumerator names. Spelled out here so this header keeps working
    // without enchantum.
    constexpr ResetSource operator|(ResetSource a,
                                    ResetSource b) {
        return static_cast<ResetSource>(static_cast<std::uint32_t>(a)
                                        | static_cast<std::uint32_t>(b));
    }

    constexpr ResetSource operator&(ResetSource a,
                                    ResetSource b) {
        return static_cast<ResetSource>(static_cast<std::uint32_t>(a)
                                        & static_cast<std::uint32_t>(b));
    }

    constexpr ResetSource operator~(ResetSource a) {
        return static_cast<ResetSource>(~static_cast<std::uint32_t>(a));
    }

    constexpr ResetSource& operator|=(ResetSource& a,
                                      ResetSource  b) {
        a = a | b;
        return a;
    }

    constexpr ResetSource& operator&=(ResetSource& a,
                                      ResetSource  b) {
        a = a & b;
        return a;
    }

    // The chip-reset mark (ResetCauseRules.hpp): each boot's first step writes WATCHDOG SCRATCH1 from what it found
    // there; registers only - it runs before .data/.bss (BootSequence.hpp), and before the clock init.
    [[gnu::always_inline]] inline void markChipReset() {
        using WD         = Kvasir::Peripheral::WATCHDOG::Registers<>;
        auto const found = get<0>(apply(read(WD::SCRATCH1::FULLREGISTER)));
#ifdef KVASIR_CHIP_RP2350
        // SYSRESETREQ clears REASON here (Table 1250): nothing a reset of the cores alone could be taken for
        constexpr bool coresOnly = false;
#else
        // a reset of the cores alone leaves the CLOCKS block on the image's sources (ResetCauseRules.hpp)
        using Clk            = Kvasir::Peripheral::CLOCKS::Registers<>;
        auto const selected  = apply(read(Clk::CLK_REF_SELECTED::clk_ref_selected),
                                     read(Clk::CLK_SYS_SELECTED::clk_sys_selected));
        bool const coresOnly = Detail::clocksKeptImageSources(
          static_cast<std::uint32_t>(selected[Clk::CLK_REF_SELECTED::clk_ref_selected]),
          static_cast<std::uint32_t>(selected[Clk::CLK_SYS_SELECTED::clk_sys_selected]));
#endif
        apply(write(WD::SCRATCH1::FULLREGISTER, ChipResetMark::next(found, coresOnly)));
    }

    // CHIP_RESET names an earlier boot's reset: none of the chip-level resets that clear the mark came since.
    inline bool chipResetIsEarlier() {
        using WD = Kvasir::Peripheral::WATCHDOG::Registers<>;
        return ChipResetMark::chipResetIsEarlier(get<0>(apply(read(WD::SCRATCH1::FULLREGISTER))));
    }

#ifndef KVASIR_CHIP_RP2350
    // RP2040: the reset that started this boot was of the cores alone (the mark's CoresOnly).
    inline bool coresOnlyReset() {
        using WD = Kvasir::Peripheral::WATCHDOG::Registers<>;
        return ChipResetMark::isCoresOnly(get<0>(apply(read(WD::SCRATCH1::FULLREGISTER))));
    }
#endif

    namespace Detail {
        // REASON and CHIP_RESET, each read once (one load each), in the RP2350's CHIP_RESET names: on the RP2040
        // HAD_POR is hadPor, HAD_RUN hadRunLow, HAD_PSM_RESTART hadRescue (ResetSource's comment), the rest false.
        struct ResetRegisters {
            bool wdForce;
            bool wdTimer;
            bool hadPor;
            bool hadBor;
            bool hadRunLow;
            bool hadDpResetReq;
            bool hadRescue;
            bool hadWatchdogResetPowmanAsync;
            bool hadWatchdogResetPowman;
            bool hadWatchdogResetSwcore;
            bool hadSwcorePd;
            bool hadGlitchDetect;
            bool hadHzdSysResetReq;
            bool hadWatchdogResetRsm;
        };

        // inlined into each reader: out of line it returns the struct through memory (+64..104 B)
        [[gnu::always_inline]] inline ResetRegisters readResetRegisters() {
            using WdReason       = Kvasir::Peripheral::WATCHDOG::Registers<>::REASON;
            auto const wdReasons = apply(read(WdReason::force), read(WdReason::timer));
#ifdef KVASIR_CHIP_RP2350
            using ChipReset      = Kvasir::Peripheral::POWMAN::Registers<>::CHIP_RESET;
            auto const chipReset = apply(read(ChipReset::had_watchdog_reset_rsm),
                                         read(ChipReset::had_hzd_sys_reset_req),
                                         read(ChipReset::had_glitch_detect),
                                         read(ChipReset::had_swcore_pd),
                                         read(ChipReset::had_watchdog_reset_swcore),
                                         read(ChipReset::had_watchdog_reset_powman),
                                         read(ChipReset::had_watchdog_reset_powman_async),
                                         read(ChipReset::had_rescue),
                                         read(ChipReset::had_dp_reset_req),
                                         read(ChipReset::had_run_low),
                                         read(ChipReset::had_bor),
                                         read(ChipReset::had_por));
            return {.wdForce       = wdReasons[WdReason::force] != 0,
                    .wdTimer       = wdReasons[WdReason::timer] != 0,
                    .hadPor        = chipReset[ChipReset::had_por] != 0,
                    .hadBor        = chipReset[ChipReset::had_bor] != 0,
                    .hadRunLow     = chipReset[ChipReset::had_run_low] != 0,
                    .hadDpResetReq = chipReset[ChipReset::had_dp_reset_req] != 0,
                    .hadRescue     = chipReset[ChipReset::had_rescue] != 0,
                    .hadWatchdogResetPowmanAsync
                    = chipReset[ChipReset::had_watchdog_reset_powman_async] != 0,
                    .hadWatchdogResetPowman = chipReset[ChipReset::had_watchdog_reset_powman] != 0,
                    .hadWatchdogResetSwcore = chipReset[ChipReset::had_watchdog_reset_swcore] != 0,
                    .hadSwcorePd            = chipReset[ChipReset::had_swcore_pd] != 0,
                    .hadGlitchDetect        = chipReset[ChipReset::had_glitch_detect] != 0,
                    .hadHzdSysResetReq      = chipReset[ChipReset::had_hzd_sys_reset_req] != 0,
                    .hadWatchdogResetRsm    = chipReset[ChipReset::had_watchdog_reset_rsm] != 0};
#else
            // RP2040: REASON (datasheet Table 548) and CHIP_RESET in VREG_AND_CHIP_RESET (Table 191)
            using ChipReset      = Kvasir::Peripheral::VREG_AND_CHIP_RESET::Registers<>::CHIP_RESET;
            auto const chipReset = apply(read(ChipReset::had_psm_restart),
                                         read(ChipReset::had_run),
                                         read(ChipReset::had_por));
            return {.wdForce                     = wdReasons[WdReason::force] != 0,
                    .wdTimer                     = wdReasons[WdReason::timer] != 0,
                    .hadPor                      = chipReset[ChipReset::had_por] != 0,
                    .hadBor                      = false,
                    .hadRunLow                   = chipReset[ChipReset::had_run] != 0,
                    .hadDpResetReq               = false,
                    .hadRescue                   = chipReset[ChipReset::had_psm_restart] != 0,
                    .hadWatchdogResetPowmanAsync = false,
                    .hadWatchdogResetPowman      = false,
                    .hadWatchdogResetSwcore      = false,
                    .hadSwcorePd                 = false,
                    .hadGlitchDetect             = false,
                    .hadHzdSysResetReq           = false,
                    .hadWatchdogResetRsm         = false};
#endif
        }
    }   // namespace Detail

    inline ResetSource resetSources() {
        ResetSource sources{ResetSource::none};
        if(chipResetIsEarlier()) { sources |= ResetSource::chipResetEarlier; }
#ifndef KVASIR_CHIP_RP2350
        if(coresOnlyReset()) { sources |= ResetSource::coresOnly; }
#endif
        auto const r = Detail::readResetRegisters();
        // Not named `set`: that is a Kvasir register verb.
        auto const add = [&](bool present, ResetSource source) {
            if(present) { sources |= source; }
        };
        add(r.wdTimer, ResetSource::wdTimer);
        add(r.wdForce, ResetSource::wdForce);
        add(r.hadPor, ResetSource::hadPor);
        add(r.hadBor, ResetSource::hadBor);
        add(r.hadRunLow, ResetSource::hadRunLow);
        add(r.hadDpResetReq, ResetSource::hadDpResetReq);
        add(r.hadRescue, ResetSource::hadRescue);
        add(r.hadWatchdogResetPowmanAsync, ResetSource::hadWatchdogResetPowmanAsync);
        add(r.hadWatchdogResetPowman, ResetSource::hadWatchdogResetPowman);
        add(r.hadWatchdogResetSwcore, ResetSource::hadWatchdogResetSwcore);
        add(r.hadSwcorePd, ResetSource::hadSwcorePd);
        add(r.hadGlitchDetect, ResetSource::hadGlitchDetect);
        add(r.hadHzdSysResetReq, ResetSource::hadHzdSysResetReq);
        add(r.hadWatchdogResetRsm, ResetSource::hadWatchdogResetRsm);
        return sources;
    }

    // The registers' one reading (Detail::readResetRegisters) summarised: ResetCauseRules.hpp decides.
    inline ResetCause resetCause() {
#ifdef KVASIR_CHIP_RP2350
        auto const r = Detail::readResetRegisters();
        return Detail::classify(Detail::Rp2350Reset{.wdForce          = r.wdForce,
                                                    .wdTimer          = r.wdTimer,
                                                    .chipResetEarlier = chipResetIsEarlier(),
                                                    .hadRescue        = r.hadRescue,
                                                    .hadDpResetReq    = r.hadDpResetReq,
                                                    .hadGlitchDetect  = r.hadGlitchDetect,
                                                    .hadBor           = r.hadBor,
                                                    .hadPor           = r.hadPor,
                                                    .hadRunLow        = r.hadRunLow,
                                                    .hadSwcorePd      = r.hadSwcorePd});
#else
        // a reset of core 0 alone leaves REASON and CHIP_RESET as they were: the guard's SCRATCH3 word and the mark's
        // CoresOnly say so (ResetCauseRules.hpp)
        auto const r = Detail::readResetRegisters();
        return Detail::classify(Detail::Rp2040Reset{.coreResetCarried = PM::isCoreResetCarried(),
                                                    .coresOnly        = coresOnlyReset(),
                                                    .wdForce          = r.wdForce,
                                                    .wdTimer          = r.wdTimer,
                                                    .chipResetEarlier = chipResetIsEarlier(),
                                                    .hadPsmRestart    = r.hadRescue,
                                                    .hadPor           = r.hadPor,
                                                    .hadRun           = r.hadRunLow});
#endif
    }

    /// resetCause() as the SDK's boot guard sees it (kvasir/Port/ResetKind.hpp). On the RP2350 the bootrom's
    /// reboot is a watchdog timeout too (it arms the watchdog's timer): a deliberate reboot calls the guard's
    /// plannedReset() first. A reset of the cores alone (SYSRESETREQ: the firmware's or a debugger's) is
    /// softwareRequest, also after a RUN-pin or a watchdog reset (ResetCauseRules.hpp: the RP2350 clears REASON, the
    /// RP2040 tells it by its clocks).
    inline ResetKind resetKind() {
        switch(resetCause()) {
        case ResetCause::watchdogForce: return ResetKind::watchdogForced;
        case ResetCause::watchdogTimer: return ResetKind::watchdogTimeout;
        case ResetCause::rescue:
        case ResetCause::dpResetReq:    return ResetKind::debugger;
        case ResetCause::coreReset:     return ResetKind::softwareRequest;
        case ResetCause::glitchDetect:
        case ResetCause::bor:           return ResetKind::brownOut;
        case ResetCause::por:
        case ResetCause::swcorePd:      return ResetKind::powerOn;
        case ResetCause::runLow:        return ResetKind::external;
        case ResetCause::unknown:       return ResetKind::unknown;
        }
        return ResetKind::unknown;
    }
}}   // namespace Kvasir::PM

/// resetCause() as the port's hook (kvasir/Port/Hooks.hpp), for the boot report:
/// Port::LateT<Port::ResetCause<Tag::User>, Dep>{}().
template<typename... Ts>
struct Kvasir::Port::ResetCause<Kvasir::Tag::User, Ts...> {
    Kvasir::PM::ResetCause operator()() const { return Kvasir::PM::resetCause(); }
};
