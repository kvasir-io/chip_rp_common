#pragma once

#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"

#if __has_include("peripherals/POWMAN.hpp")
    #include "peripherals/POWMAN.hpp"
#endif
#include "peripherals/WATCHDOG.hpp"

#include <cstdint>
#include <string_view>

namespace Kvasir { namespace PM {
    enum class ResetCause : std::uint8_t {
        // Watchdog resets
        watchdog_force,   // Software triggered watchdog reset
        watchdog_timer,   // Watchdog timeout

        // Debug/rescue resets
        rescue,         // Rescue reset from debugger
        dp_reset_req,   // Reset request from ARM debugger

        // Power-related resets
        glitch_detect,   // Power supply glitch
        bor,             // Brown-out detection
        por,             // Power-on reset
        run_low,         // RUN pin reset
        swcore_pd,       // Switched core powerdown

        // Unknown/default
        unknown
    };

    // All reset bits, not just the summary of `reset_cause()`: CHIP_RESET is read-only and never
    // cleared, so bits from an earlier reset can survive alongside the current one.
    // Values are the CHIP_RESET bit positions; wd_timer/wd_force come from WATCHDOG::REASON and use
    // bits 2 and 3, which CHIP_RESET leaves unused.
    enum class ResetSource : std::uint32_t {
        none                            = 0,
        wd_timer                        = 1U << 2U,
        wd_force                        = 1U << 3U,
        had_por                         = 1U << 16U,
        had_bor                         = 1U << 17U,
        had_run_low                     = 1U << 18U,
        had_dp_reset_req                = 1U << 19U,
        had_rescue                      = 1U << 21U,
        had_watchdog_reset_powman_async = 1U << 22U,
        had_watchdog_reset_powman       = 1U << 23U,
        had_watchdog_reset_swcore       = 1U << 24U,
        had_swcore_pd                   = 1U << 25U,
        had_glitch_detect               = 1U << 26U,
        had_hzd_sys_reset_req           = 1U << 27U,
        had_watchdog_reset_rsm          = 1U << 28U
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

    inline ResetSource reset_sources() {
        ResetSource sources{ResetSource::none};
#if __has_include("peripherals/POWMAN.hpp")
        using ChipReset = Kvasir::Peripheral::POWMAN::Registers<>::CHIP_RESET;
        using WdReason  = Kvasir::Peripheral::WATCHDOG::Registers<>::REASON;

        auto const wd_reasons = apply(read(WdReason::force), read(WdReason::timer));

        auto const chip_reset = apply(read(ChipReset::had_watchdog_reset_rsm),
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

        // Not named `set`: that is a Kvasir register verb used above.
        auto add = [&](bool present, ResetSource source) {
            if(present) { sources |= source; }
        };

        add(wd_reasons[WdReason::timer] != 0, ResetSource::wd_timer);
        add(wd_reasons[WdReason::force] != 0, ResetSource::wd_force);

        add(chip_reset[ChipReset::had_por] != 0, ResetSource::had_por);
        add(chip_reset[ChipReset::had_bor] != 0, ResetSource::had_bor);
        add(chip_reset[ChipReset::had_run_low] != 0, ResetSource::had_run_low);
        add(chip_reset[ChipReset::had_dp_reset_req] != 0, ResetSource::had_dp_reset_req);
        add(chip_reset[ChipReset::had_rescue] != 0, ResetSource::had_rescue);
        add(chip_reset[ChipReset::had_watchdog_reset_powman_async] != 0,
            ResetSource::had_watchdog_reset_powman_async);
        add(chip_reset[ChipReset::had_watchdog_reset_powman] != 0,
            ResetSource::had_watchdog_reset_powman);
        add(chip_reset[ChipReset::had_watchdog_reset_swcore] != 0,
            ResetSource::had_watchdog_reset_swcore);
        add(chip_reset[ChipReset::had_swcore_pd] != 0, ResetSource::had_swcore_pd);
        add(chip_reset[ChipReset::had_glitch_detect] != 0, ResetSource::had_glitch_detect);
        add(chip_reset[ChipReset::had_hzd_sys_reset_req] != 0, ResetSource::had_hzd_sys_reset_req);
        add(chip_reset[ChipReset::had_watchdog_reset_rsm] != 0,
            ResetSource::had_watchdog_reset_rsm);
#endif
        //TODO rp2040
        return sources;
    }

    inline ResetCause reset_cause() {
#if __has_include("peripherals/POWMAN.hpp")
        using ChipReset = Kvasir::Peripheral::POWMAN::Registers<>::CHIP_RESET;
        using WdReason  = Kvasir::Peripheral::WATCHDOG::Registers<>::REASON;

        auto wd_reasons = apply(read(WdReason::force), read(WdReason::timer));

        auto chip_reset = apply(read(ChipReset::had_rescue),
                                read(ChipReset::had_dp_reset_req),
                                read(ChipReset::had_glitch_detect),
                                read(ChipReset::had_bor),
                                read(ChipReset::had_por),
                                read(ChipReset::had_run_low),
                                read(ChipReset::had_swcore_pd));

        // Watchdog resets
        if(wd_reasons[WdReason::force]) { return ResetCause::watchdog_force; }
        if(wd_reasons[WdReason::timer]) { return ResetCause::watchdog_timer; }

        // Debug/rescue resets
        if(chip_reset[ChipReset::had_rescue]) { return ResetCause::rescue; }
        if(chip_reset[ChipReset::had_dp_reset_req]) { return ResetCause::dp_reset_req; }

        // Power-related resets
        if(chip_reset[ChipReset::had_glitch_detect]) { return ResetCause::glitch_detect; }
        if(chip_reset[ChipReset::had_bor]) { return ResetCause::bor; }
        if(chip_reset[ChipReset::had_por]) { return ResetCause::por; }
        if(chip_reset[ChipReset::had_run_low]) { return ResetCause::run_low; }
        if(chip_reset[ChipReset::had_swcore_pd]) { return ResetCause::swcore_pd; }
#endif
        //TODO rp2040
        return ResetCause::unknown;
    }
}}   // namespace Kvasir::PM
