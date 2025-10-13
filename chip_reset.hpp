#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/POWMAN.hpp"
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

    inline ResetCause reset_cause() {
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
        if(wd_reasons[WdReason::force]) {
            return ResetCause::watchdog_force;
        }
        if(wd_reasons[WdReason::timer]) {
            return ResetCause::watchdog_timer;
        }

        // Debug/rescue resets
        if(chip_reset[ChipReset::had_rescue]) {
            return ResetCause::rescue;
        }
        if(chip_reset[ChipReset::had_dp_reset_req]) {
            return ResetCause::dp_reset_req;
        }

        // Power-related resets
        if(chip_reset[ChipReset::had_glitch_detect]) {
            return ResetCause::glitch_detect;
        }
        if(chip_reset[ChipReset::had_bor]) {
            return ResetCause::bor;
        }
        if(chip_reset[ChipReset::had_por]) {
            return ResetCause::por;
        }
        if(chip_reset[ChipReset::had_run_low]) {
            return ResetCause::run_low;
        }
        if(chip_reset[ChipReset::had_swcore_pd]) {
            return ResetCause::swcore_pd;
        }

        return ResetCause::unknown;
    }
}}   // namespace Kvasir::PM
