#pragma once

#include "I2C.hpp"
#include "kvasir/Devices/I2C/LineRecovery.hpp"

#include <string_view>

namespace Kvasir { namespace I2C {

    // The bus recovery state machine (nine clocks, a STOP, a forced STOP as the last resort,
    // stuck-SDA detection, the post-abort settle gate) is shared with the SAM SERCOM driver
    // since 2026-09-20: kvasir_devices' kvasir/Devices/I2C/LineRecovery.hpp has the code and
    // the story.
    template<typename I2CConfig, typename Clock>
    using I2CBusRecovery = LineRecovery<Detail::I2CBase<I2CConfig>, Clock>;

}}   // namespace Kvasir::I2C
