#pragma once
// The ring oscillator's random bit (RP2040 datasheet 2.17.8 Table 264; RP2350 datasheet ROSC RANDOMBIT, 0x400e8000 +
// 0x20, md l.1552, l.28148): one bit per read, random only while the cores do not run from the ROSC, and NOT
// cryptographic. Each user keeps its own loop: the bit order of a value is the user's (FirstInitStep's salt on the
// RP2350 interleaves two words).
#include "kvasir/Register/Register.hpp"
#include "peripherals/ROSC.hpp"

#include <cstdint>

namespace Kvasir::Rosc {
[[gnu::always_inline]] inline std::uint32_t randomBit() {
    using ROSC = Kvasir::Peripheral::ROSC::Registers<>;
    return get<0>(apply(read(ROSC::RANDOMBIT::randombit)));
}

// STATUS.STABLE: the oscillator is running and stable (RP2040 Table 271).
[[gnu::always_inline]] inline bool stable() {
    using ROSC = Kvasir::Peripheral::ROSC::Registers<>;
    return get<0>(apply(read(ROSC::STATUS::stable))) != 0U;
}
}   // namespace Kvasir::Rosc
