#pragma once
// The boot ROM's watchdog boot vector, the same protocol on both chips: SCRATCH4 the magic 0xb007c0d3, SCRATCH5 the
// entry point XORed with -magic, SCRATCH6 the stack pointer, SCRATCH7 the entry point (Thumb bit set). On a non-POR/BOR
// reset the ROM checks both magic words and zeroes SCRATCH4 before it transfers control, so a vector is good for one
// boot (RP2040 datasheet 2.8.1.1, md l.6308-6320; RP2350 datasheet 5.2.4, md l.19242-19262).
//
// Who owns which WATCHDOG scratch word:
//   SCRATCH4..7  this vector (CoreResetGuard arms it, Rp::watchdogReboot() disarms it)
//   SCRATCH3     RP2040: CoreResetGuard's cause word (PM::CoreResetCarried below)
//   SCRATCH2/3   RP2350: the boot ROM's reboot() writes its p0/p1 there (RP2350 datasheet 5.4.8.24)
//   SCRATCH1     both: the chip-reset mark, written by every boot (PM::markChipReset, ResetCauseRules.hpp)
//   SCRATCH0     no SDK or chip path writes it: the application's
#include "kvasir/Register/Register.hpp"
#include "peripherals/WATCHDOG.hpp"

#include <cstdint>

namespace Kvasir::Rp::WatchdogBoot {
inline constexpr std::uint32_t magic = 0xB007'C0D3U;

// SCRATCH words are whole-word values. SCRATCH4 last, so a reset in between finds no half-written vector.
[[gnu::always_inline]] inline void arm(std::uint32_t entryThumb,
                                       std::uint32_t stackPointer) {
    using WD = Kvasir::Peripheral::WATCHDOG::Registers<>;
    apply(write(WD::SCRATCH7::FULLREGISTER, entryThumb));
    apply(write(WD::SCRATCH6::FULLREGISTER, stackPointer));
    apply(write(WD::SCRATCH5::FULLREGISTER, entryThumb ^ (0U - magic)));
    apply(write(WD::SCRATCH4::FULLREGISTER, Register::value<magic>()));
}

// No vector for the next boot: the ROM takes none without SCRATCH4's magic.
[[gnu::always_inline]] inline void disarm() {
    using WD = Kvasir::Peripheral::WATCHDOG::Registers<>;
    apply(write(WD::SCRATCH4::FULLREGISTER, Register::value<0>()));
}

// A vector is set for the next non-POR/BOR reset.
[[nodiscard]] inline bool armed() {
    using WD = Kvasir::Peripheral::WATCHDOG::Registers<>;
    return get<0>(apply(read(WD::SCRATCH4::FULLREGISTER))) == magic;
}
}   // namespace Kvasir::Rp::WatchdogBoot

#ifndef KVASIR_CHIP_RP2350
namespace Kvasir { namespace PM {
    namespace detail {
        inline std::uint8_t coreResetCarry{};   // .bss: 0 unread, 1 no, 2 yes
    }   // namespace detail

    // CoreResetGuard's word in WATCHDOG SCRATCH3 (RP2040 datasheet 4.7.6): this value after a reset of core 0
    // alone, else 0. Read once and cleared so a later reset that skips the routine does not inherit it.
    inline constexpr std::uint32_t coreResetCarried = 0x4B43'5253U;   // "KCRS"

    inline bool isCoreResetCarried() {
        if(detail::coreResetCarry == 0) {
            using WD = Kvasir::Peripheral::WATCHDOG::Registers<>;
            auto const carried
              = get<0>(apply(read(WD::SCRATCH3::FULLREGISTER))) == coreResetCarried;
            if(carried) { apply(write(WD::SCRATCH3::FULLREGISTER, Register::value<0>())); }
            detail::coreResetCarry = carried ? 2 : 1;
        }
        return detail::coreResetCarry == 2;
    }
}}   // namespace Kvasir::PM
#endif
