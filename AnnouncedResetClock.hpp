#pragma once
// The clock Kvasir::AnnouncedReset counts its wait with, on the RP2040 and RP2350.
#include "peripherals/TIMER.hpp"

#include <cstdint>
#include <kvasir/Register/Register.hpp>
#include <kvasir/Runtime/AnnouncedReset.hpp>   // its weak declaration

// Microsecond counter for Kvasir::AnnouncedReset: TIMERAWL, side-effect-free (RP2040 datasheet 4.6.5, TIMER_BASE
// 0x40054000 + 0x28; RP2350 datasheet 12.8, TIMER0_BASE 0x400b0000 + 0x28), safe from a fault handler; it stands still
// until the clock set-up starts the tick (AnnouncedReset then counts polls). Weak, not inline (gcc refuses weak
// inline): the linker keeps one copy. TIMER instance 0 is TIMER0 on the RP2350, the one TIMER on the RP2040.
extern "C" [[gnu::weak,
             gnu::used]] std::uint32_t
kvasir_announced_reset_time_us() {
    return get<0>(apply(read(Kvasir::Peripheral::TIMER::Registers<0>::TIMERAWL::FULLREGISTER)));
}
