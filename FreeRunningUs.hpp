#pragma once
// Kvasir::Port::FreeRunningUs<Tag::User> on the RP2040 and RP2350 (kvasir/Port/Hooks.hpp): the TIMER (the RP2350's
// TIMER0), one microsecond a tick from the tick generator, 64 bits, one counter for both cores. The time the panic
// record keeps (Panic::FullRecord, kvasir/Runtime/Panic.hpp): microseconds since the TIMER left reset in that boot -
// every image's coreClockInit releases it (detail::startTimerTick), a boot-time clock-init panic releases it first
// (detail::clockInitFailed). A TIMER still held in reset reads what a held block returns (neither datasheet says what).
//
// The raw pair, TIMERAWH / TIMERAWL ("Raw read from bits 63:32 / 31:0 of time (no side effects)", RP2350 datasheet
// 12.8.2 md l.57646 and Tables 1237/1238 md l.57921-57942; RP2040 datasheet 4.6.2 md l.25032 and Tables 537/538 md
// l.25296-25312): no latch, so any core and any context may read it; TIMELR/TIMEHR would latch the high word for the
// next reader. High, low, high: on a carry between the reads the low word is read again with the second high word -
// three or four reads, no loop (this runs on a fatal path, also with a TIMER that does not count).
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Port/Hooks.hpp"
#include "kvasir/Register/Register.hpp"

#include <cstdint>
#include <peripherals/TIMER.hpp>

namespace Kvasir::Rp::detail {
[[gnu::always_inline]] inline std::uint64_t timerRawUs() {
    using Regs       = Kvasir::Peripheral::TIMER::Registers<0>;
    std::uint32_t hi = get<0>(apply(read(Regs::TIMERAWH::FULLREGISTER)));
    std::uint32_t lo = get<0>(apply(read(Regs::TIMERAWL::FULLREGISTER)));
    if(std::uint32_t const hi2 = get<0>(apply(read(Regs::TIMERAWH::FULLREGISTER))); hi2 != hi) {
        hi = hi2;
        lo = get<0>(apply(read(Regs::TIMERAWL::FULLREGISTER)));
    }
    return (static_cast<std::uint64_t>(hi) << 32U) | lo;
}
}   // namespace Kvasir::Rp::detail

template<typename... Ts>
struct Kvasir::Port::FreeRunningUs<Kvasir::Tag::User, Ts...> {
    [[gnu::always_inline]] std::uint64_t operator()() const {
        return Kvasir::Rp::detail::timerRawUs();
    }
};
