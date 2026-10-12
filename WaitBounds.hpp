#pragma once
// The bounds of the RP chip packages' register waits (Kvasir_SDK kvasir/Register/Wait.hpp), each with its source.
// Under the default wait policy (Unbounded) every site is the loop it was; an application that injects a bounded
// policy gets these limits.
#include "kvasir/Register/Wait.hpp"

#include <cstdint>

namespace Kvasir::Chip {
// The fastest clk_sys Kvasir runs the chip at (clock_config.hpp vregInit's limits): Bound::Microseconds converts at
// this, so a slower clock (the ROSC at boot) only makes the real time longer.
#if __has_include("peripherals/POWMAN.hpp")
inline constexpr std::uint64_t CpuHzCeiling = 266'000'000;
#else
inline constexpr std::uint64_t CpuHzCeiling = 200'000'000;
#endif

template<std::uint64_t Us>
using MicrosecondsBound = Kvasir::Register::Bound::Microseconds<Us, CpuHzCeiling>;

// XOSC.STATUS.stable: STARTUP.DELAY is never written by Kvasir; its reset value 0xc4 is "approx 50 000 cycles"
// (RP2040 datasheet md l.10613, RP2350 md l.27964): 4.2 ms at 12 MHz, 50 ms at a 1 MHz crystal.
using XoscStableBound = MicrosecondsBound<100'000>;
// PLL.CS.lock: neither datasheet gives a lock time (RP2040 2.18.3, RP2350 8.6.4: "wait for LOCK"); generous.
using PllLockBound = MicrosecondsBound<10'000>;
// RESETS.RESET_DONE: "set once the peripheral is out of reset", no time given; a few clocks of the block.
using ResetDoneBound = Kvasir::Register::Bound::Polls<1'000>;
// CLK_SYS_SELECTED / CLK_REF_SELECTED after a glitchless switch, CLK_PERI_CTRL.ENABLED (RP2350) after a disable:
// two cycles of the old and new clock (RP2040 datasheet 2.15.3.2, RP2350 8.1.3.2). Slowest: clk_ref on the ROSC,
// >= 1.8 MHz (RP2040 2.17) / 4.6 MHz (RP2350 8.3), possibly divided by a leftover CLK_REF_DIV - 100 us covers /64.
using ClkSelectedBound = MicrosecondsBound<100>;
// A read back of a register just written (PSM.FRCE_OFF): a bus round trip.
using ReadBackBound = Kvasir::Register::Bound::Polls<100>;
}   // namespace Kvasir::Chip
