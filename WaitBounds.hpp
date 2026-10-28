#pragma once
// The bounds of the RP chip packages' register waits (Kvasir_SDK kvasir/Register/Wait.hpp), each with its source.
// Under the default wait policy (Unbounded) a site that takes the image's policy is the loop it was; an application
// that injects a bounded policy gets these limits. The clock init's waits (clock_config.hpp coreClockInit) are bounded
// whatever the policy: OnTimeout::Report, the outcome read back by Clocks::initResult().
#include "chip/RpTraits.hpp"
#include "kvasir/Register/Wait.hpp"

#include <cstdint>

// The fastest clk_sys Kvasir runs the chip at (the chip's Rp::ChipTraits, clock_config.hpp vregInit's limits): the
// SDK's MicrosecondsBound converts at this, so a slower clock (the ROSC at boot) only makes the real time longer.
// Specialised before any bound below names it.
template<>
struct Kvasir::Register::CpuHzCeiling<Kvasir::Tag::User> {
    static constexpr std::uint64_t value = Kvasir::Rp::ChipTraits::cpuHzCeiling;
};

namespace Kvasir::Chip {
// Clocks::XoscDelay's startup delay, XOSC.STARTUP.DELAY (clock_config.hpp detail::enableCrystal): what STATUS.STABLE
// waits for, since nothing in the XOSC measures the oscillation. 64 ms is pico-sdk's longest documented setting (1 ms
// times PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64, which boards with a slow-starting crystal use). The reset value, ~4 ms
// at 12 MHz, was too short for a slow-starting crystal after a power-on or a RUN reset; the default,
// Clocks::XoscMeasured, waits for the crystal itself instead.
inline constexpr std::uint32_t xoscStartupMs = 64;
// XOSC.STATUS.stable: the startup delay (Clocks::XoscStartupConfig's startupMs) counts
// crystal periods, so it only runs once the crystal oscillates - three times the delay, and never less than the 100 ms
// this bound always was, for a crystal that is slow to start at all.
// The wait runs with clk_sys on clk_ref on the ROSC, which Clocks::onRingOscillator() (clock_config.hpp) divides to
// Rp::ChipTraits::clkRefOnRoscHzCeiling at most (RP2040 12 MHz, RP2350 25 MHz - clk_ref's maximum), so its polls are
// counted at that clock, not the core's 200 / 266 MHz: longer at a slower ROSC by the ratio and by a poll slower than
// minCyclesPerPoll (measured on an RP2350 A2 with the 100 ms bound, ROSC 11.5 MHz, a DMA channel writing XOSC.CTRL
// every 16 cycles: ~0.85 s, about 8 cycles a poll).
template<std::uint32_t StartupMs>
using XoscStableBoundFor
  = Register::Bound::Microseconds<(3 * StartupMs < 100 ? 100 : 3 * StartupMs) * 1'000,
                                  Rp::ChipTraits::clkRefOnRoscHzCeiling>;
// PLL.CS.lock: neither datasheet gives a lock time (RP2040 2.18.3, RP2350 8.6.4: "wait for LOCK"); a choice, 10 ms. The
// clock init waits with clk_sys on a known clock (PLL_SYS on the crystal, PLL_USB on PLL_SYS): ClkSysHz is that clock.
template<std::uint64_t ClkSysHz>
using PllLockBound = Register::Bound::Microseconds<10'000, ClkSysHz>;
// RESETS.RESET_DONE: "set once the peripheral is out of reset", no time given; a few clocks of the block - a choice,
// 1 000 polls, times the poll margin every bound from a time carries (Register::pollMargin).
using ResetDoneBound = Kvasir::Register::Bound::Polls<1'000 * Register::pollMargin>;
// CLK_SYS_SELECTED / CLK_REF_SELECTED after a glitchless switch, CLK_PERI/USB/ADC_CTRL.ENABLED (RP2350) after a
// disable: two cycles of the old and new clock (RP2040 datasheet 2.15.3.2, RP2350 8.1.3.2). Slowest: clk_ref on the
// ROSC, >= 1.8 MHz (RP2040 2.17) / 4.6 MHz (RP2350 8.3), the boot ROM's CLK_REF_DIV (A3: 5) or Clocks::
// onRingOscillator()'s divisor (<= 8) on top - 100 us covers /64.
using ClkSelectedBound = Register::MicrosecondsBound<100>;
// POWMAN VREG.UPDATE_IN_PROGRESS (RP2350): "set while the regulator's operating mode or output voltage are being
// updated" (datasheet 6.3.2, md l.22847), no time given; a choice: 100 000 polls, some ms on the ROSC.
using VregUpdateBound = Kvasir::Register::Bound::Polls<100'000>;
// POWMAN SEQ_CFG.USING_FAST_POWCK reading 0 after SYSCFG.AUXCTRL bit 0 forced POWMAN onto the LPOSC (RP2350,
// chip/WatchdogDomains.hpp forcePowmanOnLposc): the datasheet states no time (Table 1317 md l.61148, Table 491
// md l.23609). A choice: what it bounds is the number of SEQ_CFG reads, 133 000 on the RP2350 (1 000 us counted at
// the 266 MHz CpuHzCeiling at minCyclesPerPoll, times the default pollMargin 2) - 1 ms only at that clock with
// 4-cycle polls. Each poll is a POWMAN register read, slower than 4 cycles, and the reset path may run below the
// ceiling, so the real give-up time is longer than 1 ms, never shorter. 1 ms was picked as the time the boot ROM's
// reboot() leaves between the two (its 1 ms watchdog timer).
using PowmanLposcBound = Register::MicrosecondsBound<1'000>;
// A read back of a register just written (PSM.FRCE_OFF): a bus round trip.
using ReadBackBound = Kvasir::Register::Bound::Polls<100>;
// PIO SMx_EXECCTRL.EXEC_STALLED after a forced instruction: none for one that does not stall (it executes at
// once, ignoring the divider: RP2040 datasheet 3.5.7, RP2350 11.5.7); one that stalls waits for the caller's
// condition, which no datasheet bounds. A choice, not a datasheet time: callers that expect a long stall pass
// their own bound.
using PioExecBound = Register::MicrosecondsBound<1'000>;
}   // namespace Kvasir::Chip
