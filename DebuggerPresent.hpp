#pragma once

// Kvasir::Chip::debuggerPresent() on the RP2040 and RP2350, and the functor for uc_log's RTT backends (their
// DebuggerPresentFunction). No includes: a firmware names the functor in its ComBackend before uc_log and the chip.
//
// Always true: neither chip keeps a "a probe was seen" bit as the SAM DSU does. "Is halting debug on now" is
// Kvasir::Core::haltingDebug() (cortex_common/Debug.hpp: `unknown` on the RP2040's Cortex-M0+, DHCSR.C_DEBUGEN on the
// RP2350's Cortex-M33), and it is no gate for logging: a printer's own reset or flash clears C_DEBUGEN until its next
// session connects, and every line written meanwhile would be lost. With
// rtt::BufferMode::skip a board without a probe drops lines instead of stalling.
namespace Kvasir::Chip {
[[nodiscard]] constexpr bool debuggerPresent() { return true; }

struct DebuggerPresent {
    [[nodiscard]] static constexpr bool operator()() noexcept { return debuggerPresent(); }
};
}   // namespace Kvasir::Chip
