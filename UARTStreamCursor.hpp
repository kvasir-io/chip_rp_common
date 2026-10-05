#pragma once
// The RX ring's cursor arithmetic of UartStream (UARTStream.hpp) as pure functions, without chip headers: the host test
// (tests/uart_stream_cursor_test.cpp) drives them with a simulated DMA channel.
#include <algorithm>
#include <cstdint>

namespace Kvasir { namespace UART { namespace Detail {
    /// Elements the RX channel has written since start: completed segments plus the running one's progress.
    /// A pending completion (channel done, callback not yet run) has transCount 0 and counts the whole
    /// segment; a stalled ring (segment counted, channel idle, not re-armed) adds nothing. Totals wrap at 2^32,
    /// which a power-of-two ring divides.
    constexpr std::uint32_t writtenTotal(std::uint32_t segDone,
                                         bool          stalled,
                                         std::uint32_t transCount,
                                         std::uint32_t seg) {
        return segDone * seg + (stalled ? 0U : seg - transCount);
    }

    /// May the channel take segment `segDone` + 1 (elements [segDone * seg, +seg)) with the reader at
    /// `readTotal`? Only if it overwrites nothing unread.
    constexpr bool mayRearm(std::uint32_t segDone,
                            std::uint32_t readTotal,
                            std::uint32_t seg,
                            std::uint32_t ring) {
        return segDone * seg + seg - readTotal <= ring;
    }

    /// The readable run at `readTotal`: up to what is written, and up to the ring's end (the next call gets the
    /// rest from the start).
    struct Run {
        std::uint32_t at;
        std::uint32_t len;
    };

    constexpr Run readable(std::uint32_t written,
                           std::uint32_t readTotal,
                           std::uint32_t ring) {
        std::uint32_t const at = readTotal & (ring - 1U);
        return {at, std::min(written - readTotal, ring - at)};
    }
}}}   // namespace Kvasir::UART::Detail
