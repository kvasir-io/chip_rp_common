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

    /// Where a dead receive channel starts again (UartStream's receive watch), with the ring emptied: the next whole
    /// segment after the one that was running - its slots may hold what the channel wrote before it died, and a run
    /// the reader still holds lies before it - or, for a stalled ring, the segment it waited for. The reader's total
    /// moves there too: nothing is left to read.
    struct Restart {
        std::uint32_t segDone;     // segments counted: the new channel writes segment segDone
        std::uint32_t readTotal;   // segDone * seg
    };

    constexpr Restart restart(std::uint32_t segDone,
                              bool          stalled,
                              std::uint32_t seg) {
        std::uint32_t const s = stalled ? segDone : segDone + 1U;
        return {s, s * seg};
    }

    /// What a restart throws away that the reader had been shown (writtenTotal() at its last look) and not
    /// committed: at most a ring (a total the reader never saw counts nothing).
    constexpr std::uint32_t dropped(std::uint32_t shown,
                                    std::uint32_t readTotal,
                                    std::uint32_t ring) {
        std::uint32_t const d = shown - readTotal;
        return d <= ring ? d : 0U;
    }
}}}   // namespace Kvasir::UART::Detail
