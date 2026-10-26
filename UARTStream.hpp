#pragma once
// A UART streamed by DMA both ways. The queue UART (UART.hpp, UartBehavior) stays the default; a port names this
// instead in its Startup list.
//
//     using Uart = Kvasir::UART::UartStream<Cfg, Clock, Dma, Kvasir::UART::TxStream<Dma::Channel::ch2, 1024>,
//                                           Kvasir::UART::RxRing<Dma::Channel::ch3, 2048, 4>>;
//     Uart::write(bytes);                                               // copies, as much as fits
//     if(auto s = Uart::writeReserve(64); !s.empty()) { Uart::writeCommit(format(s)); }   // in place
//     for(auto s = Uart::readReserve(); !s.empty(); s = Uart::readReserve()) { use(s); Uart::readCommit(s.size()); }
//     if(Uart::rxLost().overruns != 0) { ... }                          // loss is counted, never silent
//
// TX: a Kvasir::Atomic::BipBuffer; every contiguous run of committed bytes is one DMA sequence, started by
// writeCommit()/write() when the channel is idle and by the channel's completion callback otherwise. One producer
// (one thread or ISR); the channel side runs under criticalSection<TxTag>.
//
// A completion that never comes (the channel aborted elsewhere, the UART or the DMA reset under it, the interrupt
// enable lost) is told from a peer that holds the line: the producer's calls (write, writeReserve, writeCommit,
// txFree, txIdle, txFailed) watch the channel. Progress is its TRANS_COUNT moving (one transfer per character frame
// the FIFO sends: credit-based DREQs, RP2350 datasheet 12.6.4.2 md l.53900, RP2040 2.5.3.2), its completion pending
// behind masked interrupts (DmaBase::completionPending), or - with ctsPinLocation - the peer saying it is not ready:
// CTS deasserted (UARTFR.CTS reads 0: "the bit is 1 when nUARTCTS is LOW", RP2040 datasheet md l.20187, RP2350 md
// l.47992) or changed since the last look (UARTRIS.CTSRMIS, "asserted if any of the modem status signals ...
// change", RP2040 md l.19966, RP2350 l.47753). CTS counts only while the UART could send but holds characters it may
// not: out of reset, UARTEN, TXE and TXDMAE set, and its TX FIFO not empty (UARTFR.TXFE, RP2040 md l.20180, RP2350
// l.47985). The transmitter "checks nUARTCTS before transmitting each byte" (RP2350 12.1.4.1 md l.47654, RP2040 md
// l.19865), so a peer that toggles CTS drains the FIFO, and a lost completion is found once it is empty however fast
// the peer toggles; a UART reset or disabled while the peer holds CTS is found at the bound too. A peer that holds CTS
// with the FIFO full is waited for as long as it says so. Otherwise no progress for `txStallTimeout` (twice a
// character frame plus 1 ms, on Clock) ends the run in flight:
//   - all its bytes went to the UART (TRANS_COUNT 0 and READ_ADDR at the run's end: DmaBase::readAddress) - its
//     completion was lost (the interrupt enable) or is still on its way (taken on the other core, waiting for TxTag):
//     the run counts as sent and the next one starts;
//   - else the transmit side recovers as the queue UART does: the DMA set up again if it was reset
//     (DmaBase::setUpAgainIfReset), the channel aborted - a completion that raced the look and the abort is found by
//     the same check and counts as sent -, the UART's Startup steps again if it was reset, disabled or one of its
//     stream sides switched off (UARTEN, TXE/RXE, TXDMAE/RXDMAE), the run dropped and counted (txTimeouts()),
//     txFailed() until the next commit, the next run started.
// The completion callback ends a run only if the same check says it is done, under the same TxTag: a completion and a
// give-up never both end one run, and a completion that arrives after the watch ended its run (or one forced or raised
// by an abort elsewhere) finds the run in flight not done and leaves it. txTimeouts() counts every recovery, a sent
// run whose DMA had to be set up again (its interrupt enable lost) included.
//
// Either side's watch may find the UART reset, disabled or switched off and set it up again; the other side then
// counts what it lost (uartState_()). A UART reset the receive side's look repaired took the TX FIFO's characters and
// the shift register's with it: the run that was in flight then - it goes on and may end done - is not sent (txFailed,
// txTimeouts). A give-up that set up a UART whose receive side was down (reset, disabled, RXE or RXDMAE off) counts
// for the receive side as a restart (rxLost().restarts, receive()'s empty optional) at the reader's next look; the ring
// keeps its place, its channel only waited for the UART. A reset while no run is in flight can still take the last
// run's tail out of the FIFO (up to 32 characters and the shift register's): nothing says whether it held any.
//
// RX: the channel writes UARTDR into a ring (CTRL.RING_SEL = write, RING_SIZE = log2(Bytes)) one segment
// (Bytes / Segments) at a time; the completion callback re-arms the next segment only if it cannot overwrite
// unread data, else the ring stalls (counted) and the 32-entry FIFO - then RTS, when the config names it, or the
// overrun flag - takes over until readCommit() makes room. The read cursor comes from TRANS_COUNT, never from
// WRITE_ADDR: reading WRITE_ADDR of a wrapping transfer gives wrong values, TRANS_COUNT "always decrements
// linearly" and, like the addresses, counts completed writes (RP2040-E12, RP2040 data sheet md l.29519-29530; the
// RP2350 dropped the address adjustment for wrapping transfers altogether, 12.6.1 md l.53652).
// The UART's receive and receive-timeout interrupts stay off: a FIFO the DMA services must not be touched by
// software (RP2040 2.5.3.2, RP2350 12.6.4.2). Its error interrupts (overrun, break, parity, framing) count.
//
// A dead receive side is restarted by the reader's calls when they find nothing new (readReserve, available) and by
// rxLost(): the DMA held in reset (released) or reset (CTRL's RING_SIZE reads 0, DmaBase::channelState), the channel
// paused or aborted (EN clear, or idle with elements left), the UART reset, disabled or its receive side switched off.
// The channel starts again at the next whole segment with the ring emptied (UARTStreamCursor.hpp restart());
// rxLost().restarts counts it, rxLost().dropped what the reader had been shown and not committed - the characters the
// channel wrote after the reader's last look are gone with its count after a DMA reset, and the UART dropped what
// came while nobody took it (overruns, or held off by RTS). While the channel cannot say what it wrote (held in reset,
// or a count of 0 that is no segment's end) the reader is shown nothing new. A run the reader holds from before a
// restart is void: readCommit() returns false for it and commits nothing.
//
// Flow control: `rtsPinLocation` / `ctsPinLocation` in the config route the pins and set UARTCR.RTSEN/CTSEN; RTS
// is deasserted at the RX FIFO's watermark (RP2350 12.1.4.1 md l.47640-47646, RP2040 4.2.4), so a stalled ring
// holds the line instead of overrunning.
//
// Masked interrupts (a flash erase) delay only the segment ends: the channel fills the current segment by itself,
// so a ring covers (Segments - 1) * Bytes / Segments characters without a stall (`maskBudgetChars`).
#include "DMA.hpp"
#include "UART.hpp"
#include "UARTStreamCursor.hpp"
#include "kvasir/Atomic/BipBuffer.hpp"
#include "kvasir/Atomic/CriticalSection.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>

namespace Kvasir { namespace UART {

    struct TxNone {};

    struct RxNone {};

    /// The transmit side: `Bytes` of bip buffer on DMA channel `Ch`.
    template<DMA::DMAChannel Ch, std::size_t Bytes, DMA::DMAPriority Prio = DMA::DMAPriority::low>
    struct TxStream {
        static constexpr auto        channel  = Ch;
        static constexpr std::size_t bytes    = Bytes;
        static constexpr auto        priority = Prio;
    };

    /// The receive side: a ring of `Bytes` (a power of two, 64..32768) on DMA channel `Ch`, re-armed every
    /// Bytes / Segments. `Element = std::uint16_t` keeps UARTDR's error bits 8-11 (FE, PE, BE, OE) per character at
    /// twice the RAM.
    template<DMA::DMAChannel Ch,
             std::size_t     Bytes,
             std::size_t     Segments = 4,
             typename Element         = std::byte,
             DMA::DMAPriority Prio    = DMA::DMAPriority::low>
    struct RxRing {
        static_assert(std::has_single_bit(Bytes) && Bytes >= 64 && Bytes <= 32768,
                      "the ring is a power of two from 64 to 32768 bytes (CTRL.RING_SIZE 6..15)");
        static_assert(std::is_same_v<Element,
                                     std::byte>
                        || std::is_same_v<Element,
                                          std::uint16_t>,
                      "a ring holds bytes or UARTDR halfwords (data + error bits)");
        static_assert(Segments >= 2 && (Bytes / sizeof(Element)) % Segments == 0,
                      "two or more segments that divide the ring");
        static constexpr auto        channel  = Ch;
        static constexpr std::size_t bytes    = Bytes;
        static constexpr std::size_t segments = Segments;
        using element                         = Element;
        static constexpr auto priority        = Prio;
    };

    /// What the receive side lost or had to hold back, since start (wrapping counters).
    struct Lost {
        std::uint32_t
          overruns{};   // UART overrun interrupts: characters were dropped. Events, not characters: a FIFO
                        // that stays full drops on with no new interrupt
        std::uint32_t framing{};
        std::uint32_t parity{};
        std::uint32_t breaks{};
        std::uint32_t
          stalls{};   // segment ends the ring could not re-arm: the reader was a ring behind
        // the receive side found dead and set up again - by the reader's look, or by a transmit give-up that set the
        // UART up again (the header comment): what came meanwhile is lost
        std::uint32_t restarts{};
        std::uint32_t
          dropped{};   // elements a restart threw away that the reader had been shown and not committed
    };

    /// `Clock` (`now()`, a chrono clock: SystickClock, TimerClock) times the transmit side's stall timeout (header
    /// comment); a receive-only stream names it too.
    template<typename UartConfig_, typename Clock, typename Dma, typename Tx, typename Rx>
    struct UartStream : UartBase<UartConfig_> {
        using base       = UartBase<UartConfig_>;
        using Regs       = typename base::Regs;
        using UartConfig = typename base::UartConfig;
        using Config     = typename base::Config;

        static constexpr bool streamsTx = !std::is_same_v<Tx, TxNone>;
        static constexpr bool streamsRx = !std::is_same_v<Rx, RxNone>;
        static_assert(streamsTx || streamsRx,
                      "a stream with neither side");
        static_assert(
          !base::startDetached,
          "UartStream has no setAttached(): startDetached would leave its pins floating for good");
        static_assert(!streamsRx || base::hasRx,
                      "an RX ring needs rxPinLocation");
        // `||` does not keep Tx::channel from being named: a one-sided stream's TxNone / RxNone has none
        static constexpr bool distinctChannels = [] {
            if constexpr(streamsTx && streamsRx) {
                return Tx::channel != Rx::channel;
            } else {
                return true;
            }
        }();
        static_assert(distinctChannels,
                      "TX and RX on one DMA channel");

        /// How long the transmit side may make no progress, CTS asserted or no CTS, before its run in flight ends
        /// (the header comment): twice a character frame on the wire (start, data, parity and stop bits) plus 1 ms.
        static constexpr std::chrono::microseconds txStallTimeout{
          2ULL * base::bitsPerCharacter * 1'000'000ULL
            / static_cast<std::uint64_t>(UartConfig::baudRate)
          + 1'000ULL};

        struct TxTag {};

        struct RxTag {};

        // Startup

        template<typename T, typename R>
        struct ClaimsOf;

        template<typename R>
        struct ClaimsOf<TxNone, R> {
            using type = Kvasir::DMA::Claims<Dma, R::channel>;
        };

        template<typename T>
        struct ClaimsOf<T, RxNone> {
            using type = Kvasir::DMA::Claims<Dma, T::channel>;
        };

        template<typename T, typename R>
        struct ClaimsOf {
            using type = Kvasir::DMA::Claims<Dma, T::channel, R::channel>;
        };

        using Claims = brigand::append<typename base::Claims, typename ClaimsOf<Tx, Rx>::type>;

    private:
        static constexpr bool hasRts = requires { UartConfig_::rtsPinLocation; };
        static constexpr bool hasCts = requires { UartConfig_::ctsPinLocation; };

        template<typename C = UartConfig_>
        static constexpr auto flowPins() {
            if constexpr(requires { C::rtsPinLocation; } && requires { C::ctsPinLocation; }) {
                return Kvasir::MPL::list(
                  typename Config::template RtsPinConfig<std::decay_t<decltype(C::rtsPinLocation)>,
                                                         UartConfig::baudRate>{},
                  typename Config::template CtsPinConfig<
                    std::decay_t<decltype(C::ctsPinLocation)>>{});
            } else if constexpr(requires { C::rtsPinLocation; }) {
                return Kvasir::MPL::list(
                  typename Config::template RtsPinConfig<std::decay_t<decltype(C::rtsPinLocation)>,
                                                         UartConfig::baudRate>{});
            } else if constexpr(requires { C::ctsPinLocation; }) {
                return Kvasir::MPL::list(typename Config::template CtsPinConfig<
                                         std::decay_t<decltype(C::ctsPinLocation)>>{});
            } else {
                return brigand::list<>{};
            }
        }

        static constexpr auto flowControl() {
            if constexpr(hasRts && hasCts) {
                return Kvasir::MPL::list(set(Regs::UARTCR::rtsen), set(Regs::UARTCR::ctsen));
            } else if constexpr(hasRts) {
                return Kvasir::MPL::list(set(Regs::UARTCR::rtsen));
            } else if constexpr(hasCts) {
                return Kvasir::MPL::list(set(Regs::UARTCR::ctsen));
            } else {
                return brigand::list<>{};
            }
        }

        static constexpr auto dmaControl() {
            if constexpr(streamsTx && streamsRx) {
                return Kvasir::MPL::list(set(Regs::UARTDMACR::txdmae),
                                         set(Regs::UARTDMACR::rxdmae),
                                         flowControl());
            } else if constexpr(streamsTx) {
                return Kvasir::MPL::list(set(Regs::UARTDMACR::txdmae), flowControl());
            } else {
                return Kvasir::MPL::list(set(Regs::UARTDMACR::rxdmae), flowControl());
            }
        }

        // errors only: the DMA owns the RX FIFO
        static constexpr auto errorInterrupts() {
            if constexpr(streamsRx) {
                return Kvasir::MPL::list(set(Regs::UARTIMSC::oeim),
                                         set(Regs::UARTIMSC::beim),
                                         set(Regs::UARTIMSC::peim),
                                         set(Regs::UARTIMSC::feim));
            } else {
                return brigand::list<>{};
            }
        }

        static constexpr auto nvicEnable() {
            if constexpr(streamsRx) {
                return Kvasir::Nvic::makeEnable(typename base::InterruptIndexs{});
            } else {
                return brigand::list<>{};
            }
        }

    public:
        static constexpr auto initStepPinConfig
          = Kvasir::MPL::list(base::initStepPinConfig, flowPins());
        static constexpr auto initStepPeripheryConfig
          = base::peripheryConfig(dmaControl(), errorInterrupts());
        static constexpr auto initStepPeripheryEnable
          = Kvasir::MPL::list(set(Regs::UARTCR::uarten), nvicEnable());

        static void runtimeInit() {
            if constexpr(streamsRx) { startRxAt_(0); }
        }

    private:
        // RP2350: TRANS_COUNT 31:28 is MODE (0 here: start, retrigger and rearm write it so), 27:0 the count
        // (12.6.2.2.1)
        static constexpr std::uint32_t transCountMask = 0x0FFF'FFFFU;

        static bool dmaHeldInReset_() {
            return apply(read(Rp::Block<typename Dma::Regs>::reset)) != 0;
        }

        // The block as at startup again, after a reset (held or pulsed) or a disable under it: out of reset, the
        // format, DMA, flow control and error interrupts, UARTEN. The pins are IO_BANK0's and are not reset with it.
        static void setUpAgain_() {
            apply(base::powerClockEnable);
            Rp::Block<Regs>::waitResetDone();
            apply(initStepPeripheryConfig);
            apply(set(Regs::UARTCR::uarten));
        }

        /// What the UART needs, and what a set-up would make good (the header comment). `needsSetUp`: it was reset,
        /// disabled, or a side of it this stream uses switched off - the enables its Startup steps write: UARTCR.UARTEN
        /// (resets to 0: RP2040 datasheet Table 433 md l.20316, RP2350 Table 1036 md l.48119), TXE / RXE (reset 1, md
        /// l.20299-20300 / l.48103-48104), UARTDMACR.TXDMAE / RXDMAE (reset 0, md l.20495-20496 / l.48298-48299).
        /// `wasReset`: held in reset, or reset since its set-up - UARTIBRD reads 0, its reset value (RP2040 Table 430 md
        /// l.20225, RP2350 Table 1033 md l.48035), and a set-up writes a divisor of 1 or more: its FIFOs and shift
        /// registers are gone. A disabled UART "completes the current character before stopping" (UARTEN, above) and
        /// keeps its FIFOs. `rxDown`: the receive side took nothing meanwhile (reset, UARTEN, RXE or RXDMAE). Its
        /// registers are read only out of reset. Either side's watch reads it, under its own tag.
        struct UartState {
            bool needsSetUp;
            bool wasReset;
            bool rxDown;
        };

        static UartState uartState_() {
            if(apply(read(Rp::Block<Regs>::reset)) != 0) { return {true, true, streamsRx}; }
            auto const r     = apply(read(Regs::UARTCR::uarten,
                                          Regs::UARTCR::txe,
                                          Regs::UARTCR::rxe,
                                          Regs::UARTDMACR::txdmae,
                                          Regs::UARTDMACR::rxdmae,
                                          Regs::UARTIBRD::baud_divint));
            bool const on    = r.template get<0>();
            bool const reset = r.template get<5>() == 0;
            bool const txUp  = !streamsTx || (r.template get<1>() && r.template get<3>());
            bool const rxUp  = !streamsRx || (r.template get<2>() && r.template get<4>());
            return {!(on && txUp && rxUp), reset, streamsRx && (reset || !on || !rxUp)};
        }

        // The UART set up again by one side with a loss for the other (the header comment): each counter is written by
        // one side only, under its tag, before its setUpAgain_() (a completion the set-up lets through comes after it),
        // and read by the other.
        // the reader's look found it reset
        inline static std::atomic<std::uint32_t> resetsRepairedByRx_{};
        // a give-up found its receive side down
        inline static std::atomic<std::uint32_t> rxDownRepairedByTx_{};

        static void bump_(std::atomic<std::uint32_t>& counter) {
            counter.store(counter.load(std::memory_order_relaxed) + 1U, std::memory_order_seq_cst);
        }

        // transmit

        template<typename T>
        struct TxState {
            Kvasir::Atomic::BipBuffer<std::byte, T::bytes> buf{};
            std::size_t                                    inflight{};
            std::uint32_t runEnd{};   // READ_ADDR once the channel has read the whole run in flight
            bool          active{};
            // the watch (watchTx_), all under TxTag
            std::uint32_t              runs{};        // DMA sequences started
            std::uint32_t              seenRuns{};    // the run the watch saw last
            std::uint32_t              seenCount{};   // its TRANS_COUNT then
            typename Clock::time_point since{};       // when the watch last saw progress
            bool                       failed{};      // a run given up since the last commit
            std::uint32_t              timeouts{};    // recoveries since start
            std::uint32_t resetsSeen{};   // resetsRepairedByRx_ when the run in flight started
        };

        template<typename T>
            requires std::is_same_v<T, TxNone>
        struct TxState<T> {};

        // requires: an explicit instantiation of a receive-only stream (a conformance tree) reaches every member that
        // is not constrained away, and TxNone has no priority
        static constexpr auto txConfig()
            requires streamsTx
        {
            return Kvasir::DMA::ChannelConfig{.priority       = Tx::priority,
                                              .trigger        = base::txDmaTrigger,
                                              .size           = Kvasir::DMA::DMATransferSize::size8,
                                              .incrementRead  = true,
                                              .incrementWrite = false};
        }

        // under TxTag: the channel side of the bip buffer. The callback stays installed from run to run (retrigger):
        // it is published only into an empty slot - the first run, or after an abort emptied it - so a completion
        // being called on another core never meets its slot half written.
        static void startNextTx()
            requires streamsTx
        {
            auto const s = tx_.buf.readReserve();
            if(s.empty()) {
                tx_.active = false;
                return;
            }
            auto const from = reinterpret_cast<std::uint32_t>(s.data());
            tx_.inflight    = s.size();
            tx_.runEnd      = from + static_cast<std::uint32_t>(s.size());
            tx_.active      = true;
            tx_.runs        = tx_.runs + 1U;
            if constexpr(streamsRx) {
                tx_.resetsSeen = resetsRepairedByRx_.load(std::memory_order_acquire);
            }
            // the bytes the producer stored are seen by the DMA, another bus master, before the trigger
            std::atomic_thread_fence(std::memory_order_seq_cst);
            if(Dma::template hasCallback<Tx::channel>()) {
                Dma::template retrigger<Tx::channel, txConfig()>(base::txDmaTarget, from, s.size());
            } else {
                Dma::template start<Tx::channel, txConfig()>(base::txDmaTarget,
                                                             from,
                                                             s.size(),
                                                             &onTxDone);
            }
        }

        /// Under TxTag: every byte of the run in flight went to the UART - the channel's count at 0 (TRANS_COUNT
        /// "updating automatically each time a write transfer completes", RP2040 datasheet Table 123 md l.5390,
        /// RP2350 Table 1150 md l.54843) and its read address at the run's end (DmaBase::readAddress). A DMA reset
        /// clears both, and READ_ADDR 0 is no run's end; a DMA held in reset is not read.
        static bool txRunDone_()
            requires streamsTx
        {
            if(dmaHeldInReset_()) { return false; }
            return (Dma::template remaining<Tx::channel>() & transCountMask) == 0
                && Dma::template readAddress<Tx::channel>() == tx_.runEnd;
        }

        /// Under TxTag: the reader's look set up again a UART that was reset since the run in flight started (the header
        /// comment) - what the FIFO and the shift register held is gone, whether or not the run then ends done.
        static bool txRunHitByReset_()
            requires streamsTx
        {
            if constexpr(streamsRx) {
                return resetsRepairedByRx_.load(std::memory_order_acquire) != tx_.resetsSeen;
            } else {
                return false;
            }
        }

        // under TxTag: the run in flight is over, the next one starts
        static void endTxRun_()
            requires streamsTx
        {
            tx_.buf.readCommit(tx_.inflight);
            tx_.inflight = 0;
            startNextTx();
        }

        // The channel's completion. It ends the run in flight only if that run is done: a completion that arrives
        // after the watch ended the run itself (taken on the other core while the watch held TxTag), or one forced
        // or raised by an abort elsewhere, finds the run in flight not done - or no run - and leaves it. A done run a
        // UART reset hit is not sent.
        static void onTxDone()
            requires streamsTx
        {
            Kvasir::criticalSection<TxTag>([] {
                if(!tx_.active || !txRunDone_()) { return; }
                if(txRunHitByReset_()) {
                    tx_.failed   = true;
                    tx_.timeouts = tx_.timeouts + 1U;
                }
                endTxRun_();
            });
        }

        static void kick()
            requires streamsTx
        {
            Kvasir::criticalSection<TxTag>([] {
                if(!tx_.active) { startNextTx(); }
            });
        }

        /// Under TxTag: whether the transmit side moved since the last look, or may not move because the peer says
        /// so - CTS counts only while the UART could send and holds characters it may not send (the header comment).
        /// A block held in reset is not read: the DMA's moves nothing, the UART's says nothing about CTS.
        static bool txProgressed_()
            requires streamsTx
        {
            if(dmaHeldInReset_()) { return false; }
            std::uint32_t const count = Dma::template remaining<Tx::channel>() & transCountMask;
            if(tx_.runs != tx_.seenRuns || count != tx_.seenCount) {
                tx_.seenRuns  = tx_.runs;
                tx_.seenCount = count;
                return true;
            }
            if(Dma::template completionPending<Tx::channel>()) { return true; }
            if constexpr(hasCts) {
                if(apply(read(Rp::Block<Regs>::reset)) != 0) { return false; }
                auto const u            = apply(read(Regs::UARTCR::uarten,
                                                     Regs::UARTCR::txe,
                                                     Regs::UARTDMACR::txdmae,
                                                     Regs::UARTFR::txfe,
                                                     Regs::UARTFR::cts));
                bool const waitsForPeer = u.template get<0>() && u.template get<1>()
                                       && u.template get<2>() && !u.template get<3>();
                if(!waitsForPeer) { return false; }
                if(apply(read(Regs::UARTRIS::ctsrmis))) {
                    apply(set(Regs::UARTICR::ctsmic));
                    return true;
                }
                return !u.template get<4>();
            } else {
                return false;
            }
        }

        /// Under TxTag, no progress for txStallTimeout: the run in flight ends (the header comment). The DMA first - a
        /// block held in reset takes no write, the abort included -, a run that is not done aborted, then the UART
        /// (its registers are read only out of reset; a receive side it finds down counts for the reader), then the
        /// same done check once more: a completion that came in between the look and the abort is a run sent.
        static void endStalledTx_()
            requires streamsTx
        {
            bool const dmaRepaired = Dma::setUpAgainIfReset();
            if(!txRunDone_()) {
                static_cast<void>(
                  Dma::template abort<Tx::channel>());   // stopped, a raced completion dropped
            }
            auto const uart = uartState_();
            if(uart.needsSetUp) {
                if constexpr(streamsRx) {
                    if(uart.rxDown) { bump_(rxDownRepairedByTx_); }
                }
                setUpAgain_();
            }
            bool const sent = !uart.needsSetUp && !txRunHitByReset_() && txRunDone_();
            if(!sent) { tx_.failed = true; }
            if(!sent || dmaRepaired) { tx_.timeouts = tx_.timeouts + 1U; }
            endTxRun_();
        }

        // The timeout, and the tick in progress: a give-up never comes early.
        static constexpr auto stallTicks
          = std::chrono::ceil<typename Clock::duration>(txStallTimeout) +
            typename Clock::duration{1};

        // A commit through `commit` (it returns the bytes it took): a give-up reported before it is cleared by a commit
        // that takes bytes; one the watch finds during this call stays reported (txFailed()), and the space a lost run
        // held is free before the bytes go in.
        template<typename F>
        static std::size_t committing_(F commit)
            requires streamsTx
        {
            bool const earlier
              = Kvasir::criticalSection<TxTag>([] { return std::exchange(tx_.failed, false); });
            watchTx_();
            std::size_t const n = commit();
            if(n != 0) {
                kick();
            } else if(earlier) {
                Kvasir::criticalSection<TxTag>([] { tx_.failed = true; });
            }
            return n;
        }

        /// The transmit side's watch, from the producer's calls (the header comment). The clock is read in the section,
        /// next to the registers: read before it, an interrupt between the two that outlasts the bound dated the
        /// progress this look then saw to before the interrupt, and the next look - microseconds later, nothing moved
        /// yet - gave a healthy run up. An idle stream reads no clock.
        static void watchTx_()
            requires streamsTx
        {
            Kvasir::criticalSection<TxTag>([] {
                if(!tx_.active) { return; }
                auto const now = Clock::now();
                if(txProgressed_()) {
                    tx_.since = now;
                    return;
                }
                if(now - tx_.since < stallTicks) { return; }
                endStalledTx_();
            });
        }

    public:
        /// Exactly n contiguous bytes of the TX buffer, or an empty span (see BipBuffer::writeReserve).
        static std::span<std::byte> writeReserve(std::size_t n)
            requires streamsTx
        {
            watchTx_();
            return tx_.buf.writeReserve(n);
        }

        /// Send the first k bytes of the last reservation.
        static void writeCommit(std::size_t k)
            requires streamsTx
        {
            static_cast<void>(committing_([k] {
                tx_.buf.writeCommit(k);
                return k;
            }));
        }

        /// Copy as much of `in` as fits and send it; how many bytes were taken.
        static std::size_t write(std::span<std::byte const> in)
            requires streamsTx
        {
            return committing_([in] { return tx_.buf.write(in); });
        }

        /// Free TX buffer bytes now (not all contiguous).
        static std::size_t txFree()
            requires streamsTx
        {
            watchTx_();
            return Tx::bytes - tx_.buf.size();
        }

        /// Everything committed has left the pin.
        [[nodiscard]] static bool txIdle()
            requires streamsTx
        {
            watchTx_();
            bool const quiet
              = Kvasir::criticalSection<TxTag>([] { return !tx_.active && tx_.buf.empty(); });
            return quiet && !static_cast<bool>(get<0>(apply(read(Regs::UARTFR::busy))));
        }

        /// Recoveries of the transmit side since start (the header comment): runs given up, and sent runs whose DMA
        /// had to be set up again (a completion interrupt lost).
        [[nodiscard]] static std::uint32_t txTimeouts()
            requires streamsTx
        {
            return Kvasir::criticalSection<TxTag>([] { return tx_.timeouts; });
        }

        /// A run was given up since the last writeCommit() or write() that took bytes: what it held may not have
        /// left the pin.
        [[nodiscard]] static bool txFailed()
            requires streamsTx
        {
            watchTx_();
            return Kvasir::criticalSection<TxTag>([] { return tx_.failed; });
        }

        // receive

    private:
        template<typename R>
        struct RxParams {
            using Element                        = std::byte;
            static constexpr std::uint32_t ring  = 2;
            static constexpr std::uint32_t seg   = 1;
            static constexpr std::size_t   bytes = 2;
        };

        template<typename R>
            requires(!std::is_same_v<R, RxNone>)
        struct RxParams<R> {
            using Element                        = typename R::element;
            static constexpr std::size_t   bytes = R::bytes;
            static constexpr std::uint32_t ring
              = static_cast<std::uint32_t>(R::bytes / sizeof(Element));
            static constexpr std::uint32_t seg = static_cast<std::uint32_t>(ring / R::segments);
        };

    public:
        using Element                               = typename RxParams<Rx>::Element;
        static constexpr std::uint32_t ringElems    = RxParams<Rx>::ring;
        static constexpr std::uint32_t segmentElems = RxParams<Rx>::seg;
        /// Characters the ring takes while its DMA interrupt is held off (masked interrupts, a flash erase).
        static constexpr std::uint32_t maskBudgetChars = ringElems - segmentElems;

    private:
        // UARTDR's FE (8), PE (9), BE (10), OE (11) in a halfword element
        static constexpr std::uint16_t elementErrors = 0xFU << 8;

        // CTRL.RING_SIZE of the ring's channel: log2 of the ring in bytes (never 0: the ring is 64 bytes or more)
        static constexpr unsigned ringBits
          = static_cast<unsigned>(std::countr_zero(RxParams<Rx>::bytes));

        struct RxState {
            std::uint32_t              segDone{};     // ISR (under RxTag)
            bool                       stalled{};     // ISR / readCommit (under RxTag)
            std::atomic<std::uint32_t> readTotal{};   // consumer
            Lost                       lost{};        // UART ISR / DMA ISR
            std::uint32_t errorsSeen{};   // consumer: receive()'s view of the error counters
            std::uint32_t timeouts{};     // UART ISR: receive timeouts, only with UARTIMSC.RTIM set
            // the receive side's watch (watchRx_), consumer, under RxTag
            std::uint32_t shown{};   // writtenTotal_() at the reader's last look
            std::uint32_t epoch{};   // restarts so far: a run reserved before the last one is void
            std::uint32_t spanEpoch{};       // the epoch of the last readReserve()
            std::uint32_t txRepairsSeen{};   // rxDownRepairedByTx_ counted so far
        };

        // requires: as txConfig() for a transmit-only stream (RxNone)
        static constexpr auto rxConfig()
            requires streamsRx
        {
            return Kvasir::DMA::ChannelConfig{.priority = Rx::priority,
                                              .trigger  = base::rxDmaTrigger,
                                              .size     = sizeof(Element) == 1
                                                          ? Kvasir::DMA::DMATransferSize::size8
                                                          : Kvasir::DMA::DMATransferSize::size16,
                                              .incrementRead  = false,
                                              .incrementWrite = true,
                                              .ringBits       = ringBits,
                                              .ringOnWrite    = true};
        }

        // the channel writing segment `at / SegmentElems` from element `at` of the ring on (it wraps there by itself)
        static void startRxAt_(std::uint32_t at)
            requires streamsRx
        {
            Dma::template configure<Rx::channel, rxConfig()>(
              reinterpret_cast<std::uint32_t>(ring_.data() + (at & (ringElems - 1U))),
              base::rxDmaSource,
              segmentElems,
              &onRxSegmentDone);
            Dma::template trigger<Rx::channel>();
        }

        // Under RxTag, with the ring's count at 0 and a segment running: whether that 0 is the segment's end - the
        // channel still enabled and the ring's (RING_SIZE as configured). A DMA reset puts CTRL (EN, RING_SIZE) and
        // TRANS_COUNT to 0; an abort (DmaBase::abort clears EN first) "clears the transfer counter" in the middle of a
        // segment (RP2040 datasheet 2.5.5.3 md l.4917, RP2350 12.6.8.3 md l.54191). Either 0 says nothing about what
        // the channel wrote.
        static bool zeroIsSegmentEnd_()
            requires streamsRx
        {
            auto const ch = Dma::template channelState<Rx::channel>();
            return ch.enabled && ch.ringBits == ringBits;
        }

        // A segment's end: the count at 0 with the channel still the ring's and enabled (zeroIsSegmentEnd_), and a
        // segment running (a stalled ring has none). A completion raised by an abort elsewhere, or left from before a
        // restart, is none; the reader's watch takes care of a channel that died.
        static void onRxSegmentDone()
            requires streamsRx
        {
            Kvasir::criticalSection<RxTag>([] {
                if(rx_.stalled || dmaHeldInReset_()
                   || (Dma::template remaining<Rx::channel>() & transCountMask) != 0
                   || !zeroIsSegmentEnd_())
                {
                    return;
                }
                rx_.segDone = rx_.segDone + 1U;
                if(Detail::mayRearm(rx_.segDone,
                                    rx_.readTotal.load(std::memory_order_acquire),
                                    segmentElems,
                                    ringElems))
                {
                    Dma::template rearm<Rx::channel>(
                      segmentElems);   // goes on at the wrapped write address
                } else {
                    rx_.stalled = true;
                    ++rx_.lost.stalls;
                }
            });
        }

        // Under RxTag: elements the channel has written since start, or what the reader was last shown while the
        // channel cannot say (the DMA held in reset; a count of 0 that is no segment's end: CTRL reset with it, or
        // the channel aborted).
        static std::uint32_t writtenTotal_()
            requires streamsRx
        {
            if(dmaHeldInReset_()) { return rx_.shown; }
            std::uint32_t const tc = Dma::template remaining<Rx::channel>() & transCountMask;
            if(tc == 0 && !rx_.stalled && !zeroIsSegmentEnd_()) { return rx_.shown; }
            rx_.shown = Detail::writtenTotal(rx_.segDone, rx_.stalled, tc, segmentElems);
            return rx_.shown;
        }

        // Under RxTag: the channel set up again at the next whole segment, the ring emptied (the header comment).
        static void restartRx_()
            requires streamsRx
        {
            std::uint32_t const read = rx_.readTotal.load(std::memory_order_relaxed);
            rx_.lost.dropped += Detail::dropped(rx_.shown, read, ringElems);
            ++rx_.lost.restarts;
            ++rx_.epoch;
            auto const r = Detail::restart(rx_.segDone, rx_.stalled, segmentElems);
            rx_.segDone  = r.segDone;
            rx_.stalled  = false;
            rx_.shown    = r.readTotal;
            rx_.readTotal.store(r.readTotal, std::memory_order_release);
            // a paused channel stopped, a completion it raised dropped; its callback is installed again below
            static_cast<void>(Dma::template abort<Rx::channel>());
            startRxAt_(r.readTotal);
        }

        /// Under RxTag, from the reader's calls (the header comment): the DMA out of reset and its interrupt enable as
        /// at startup (DmaBase::setUpAgainIfReset - a segment end it missed comes now) when the DMA is held in reset or
        /// the ring's own bit in INTEn is gone (a pulsed reset clears it) - not on every look: the repair would also
        /// write the bits of channels an abort elsewhere clears on purpose, if abort() did not hold them (RP2040-E13,
        /// DmaBase::abort). The UART set up again if it needs it (a reset it finds counts for the transmit side), the
        /// give-ups that found the receive side down counted as restarts, the ring's channel restarted if the receive
        /// side was down or the channel is dead: reset (RING_SIZE 0), paused or aborted (EN clear), or idle with
        /// elements left while no stall holds it.
        static void watchRx_()
            requires streamsRx
        {
            if(dmaHeldInReset_() || !Dma::template completionEnabled<Rx::channel>()) {
                static_cast<void>(Dma::setUpAgainIfReset());
            }
            auto const uart = uartState_();
            if(uart.needsSetUp) {
                if constexpr(streamsTx) {
                    if(uart.wasReset) { bump_(resetsRepairedByRx_); }
                }
                setUpAgain_();
            }
            if constexpr(streamsTx) {
                std::uint32_t const byTx = rxDownRepairedByTx_.load(std::memory_order_acquire);
                rx_.lost.restarts += byTx - rx_.txRepairsSeen;
                rx_.txRepairsSeen = byTx;
            }
            bool       dead = uart.rxDown;
            auto const ch   = Dma::template channelState<Rx::channel>();
            if(ch.ringBits != ringBits || !ch.enabled
               || (!ch.busy && !rx_.stalled
                   && (Dma::template remaining<Rx::channel>() & transCountMask) != 0))
            {
                dead = true;
            }
            if(dead) { restartRx_(); }
        }

        static void onUartIsr()
            requires streamsRx
        {
            auto const mis = apply(read(Regs::UARTMIS::oemis,
                                        Regs::UARTMIS::bemis,
                                        Regs::UARTMIS::pemis,
                                        Regs::UARTMIS::femis,
                                        Regs::UARTMIS::rtmis));
            auto&      l   = rx_.lost;
            if(mis.template get<0>()) {
                apply(set(Regs::UARTICR::oeic));
                ++l.overruns;
            }
            if(mis.template get<1>()) {
                apply(set(Regs::UARTICR::beic));
                ++l.breaks;
            }
            if(mis.template get<2>()) {
                apply(set(Regs::UARTICR::peic));
                ++l.parity;
            }
            if(mis.template get<3>()) {
                apply(set(Regs::UARTICR::feic));
                ++l.framing;
            }
            if(
              mis.template get<
                4>()) {   // off unless a diagnostic sets RTIM: clear and count, never read the FIFO
                apply(set(Regs::UARTICR::rtic));
                ++rx_.timeouts;
            }
        }

        // onUartIsr exists only with an RX side: its address is named in the taken branch alone
        static constexpr auto isrs() {
            if constexpr(streamsRx) {
                return Nvic::IsrsFor<std::addressof(onUartIsr), typename base::InterruptIndexs>{};
            } else {
                return brigand::list<>{};
            }
        }

        static Lost lostSnapshot_()
            requires streamsRx
        {
            return Kvasir::criticalSection<RxTag>([] { return rx_.lost; });
        }

    public:
        using Isr = decltype(isrs());

        /// The oldest contiguous run of received elements, in the ring itself (empty when there is none). With
        /// `std::uint16_t` elements, bits 7:0 are the character and 11:8 its OE/BE/PE/FE flags. Finding none, it
        /// looks whether the receive side died (the header comment).
        static std::span<Element const> readReserve()
            requires streamsRx
        {
            auto const run = Kvasir::criticalSection<RxTag>([] {
                rx_.spanEpoch = rx_.epoch;
                auto const w  = writtenTotal_();
                auto const r
                  = Detail::readable(w, rx_.readTotal.load(std::memory_order_relaxed), ringElems);
                if(r.len == 0) { watchRx_(); }
                return r;
            });
            // the DMA's writes before the count that says they are done
            std::atomic_thread_fence(std::memory_order_acquire);
            return {ring_.data() + run.at, run.len};
        }

        /// Free the first k elements of the last readReserve(); restarts a stalled ring once it has room. False, and
        /// nothing freed, for a run reserved before a restart emptied the ring (rxLost().dropped counts it): what it
        /// showed may have been overwritten while it was read.
        static bool readCommit(std::size_t k)
            requires streamsRx
        {
            return Kvasir::criticalSection<RxTag>([k] {
                if(rx_.spanEpoch != rx_.epoch) { return false; }
                std::uint32_t const r
                  = rx_.readTotal.load(std::memory_order_relaxed) + static_cast<std::uint32_t>(k);
                rx_.readTotal.store(r, std::memory_order_release);
                if(rx_.stalled && Detail::mayRearm(rx_.segDone, r, segmentElems, ringElems)) {
                    rx_.stalled = false;
                    Dma::template rearm<Rx::channel>(segmentElems);
                }
                return true;
            });
        }

        /// Received elements not yet committed. Finding none, it looks whether the receive side died.
        static std::size_t available()
            requires streamsRx
        {
            return Kvasir::criticalSection<RxTag>([] {
                std::uint32_t const n
                  = writtenTotal_() - rx_.readTotal.load(std::memory_order_relaxed);
                if(n == 0) { watchRx_(); }
                return n;
            });
        }

        /// A snapshot of the loss counters, after a look whether the receive side died (a run the reader holds is void
        /// after a restart this look makes).
        static Lost rxLost()
            requires streamsRx
        {
            Kvasir::criticalSection<RxTag>([] { watchRx_(); });
            return lostSnapshot_();
        }

        /// Receive-timeout interrupts counted since start. The stream keeps UARTIMSC.RTIM off; a diagnostic that
        /// sets it (`apply(set(Uart::Regs::UARTIMSC::rtim))`) learns whether the FIFO ever sat non-empty for 32 bit
        /// periods while the DMA drained it.
        static std::uint32_t receiveTimeouts()
            requires streamsRx
        {
            return Kvasir::criticalSection<RxTag>([] { return rx_.timeouts; });
        }

        /// The queue UART's receive(): the next byte, or one empty optional where errors or a restart of the receive
        /// side were counted since the last call (where the reader noticed, not where the character was) - Gnss::Nmea
        /// / Ubx read this. With halfword elements a character carrying an error flag is an empty optional itself.
        static bool receive(std::optional<std::byte>& out)
            requires streamsRx
        {
            auto const          l      = lostSnapshot_();
            std::uint32_t const errors = std::is_same_v<Element, std::byte>
                                         ? l.overruns + l.framing + l.parity + l.breaks + l.restarts
                                         : l.overruns + l.restarts;
            if(errors != rx_.errorsSeen) {
                rx_.errorsSeen = errors;
                out            = std::nullopt;
                return true;
            }
            auto const s = readReserve();
            if(s.empty()) { return false; }
            if constexpr(std::is_same_v<Element, std::byte>) {
                out = s.front();
            } else if((s.front() & elementErrors) != 0) {
                out = std::nullopt;
            } else {
                out = static_cast<std::byte>(s.front() & 0xFFU);
            }
            static_cast<void>(readCommit(1));
            return true;
        }

    private:
        inline static TxState<Tx> tx_{};
        inline static RxState     rx_{};
        alignas(RxParams<Rx>::bytes) inline static std::array<Element,
                                                              streamsRx ? ringElems : 0> ring_{};
    };
}}   // namespace Kvasir::UART
