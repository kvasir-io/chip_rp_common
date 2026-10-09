#pragma once
// A UART streamed by DMA both ways. The queue UART (UART.hpp, UartBehavior) stays the default; a port names this
// instead in its Startup list.
//
//     using Uart = Kvasir::UART::UartStream<Cfg, Dma, Kvasir::UART::TxStream<Dma::Channel::ch2, 1024>,
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
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>

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
    };

    template<typename UartConfig_, typename Dma, typename Tx, typename Rx>
    struct UartStream : UartBase<UartConfig_> {
        using base       = UartBase<UartConfig_>;
        using Regs       = typename base::Regs;
        using UartConfig = typename base::UartConfig;
        using Config     = typename base::Config;

        static constexpr bool HasTx = !std::is_same_v<Tx, TxNone>;
        static constexpr bool HasRx = !std::is_same_v<Rx, RxNone>;
        static_assert(HasTx || HasRx,
                      "a stream with neither side");
        static_assert(
          !base::StartDetached,
          "UartStream has no setAttached(): startDetached would leave its pins floating for good");
        static_assert(!HasRx || base::hasRx,
                      "an RX ring needs rxPinLocation");
        static_assert(!(HasTx && HasRx) || Tx::channel != Rx::channel,
                      "TX and RX on one DMA channel");

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
        static constexpr bool HasRts = requires { UartConfig_::rtsPinLocation; };
        static constexpr bool HasCts = requires { UartConfig_::ctsPinLocation; };

        template<typename C = UartConfig_>
        static constexpr auto flowPins() {
            if constexpr(requires { C::rtsPinLocation; } && requires { C::ctsPinLocation; }) {
                return Kvasir::MPL::list(typename Config::template GetRtsPinConfig<
                                           std::decay_t<decltype(C::rtsPinLocation)>,
                                           UartConfig::baudRate>::pinConfig{},
                                         typename Config::template GetCtsPinConfig<
                                           std::decay_t<decltype(C::ctsPinLocation)>>::pinConfig{});
            } else if constexpr(requires { C::rtsPinLocation; }) {
                return Kvasir::MPL::list(typename Config::template GetRtsPinConfig<
                                         std::decay_t<decltype(C::rtsPinLocation)>,
                                         UartConfig::baudRate>::pinConfig{});
            } else if constexpr(requires { C::ctsPinLocation; }) {
                return Kvasir::MPL::list(typename Config::template GetCtsPinConfig<
                                         std::decay_t<decltype(C::ctsPinLocation)>>::pinConfig{});
            } else {
                return brigand::list<>{};
            }
        }

        static constexpr auto flowControl() {
            if constexpr(HasRts && HasCts) {
                return Kvasir::MPL::list(set(Regs::UARTCR::rtsen), set(Regs::UARTCR::ctsen));
            } else if constexpr(HasRts) {
                return Kvasir::MPL::list(set(Regs::UARTCR::rtsen));
            } else if constexpr(HasCts) {
                return Kvasir::MPL::list(set(Regs::UARTCR::ctsen));
            } else {
                return brigand::list<>{};
            }
        }

        static constexpr auto dmaControl() {
            if constexpr(HasTx && HasRx) {
                return Kvasir::MPL::list(set(Regs::UARTDMACR::txdmae),
                                         set(Regs::UARTDMACR::rxdmae),
                                         flowControl());
            } else if constexpr(HasTx) {
                return Kvasir::MPL::list(set(Regs::UARTDMACR::txdmae), flowControl());
            } else {
                return Kvasir::MPL::list(set(Regs::UARTDMACR::rxdmae), flowControl());
            }
        }

        // errors only: the DMA owns the RX FIFO
        static constexpr auto errorInterrupts() {
            if constexpr(HasRx) {
                return Kvasir::MPL::list(set(Regs::UARTIMSC::oeim),
                                         set(Regs::UARTIMSC::beim),
                                         set(Regs::UARTIMSC::peim),
                                         set(Regs::UARTIMSC::feim));
            } else {
                return brigand::list<>{};
            }
        }

        static constexpr auto nvicEnable() {
            if constexpr(HasRx) {
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
            if constexpr(HasRx) { startRx(); }
        }

        // transmit

    private:
        template<typename T>
        struct TxState {
            Kvasir::Atomic::BipBuffer<std::byte, T::bytes> buf{};
            std::size_t                                    inflight{};
            bool                                           active{};
        };

        template<typename T>
            requires std::is_same_v<T, TxNone>
        struct TxState<T> {};

        static constexpr auto txConfig() {
            return Kvasir::DMA::ChannelConfig{.priority       = Tx::priority,
                                              .trigger        = base::TxDmaTrigger,
                                              .size           = Kvasir::DMA::DMATransferSize::_8,
                                              .incrementRead  = true,
                                              .incrementWrite = false};
        }

        // under TxTag: the channel side of the bip buffer
        static void startNextTx()
            requires HasTx
        {
            auto const s = tx_.buf.readReserve();
            if(s.empty()) {
                tx_.active = false;
                return;
            }
            tx_.inflight = s.size();
            tx_.active   = true;
            // the bytes the producer stored are seen by the DMA, another bus master, before the trigger
            std::atomic_thread_fence(std::memory_order_seq_cst);
            Dma::template configure<Tx::channel, txConfig()>(
              base::TxDmaTarget,
              reinterpret_cast<std::uint32_t>(s.data()),
              s.size(),
              &onTxDone);
            Dma::template trigger<Tx::channel>();
        }

        static void onTxDone()
            requires HasTx
        {
            Kvasir::criticalSection<TxTag>([] {
                tx_.buf.readCommit(tx_.inflight);
                tx_.inflight = 0;
                startNextTx();
            });
        }

        static void kick()
            requires HasTx
        {
            Kvasir::criticalSection<TxTag>([] {
                if(!tx_.active) { startNextTx(); }
            });
        }

    public:
        /// Exactly n contiguous bytes of the TX buffer, or an empty span (see BipBuffer::writeReserve).
        static std::span<std::byte> writeReserve(std::size_t n)
            requires HasTx
        {
            return tx_.buf.writeReserve(n);
        }

        /// Send the first k bytes of the last reservation.
        static void writeCommit(std::size_t k)
            requires HasTx
        {
            tx_.buf.writeCommit(k);
            kick();
        }

        /// Copy as much of `in` as fits and send it; how many bytes were taken.
        static std::size_t write(std::span<std::byte const> in)
            requires HasTx
        {
            auto const n = tx_.buf.write(in);
            if(n != 0) { kick(); }
            return n;
        }

        /// Free TX buffer bytes now (not all contiguous).
        static std::size_t txFree()
            requires HasTx
        {
            return Tx::bytes - tx_.buf.size();
        }

        /// Everything committed has left the pin.
        [[nodiscard]] static bool txIdle()
            requires HasTx
        {
            bool const quiet
              = Kvasir::criticalSection<TxTag>([] { return !tx_.active && tx_.buf.empty(); });
            return quiet && !static_cast<bool>(get<0>(apply(read(Regs::UARTFR::busy))));
        }

        // receive

    private:
        template<typename R>
        struct RxParams {
            using Element                        = std::byte;
            static constexpr std::uint32_t Ring  = 2;
            static constexpr std::uint32_t Seg   = 1;
            static constexpr std::size_t   Bytes = 2;
        };

        template<typename R>
            requires(!std::is_same_v<R, RxNone>)
        struct RxParams<R> {
            using Element                        = typename R::element;
            static constexpr std::size_t   Bytes = R::bytes;
            static constexpr std::uint32_t Ring
              = static_cast<std::uint32_t>(R::bytes / sizeof(Element));
            static constexpr std::uint32_t Seg = static_cast<std::uint32_t>(Ring / R::segments);
        };

    public:
        using Element                               = typename RxParams<Rx>::Element;
        static constexpr std::uint32_t RingElems    = RxParams<Rx>::Ring;
        static constexpr std::uint32_t SegmentElems = RxParams<Rx>::Seg;
        /// Characters the ring takes while its DMA interrupt is held off (masked interrupts, a flash erase).
        static constexpr std::uint32_t maskBudgetChars = RingElems - SegmentElems;

    private:
        // UARTDR's FE (8), PE (9), BE (10), OE (11) in a halfword element
        static constexpr std::uint16_t ElementErrors = 0xFU << 8;

        struct RxState {
            std::uint32_t              segDone{};     // ISR (under RxTag)
            bool                       stalled{};     // ISR / readCommit (under RxTag)
            std::atomic<std::uint32_t> readTotal{};   // consumer
            Lost                       lost{};        // UART ISR / DMA ISR
            std::uint32_t errorsSeen{};   // consumer: receive()'s view of the error counters
            std::uint32_t timeouts{};     // UART ISR: receive timeouts, only with UARTIMSC.RTIM set
        };

        static constexpr auto rxConfig() {
            return Kvasir::DMA::ChannelConfig{
              .priority       = Rx::priority,
              .trigger        = base::RxDmaTrigger,
              .size           = sizeof(Element) == 1 ? Kvasir::DMA::DMATransferSize::_8
                                                     : Kvasir::DMA::DMATransferSize::_16,
              .incrementRead  = false,
              .incrementWrite = true,
              .ringBits       = static_cast<unsigned>(std::countr_zero(RxParams<Rx>::Bytes)),
              .ringOnWrite    = true};
        }

        static void startRx()
            requires HasRx
        {
            Dma::template configure<Rx::channel, rxConfig()>(
              reinterpret_cast<std::uint32_t>(ring_.data()),
              base::RxDmaSource,
              SegmentElems,
              &onRxSegmentDone);
            Dma::template trigger<Rx::channel>();
        }

        static void onRxSegmentDone()
            requires HasRx
        {
            Kvasir::criticalSection<RxTag>([] {
                rx_.segDone = rx_.segDone + 1U;
                if(Detail::mayRearm(rx_.segDone,
                                    rx_.readTotal.load(std::memory_order_acquire),
                                    SegmentElems,
                                    RingElems))
                {
                    Dma::template rearm<Rx::channel>(
                      SegmentElems);   // goes on at the wrapped write address
                } else {
                    rx_.stalled = true;
                    ++rx_.lost.stalls;
                }
            });
        }

        static std::uint32_t writtenTotal()
            requires HasRx
        {
            return Kvasir::criticalSection<RxTag>([] {
                // RP2350: TRANS_COUNT 31:28 is MODE (0 here, rearm writes it so), 27:0 the count (12.6.2.2.1)
                std::uint32_t const tc = Dma::template remaining<Rx::channel>() & 0x0FFF'FFFFU;
                return Detail::writtenTotal(rx_.segDone, rx_.stalled, tc, SegmentElems);
            });
        }

        static void onUartIsr()
            requires HasRx
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

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            if constexpr(HasRx) {
                return brigand::list<
                  Kvasir::Nvic::Isr<std::addressof(onUartIsr), Nvic::Index<Ts::value>>...>{};
            } else {
                return brigand::list<>{};
            }
        }

    public:
        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));

        /// The oldest contiguous run of received elements, in the ring itself (empty when there is none). With
        /// `std::uint16_t` elements, bits 7:0 are the character and 11:8 its OE/BE/PE/FE flags.
        static std::span<Element const> readReserve()
            requires HasRx
        {
            std::uint32_t const w = writtenTotal();
            // the DMA's writes before the count that says they are done
            std::atomic_thread_fence(std::memory_order_acquire);
            auto const run
              = Detail::readable(w, rx_.readTotal.load(std::memory_order_relaxed), RingElems);
            return {ring_.data() + run.at, run.len};
        }

        /// Free the first k elements of the last readReserve(); restarts a stalled ring once it has room.
        static void readCommit(std::size_t k)
            requires HasRx
        {
            std::uint32_t const r
              = rx_.readTotal.load(std::memory_order_relaxed) + static_cast<std::uint32_t>(k);
            rx_.readTotal.store(r, std::memory_order_release);
            Kvasir::criticalSection<RxTag>([r] {
                if(rx_.stalled && Detail::mayRearm(rx_.segDone, r, SegmentElems, RingElems)) {
                    rx_.stalled = false;
                    Dma::template rearm<Rx::channel>(SegmentElems);
                }
            });
        }

        /// Received elements not yet committed.
        static std::size_t available()
            requires HasRx
        {
            return writtenTotal() - rx_.readTotal.load(std::memory_order_relaxed);
        }

        /// A snapshot of the loss counters.
        static Lost rxLost()
            requires HasRx
        {
            return Kvasir::criticalSection<RxTag>([] {
                return Lost{rx_.lost.overruns,
                            rx_.lost.framing,
                            rx_.lost.parity,
                            rx_.lost.breaks,
                            rx_.lost.stalls};
            });
        }

        /// Receive-timeout interrupts counted since start. The stream keeps UARTIMSC.RTIM off; a diagnostic that
        /// sets it (`apply(set(Uart::Regs::UARTIMSC::rtim))`) learns whether the FIFO ever sat non-empty for 32 bit
        /// periods while the DMA drained it.
        static std::uint32_t receiveTimeouts()
            requires HasRx
        {
            return Kvasir::criticalSection<RxTag>([] { return rx_.timeouts; });
        }

        /// The queue UART's receive(): the next byte, or one empty optional where errors were counted since the
        /// last call (where the reader noticed, not where the character was) - Gnss::Nmea / Ubx read this. With
        /// halfword elements a character carrying an error flag is an empty optional itself.
        static bool receive(std::optional<std::byte>& out)
            requires HasRx
        {
            auto const          l      = rxLost();
            std::uint32_t const errors = std::is_same_v<Element, std::byte>
                                         ? l.overruns + l.framing + l.parity + l.breaks
                                         : l.overruns;
            if(errors != rx_.errorsSeen) {
                rx_.errorsSeen = errors;
                out            = std::nullopt;
                return true;
            }
            auto const s = readReserve();
            if(s.empty()) { return false; }
            if constexpr(std::is_same_v<Element, std::byte>) {
                out = s.front();
            } else if((s.front() & ElementErrors) != 0) {
                out = std::nullopt;
            } else {
                out = static_cast<std::byte>(s.front() & 0xFFU);
            }
            readCommit(1);
            return true;
        }

    private:
        inline static TxState<Tx> tx_{};
        inline static RxState     rx_{};
        alignas(RxParams<Rx>::Bytes) inline static std::array<Element,
                                                              HasRx ? RingElems : 0> ring_{};
    };
}}   // namespace Kvasir::UART
