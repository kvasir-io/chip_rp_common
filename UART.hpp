#pragma once

#include "Clocks.hpp"
#include "DMA.hpp"
#include "Io.hpp"
#include "PinConfig.hpp"
#include "WaitBounds.hpp"
#include "chip/Blocks.hpp"   // the chip's block table (Rp::Block)
#include "core/Nvic.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Hal/Clocks.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Hal/Uart.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Log/LogLimited.hpp"
#include "kvasir/Log/RateLimiter.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Prescaler.hpp"
#include "peripherals/UART.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <chrono>

namespace Kvasir { namespace UART {

    // The interrupt trigger point of a FIFO (UARTIFLS), as a fraction of its 32 entries:
    // the receive interrupt fires when the RX FIFO holds at least this much, the transmit
    // one when the TX FIFO holds at most this much.
    enum class FifoLevel : std::uint32_t {
        ratio1to8 = 0,
        ratio1to4 = 1,
        ratio1to2 = 2,
        ratio3to4 = 3,
        ratio7to8 = 4,
    };

    namespace Detail {

        // Use chip-agnostic pin configuration

        // The PL011 divisor UARTCLK / (16 x baud) as a 16-bit integer and a 6-bit fraction
        // (RP2350 data sheet 12.1.3.2.1, UARTIBRD/UARTFBRD Tables 1033/1034). One integer in
        // 1/64 steps, so a fraction that rounds to 64/64 carries into the integer part.
        static constexpr Prescaler::Fixed pl011Divider{.intMin   = 1,
                                                       .intMax   = 65535,
                                                       .fracBits = 6,
                                                       .scale    = 16};

        static constexpr auto baudDivider(std::uint32_t fClockSpeed,
                                          std::uint32_t fBaud) {
            return Prescaler::fromFixedPoint(fClockSpeed, fBaud, pl011Divider);
        }

        static constexpr std::pair<std::uint16_t,
                                   std::uint8_t>
        calcBaudRegs(std::uint32_t fClockSpeed,
                     std::uint32_t fBaud) {
            auto const d = baudDivider(fClockSpeed, fBaud).setting;
            return {static_cast<std::uint16_t>(d.integer), static_cast<std::uint8_t>(d.fraction)};
        }

        template<unsigned Instance>
        struct Config {
            using Regs = Kvasir::Peripheral::UART::Registers<Instance>;

            // TX/RTS are push-pull outputs: 4 mA with slow slew (the pad default) is
            // plenty up to ~1 MBaud; faster links get 8 mA with fast slew.
            static constexpr Io::DriveStrength uartDrive(std::uint32_t fBaud) {
                return fBaud <= 1'000'000 ? Io::DriveStrength::mA4 : Io::DriveStrength::mA8;
            }

            static constexpr bool uartSlewFast(std::uint32_t fBaud) { return fBaud > 1'000'000; }

            // CTS (an input) and RTS (an output, the TX drive): F2, or nothing for an Io::NotUsed<>
            template<typename CTSPIN>
            using CtsPinConfig = typename Io::Detail::
              PinConfigFor<CTSPIN, PinConfig::Function::uartCts, Instance>::type;

            template<typename RTSPIN, std::uint32_t FBaud>
            using RtsPinConfig = typename Io::Detail::PinConfigFor<RTSPIN,
                                                                   PinConfig::Function::uartRts,
                                                                   Instance,
                                                                   uartDrive(FBaud),
                                                                   uartSlewFast(FBaud)>::type;

            template<typename RXPIN>
            struct GetRxPinConfig;

            template<typename Dummy>
            struct GetRxPinConfig<Io::NotUsed<Dummy>> {
                using enable    = decltype(clear(Regs::UARTCR::rxe));
                using PinConfig = brigand::list<>;
                using interrupt = brigand::list<>;
                template<typename InterruptIndex>
                using InterruptEnable = brigand::list<>;
            };

            template<int Port, int Pin>
            struct GetRxPinConfig<Kvasir::Io::PinLocation<Port, Pin>> {
                using enable    = decltype(set(Regs::UARTCR::rxe));
                using PinConfig = typename Io::Detail::PinConfigFor<Io::PinLocation<Port, Pin>,
                                                                    PinConfig::Function::uartRx,
                                                                    Instance>::type;
                using interrupt = brigand::list<decltype(set(Regs::UARTIMSC::rxim)),
                                                decltype(set(Regs::UARTIMSC::oeim))>;
                template<typename InterruptIndex>
                using InterruptEnable = decltype(Kvasir::Nvic::makeEnable(InterruptIndex{}));
            };

            template<typename TXPIN, std::uint32_t FBaud>
            struct GetTxPinConfig;

            template<typename Dummy, std::uint32_t FBaud>
            struct GetTxPinConfig<Io::NotUsed<Dummy>, FBaud> {
                using enable    = decltype(set(Regs::UARTCR::txe));
                using PinConfig = brigand::list<>;
            };

            template<int Port, int Pin, std::uint32_t FBaud>
            struct GetTxPinConfig<Kvasir::Io::PinLocation<Port, Pin>, FBaud> {
                using enable    = decltype(set(Regs::UARTCR::txe));
                using PinConfig = typename Io::Detail::PinConfigFor<Io::PinLocation<Port, Pin>,
                                                                    PinConfig::Function::uartTx,
                                                                    Instance,
                                                                    uartDrive(FBaud),
                                                                    uartSlewFast(FBaud)>::type;
            };

            template<Hal::Uart::DataBits Dbits>
            struct GetDataBitConfig {
                // UARTLCR_H.WLEN is 2 bits, 5 to 8 data bits (RP2040 datasheet table 432, RP2350 table 1035)
                static_assert(Dbits != Hal::Uart::DataBits::bits9,
                              "the PL011 has no 9-bit frames (UARTLCR_H.WLEN)");
                static constexpr auto config_ = []() {
                    if constexpr(Dbits == Hal::Uart::DataBits::bits5) {
                        return write(Regs::UARTLCR_H::wlen, Register::value<0>());
                    } else if constexpr(Dbits == Hal::Uart::DataBits::bits6) {
                        return write(Regs::UARTLCR_H::wlen, Register::value<1>());
                    } else if constexpr(Dbits == Hal::Uart::DataBits::bits7) {
                        return write(Regs::UARTLCR_H::wlen, Register::value<2>());
                    } else if constexpr(Dbits == Hal::Uart::DataBits::bits8) {
                        return write(Regs::UARTLCR_H::wlen, Register::value<3>());
                    }
                }();
                using config = decltype(config_);
            };

            template<Hal::Uart::StopBits Sbits>
            struct GetStopBitConfig {
                static constexpr auto config_ = []() {
                    if constexpr(Sbits == Hal::Uart::StopBits::stop1) {
                        return clear(Regs::UARTLCR_H::stp2);
                    } else if constexpr(Sbits == Hal::Uart::StopBits::stop2) {
                        return set(Regs::UARTLCR_H::stp2);
                    }
                }();
                using config = decltype(config_);
            };

            template<Hal::Uart::Parity Pa>
            struct GetParityConfig {
                static constexpr auto config_ = []() {
                    if constexpr(Pa == Hal::Uart::Parity::none) {
                        return brigand::list<decltype(clear(Regs::UARTLCR_H::pen))>{};
                    } else if constexpr(Pa == Hal::Uart::Parity::odd) {
                        return brigand::list<decltype(set(Regs::UARTLCR_H::pen)),
                                             decltype(clear(Regs::UARTLCR_H::eps))>{};
                    } else if constexpr(Pa == Hal::Uart::Parity::even) {
                        return brigand::list<decltype(set(Regs::UARTLCR_H::pen)),
                                             decltype(set(Regs::UARTLCR_H::eps))>{};
                    }
                }();
                using config = decltype(config_);
            };

            template<std::uint32_t FClockSpeed, std::uint32_t FBaud>
            struct GetBaudConfig {
                static constexpr auto config_ = []() {
                    constexpr auto baudRegs = calcBaudRegs(FClockSpeed, FBaud);
                    return list(
                      write(Regs::UARTIBRD::baud_divint, Register::value<std::get<0>(baudRegs)>()),
                      write(Regs::UARTFBRD::baud_divfrac,
                            Register::value<std::get<1>(baudRegs)>()));
                }();
                using config = decltype(config_);
            };
        };

    }   // namespace Detail

    template<typename UartConfig_>
    struct UartBase {
        struct UartConfig : UartConfig_ {
            static constexpr auto userConfigOverride
              = KVASIR_CONFIG_OR(UartConfig_, userConfigOverride, brigand::list<>{});

            static constexpr auto maxBaudRateError
              = KVASIR_CONFIG_OR(UartConfig_, maxBaudRateError, std::ratio<5, 1000>{});

            // The PL011's 32-entry FIFOs (UARTLCR_H.FEN), on by default: the receive interrupt
            // fires at rxFifoLevel and the receive-timeout interrupt (32 bit periods of silence)
            // hands over what is left below it, so a burst costs a few interrupts instead of one
            // per byte. `fifo = false` makes the holding registers one deep (RP2350 data sheet
            // 12.1.2.4/12.1.2.5): every received byte is an interrupt, with no timeout latency.
            // The DMA transmit path pulls from the TX FIFO either way (12.1.5).
            static constexpr bool fifo
              = static_cast<bool>(KVASIR_CONFIG_OR(UartConfig_, fifo, true));

            static constexpr FifoLevel rxFifoLevel
              = KVASIR_CONFIG_OR(UartConfig_, rxFifoLevel, FifoLevel::ratio1to2);

            static constexpr FifoLevel txFifoLevel
              = KVASIR_CONFIG_OR(UartConfig_, txFifoLevel, FifoLevel::ratio1to2);
        };

        // needed config
        // clockSpeed
        // baudRate
        // usartInstance
        // txPinLocation
        // rxPinLocation
        // userConfigOverride
        // dataBits
        // parity
        // stopBits
        static constexpr auto instance = UartConfig::instance;
        using Regs                     = Kvasir::Peripheral::UART::Registers<instance>;
        static_assert(instance < 2,
                      "UART Instance must be 0 or 1");

        using InterruptIndexs = typename Rp::Block<Regs>::Irqs;

        // Startup: this block, and the clock the baud divisor is computed from (the PL011
        // counts clk_peri).
        using Provides = brigand::list<Hal::Owns<Regs>>;
        using Claims   = Hal::Claim<Clocks::ClkPeri, UartConfig::clockSpeed>;

        using Config = Detail::Config<instance>;

        static constexpr auto rxDmaTrigger = Rp::Block<Regs>::dreqRx;
        static constexpr auto txDmaTrigger = Rp::Block<Regs>::dreqTx;
        /// The data register, as a DMA channel's write address (TX) or read address (RX).
        static constexpr std::uint32_t txDmaTarget = Regs::UARTDR::Addr::value;
        static constexpr std::uint32_t rxDmaSource = Regs::UARTDR::Addr::value;

        // the achieved baud rate against maxBaudRateError; a failure prints wanted, got and ppm
        static constexpr bool baudInTolerance = [] {
            Prescaler::assertInTolerance<
              Detail::baudDivider(UartConfig::clockSpeed, UartConfig::baudRate).achieved,
              UartConfig::baudRate,
              Prescaler::Tolerance{UartConfig::maxBaudRateError},
              "UART baud rate">();
            return true;
        }();
        // a static data member of a class template is initialised only when used: this use is
        // what runs the check
        static_assert(baudInTolerance);
        static_assert(Io::pinServes<PinConfig::Function::uartTx,
                                    instance>(UartConfig::txPinLocation));
        static_assert(Io::pinServes<PinConfig::Function::uartRx,
                                    instance>(UartConfig::rxPinLocation));

        static constexpr auto powerClockEnable = list(clear(Rp::Block<Regs>::reset));

        static constexpr bool hasRx
          = !std::is_same_v<std::remove_cvref_t<decltype(UartConfig::rxPinLocation)>,
                            std::remove_cvref_t<decltype(Io::NotUsed<>{})>>;

        /// One character frame on the wire: the start bit, the data bits, the parity bit, the stop bits (the
        /// transmit timeouts of UartBehavior and UartStream count in it).
        static constexpr std::uint32_t bitsPerCharacter
          = 1U + 5U + static_cast<std::uint32_t>(UartConfig::dataBits)
          + (UartConfig::parity == Hal::Uart::Parity::none ? 0U : 1U)
          + (UartConfig::stopBits == Hal::Uart::StopBits::stop2 ? 2U : 1U);

        static constexpr auto fifoConfig() {
            if constexpr(UartConfig::fifo) {
                return list(
                  set(Regs::UARTLCR_H::fen),
                  write(Regs::UARTIFLS::rxiflsel,
                        Register::value<static_cast<std::uint32_t>(UartConfig::rxFifoLevel)>()),
                  write(Regs::UARTIFLS::txiflsel,
                        Register::value<static_cast<std::uint32_t>(UartConfig::txFifoLevel)>()));
            } else {
                return list(clear(Regs::UARTLCR_H::fen));
            }
        }

        // With a FIFO, a burst shorter than the trigger level would sit there until more
        // arrives; the receive timeout interrupt delivers it after 32 bit periods of silence.
        static constexpr auto rxTimeoutInterrupt() {
            if constexpr(UartConfig::fifo && hasRx) {
                return list(set(Regs::UARTIMSC::rtim));
            } else {
                return brigand::list<>{};
            }
        }

        /// `UartConfig::startDetached`: the port comes up let go of (UartBehavior::setAttached) --
        /// pins floating, receive masked -- for a peer that is not powered at startup; the
        /// application attaches it once it is. Without it, the first break of a dead peer is
        /// logged before the application's first chance to detach.
        static constexpr bool startDetached
          = static_cast<bool>(KVASIR_CONFIG_OR(UartConfig, startDetached, false));

        /// The pins as the UART's (what setAttached(true) applies).
        static constexpr auto uartPinConfig
          = list(typename Config::template GetTxPinConfig<
                   std::decay_t<decltype(UartConfig::txPinLocation)>,
                   UartConfig::baudRate>::PinConfig{},
                 typename Config::template GetRxPinConfig<
                   std::decay_t<decltype(UartConfig::rxPinLocation)>>::PinConfig{});

        template<typename Pin>
        static constexpr auto floatingPin(Pin) {
            if constexpr(std::is_same_v<std::remove_cvref_t<Pin>, Io::NotUsed<>>) {
                return brigand::list<>{};
            } else {
                return action(Kvasir::Io::Action::Input<Kvasir::Io::PullConfiguration::pullNone>{},
                              std::remove_cvref_t<Pin>{});
            }
        }

        /// The pins let go of: inputs without pulls, nothing driven and nothing pulled.
        static constexpr auto floatingPinConfig
          = list(floatingPin(UartConfig::txPinLocation), floatingPin(UartConfig::rxPinLocation));

        static constexpr auto initStepPinConfig = [] {
            if constexpr(startDetached) {
                return floatingPinConfig;
            } else {
                return uartPinConfig;
            }
        }();

        // The init list in pieces: UartStream builds its own from the same format and FIFO parts with a DMA
        // control and interrupt set of its own.
        static constexpr auto formatConfig() {
            return list(clear(Regs::UARTCR::uarten),
                        Kvasir::Register::SequencePoint{},

                        typename Config::template GetBaudConfig<UartConfig::clockSpeed,
                                                                UartConfig::baudRate>::config{},
                        Kvasir::Register::SequencePoint{},
                        // LCR_H writes below will latch the baud divisors (PL011 requirement)
                        typename Config::template GetTxPinConfig<
                          std::decay_t<decltype(UartConfig::txPinLocation)>,
                          UartConfig::baudRate>::enable{},
                        typename Config::template GetRxPinConfig<
                          std::decay_t<decltype(UartConfig::rxPinLocation)>>::enable{},
                        typename Config::template GetDataBitConfig<UartConfig::dataBits>::config{},
                        typename Config::template GetStopBitConfig<UartConfig::stopBits>::config{},
                        typename Config::template GetParityConfig<UartConfig::parity>::config{});
        }

        static constexpr auto dmaControl() { return list(set(Regs::UARTDMACR::txdmae)); }

        static constexpr auto rxInterrupts() {
            // qualified: on a TX-only port both parts are brigand::list<>, and ADL finds no list() for them
            return Kvasir::MPL::list(
              typename Config::template GetRxPinConfig<
                std::decay_t<decltype(UartConfig::rxPinLocation)>>::interrupt{},
              rxTimeoutInterrupt());
        }

        template<typename DmaControl,
                 typename Interrupts>
        static constexpr auto peripheryConfig(DmaControl,
                                              Interrupts) {
            return list(formatConfig(),
                        DmaControl{},
                        fifoConfig(),
                        Interrupts{},
                        UartConfig::userConfigOverride);
        }

        static constexpr auto initStepPeripheryConfig = [] {
            if constexpr(startDetached) {
                return peripheryConfig(dmaControl(), brigand::list<>{});
            } else {
                return peripheryConfig(dmaControl(), rxInterrupts());
            }
        }();

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<UartConfig::isrPriority>(InterruptIndexs{}),
                 Nvic::makeClearPending(InterruptIndexs{}));

        static constexpr auto initStepPeripheryEnable = list(
          set(Regs::UARTCR::uarten),
          typename Config::template GetRxPinConfig<std::decay_t<
            decltype(UartConfig::rxPinLocation)>>::template InterruptEnable<InterruptIndexs>{});
    };

    // The transmit side: send() hands the bytes to a DMA channel paced by the UART's TX request, and the channel's
    // completion callback ends the transfer - only if the channel is done with it (its count at 0 and its read address
    // at the transfer's end: a completion forced, raised by an abort elsewhere, or arriving after the transfer was
    // given up and the next one started ends nothing). A completion that never comes - the channel aborted elsewhere,
    // the UART reset under it or its transmit side switched off (no TX request any more: the channel stalls), the DMA
    // reset under it (channel and INTEn gone: no interrupt) - is given up after twice the transfer's time on the wire
    // plus 1 ms (txTimeout(), on TimeClock): the DMA set up again if it was reset (DmaBase::setUpAgainIfReset), the
    // channel aborted (DmaBase::abort) unless it is done, the UART set up again as at startup if it was reset, disabled
    // or its transmit side switched off (UARTCR.UARTEN reads 0, its reset value: RP2040 datasheet Table 433 md
    // l.20316, RP2350 Table 1036 md l.48119; UARTCR.TXE or UARTDMACR.TXDMAE cleared, md l.20300 / l.20495, l.48104 /
    // l.48298); operationState() then says `failed` until the next send(), and txTimeouts() counts it. A transfer the
    // channel finished whose completion was lost (or came in between the deadline and the abort) is no failure; one
    // whose DMA had to be set up again for it still counts in txTimeouts(). Nothing waits for a person or a reset. The
    // timeout is judged in thread context, wherever operationState(), transferInProgress(), txIdle() or send() is
    // called.
    //
    // The receive side: with nothing queued, receive() looks whether the UART still receives - out of reset, UARTEN
    // and RXE set and, while attached, its receive interrupt enabled (UARTIMSC.RXIM) - and sets it up again if not,
    // handing out one empty optional for what was lost meanwhile (rxRepairs() counts it).
    //
    // Either side may set the UART up again; the other counts what it lost. A give-up that finds the receive side down
    // counts in rxRepairs() and puts one empty optional into the stream: the receive interrupt queues it before the first
    // character after the set-up, or receive() hands it out when nothing came yet. A reset receive() repairs while a
    // transfer is in flight took the TX FIFO's characters and the shift register's with it (a reset since the set-up:
    // UARTIBRD reads 0, its reset value, RP2040 datasheet Table 430 md l.20225, RP2350 Table 1033 md l.48035): the
    // transfer goes on, and ends `failed`, counted in txTimeouts(). A disabled UART keeps its FIFOs ("completes the
    // current character before stopping", UARTEN, md l.20316 / l.48119). A reset after the DMA's part of a transfer was
    // done can still take its last characters out of the FIFO: nothing says whether it held any.
    template<typename UartConfig,
             typename TimeClock,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             std::size_t            BufferSize>
    struct UartBehaviorImpl : Kvasir::UART::UartBase<UartConfig> {
        using base = Kvasir::UART::UartBase<UartConfig>;
        using Regs = typename base::Regs;

        using Claims = brigand::append<typename base::Claims, Kvasir::DMA::Claims<Dma, DmaChannel>>;

        // BufferSize 0 = a send-only UART: no receive queue (a Queue needs at least one slot), receive() finds nothing
        struct NoRxBuffer {
            static bool popInto(std::optional<std::byte>&) { return false; }
        };

        inline static std::conditional_t<
          BufferSize == 0,
          NoRxBuffer,
          Kvasir::Atomic::
            Queue<std::optional<std::byte>, BufferSize, Kvasir::Atomic::OverflowPolicyIgnore>>
          rxbuffer_{};

        /// The next received byte, if one is waiting. An empty optional inside `out` is a
        /// framing / overrun error the interrupt recorded in the stream's order, or a receive side found dead and set
        /// up again (the comment above the struct).
        static bool receive(std::optional<std::byte>& out) {
            if(rxbuffer_.popInto(out)) { return true; }
            if constexpr(BufferSize != 0 && base::hasRx) {
                if(takeRxLoss_()) {
                    out = std::nullopt;
                    return true;
                }
                if(!receiving_()) {
                    // read before the set-up, which writes the divisor; a transfer in flight lost what the TX FIFO held
                    if(busy.load(std::memory_order_relaxed) && wasReset_()) {
                        txHitByReset_ = true;
                    }
                    setUpAgain_();
                    rxRepairs_ = rxRepairs_ + 1;
                    out        = std::nullopt;
                    return true;
                }
            }
            return false;
        }

        /// Receive sides found dead and set up again since boot: by receive(), or by a transmit give-up that set the
        /// UART up again (the comment above the struct).
        [[nodiscard]] static std::uint32_t rxRepairs() { return rxRepairs_; }

        enum class OperationState { succeeded, failed, ongoing };

        /// How long a transmit of `bytes` may take before it counts as lost: twice its time on the wire, plus 1 ms
        /// for the channel's start and the clock's tick.
        static constexpr std::chrono::microseconds txTimeout(std::size_t bytes) {
            return std::chrono::microseconds{2ULL * bytes * base::bitsPerCharacter * 1'000'000ULL
                                               / static_cast<std::uint64_t>(UartConfig::baudRate)
                                             + 1'000ULL};
        }

        // Atomic because the DMA completion callback runs in interrupt context
        // and every reader is in thread context.
        inline static std::atomic<bool> busy{false};

        /// `succeeded` once the DMA has handed over the last byte (with the FIFO on, up to 32 of them are still to go
        /// out on the pin - transferInProgress() waits for those too); `failed` after a transfer given up or hit by a
        /// UART reset that receive() repaired (above), until the next send().
        static OperationState operationState() {
            if(busy.load(std::memory_order_relaxed)) {
                if(TimeClock::now() < deadline_) { return OperationState::ongoing; }
                abandon_();
            } else if(txHitByReset_) {
                // ended by its completion, but a reset receive() repaired took characters
                txHitByReset_ = false;
                failed_       = true;
                txTimeouts_   = txTimeouts_ + 1;
            }
            return failed_ ? OperationState::failed : OperationState::succeeded;
        }

        // Is the transmitter free? Use this to sequence transfers: the DMA's part (operationState(), bounded by the
        // timeout above), then the PL011's shift register (UARTFR.BUSY).
        [[nodiscard]] static bool transferInProgress() {
            return operationState() == OperationState::ongoing
                || static_cast<bool>(get<0>(apply(read(Regs::UARTFR::busy))));
        }

        // Abandon an in-flight transmit and leave the peripheral usable.
        // Without it a caller that gives up cannot reclaim the channel: the
        // sequence runs on and the next start() races a transfer that never
        // stopped. See DMA::abort.
        static void abortTransfer() {
            Dma::template abort<DmaChannel>();
            busy.store(false, std::memory_order_relaxed);
            txHitByReset_ = false;
        }

        /// Transmits given up since boot, and transfers a UART reset hit (the comment above the struct).
        [[nodiscard]] static std::uint32_t txTimeouts() { return txTimeouts_; }

        static void sendNocopy(std::span<std::byte const> span) {
            // Never re-arm a live channel. transferInProgress() also counts a reset that hit the last transfer
            // (operationState(); abortTransfer() drops it for one given up here).
            if(transferInProgress()) { abortTransfer(); }
            deadline_ = TimeClock::now()
                      + std::chrono::ceil<typename TimeClock::duration>(txTimeout(span.size())) +
                        typename TimeClock::duration{1};   // the tick in progress
            failed_   = false;
            auto const from = reinterpret_cast<std::uint32_t>(span.data());
            end_            = from + static_cast<std::uint32_t>(span.size());
            busy.store(true, std::memory_order_relaxed);

            std::atomic_signal_fence(std::memory_order_release);

            Dma::template start<DmaChannel,
                                ::Kvasir::DMA::ChannelConfig{.priority = DmaPriority,
                                                             .trigger  = base::txDmaTrigger,
                                                             .size     = Dma::TransferSize::size8,
                                                             .incrementRead  = true,
                                                             .incrementWrite = false}>(
              base::txDmaTarget,
              from,
              span.size(),
              []() {
                  // only the end of the transfer in flight (the comment above the struct)
                  if(channelDone_()) { busy.store(false, std::memory_order_relaxed); }
              });
        }

        // Hal::Uart::ByteSink (kvasir/Hal/Uart.hpp): all of `s` while nothing is on the way, else
        // nothing (an empty `s` starts no transfer). No copy: the bytes stay the caller's until txIdle().
        static std::size_t send(std::span<std::byte const> s) {
            if(s.empty() || transferInProgress()) { return 0; }
            sendNocopy(s);
            return s.size();
        }

        // The DMA has handed over everything and the PL011 shifter is empty: UARTFR.BUSY stays
        // set "until the complete byte, including all the stop bits, has been sent" (RP2040
        // datasheet table 428, RP2350 table 1031).
        [[nodiscard]] static bool txIdle() { return !transferInProgress(); }

    protected:
        // setAttached() (the receiving UART's): whether the receive interrupts are on. A transmit-only UART never
        // starts detached (UartBehaviorSelector), so it reads true for good.
        inline static std::atomic<bool> attached_{!base::startDetached};

        // A give-up set up a UART whose receive side was down (the comment above the struct): one empty optional owed
        // to the reader - taken by the receive interrupt (queued before the first character after the set-up) or by
        // receive() with nothing queued, whichever comes first.
        inline static std::atomic<bool> rxLossPending_{false};

        static bool takeRxLoss_() {
            return rxLossPending_.load(std::memory_order_relaxed)
                && rxLossPending_.exchange(false, std::memory_order_acq_rel);
        }

        // The block as at startup again, after a reset (held or pulsed) or a disable under it: out of reset, the
        // format, DMA and FIFO configuration, the receive interrupts while attached, UARTEN. The pins are IO_BANK0's
        // and are not reset with the block.
        static void setUpAgain_() {
            apply(base::powerClockEnable);
            Rp::Block<Regs>::waitResetDone();
            if(attached_.load(std::memory_order_relaxed)) {
                apply(base::peripheryConfig(base::dmaControl(), base::rxInterrupts()));
            } else {
                apply(base::peripheryConfig(base::dmaControl(), brigand::list<>{}));
            }
            apply(set(Regs::UARTCR::uarten));
        }

    private:
        inline static typename TimeClock::time_point deadline_{};
        inline static std::uint32_t end_{};   // the transfer's end: READ_ADDR once it is read
        inline static bool          failed_{};
        inline static std::uint32_t volatile txTimeouts_{};
        inline static std::uint32_t volatile rxRepairs_{};
        // receive() repaired a reset while the transfer in flight went
        inline static bool txHitByReset_{};

        // RP2350: TRANS_COUNT 31:28 is MODE (0: start writes it so), 27:0 the count (12.6.2.2.1)
        static constexpr std::uint32_t transCountMask = 0x0FFF'FFFFU;

        // The channel is done with the transfer in flight: its count at 0 (TRANS_COUNT "updating automatically each
        // time a write transfer completes", RP2040 datasheet Table 123 md l.5390, RP2350 Table 1150 md l.54843) and its
        // read address at the transfer's end (DmaBase::readAddress; 0 after a DMA reset). A DMA held in reset is not
        // read.
        static bool channelDone_() {
            if(apply(read(Rp::Block<typename Dma::Regs>::reset)) != 0) { return false; }
            return (Dma::template remaining<DmaChannel>() & transCountMask) == 0
                && Dma::template readAddress<DmaChannel>() == end_;
        }

        // The UART sends: out of reset, UARTEN, TXE and TXDMAE set (the comment above the struct).
        static bool sending_() {
            if(apply(read(Rp::Block<Regs>::reset)) != 0) { return false; }
            auto const r
              = apply(read(Regs::UARTCR::uarten, Regs::UARTCR::txe, Regs::UARTDMACR::txdmae));
            return r.template get<0>() && r.template get<1>() && r.template get<2>();
        }

        // The UART receives: out of reset, UARTEN and RXE set, and while attached the receive interrupt enabled.
        static bool receiving_() {
            if(apply(read(Rp::Block<Regs>::reset)) != 0) { return false; }
            auto const r
              = apply(read(Regs::UARTCR::uarten, Regs::UARTCR::rxe, Regs::UARTIMSC::rxim));
            return r.template get<0>() && r.template get<1>()
                && (r.template get<2>() || !attached_.load(std::memory_order_relaxed));
        }

        // Reset since the set-up, or held in reset (the comment above the struct): UARTIBRD read only out of reset.
        static bool wasReset_() {
            return apply(read(Rp::Block<Regs>::reset)) != 0
                || get<0>(apply(read(Regs::UARTIBRD::baud_divint))) == 0;
        }

        // The lost completion (the comment above the struct): the DMA first - a block held in reset takes no write,
        // the abort included - then the channel unless it is done, then the UART (its registers are read only out of
        // reset; a receive side it finds down is owed to the reader). A transfer the channel finished counts as sent:
        // its completion lost, or one that came in between the deadline and the abort (dropped by it), found by the
        // same check after it - unless a reset receive() repaired hit it.
        static void abandon_() {
            bool const dmaRepaired = Dma::setUpAgainIfReset();
            if(!channelDone_()) {
                static_cast<void>(
                  Dma::template abort<DmaChannel>());   // the channel stopped, its callback cleared
            }
            bool const uartRepaired = !sending_();
            if(uartRepaired) {
                if constexpr(BufferSize != 0 && base::hasRx) {
                    if(!receiving_()) {
                        rxRepairs_ = rxRepairs_ + 1;
                        // before the set-up lets characters in
                        rxLossPending_.store(true, std::memory_order_seq_cst);
                    }
                }
                setUpAgain_();
            }
            bool const sent = !uartRepaired && !txHitByReset_
                           && (!busy.load(std::memory_order_relaxed) || channelDone_());
            txHitByReset_   = false;
            busy.store(false, std::memory_order_relaxed);
            if(sent && !dmaRepaired) { return; }
            txTimeouts_ = txTimeouts_ + 1;
            if(!sent) { failed_ = true; }
        }
    };

    template<typename UartConfig,
             typename TimeClock,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             std::size_t            BufferSize,
             bool                   Isr>
    struct UartBehaviorSelector;

    template<typename UartConfig,
             typename TimeClock,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             std::size_t            BufferSize>
    struct UartBehaviorSelector<UartConfig,
                                TimeClock,
                                Dma,
                                DmaChannel,
                                DmaPriority,
                                BufferSize,
                                false>
      : UartBehaviorImpl<UartConfig, TimeClock, Dma, DmaChannel, DmaPriority, 0> {
        // setAttached() lives with the receive side: a transmit-only port that started detached
        // could never take its pin back.
        static_assert(
          !UartBehaviorImpl<UartConfig,
                            TimeClock,
                            Dma,
                            DmaChannel,
                            DmaPriority,
                            0>::startDetached,
          "startDetached needs a receiving UART (rxPinLocation): only it has setAttached()");
    };

    template<typename UartConfig,
             typename TimeClock,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             std::size_t            BufferSize>
    struct UartBehaviorSelector<UartConfig,
                                TimeClock,
                                Dma,
                                DmaChannel,
                                DmaPriority,
                                BufferSize,
                                true>
      : UartBehaviorImpl<UartConfig, TimeClock, Dma, DmaChannel, DmaPriority, BufferSize> {
        using base
          = UartBehaviorImpl<UartConfig, TimeClock, Dma, DmaChannel, DmaPriority, BufferSize>;
        using Regs = typename base::Regs;

        // An overrun repeats per character on a noisy line; no clock here, so count based.
        inline static Kvasir::CountLimiter<> overrunLog_{};
        inline static Kvasir::CountLimiter<> characterErrorLog_{};

        static constexpr std::uint32_t characterErrors
          = Regs::UARTDR::fe.mask | Regs::UARTDR::pe.mask | Regs::UARTDR::be.mask;

        // The PL011's receive FIFO: "32x12 Rx" (RP2040 datasheet 4.2 md l.19571 and 4.2.2.5,
        // RP2350 12.1 md l.47348). The cap of one drain pass.
        static constexpr std::uint32_t rxFifoDepth = 32;

        // How often the receive interrupt ran: with the FIFO on, a fraction of the bytes.
        inline static std::atomic<std::uint32_t> rxInterrupts{0};

        static void onIsr() {
            rxInterrupts.fetch_add(1, std::memory_order_relaxed);
            // what a give-up's set-up lost, before the first character after it (the comment above UartBehaviorImpl)
            if(base::takeRxLoss_()) { base::rxbuffer_.push(std::nullopt); }
            auto const intflag
              = apply(read(Regs::UARTMIS::rxmis, Regs::UARTMIS::oemis, Regs::UARTMIS::rtmis));
            if(intflag.template get<1>()) {
                KVASIR_LOG_LIMITED(overrunLog_.allow(), UC_LOG_E, "uart rx overrun");
                apply(set(Regs::UARTICR::oeic));
                base::rxbuffer_.push(std::nullopt);
            }
            if(intflag.template get<2>()) { apply(set(Regs::UARTICR::rtic)); }
            // What the FIFO (or the one-deep holding register) has, at most RxFifoDepth reads per
            // pass, so a peer that never stops sending cannot keep the handler running: what is
            // left re-raises the interrupt - UARTRXINTR stays asserted while the level is at or
            // above the trigger, UARTRTINTR fires when data stays unread for 32 bit periods
            // (RP2040 datasheet 4.2.6.2 / 4.2.6.4; the same PL011 on the RP2350).
            // One read per character (it pops the FIFO): the byte in bits 7:0 and its framing,
            // parity and break errors in bits 8-10 (UARTDR, RP2040 datasheet md l.20107-20135;
            // the same PL011 on the RP2350). A character with one of them is pushed as empty.
            for(std::uint32_t n = 0; n != rxFifoDepth && !apply(read(Regs::UARTFR::rxfe)); ++n) {
                auto const dr = static_cast<std::uint32_t>(apply(read(Regs::UARTDR::FULLREGISTER)));
                if((dr & characterErrors) != 0) {
                    KVASIR_LOG_LIMITED(characterErrorLog_.allow(),
                                       UC_LOG_E,
                                       "uart rx character error: DR {:#06x}",
                                       dr);
                    base::rxbuffer_.push(std::nullopt);
                } else {
                    base::rxbuffer_.push(std::byte(dr & 0xFFU));
                }
            }
        }

        /// Let go of the line, or take it back, for a peer that is not always powered.
        ///
        /// Detached, neither pin is the UART's: both are inputs without pulls, so nothing is
        /// driven into an unpowered module (an idle TX is high, and a high line into a chip
        /// without supply feeds it through its input) and nothing is taken from it -- its TX,
        /// pulled low by the dead module, no longer reads as an endless break (a character error
        /// per frame time). The receive interrupts are masked and a transmit in progress is
        /// abandoned: its DMA stops, and what the transmitter still holds (up to 32 bytes in the
        /// FIFO and the character in the shift register) is thrown away by a block reset - the
        /// PL011 has no FIFO flush (UARTLCR_H.FEN only switches to character mode, RP2040
        /// datasheet 4.2.8 / RP2350 12.1.8). The block is configured again after it, receive still
        /// masked; transferInProgress() is false when this returns. Attached, the pins are the UART's again, whatever the FIFO collected
        /// meanwhile and the error flags it left are dropped, and receiving starts clean.
        /// Thread context; the interrupt may run once more on what was pending at the switch.
        static void setAttached(bool on) {
            if(on == base::attached_.load(std::memory_order_relaxed)) { return; }
            if(!on) {
                apply(clear(Regs::UARTIMSC::rxim),
                      clear(Regs::UARTIMSC::rtim),
                      clear(Regs::UARTIMSC::oeim));
                base::attached_.store(false, std::memory_order_relaxed);
                bool const sending = base::transferInProgress();
                if(sending) { base::abortTransfer(); }
                apply(base::floatingPinConfig);
                if(sending) { flushTransmitter_(); }
                return;
            }
            apply(base::uartPinConfig);
            // Drop one FIFO's worth (RxFifoDepth reads at most: a peer sending without pause
            // cannot hold the thread here); a character still arriving is received normally.
            for(std::uint32_t n = 0; n != rxFifoDepth && !apply(read(Regs::UARTFR::rxfe)); ++n) {
                static_cast<void>(apply(read(Regs::UARTDR::FULLREGISTER)));
            }
            apply(set(Regs::UARTRSR::oe, Regs::UARTRSR::be, Regs::UARTRSR::pe, Regs::UARTRSR::fe));
            apply(set(Regs::UARTICR::rxic,
                      Regs::UARTICR::rtic,
                      Regs::UARTICR::oeic,
                      Regs::UARTICR::feic,
                      Regs::UARTICR::peic,
                      Regs::UARTICR::beic));
            base::attached_.store(true, std::memory_order_relaxed);
            apply(base::rxInterrupts());
        }

        [[nodiscard]] static bool attached() {
            return base::attached_.load(std::memory_order_relaxed);
        }

    private:
        // A block reset empties both FIFOs and the shift register; the format, FIFO and DMA
        // configuration is written again as at startup (setUpAgain_), without the receive
        // interrupts. The pins are SIO by now, so the character cut short never reaches the line.
        static void flushTransmitter_() {
            apply(set(Rp::Block<Regs>::reset));
            base::setUpAgain_();   // detached by now: without the receive interrupts
        }

    public:
        using Isr = Nvic::IsrsFor<std::addressof(onIsr), typename base::InterruptIndexs>;
    };

    /// A DMA-fed UART: `TimeClock` (`now()`, a chrono clock: SystickClock, TimerClock) times the transmit timeout
    /// (UartBehaviorImpl); `BufferSize` 0 or no rxPinLocation: transmit only.
    template<typename UartConfig,
             typename TimeClock,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             std::size_t            BufferSize>
    struct UartBehavior
      : UartBehaviorSelector<
          UartConfig,
          TimeClock,
          Dma,
          DmaChannel,
          DmaPriority,
          BufferSize,
          !std::is_same_v<std::remove_cvref_t<decltype(UartConfig::rxPinLocation)>,
                          std::remove_cvref_t<decltype(Io::NotUsed<>{})>>> {};

}}   // namespace Kvasir::UART
