#pragma once
// Link kvasir::devices (not only kvasir::bus): the clock ceiling is a Units::Hertz (Quantities.hpp, mp-units).
// A queued SPI master made of a PIO state machine and two DMA channels: the same front and QueueCore as SPIQueued
// (the PL022's), for a device the PL022 cannot clock as fast as the device can.
//
// Why: the PL022 samples MISO at the rising edge of SCK. The bit the device shifts out after the falling edge reaches
// the pin only after SCK has travelled out, the device's output delay and the trip back - on a board where that round
// trip is longer than half a bit, the rising-edge sample takes the bit in transition. A PIO machine takes its sample
// on any cycle. This one runs SPI mode 3 (SCK idles high; out on the falling edge, latched on the rising one), MSB
// first, 8-bit frames, four machine cycles a bit at divider 1 (SCK = clk_sys / 4), and samples each bit during the
// NEXT one: 5 or 6 machine cycles after the falling edge that put it out (the next bit's first low cycle, or its
// rising edge), the last bit one cycle later than that (it has no next bit, SCK stays high). Config states the
// board's measured round trip (`misoRoundTripMinNs`/`MaxNs`, as the machine sees the pin: the synchroniser is
// bypassed on MISO); the sample cycle is the one inside it nearest its middle, a static_assert when there is none.
//
// The machine: it waits at `wait 1 irq rel` for start()'s go (IRQ_FORCE), takes the transfer's bit count (autopull has
// fetched it into the empty OSR), then runs. The next count comes only after the frame's completion, so it is never
// taken for a frame.
// A stall in the middle of a frame would move the sample point (a stalled `out` still drives its falling edge, a
// stalled `in` samples late), so start() lets the go through only once the TX DMA has filled the FIFO, and the RX
// completion reports any FDEBUG TXSTALL/RXSTALL of the frame as an overrun: the frame is failed, never wrong.
//
// A short request on an idle bus runs polled (QueueCore's `pollUpTo`, PioSpiTiming): the count and the first three
// frames go into the TX FIFO before the go, the rest as the machine takes them, the reply out of the RX FIFO.
#include "DMA.hpp"
#include "PioStateMachine.hpp"
#include "SPIQueued.hpp"   // QueuedDma, the DMA channel pair both masters take
#include "kvasir/Atomic/Atomic.hpp"
#include "kvasir/Devices/Quantities.hpp"
#include "kvasir/Devices/SPI/QueueCore.hpp"
#include "kvasir/Devices/SPI/QueuedFront.hpp"
#include "pio/PioSpiProgram.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <uc_log/uc_log.hpp>

namespace Kvasir { namespace SPI {

    /// QueueCore's timing for the PIO master: a request of up to 1 us on the wire runs polled (seven frames at
    /// 62.5 MHz: a W5500 register access); the rest as SPIQueued's.
    struct PioSpiTiming : QueueCoreDefaults {
        static constexpr std::chrono::microseconds pollUpTo{1};
    };

    namespace Detail {
        /// PioSpiQueued without its front: the state machine, the DMA, the Startup steps and the QueueCore over the
        /// Hw policy (kvasir_devices SPI/QueuedFront.hpp says why the two are apart).
        template<typename PioSpiConfig,
                 typename Clock,
                 typename Dma,
                 typename DmaChannels,
                 std::size_t QueueDepth,
                 std::size_t CallbackSize,
                 typename Timing>
        struct PioSpiQueuedHalf {
            static constexpr unsigned sampleLag = pioSpiSampleLag(PioSpiConfig::clockSpeed,
                                                                  PioSpiConfig::misoRoundTripMinNs,
                                                                  PioSpiConfig::misoRoundTripMaxNs);
            static_assert(PioSpiConfig::misoRoundTripMinNs <= PioSpiConfig::misoRoundTripMaxNs,
                          "misoRoundTripMinNs above misoRoundTripMaxNs");
            static_assert(
              sampleLag != 0,
              "PioSpiQueued: at this clk_sys neither 5 nor 6 machine cycles after a falling edge "
              "is "
              "inside the bit the board's round trip (misoRoundTripMinNs..MaxNs) delivers");
            using Program = PioSpiProgram<sampleLag == 0 ? 6 : sampleLag>;

            struct SmConfig {
                static constexpr auto clockSpeed  = PioSpiConfig::clockSpeed;
                static constexpr auto pioInstance = PioSpiConfig::pioInstance;
                static constexpr auto smInstance  = PioSpiConfig::smInstance;
                static constexpr auto programOffset
                  = KVASIR_CONFIG_OR(PioSpiConfig, programOffset, 0);
                static constexpr double clockDiv    = 1.0;
                static constexpr auto   sidesetPins = brigand::list<typename PioSpiConfig::Sck>{};
                static constexpr auto   outPins     = brigand::list<typename PioSpiConfig::Mosi>{};
                static constexpr auto   inPins      = brigand::list<typename PioSpiConfig::Miso>{};
                static constexpr auto   inputSyncBypass
                  = brigand::list<typename PioSpiConfig::Miso>{};
                static constexpr auto pinsInitialHigh = brigand::list<typename PioSpiConfig::Sck>{};
                static constexpr bool autopull        = true;
                static constexpr unsigned pullThreshold = 8;   // one frame from each FIFO word
                static constexpr bool     outShiftRight
                  = false;   // MSB first: an 8-bit write fills bits 31..24
                static constexpr bool     autopush = true;
                static constexpr unsigned pushThreshold
                  = 8;   // one frame into bits 7..0 of each word
                static constexpr bool inShiftRight = false;
                static constexpr auto driveStrength
                  = KVASIR_CONFIG_OR(PioSpiConfig, driveStrength, Kvasir::Io::DriveStrength::mA8);
                static constexpr bool slewFast = KVASIR_CONFIG_OR(PioSpiConfig, slewFast, true);
            };

            using Sm = Kvasir::Pio::StateMachine<Program, SmConfig>;

            static constexpr auto txChannel = DmaChannels::txChannel;
            static constexpr auto rxChannel = DmaChannels::rxChannel;
            static constexpr auto priority  = DmaChannels::priority;

            // The Startup entries: the machine's (program, pins, start) and the two DMA channels.
            using Provides = typename Sm::Provides;
            using Claims   = brigand::append<typename Sm::Claims,
                                             Kvasir::DMA::Claims<Dma, txChannel, rxChannel>>;
            static constexpr auto powerClockEnable        = Sm::powerClockEnable;
            static constexpr auto initStepPinConfig       = Sm::initStepPinConfig;
            static constexpr auto initStepPeripheryConfig = Sm::initStepPeripheryConfig;
            static constexpr auto initStepPeripheryEnable = Sm::initStepPeripheryEnable;

            static void preEnableRuntimeInit() { Sm::preEnableRuntimeInit(); }

            static void runtimeInit() { Sm::runtimeInit(); }

            /// The one rate: four machine cycles a bit at divider 1.
            static constexpr std::uint32_t rateHz
              = static_cast<std::uint32_t>(PioSpiConfig::clockSpeed / Program::cyclesPerBit);

            struct Setup {
                std::uint32_t hz{rateHz};
                std::uint32_t usPerBitQ10{Kvasir::SPI::usPerBitQ10(rateHz)};

                /// One rate, one mode: every device on this bus shares the setup.
                [[gnu::always_inline]] constexpr bool operator==(Setup const&) const {
                    return true;
                }
            };

            /// The machine runs mode 3 at clk_sys / 4 only: a device must take mode 3 and at least that clock
            /// (ceilingHz refuses a lower ceiling at compile time, rateOutOfReach).
            static consteval Setup setup(Hal::ClockMode mode,
                                         Units::Hertz   maxClock) {
                if(mode != Hal::ClockMode::mode3) { onlyMode3OnThePioSpiMaster(); }
                (void)ceilingHz(maxClock.numerical_value_in(Units::si::hertz),
                                rateHz,
                                static_cast<std::uint32_t>(PioSpiConfig::clockSpeed),
                                Program::cyclesPerBit,
                                Program::cyclesPerBit);
                return Setup{};
            }

            struct Snapshot {
                std::uint32_t fstat{};    ///< the instance's FIFO flags
                std::uint32_t fdebug{};   ///< stall / over / under flags
                std::uint32_t pc{};
                std::uint32_t txLeft{};
                std::uint32_t rxLeft{};
                bool          txBusy{};
                bool          rxBusy{};
            };

            struct Hw {
                static constexpr unsigned instance = PioSpiConfig::pioInstance;
                using Setup                        = PioSpiQueuedHalf::Setup;
                using Snapshot                     = PioSpiQueuedHalf::Snapshot;
                static constexpr bool supportsWide = false;   // 8-bit frames only
                // the RP2350 DMA counts 28 bits (TRANS_COUNT's top four are MODE: RP2350 datasheet 12.6.2.2,
                // md l.53736); the machine's bit count, frames x 8 - 1, fits X with room
                static constexpr std::uint32_t maxFrames = (1U << 28U) - 1U;

                static void mask() { irqs_ = Kvasir::Core::IrqState::saveAndMask(); }

                static void unmask() { irqs_.restore(); }

                static void configure(Setup const&) {}

                static void start(Transfer const& t,
                                  std::uint32_t   gen,
                                  void (*)(std::uint32_t,
                                           bool)) {
                    clearStalls_();
                    lastFrames_ = t.frames;
                    Sm::tryPush(bitCount_(t));   // the machine is idle, the FIFO empty: always room
                    std::atomic_signal_fence(std::memory_order_release);
                    startRx_(t, gen);
                    startTx_(t);
                    // the go only once the FIFO holds the frames the first bits need: a stall at the frame's start
                    // would read as a stall in it
                    for(std::uint32_t spins = 0; spins < fillWaitSpins; ++spins) {
                        if(Sm::txFull() || Dma::template ready<txChannel>()) { break; }
                    }
                    go_();
                }

                /// QueueCore's polled path, masked, nothing on the wire: machine frames of at most four bytes, each
                /// loaded whole before its go - the count first (autopull takes it into the OSR, which leaves all four
                /// FIFO entries to the frames), then the frames - and its replies popped after. Nothing has to keep pace
                /// with the bus, whatever the build's speed; between two such frames SCK rests high (mode 3) under the
                /// same CS. False when the count was not taken or a reply did not come (bounded), or the machine stalled.
                static bool transferPolled(Transfer const& t) {
                    clearStalls_();
                    for(std::uint32_t done = 0; done != t.frames;) {
                        auto const n = std::min<std::uint32_t>(t.frames - done, fifoDepth);
                        Sm::tryPush(n * 8U - 1U);
                        if(!countTaken_()) {
                            logPolledLoss_(t, done, done);
                            return false;
                        }
                        for(std::uint32_t i = 0; i != n; ++i) {
                            Sm::tryPush(frameWord_(t, done + i));
                        }
                        go_();
                        for(std::uint32_t received = 0; received != n;) {
                            std::uint32_t idle = 0;
                            while(Sm::rxEmpty()) {
                                if(++idle == polledWaitSpins) {
                                    logPolledLoss_(t, done + n, done + received);
                                    return false;
                                }
                            }
                            auto const v = get<0>(apply(read(Fifo::RXF::fifo)));
                            t.rx[t.rxIncrement ? done + received : 0]
                              = static_cast<std::byte>(v & 0xFFU);
                            ++received;
                        }
                        done += n;
                    }
                    if(stalled_()) {
                        logPolledLoss_(t, t.frames, t.frames);
                        return false;
                    }
                    return true;
                }

                static void abort() {
                    Dma::template abort<txChannel>();
                    Dma::template abort<rxChannel>();
                    restart_();
                }

                static void reinit() { abort(); }

                static Snapshot snapshot() {
                    return Snapshot{
                      .fstat  = get<0>(apply(read(PioRegs::FSTAT::FULLREGISTER))),
                      .fdebug = get<0>(apply(read(PioRegs::FDEBUG::FULLREGISTER))),
                      .pc     = Sm::pc(),
                      .txLeft = Dma::template remaining<txChannel>(),
                      .rxLeft = Dma::template remaining<rxChannel>(),
                      .txBusy = !Dma::template ready<txChannel>(),
                      .rxBusy = !Dma::template ready<rxChannel>(),
                    };
                }

                static void log([[maybe_unused]] Snapshot const& s) {
                    UC_LOG_W(
                      "pio{} spi at the timeout: FSTAT {:#010x}, FDEBUG {:#010x}, pc {}, TX DMA {} "
                      "left{}, "
                      "RX DMA {} left{}",
                      instance,
                      s.fstat,
                      s.fdebug,
                      s.pc,
                      s.txLeft,
                      std::string_view{s.txBusy ? " (busy)" : ""},
                      s.rxLeft,
                      std::string_view{s.rxBusy ? " (busy)" : ""});
                }

            private:
                using PioRegs = Kvasir::Peripheral::PIO::Registers<PioSpiConfig::pioInstance>;
                using Fifo    = typename PioRegs::template FIFO<PioSpiConfig::smInstance>;
                static constexpr unsigned sm = PioSpiConfig::smInstance;

                /// Entries of a (not joined) PIO FIFO (RP2350 datasheet 11.4: four words each way).
                static constexpr std::uint32_t fifoDepth = 4;

                /// Spins for the TX DMA to fill the FIFO before the go (tens of cycles in practice), and for a
                /// polled frame to come back (a few frame times at 62.5 MHz; generous for a slow build).
                static constexpr std::uint32_t fillWaitSpins = 1'000;
                /// DMA frames the machine stalled in, logged (the first few: each is also a failed request).
                static constexpr std::uint32_t stallLogs = 5;
                inline static std::uint32_t    stallsLogged_{};
                inline static std::uint32_t    lastFrames_{};
                static constexpr std::uint32_t polledWaitSpins = 20'000;

                inline static Kvasir::Core::IrqState irqs_{};

                /// Where a failed polled transfer stopped: the frames moved, the FIFO levels and flags, the PC.
                static void logPolledLoss_([[maybe_unused]] Transfer const& t,
                                           [[maybe_unused]] std::uint32_t   sent,
                                           [[maybe_unused]] std::uint32_t   received) {
                    UC_LOG_W(
                      "pio{} spi polled transfer of {} frame(s): {} sent, {} received, FLEVEL "
                      "{:#010x}, "
                      "FDEBUG {:#010x}, pc {}",
                      instance,
                      t.frames,
                      sent,
                      received,
                      get<0>(apply(read(PioRegs::FLEVEL::FULLREGISTER))),
                      get<0>(apply(read(PioRegs::FDEBUG::FULLREGISTER))),
                      Sm::pc());
                }

                /// After a count was pushed onto an empty FIFO: autopull has taken it into the OSR (bounded).
                [[nodiscard]] static bool countTaken_() {
                    for(std::uint32_t spins = 0; spins < fillWaitSpins; ++spins) {
                        if(Sm::txEmpty()) { return true; }
                    }
                    return false;
                }

                [[gnu::always_inline]] static std::uint32_t bitCount_(Transfer const& t) {
                    return t.frames * 8U - 1U;
                }

                /// A frame as the 8-bit DMA writes it: the byte in every lane (the machine shifts out bits 31..24).
                [[gnu::always_inline]] static std::uint32_t frameWord_(Transfer const& t,
                                                                       std::uint32_t   i) {
                    return std::to_integer<std::uint32_t>(t.tx[t.txIncrement ? i : 0])
                         * 0x01010101U;
                }

                /// IRQ flag `sm` (`wait 1 irq 0 rel` of machine `sm`): forced, the machine clears it as it passes.
                static void go_() {
                    apply(set(Kvasir::Register::bit<sm>(PioRegs::IRQ_FORCE::irq_force)));
                }

                static void clearStalls_() {
                    apply(
                      Kvasir::Register::reset(Kvasir::Register::bit<sm>(PioRegs::FDEBUG::txstall)),
                      Kvasir::Register::reset(Kvasir::Register::bit<sm>(PioRegs::FDEBUG::rxstall)));
                }

                [[nodiscard]] static bool stalled_() {
                    auto const r = apply(read(Kvasir::Register::bit<sm>(PioRegs::FDEBUG::txstall)),
                                         read(Kvasir::Register::bit<sm>(PioRegs::FDEBUG::rxstall)));
                    return get<0>(r) != 0 || get<1>(r) != 0;
                }

                /// Back to `idle` with empty FIFOs, no pending go, no stall flags.
                static void restart_() {
                    Sm::setEnabled(false);
                    Sm::clearFifos();
                    apply(Kvasir::Register::reset(Kvasir::Register::bit<sm>(PioRegs::IRQ::irq)));
                    clearStalls_();
                    Sm::restartAt(Program::offset("idle"));
                    Sm::setEnabled(true);
                }

                template<bool Inc>
                static void rx_(Transfer const& t,
                                std::uint32_t   gen) {
                    Dma::template start<rxChannel,
                                        ::Kvasir::DMA::ChannelConfig{
                                          .priority       = priority,
                                          .trigger        = Sm::template rxDmaTrigger<Dma>(),
                                          .size           = Dma::TransferSize::size8,
                                          .incrementRead  = false,
                                          .incrementWrite = Inc}>(
                      reinterpret_cast<std::uint32_t>(t.rx),
                      Sm::rxFifoAddress,
                      t.frames,
                      [gen]() {
                          bool const stalled = stalled_();
                          if(stalled && stallsLogged_ < stallLogs) {
                              ++stallsLogged_;
                              UC_LOG_W(
                                "pio{} spi DMA frame of {} stalled in the machine: FLEVEL "
                                "{:#010x}, "
                                "FDEBUG {:#010x}, pc {}",
                                instance,
                                lastFrames_,
                                get<0>(apply(read(PioRegs::FLEVEL::FULLREGISTER))),
                                get<0>(apply(read(PioRegs::FDEBUG::FULLREGISTER))),
                                Sm::pc());
                          }
                          clearStalls_();
                          PioSpiQueuedHalf::Core::complete(gen, stalled);
                      });
                }

                template<bool Inc>
                static void tx_(Transfer const& t) {
                    Dma::template start<txChannel,
                                        ::Kvasir::DMA::ChannelConfig{
                                          .priority       = priority,
                                          .trigger        = Sm::template txDmaTrigger<Dma>(),
                                          .size           = Dma::TransferSize::size8,
                                          .incrementRead  = Inc,
                                          .incrementWrite = false}>(
                      Sm::txFifoAddress,
                      reinterpret_cast<std::uint32_t>(t.tx),
                      t.frames);
                }

                static void startRx_(Transfer const& t,
                                     std::uint32_t   gen) {
                    t.rxIncrement ? rx_<true>(t, gen) : rx_<false>(t, gen);
                }

                static void startTx_(Transfer const& t) {
                    t.txIncrement ? tx_<true>(t) : tx_<false>(t);
                }
            };

            using Core = QueueCore<Hw, Clock, QueueDepth, CallbackSize, Timing>;

        private:
            /// Not constexpr: a call in setup() is the compile error that names the refusal.
            static void onlyMode3OnThePioSpiMaster() {}
        };
    }   // namespace Detail

    /// PioSpiConfig: clockSpeed (clk_sys), pioInstance, smInstance, programOffset (0), the pin types Sck, Mosi,
    /// Miso, driveStrength (8 mA) and slewFast (true), and the board's MISO round trip misoRoundTripMinNs/MaxNs.
    template<typename PioSpiConfig,
             typename Clock,
             typename Dma,
             typename DmaChannels,
             std::size_t QueueDepth   = 8,
             std::size_t CallbackSize = 16,
             typename Timing          = PioSpiTiming>
    struct PioSpiQueued
      : Detail::
          PioSpiQueuedHalf<PioSpiConfig, Clock, Dma, DmaChannels, QueueDepth, CallbackSize, Timing>
      , QueuedFront<typename Detail::PioSpiQueuedHalf<PioSpiConfig,
                                                      Clock,
                                                      Dma,
                                                      DmaChannels,
                                                      QueueDepth,
                                                      CallbackSize,
                                                      Timing>::Core> {};

}}   // namespace Kvasir::SPI
