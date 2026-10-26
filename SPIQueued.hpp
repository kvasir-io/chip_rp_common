#pragma once
// Link kvasir::devices (not only kvasir::bus): the clock ceiling is a Units::Hertz (Quantities.hpp, mp-units).
// A queued SPI master for the PL022 (RP2040 / RP2350 SPI0/1) with DMA both ways, the SPI
// counterpart of I2CQueued. Each request brings its own mode, clock ceiling and CS functions;
// CS goes up in the RX DMA completion. Queue, holds, timeouts and watchdog are kvasir_devices'
// SPI/QueueCore.hpp. SPIBehavior (SPI.hpp) stays for net's W5500 and TC6.
#include "SPI.hpp"
#include "WaitBounds.hpp"
#include "kvasir/Atomic/Atomic.hpp"
#include "kvasir/Devices/Quantities.hpp"
#include "kvasir/Devices/SPI/QueueCore.hpp"
#include "kvasir/Devices/SPI/QueuedFront.hpp"

#include <cstdint>
#include <string_view>
#include <uc_log/uc_log.hpp>

namespace Kvasir { namespace SPI {

    /// DMA channel A sends, B receives; the completion of B ends a transfer.
    template<typename Dma,
             typename Dma::Channel  Tx,
             typename Dma::Channel  Rx,
             typename Dma::Priority Prio>
    struct QueuedDma {
        static constexpr auto txChannel = Tx;
        static constexpr auto rxChannel = Rx;
        static constexpr auto priority  = Prio;
    };

    namespace Detail {
        /// SPIQueued without its front: the PL022 registers, the DMA, the Startup steps (SPIBase) and the
        /// QueueCore over the Hw policy (kvasir_devices SPI/QueuedFront.hpp says why the two are apart).
        template<typename SPIConfig,
                 typename Clock,
                 typename Dma,
                 typename DmaChannels,
                 std::size_t QueueDepth,
                 std::size_t CallbackSize,
                 typename Timing>
        struct SPIQueuedHalf : SPIBase<SPIConfig> {
            using base = SPIBase<SPIConfig>;
            using Regs = typename base::Regs;

            static constexpr auto txChannel = DmaChannels::txChannel;
            static constexpr auto rxChannel = DmaChannels::rxChannel;
            static constexpr auto priority  = DmaChannels::priority;

            using Claims = brigand::append<typename base::Claims,
                                           Kvasir::DMA::Claims<Dma, txChannel, rxChannel>>;

            /// A device's mode and dividers at compile time: SCK = clk_peri / (CPSDVSR * (1 + SCR)),
            /// CPSDVSR even 2..254, SCR 0..255 (TRM 3.3.1 / 3.3.5), fastest not above the ceiling.
            struct Setup {
                std::uint8_t  scr{};
                std::uint8_t  cpsdvsr{2};
                std::uint8_t  spo{};
                std::uint8_t  sph{};
                std::uint32_t hz{SPIConfig::baudRate};
                std::uint32_t usPerBitQ10{Kvasir::SPI::usPerBitQ10(SPIConfig::baudRate)};

                constexpr bool operator==(Setup const&) const = default;
            };

            static consteval Setup setup(Hal::ClockMode mode,
                                         Units::Hertz   maxClock) {
                std::uint32_t const clk = SPIConfig::clockSpeed;
                // the slowest SCK is clk / (254 x 256), the fastest clk / 2 (CPSDVSR, 1 + SCR)
                auto const max = ceilingHz(maxClock.numerical_value_in(Units::si::hertz),
                                           static_cast<std::uint32_t>(SPIConfig::baudRate),
                                           clk,
                                           254U * 256U,
                                           2U);
                // SPI.hpp's search: the closest clock not above max, the smallest CPSDVSR on a tie
                auto const [scr, cps]
                  = base::Config::decodeDivider(base::Config::baudDivider(clk, max).setting);
                auto const bits = modeBits(mode);
                return Setup{.scr         = static_cast<std::uint8_t>(scr),
                             .cpsdvsr     = static_cast<std::uint8_t>(cps),
                             .spo         = bits.cpol,
                             .sph         = bits.cpha,
                             .hz          = clk / (cps * (scr + 1U)),
                             .usPerBitQ10 = Kvasir::SPI::usPerBitQ10(clk / (cps * (scr + 1U)))};
            }

            struct Snapshot {
                std::uint32_t sr{};    ///< SSPSR: TFE TNF RNE RFF BSY
                std::uint32_t ris{};   ///< SSPRIS: ROR RT RX TX
                std::uint32_t cr0{};
                std::uint32_t txLeft{};   ///< TRANS_COUNT of the TX channel
                std::uint32_t rxLeft{};
                bool          txBusy{};
                bool          rxBusy{};
                /// Aborts whose wait for the shifter ran out (a wedged block, or a bus below ~64 kHz).
                std::uint32_t shifterWaitsExhausted{};
            };

            struct Hw {
                static constexpr unsigned instance = base::instance;
                using Setup                        = SPIQueuedHalf::Setup;
                using Snapshot                     = SPIQueuedHalf::Snapshot;

                // Masks every interrupt, not only the DMA's: a device may submit() from its own
                // interrupt at a higher priority (the ADS8675's RVS pin). Bus and DMA interrupt
                // must be on one core.
                static void mask() { irqs_ = Kvasir::Core::IrqState::saveAndMask(); }

                static void unmask() { irqs_.restore(); }

                static void configure(Setup const& s) {
                    // Configured with SSE clear (TRM DDI0194H 2.3.2), before CS drops, so SCK
                    // settles to the new idle level first.
                    apply(clear(Regs::SSPCR1::sse));
                    apply(write(Regs::SSPCPSR::cpsdvsr, s.cpsdvsr));
                    apply(write(Regs::SSPCR0::scr, s.scr),
                          write(Regs::SSPCR0::spo, s.spo),
                          write(Regs::SSPCR0::sph, s.sph));
                    apply(set(Regs::SSPCR1::sse));
                }

                static void start(Transfer const& t,
                                  std::uint32_t   gen,
                                  void (*)(std::uint32_t,
                                           bool)) {
                    // Leftovers of an aborted transfer would be this one's first bytes.
                    for(int i = 0; i < 16 && apply(read(Regs::SSPSR::rne)); ++i) {
                        apply(read(Regs::SSPDR::data));
                    }
                    apply(set(Regs::SSPICR::roric));
                    // DSS 8 or 16 bit (TRM Table 3-2), with SSE clear, only when it changes.
                    auto const dss = static_cast<std::uint8_t>(t.wide ? 15U : 7U);
                    if(dss != dss_) {
                        apply(clear(Regs::SSPCR1::sse));
                        apply(write(Regs::SSPCR0::dss, dss));
                        apply(set(Regs::SSPCR1::sse));
                        dss_ = dss;
                    }
                    std::atomic_signal_fence(std::memory_order_release);
                    // RX first: the receiver is armed before the first bit is clocked in.
                    startRx_(t, gen);
                    startTx_(t);
                }

                static void abort() {
                    Dma::template abort<txChannel>();
                    Dma::template abort<rxChannel>();
                    // The block still clocks out up to 8 queued frames and has no flush but a reset
                    // (TRM 2.3.1); the next transfer would be shifted by them. Wait for SSPSR.BSY = 0
                    // (TRM Table 3-4, ~2 ms bound = 8 x 16 bits at 64 kHz), then drop what came in.
                    // A block that never goes idle is counted and left to the dead-bus watchdog.
                    bool idle = false;
                    for(std::uint32_t spins = 0; spins < shifterWaitSpins; ++spins) {
                        if(!apply(read(Regs::SSPSR::bsy))) {
                            idle = true;
                            break;
                        }
                    }
                    if(!idle) { ++shifterWaitsExhausted_; }
                    for(int i = 0; i < 64; ++i) {
                        if(!apply(read(Regs::SSPSR::rne))) { break; }
                        apply(read(Regs::SSPDR::data));
                    }
                }

                static void reinit() {
                    Dma::template abort<txChannel>();
                    Dma::template abort<rxChannel>();
                    apply(write(Rp::Block<Regs>::reset, 1U));
                    apply(write(Rp::Block<Regs>::reset, 0U));
                    Rp::Block<Regs>::waitResetDone();
                    apply(base::initStepPeripheryConfig);   // DSS 7 among it
                    apply(base::initStepPeripheryEnable);
                    dss_ = 7;
                }

                static Snapshot snapshot() {
                    return Snapshot{
                      .sr                    = get<0>(apply(read(Regs::SSPSR::FULLREGISTER))),
                      .ris                   = get<0>(apply(read(Regs::SSPRIS::FULLREGISTER))),
                      .cr0                   = get<0>(apply(read(Regs::SSPCR0::FULLREGISTER))),
                      .txLeft                = Dma::template remaining<txChannel>(),
                      .rxLeft                = Dma::template remaining<rxChannel>(),
                      .txBusy                = !Dma::template ready<txChannel>(),
                      .rxBusy                = !Dma::template ready<rxChannel>(),
                      .shifterWaitsExhausted = shifterWaitsExhausted_,
                    };
                }

                static void log([[maybe_unused]] Snapshot const& s) {
                    UC_LOG_W(
                      "spi{} at the timeout: SSPSR {:#04x}, SSPRIS {:#03x}, SSPCR0 {:#06x}, "
                      "TX DMA {} left{}, RX DMA {} left{}, {} abort(s) with the shifter never idle",
                      instance,
                      s.sr,
                      s.ris,
                      s.cr0,
                      s.txLeft,
                      std::string_view{s.txBusy ? " (busy)" : ""},
                      s.rxLeft,
                      std::string_view{s.rxBusy ? " (busy)" : ""},
                      s.shifterWaitsExhausted);
                }

            private:
                /// Spins of the abort's wait for SSPSR.BSY: ~2 ms of APB reads.
                static constexpr std::uint32_t shifterWaitSpins = 20'000;

                inline static Kvasir::Core::IrqState irqs_{};   ///< PRIMASK as mask() found it
                inline static std::uint8_t  dss_{7};   ///< SSPCR0.DSS as last written (init: 7)
                inline static std::uint32_t shifterWaitsExhausted_{};

                template<bool                       Inc,
                         typename Dma::TransferSize Size>
                static void rx_(Transfer const& t,
                                std::uint32_t   gen) {
                    Dma::template start<rxChannel,
                                        ::Kvasir::DMA::ChannelConfig{.priority = priority,
                                                                     .trigger  = base::rxDmaTrigger,
                                                                     .size     = Size,
                                                                     .incrementRead  = false,
                                                                     .incrementWrite = Inc}>(
                      reinterpret_cast<std::uint32_t>(t.rx),
                      Regs::SSPDR::Addr::value,
                      t.frames,
                      [gen]() {
                          bool const overrun = get<0>(apply(read(Regs::SSPRIS::rorris))) != 0;
                          if(overrun) { apply(set(Regs::SSPICR::roric)); }
                          SPIQueuedHalf::Core::complete(gen, overrun);
                      });
                }

                template<bool                       Inc,
                         typename Dma::TransferSize Size>
                static void tx_(Transfer const& t) {
                    Dma::template start<txChannel,
                                        ::Kvasir::DMA::ChannelConfig{.priority = priority,
                                                                     .trigger  = base::txDmaTrigger,
                                                                     .size     = Size,
                                                                     .incrementRead  = Inc,
                                                                     .incrementWrite = false}>(
                      Regs::SSPDR::Addr::value,
                      reinterpret_cast<std::uint32_t>(t.tx),
                      t.frames);
                }

                static void startRx_(Transfer const& t,
                                     std::uint32_t   gen) {
                    using TS = typename Dma::TransferSize;
                    if(t.wide) {
                        t.rxIncrement ? rx_<true, TS::size16>(t, gen)
                                      : rx_<false, TS::size16>(t, gen);
                    } else {
                        t.rxIncrement ? rx_<true, TS::size8>(t, gen)
                                      : rx_<false, TS::size8>(t, gen);
                    }
                }

                static void startTx_(Transfer const& t) {
                    using TS = typename Dma::TransferSize;
                    if(t.wide) {
                        t.txIncrement ? tx_<true, TS::size16>(t) : tx_<false, TS::size16>(t);
                    } else {
                        t.txIncrement ? tx_<true, TS::size8>(t) : tx_<false, TS::size8>(t);
                    }
                }
            };

            using Core = QueueCore<Hw, Clock, QueueDepth, CallbackSize, Timing>;
        };
    }   // namespace Detail

    /// SPIConfig as for SPIBehavior; baudRate = the fastest any device may be clocked, `mode`
    /// only the idle level at start (every request brings its own).
    template<typename SPIConfig,
             typename Clock,
             typename Dma,
             typename DmaChannels,
             std::size_t QueueDepth   = 8,
             std::size_t CallbackSize = 16,
             typename Timing          = QueueCoreDefaults>
    struct SPIQueued
      : Detail::SPIQueuedHalf<SPIConfig, Clock, Dma, DmaChannels, QueueDepth, CallbackSize, Timing>
      , QueuedFront<
          typename Detail::
            SPIQueuedHalf<SPIConfig, Clock, Dma, DmaChannels, QueueDepth, CallbackSize, Timing>::
              Core> {};

}}   // namespace Kvasir::SPI
