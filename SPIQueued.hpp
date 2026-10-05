#pragma once
// A queued SPI master for the PL022 (RP2040 / RP2350 SPI0/1) with DMA both ways, the SPI
// counterpart of I2CQueued. Each request brings its own mode, clock ceiling and CS functions;
// CS goes up in the RX DMA completion. Queue, holds, timeouts and watchdog are kvasir_devices'
// SPI/QueueCore.hpp. SPIBehavior (SPI.hpp) stays for net's W5500 and TC6.
#include "SPI.hpp"
#include "WaitBounds.hpp"
#include "kvasir/Atomic/Atomic.hpp"
#include "kvasir/Devices/Quantities.hpp"
#include "kvasir/Devices/SPI/QueueCore.hpp"
#include "kvasir/StartUp/Hooks.hpp"

#include <cstdint>
#include <string_view>

namespace Kvasir { namespace SPI {

    /// DMA channel A sends, B receives; the completion of B ends a transfer.
    template<typename Dma,
             typename Dma::Channel  Tx,
             typename Dma::Channel  Rx,
             typename Dma::Priority Prio>
    struct QueuedDma {
        static constexpr auto TxChannel = Tx;
        static constexpr auto RxChannel = Rx;
        static constexpr auto Priority  = Prio;
    };

    /// SPIConfig as for SPIBehavior; baudRate = the fastest any device may be clocked, `mode`
    /// only the idle level at start (every request brings its own).
    template<typename SPIConfig,
             typename Clock,
             typename Dma,
             typename DmaChannels,
             std::size_t QueueDepth   = 8,
             std::size_t CallbackSize = 16,
             typename Timing          = QueueCoreDefaults>
    struct SPIQueued : SPIBase<SPIConfig> {
        using base = SPIBase<SPIConfig>;
        using Regs = typename base::Regs;

        static constexpr auto TxChannel = DmaChannels::TxChannel;
        static constexpr auto RxChannel = DmaChannels::RxChannel;
        static constexpr auto Priority  = DmaChannels::Priority;

        using Claims
          = brigand::append<typename base::Claims, Kvasir::DMA::Claims<Dma, TxChannel, RxChannel>>;

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

        static consteval Setup setup(ClockMode    mode,
                                     Units::Hertz maxClock) {
            auto const max = std::min(maxClock.numerical_value_in(Units::si::hertz),
                                      static_cast<std::uint32_t>(SPIConfig::baudRate));

            std::uint32_t const clk = SPIConfig::clockSpeed;
            if(std::uint64_t{max} * 254U * 256U < clk) {
                // the note "in call to 'rateOutOfReach(wanted Hz, slowest mHz, fastest mHz)'"
                Prescaler::rateOutOfReach(max,
                                          std::uint64_t{clk} * 1000U / (254U * 256U),
                                          std::uint64_t{clk} * 1000U / 2U);
                return Setup{};
            }
            // SPI.hpp's search: the closest clock not above max, the smallest CPSDVSR on a tie
            auto const [scr, cps]
              = base::Config::decodeDivider(base::Config::baudDivider(clk, max).setting);
            auto const m = static_cast<std::uint8_t>(mode);
            return Setup{.scr         = static_cast<std::uint8_t>(scr),
                         .cpsdvsr     = static_cast<std::uint8_t>(cps),
                         .spo         = static_cast<std::uint8_t>((m >> 1U) & 1U),
                         .sph         = static_cast<std::uint8_t>(m & 1U),
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
            static constexpr unsigned Instance = base::Instance;
            using Setup                        = SPIQueued::Setup;
            using Snapshot                     = SPIQueued::Snapshot;

            // Masks every interrupt, not only the DMA's: a device may submit() from its own
            // interrupt at a higher priority (the ADS8675's RVS pin). Bus and DMA interrupt
            // must be on one core.
            static void mask() { enabled_ = Kvasir::Nvic::disable_all_and_get_old_state(); }

            static void unmask() {
                if(enabled_) { Kvasir::Nvic::enable_all(); }
            }

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
                Dma::template abort<TxChannel>();
                Dma::template abort<RxChannel>();
                // The block still clocks out up to 8 queued frames and has no flush but a reset
                // (TRM 2.3.1); the next transfer would be shifted by them. Wait for SSPSR.BSY = 0
                // (TRM Table 3-4, ~2 ms bound = 8 x 16 bits at 64 kHz), then drop what came in.
                // A block that never goes idle is counted and left to the dead-bus watchdog.
                bool idle = false;
                for(std::uint32_t spins = 0; spins < ShifterWaitSpins; ++spins) {
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
                Dma::template abort<TxChannel>();
                Dma::template abort<RxChannel>();
                apply(resetBit_(true));
                apply(resetBit_(false));
                Kvasir::Register::waitUntil<Kvasir::Chip::ResetDoneBound>(
                  Kvasir::Register::isSet(resetDoneBit_()));
                apply(base::initStepPeripheryConfig);   // DSS 7 among it
                apply(base::initStepPeripheryEnable);
                dss_ = 7;
            }

            static Snapshot snapshot() {
                return Snapshot{
                  .sr                    = get<0>(apply(read(Regs::SSPSR::FULLREGISTER))),
                  .ris                   = get<0>(apply(read(Regs::SSPRIS::FULLREGISTER))),
                  .cr0                   = get<0>(apply(read(Regs::SSPCR0::FULLREGISTER))),
                  .txLeft                = Dma::template remaining<TxChannel>(),
                  .rxLeft                = Dma::template remaining<RxChannel>(),
                  .txBusy                = !Dma::template ready<TxChannel>(),
                  .rxBusy                = !Dma::template ready<RxChannel>(),
                  .shifterWaitsExhausted = shifterWaitsExhausted_,
                };
            }

            static void log([[maybe_unused]] Snapshot const& s) {
                UC_LOG_W(
                  "spi{} at the timeout: SSPSR {:#04x}, SSPRIS {:#03x}, SSPCR0 {:#06x}, "
                  "TX DMA {} left{}, RX DMA {} left{}, {} abort(s) with the shifter never idle",
                  Instance,
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
            static constexpr std::uint32_t ShifterWaitSpins = 20'000;

            inline static bool          enabled_{};
            inline static std::uint8_t  dss_{7};   ///< SSPCR0.DSS as last written (init: 7)
            inline static std::uint32_t shifterWaitsExhausted_{};

            template<bool                       Inc,
                     typename Dma::TransferSize Size>
            static void rx_(Transfer const& t,
                            std::uint32_t   gen) {
                Dma::template start<RxChannel, Priority, base::RxDmaTrigger, Size, Inc, false>(
                  reinterpret_cast<std::uint32_t>(t.rx),
                  Regs::SSPDR::Addr::value,
                  t.frames,
                  [gen]() {
                      bool const overrun = get<0>(apply(read(Regs::SSPRIS::rorris))) != 0;
                      if(overrun) { apply(set(Regs::SSPICR::roric)); }
                      SPIQueued::Core::complete(gen, overrun);
                  });
            }

            template<bool                       Inc,
                     typename Dma::TransferSize Size>
            static void tx_(Transfer const& t) {
                Dma::template start<TxChannel, Priority, base::TxDmaTrigger, Size, false, Inc>(
                  Regs::SSPDR::Addr::value,
                  reinterpret_cast<std::uint32_t>(t.tx),
                  t.frames);
            }

            static void startRx_(Transfer const& t,
                                 std::uint32_t   gen) {
                using TS = typename Dma::TransferSize;
                if(t.wide) {
                    t.rxIncrement ? rx_<true, TS::_16>(t, gen) : rx_<false, TS::_16>(t, gen);
                } else {
                    t.rxIncrement ? rx_<true, TS::_8>(t, gen) : rx_<false, TS::_8>(t, gen);
                }
            }

            static void startTx_(Transfer const& t) {
                using TS = typename Dma::TransferSize;
                if(t.wide) {
                    t.txIncrement ? tx_<true, TS::_16>(t) : tx_<false, TS::_16>(t);
                } else {
                    t.txIncrement ? tx_<true, TS::_8>(t) : tx_<false, TS::_8>(t);
                }
            }

            static constexpr auto resetBit_(bool assert) {
                using R = Peripheral::RESETS::Registers<>::RESET;
                if constexpr(Instance == 0) {
                    return assert ? write(R::spi0, 1U) : write(R::spi0, 0U);
                } else {
                    return assert ? write(R::spi1, 1U) : write(R::spi1, 0U);
                }
            }

            static constexpr auto resetDoneBit_() {
                using R = Peripheral::RESETS::Registers<>::RESET_DONE;
                if constexpr(Instance == 0) {
                    return R::spi0;
                } else {
                    return R::spi1;
                }
            }
        };

        using Core    = QueueCore<Hw, Clock, QueueDepth, CallbackSize, Timing>;
        using Request = typename Core::RequestT;

        static bool submit(Request const& r) { return Core::submit(r); }

        /// QueueCoreFeatures::cancel / ::deadlines (kvasir_devices BusTypes.hpp): a ticket, and cancel(ticket).
        using Result = typename Core::Result;

        static Bus::Ticket submitTracked(Request const& r)
            requires(Core::Tracked)
        {
            return Core::submitTracked(r);
        }

        static Bus::Cancel cancel(Bus::Ticket t)
            requires(Core::Features.cancel)
        {
            return Core::cancel(t);
        }

        static void releaseHold(Lines const& l) { Core::releaseHold(l); }

        static void handler() { Core::handler(); }

        // once per main-loop turn: Startup::run<Kvasir::Hook::MainLoop>() calls it (StartUp/Hooks.hpp);
        // a firmware that runs the hook must not also call handler() by hand
        using Extends = Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &handler>;

        static void reset() { Core::reset(); }
    };

}}   // namespace Kvasir::SPI
