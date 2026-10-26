#pragma once

#include "Clocks.hpp"
#include "DMA.hpp"
#include "Io.hpp"
#include "PinConfig.hpp"
#include "chip/Blocks.hpp"   // the chip's block table (Rp::Block)
#include "core/Nvic.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Hal/Spi.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Prescaler.hpp"
#include "peripherals/SPI.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <ranges>
#include <span>

namespace Kvasir { namespace SPI {

    namespace Detail {

        // Use chip-agnostic pin configuration

        template<unsigned Instance>
        struct Config {
            using Regs = Kvasir::Peripheral::SPI::Registers<Instance>;

            // MOSI/SCLK/CS are push-pull outputs: 4 mA with slow slew (the pad
            // default) is plenty up to ~8 MHz; faster clocks get 8 mA with fast
            // slew. Only the default - an SPIConfig may override both, since on
            // fast buses a stronger drive trades timing margin against ringing
            // depending on the wiring.
            static constexpr Io::DriveStrength spiDrive(std::uint32_t fBaud) {
                return fBaud <= 8'000'000 ? Io::DriveStrength::mA4 : Io::DriveStrength::mA8;
            }

            static constexpr bool spiSlewFast(std::uint32_t fBaud) { return fBaud > 8'000'000; }

            template<Hal::ClockMode Mode>
            struct GetModeConfig {
                static constexpr auto config_ = []() {
                    if constexpr(Mode == Hal::ClockMode::mode0) {
                        return brigand::list<
                          decltype(write(Regs::SSPCR0::spo, Kvasir::Register::value<0>())),
                          decltype(write(Regs::SSPCR0::sph, Kvasir::Register::value<0>()))>{};
                    } else if constexpr(Mode == Hal::ClockMode::mode1) {
                        return brigand::list<
                          decltype(write(Regs::SSPCR0::spo, Kvasir::Register::value<0>())),
                          decltype(write(Regs::SSPCR0::sph, Kvasir::Register::value<1>()))>{};
                    } else if constexpr(Mode == Hal::ClockMode::mode2) {
                        return brigand::list<
                          decltype(write(Regs::SSPCR0::spo, Kvasir::Register::value<1>())),
                          decltype(write(Regs::SSPCR0::sph, Kvasir::Register::value<0>()))>{};
                    } else if constexpr(Mode == Hal::ClockMode::mode3) {
                        return brigand::list<
                          decltype(write(Regs::SSPCR0::spo, Kvasir::Register::value<1>())),
                          decltype(write(Regs::SSPCR0::sph, Kvasir::Register::value<1>()))>{};
                    }
                }();
                using config = decltype(config_);
            };

            // SCK = SSPCLK / (CPSDVSR x (1 + SCR)), CPSDVSR even 2..254, SCR 0..255 (PL022 TRM
            // DDI0194H 3.3.1, Table 3-6). One index over the grid, CPSDVSR outer: the closest rate
            // not above the request (a peripheral's maximum is never overshot), else the closest.
            static constexpr std::pair<std::uint32_t,
                                       std::uint32_t>
            decodeDivider(std::uint32_t i) {
                return {i % 256, 2 * (i / 256 + 1)};   // {scr, cpsdvsr}
            }

            // Per CPSDVSR only the two SCRs around the ideal divisor can win (the rate falls with
            // SCR), so the search sees 2 x 127 candidates instead of 32 512, in the same order:
            // the result is the full grid's (util_prescaler_tests compares the two).
            static constexpr auto baudDivider(std::uint32_t fClockSpeed,
                                              std::uint32_t fBaud) {
                auto const candidates
                  = std::views::iota(0U, 127U * 2U) | std::views::transform([=](std::uint32_t i) {
                        std::uint64_t const cps  = std::uint64_t{i / 2 + 1} * 2;
                        std::uint64_t const den  = cps * fBaud;
                        std::uint64_t const q    = fClockSpeed / den;   // 1 + SCR, floor
                        std::uint64_t const ceil = q + (fClockSpeed % den != 0 ? 1 : 0);
                        std::uint64_t const n    = i % 2 == 0 ? q : ceil;
                        auto const          scr
                          = static_cast<std::uint32_t>(std::clamp<std::uint64_t>(n, 1, 256) - 1);
                        return (i / 2) * 256 + scr;
                    });
                return Prescaler::search(
                  fClockSpeed,
                  fBaud,
                  candidates,
                  [](std::uint32_t i) {
                      auto const [scr, cps] = decodeDivider(i);
                      return Prescaler::Rational{1, std::uint64_t{cps} * (1 + scr)};
                  },
                  Prescaler::Pick::notAbove);
            }

            static constexpr std::pair<std::uint8_t,
                                       std::uint8_t>
            calcBaudRegs(std::uint32_t fClockSpeed,
                         std::uint32_t fBaud) {
                auto const [scr, cps] = decodeDivider(baudDivider(fClockSpeed, fBaud).setting);
                return {static_cast<std::uint8_t>(scr), static_cast<std::uint8_t>(cps)};
            }

            template<std::uint32_t FClockSpeed,
                     std::uint32_t FBaud>
            static constexpr auto getBaudConfig() {
                constexpr auto baudRegs = calcBaudRegs(FClockSpeed, FBaud);
                return list(
                  write(Regs::SSPCR0::scr, Register::value<std::get<0>(baudRegs)>()),
                  write(Regs::SSPCPSR::cpsdvsr, Register::value<std::get<1>(baudRegs)>()));
            }
        };
    }   // namespace Detail

    template<typename SPIConfig_>
    struct SPIBase {
        struct SPIConfig : SPIConfig_ {
            static constexpr auto userConfigOverride
              = KVASIR_CONFIG_OR(SPIConfig_, userConfigOverride, brigand::list<>{});

            static constexpr auto maxBaudRateError
              = KVASIR_CONFIG_OR(SPIConfig_, maxBaudRateError, std::ratio<1, 100>{});
            static constexpr auto misoPinLocation
              = KVASIR_CONFIG_OR(SPIConfig_, misoPinLocation, Io::NotUsed<>{});
            static constexpr auto mosiPinLocation
              = KVASIR_CONFIG_OR(SPIConfig_, mosiPinLocation, Io::NotUsed<>{});
            static constexpr auto csPinLocation
              = KVASIR_CONFIG_OR(SPIConfig_, csPinLocation, Io::NotUsed<>{});
        };

        // needed config
        // clockSpeed
        // baudRate
        // instance
        // mode
        // misoPinLocation
        // mosiPinLocation
        // sclkPinLocation
        // csPinLocation
        // userConfigOverride
        static constexpr auto instance = SPIConfig::instance;
        static_assert(instance < 2,
                      "invalid SPI instance");
        using Regs = Kvasir::Peripheral::SPI::Registers<instance>;

        using InterruptIndexs = typename Rp::Block<Regs>::Irqs;

        // Startup: this block, and the clock the baud divisor is computed from (the PL022
        // counts clk_peri).
        using Provides = brigand::list<Hal::Owns<Regs>>;
        using Claims   = Hal::Claim<Clocks::ClkPeri, SPIConfig::clockSpeed>;

        using Config = Detail::Config<instance>;

        static constexpr auto rxDmaTrigger = Rp::Block<Regs>::dreqRx;
        static constexpr auto txDmaTrigger = Rp::Block<Regs>::dreqTx;

        // the achieved SCK against maxBaudRateError; a failure prints wanted, got and ppm
        static constexpr bool baudInTolerance = [] {
            Prescaler::assertInTolerance<
              Config::baudDivider(SPIConfig::clockSpeed, SPIConfig::baudRate).achieved,
              SPIConfig::baudRate,
              Prescaler::Tolerance{SPIConfig::maxBaudRateError},
              "SPI SCK">();
            return true;
        }();
        // a static data member of a class template is initialised only when used: this use is
        // what runs the check
        static_assert(baudInTolerance);
        static_assert(Io::pinServes<PinConfig::Function::spiRx,
                                    instance>(SPIConfig::misoPinLocation));
        static_assert(Io::pinServes<PinConfig::Function::spiTx,
                                    instance>(SPIConfig::mosiPinLocation));
        static_assert(Io::pinServes<PinConfig::Function::spiSck,
                                    instance>(SPIConfig::sclkPinLocation));
        static_assert(Io::pinServes<PinConfig::Function::spiCs,
                                    instance>(SPIConfig::csPinLocation));

        static_assert(!Io::pinLocationEqual(SPIConfig::misoPinLocation,
                                            SPIConfig::mosiPinLocation),
                      "MISO and MOSI are the same pin");
        static_assert(!Io::pinLocationEqual(SPIConfig::misoPinLocation,
                                            SPIConfig::sclkPinLocation),
                      "MISO and SCLK are the same pin");
        static_assert(!Io::pinLocationEqual(SPIConfig::misoPinLocation,
                                            SPIConfig::csPinLocation),
                      "MISO and CS are the same pin");
        static_assert(!Io::pinLocationEqual(SPIConfig::mosiPinLocation,
                                            SPIConfig::sclkPinLocation),
                      "MOSI and SCLK are the same pin");
        static_assert(!Io::pinLocationEqual(SPIConfig::mosiPinLocation,
                                            SPIConfig::csPinLocation),
                      "MOSI and CS are the same pin");
        static_assert(!Io::pinLocationEqual(SPIConfig::sclkPinLocation,
                                            SPIConfig::csPinLocation),
                      "SCLK and CS are the same pin");

        static constexpr auto powerClockEnable = list(clear(Rp::Block<Regs>::reset));

        // Pad drive/slew for the output pins: optional `driveStrength` /
        // `slewFast` members override the rate-tiered default from
        // Detail::Config::spiDrive.
        static constexpr Io::DriveStrength pinDrive
          = KVASIR_CONFIG_OR(SPIConfig, driveStrength, Config::spiDrive(SPIConfig::baudRate));
        static constexpr bool pinSlewFast
          = KVASIR_CONFIG_OR(SPIConfig, slewFast, Config::spiSlewFast(SPIConfig::baudRate));

        static constexpr Io::PullConfiguration misoPull
          = KVASIR_CONFIG_OR(SPIConfig, misoPull, Io::PullConfiguration::pullNone);

        // MISO: SPIConfig::misoPull (default none) - an SD card releases MISO when not selected (SD spec 7.2.4), and
        // the RP2350's reset pull-down would read that as 0x00 = "no error"
        template<typename Location,
                 PinConfig::Function   F,
                 Io::DriveStrength     Drive,
                 bool                  Slew,
                 Io::PullConfiguration Pull = Io::PullConfiguration::pullNone>
        using PinSetUp = typename Io::Detail::
          PinConfigFor<std::decay_t<Location>, F, instance, Drive, Slew, Pull>::type;

        static constexpr auto initStepPinConfig
          = list(PinSetUp<decltype(SPIConfig::misoPinLocation),
                          PinConfig::Function::spiRx,
                          Io::DriveStrength::mA4,
                          false,
                          misoPull>{},
                 PinSetUp<decltype(SPIConfig::mosiPinLocation),
                          PinConfig::Function::spiTx,
                          pinDrive,
                          pinSlewFast>{},
                 PinSetUp<decltype(SPIConfig::sclkPinLocation),
                          PinConfig::Function::spiSck,
                          pinDrive,
                          pinSlewFast>{},
                 PinSetUp<decltype(SPIConfig::csPinLocation),
                          PinConfig::Function::spiCs,
                          pinDrive,
                          pinSlewFast>{});

        static constexpr auto initStepPeripheryConfig
          = list(Config::template getBaudConfig<SPIConfig::clockSpeed, SPIConfig::baudRate>(),

                 typename Config::template GetModeConfig<SPIConfig::mode>::config{},

                 write(Regs::SSPCR0::frf, Register::value<0>()),
                 write(Regs::SSPCR0::dss, Register::value<7>()),
                 clear(Regs::SSPCR1::sod),
                 clear(Regs::SSPCR1::ms),
                 clear(Regs::SSPCR1::sse),
                 clear(Regs::SSPCR1::lbm),

                 set(Regs::SSPDMACR::txdmae),
                 set(Regs::SSPDMACR::rxdmae),
                 SPIConfig::userConfigOverride);

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<SPIConfig::isrPriority>(InterruptIndexs{}),
                 Nvic::makeClearPending(InterruptIndexs{}));

        static constexpr auto initStepPeripheryEnable = list(set(Regs::SSPCR1::sse));
    };

    template<typename SPIConfig, typename Dma, typename DMAConfig>
    struct SPIBehavior : SPIBase<SPIConfig> {
        using base = SPIBase<SPIConfig>;
        using Regs = typename base::Regs;

        static constexpr auto dmaChannelA = DMAConfig::channelA;
        static constexpr auto dmaChannelB = DMAConfig::channelB;
        static constexpr auto dmaPriority = DMAConfig::priority;

        using Claims = brigand::append<typename base::Claims,
                                       Kvasir::DMA::Claims<Dma, dmaChannelA, dmaChannelB>>;

        enum class OperationState { succeeded, failed, ongoing };

    private:
        inline static std::atomic<bool> busy{false};
        inline static std::atomic<bool> booked{false};
        inline static std::atomic<bool> error{false};

    public:
        static bool acquire() { return !booked.exchange(true, std::memory_order_acquire); }

        static void release() { booked.store(false, std::memory_order_release); }

        static OperationState operationState() {
            auto const bsy = get<0>(apply(read(Regs::SSPSR::bsy)));
            if(!busy && !bsy) {
                return error.load(std::memory_order_relaxed) ? OperationState::failed
                                                             : OperationState::succeeded;
            }
            return OperationState::ongoing;
        }

        // Is a transfer in flight? Use this to decide whether the next one may
        // start, not operationState(): that reports the outcome of the last
        // transfer, and `error` (set by an RX FIFO overrun) latches until the
        // next transfer starts - so gating on `succeeded` waits for something
        // only the caller can cause, with no timeout and no way out from the
        // far end of the bus. Ask operationState() only whether the data that
        // arrived can be trusted.
        [[nodiscard]] static bool transferInProgress() {
            return busy.load(std::memory_order_relaxed) || get<0>(apply(read(Regs::SSPSR::bsy)));
        }

        // Acknowledge a failed transfer. Not needed for sequencing - the next
        // transfer clears the flag anyway - but lets a caller treat
        // operationState() as reporting only what happened since it last looked.
        static void clearError() { error.store(false, std::memory_order_relaxed); }

        // One consistent snapshot of what decides whether the bus is stuck, to
        // be logged before aborting. A stall has several causes - RX DMA
        // starved short of its count, TX DMA never draining, a lost completion
        // callback - that are only distinguishable while it is still stuck.
        struct BusDebug {
            bool          busyFlag;
            bool          sspBusy;
            bool          rxFifoNotEmpty;
            bool          rxOverrun;
            std::uint32_t txDmaRemaining;
            std::uint32_t rxDmaRemaining;
            bool          txDmaBusy;
            bool          rxDmaBusy;
        };

        [[nodiscard]] static BusDebug busDebug() {
            return {.busyFlag       = busy.load(std::memory_order_relaxed),
                    .sspBusy        = static_cast<bool>(get<0>(apply(read(Regs::SSPSR::bsy)))),
                    .rxFifoNotEmpty = static_cast<bool>(get<0>(apply(read(Regs::SSPSR::rne)))),
                    .rxOverrun      = static_cast<bool>(get<0>(apply(read(Regs::SSPRIS::rorris)))),
                    .txDmaRemaining = Dma::template remaining<dmaChannelA>(),
                    .rxDmaRemaining = Dma::template remaining<dmaChannelB>(),
                    .txDmaBusy      = !Dma::template ready<dmaChannelA>(),
                    .rxDmaBusy      = !Dma::template ready<dmaChannelB>()};
        }

        // Abandon an in-flight transfer and leave the bus usable. Without it a
        // caller that gives up cannot stop the DMA: the channel keeps running,
        // `busy` is never cleared by a completion nobody is listening for, and
        // every later transfer inherits a channel that is still live.
        static void abortTransfer() {
            Dma::template abort<dmaChannelA>();
            Dma::template abort<dmaChannelB>();
            busy.store(false, std::memory_order_relaxed);
            error.store(false, std::memory_order_relaxed);
            // Leftovers in the RX FIFO would be read as the first bytes of the
            // next transfer. Bounded: the FIFO is 8 deep and with both channels
            // aborted the master stops clocking once the TX FIFO drains.
            for(int i = 0; i < 64; ++i) {
                if(!apply(read(Regs::SSPSR::rne))) { break; }
                apply(read(Regs::SSPDR::data));
            }
        }

        static void sendNocopy(std::span<std::byte const> inData) {
            sendNocopyImpl<true>(inData.data(), inData.size(), std::nullopt);
        }

        template<typename F>
        static void sendNocopy(std::span<std::byte const> inData,
                               F                          f) {
            sendNocopyImpl<true>(inData.data(), inData.size(), f);
        }

        static void sendNocopyStatic(std::byte const* staticValue,
                                     std::size_t      size) {
            sendNocopyImpl<false>(staticValue, size, std::nullopt);
        }

        template<typename F>
        static void sendNocopyStatic(std::byte const* staticValue,
                                     std::size_t      size,
                                     F                f) {
            sendNocopyImpl<false>(staticValue, size, f);
        }

        static void sendReceiveNocopy(std::span<std::byte> inOutData) {
            sendReceiveNocopy(inOutData, inOutData, std::nullopt);
        }

        template<typename F>
        static void sendReceiveNocopy(std::span<std::byte> inOutData,
                                      F                    f) {
            sendReceiveNocopy(inOutData, inOutData, f);
        }

        static void sendReceiveNocopyStatic(std::byte const*     staticValue,
                                            std::span<std::byte> outData) {
            sendReceiveNocopyImpl<false>(staticValue, outData.data(), outData.size(), std::nullopt);
        }

        template<typename F>
        static void sendReceiveNocopyStatic(std::byte const*     staticValue,
                                            std::span<std::byte> outData,
                                            F                    f) {
            sendReceiveNocopyImpl<false>(staticValue, outData.data(), outData.size(), f);
        }

        static void sendReceiveNocopy(std::span<std::byte const> inData,
                                      std::span<std::byte>       outData) {
            assert(inData.size() == outData.size());
            sendReceiveNocopyImpl<true>(inData.data(), outData.data(), inData.size(), std::nullopt);
        }

        template<typename F>
        static void sendReceiveNocopy(std::span<std::byte const> inData,
                                      std::span<std::byte>       outData,
                                      F                          f) {
            assert(inData.size() == outData.size());
            sendReceiveNocopyImpl<true>(inData.data(), outData.data(), inData.size(), f);
        }

    private:
        template<bool Increment,
                 typename F>
        static void sendNocopyImpl(std::byte const* first,
                                   std::size_t      size,
                                   F                f) {
            assert(!busy);

            // Drain any stale data left in the RX FIFO. The RX FIFO keeps filling during a
            // transmit-only transfer (rxdmae is enabled but no RX channel is armed), so clearing
            // it here keeps a later send_receive from reading garbage. Bounded by the block: it is
            // master only (SSPCR1.MS clear), so no frame arrives without the core's TX data; with
            // `busy` clear at most the 8-deep RX FIFO is full and up to 8 TX entries plus the one
            // in the shifter still clock out - 17 reads at most (PL022 FIFOs "8-locations deep":
            // RP2040 datasheet 4.4.2.5 md l.23438, RP2350 12.3.3.5 md l.51333).
            while(apply(read(Regs::SSPSR::rne))) { apply(read(Regs::SSPDR::data)); }

            error.store(false, std::memory_order_relaxed);

            busy = true;

            std::atomic_signal_fence(std::memory_order_release);

            Dma::template start<dmaChannelA,
                                ::Kvasir::DMA::ChannelConfig{.priority = dmaPriority,
                                                             .trigger  = base::txDmaTrigger,
                                                             .size     = Dma::TransferSize::size8,
                                                             .incrementRead  = Increment,
                                                             .incrementWrite = false}>(
              Regs::SSPDR::Addr::value,
              reinterpret_cast<std::uint32_t>(first),
              size,
              [f]() {
                  busy = false;
                  if constexpr(!std::is_same_v<F, std::nullopt_t>) {
                      f();
                  } else {
                      (void)f;
                  }
              });
        }

        template<bool Increment,
                 typename F>
        static void sendReceiveNocopyImpl(std::byte const* first,
                                          std::byte*       firstOut,
                                          std::size_t      size,
                                          F                f) {
            assert(!busy);

            // Bounded like send_nocopy_impl's drain: master only, at most 17 reads.
            while(apply(read(Regs::SSPSR::rne))) { apply(read(Regs::SSPDR::data)); }

            error.store(false, std::memory_order_relaxed);
            apply(set(Regs::SSPICR::roric));   // clear any stale receive-overrun flag

            busy = true;

            std::atomic_signal_fence(std::memory_order_release);

            // Arm the RX channel (B) before the TX channel (A) so the receiver is ready before
            // the SSP starts clocking out data and filling the RX FIFO.
            Dma::template start<dmaChannelB,
                                ::Kvasir::DMA::ChannelConfig{.priority = dmaPriority,
                                                             .trigger  = base::rxDmaTrigger,
                                                             .size     = Dma::TransferSize::size8,
                                                             .incrementRead  = false,
                                                             .incrementWrite = true}>(
              reinterpret_cast<std::uint32_t>(firstOut),
              Regs::SSPDR::Addr::value,
              size,
              [f]() {
                  if(get<0>(apply(read(Regs::SSPRIS::rorris)))) {
                      error.store(true, std::memory_order_relaxed);
                      apply(set(Regs::SSPICR::roric));
                  }
                  busy = false;

                  if constexpr(!std::is_same_v<F, std::nullopt_t>) {
                      f();
                  } else {
                      (void)f;
                  }
              });

            Dma::template start<dmaChannelA,
                                ::Kvasir::DMA::ChannelConfig{.priority = dmaPriority,
                                                             .trigger  = base::txDmaTrigger,
                                                             .size     = Dma::TransferSize::size8,
                                                             .incrementRead  = Increment,
                                                             .incrementWrite = false}>(
              Regs::SSPDR::Addr::value,
              reinterpret_cast<std::uint32_t>(first),
              size);
        }
    };
}}   // namespace Kvasir::SPI
