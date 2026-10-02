#pragma once

#include "Clocks.hpp"
#include "DMA.hpp"
#include "Io.hpp"
#include "PinConfig.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Util/Prescaler.hpp"
#include "peripherals/SPI.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <ranges>
#include <span>

namespace Kvasir { namespace SPI {

    // Startup resource: the SPI block itself. Two drivers on one instance would each
    // configure it (kvasir/StartUp/Resources.hpp).
    struct InstanceTag {};

    enum class Mode {
        _0,   //CPOL0_CPHA0
        _1,   //CPOL0_CPHA1
        _2,   //CPOL1_CPHA0
        _3    //CPOL1_CPHA1
    };

    namespace Detail {

        // Use chip-agnostic pin configuration

        template<unsigned Instance>
        struct Config {
            using Regs = Kvasir::Peripheral::SPI::Registers<Instance>;

            static constexpr bool isValidPinLocationMISO(Io::NotUsed<>) { return true; }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationMISO(Kvasir::Register::PinLocation<Port,
                                                                                       Pin>) {
                return Instance == 0 ? PinConfig::isValidSpiPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::SpiPinType::Rx0)
                                     : PinConfig::isValidSpiPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::SpiPinType::Rx1);
            }

            static constexpr bool isValidPinLocationMOSI(Io::NotUsed<>) { return true; }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationMOSI(Kvasir::Register::PinLocation<Port,
                                                                                       Pin>) {
                return Instance == 0 ? PinConfig::isValidSpiPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::SpiPinType::Tx0)
                                     : PinConfig::isValidSpiPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::SpiPinType::Tx1);
            }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationSCLK(Kvasir::Register::PinLocation<Port,
                                                                                       Pin>) {
                return Instance == 0 ? PinConfig::isValidSpiPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::SpiPinType::Sck0)
                                     : PinConfig::isValidSpiPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::SpiPinType::Sck1);
            }

            static constexpr bool isValidPinLocationCS(Io::NotUsed<>) { return true; }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationCS(Kvasir::Register::PinLocation<Port,
                                                                                     Pin>) {
                return Instance == 0 ? PinConfig::isValidSpiPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::SpiPinType::Cs0)
                                     : PinConfig::isValidSpiPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::SpiPinType::Cs1);
            }

            // MOSI/SCLK/CS are push-pull outputs: 4 mA with slow slew (the pad
            // default) is plenty up to ~8 MHz; faster clocks get 8 mA with fast
            // slew. Only the default - an SPIConfig may override both, since on
            // fast buses a stronger drive trades timing margin against ringing
            // depending on the wiring.
            static constexpr Io::DriveStrength spiDrive(std::uint32_t f_baud) {
                return f_baud <= 8'000'000 ? Io::DriveStrength::mA_4 : Io::DriveStrength::mA_8;
            }

            static constexpr bool spiSlewFast(std::uint32_t f_baud) { return f_baud > 8'000'000; }

            // SPIConfig::misoPull (default none): an SD card releases MISO when not selected
            // (SD spec 7.2.4), and the RP2350's reset pull-down would read that as 0x00 = "no error".
            template<typename MISOPIN, Io::PullConfiguration Pull = Io::PullConfiguration::PullNone>
            struct GetMISOPinConfig;

            template<typename dummy, Io::PullConfiguration Pull>
            struct GetMISOPinConfig<Io::NotUsed<dummy>, Pull> {
                using pinConfig = brigand::list<>;
            };

            template<int Port, int Pin, Io::PullConfiguration Pull>
            struct GetMISOPinConfig<Kvasir::Register::PinLocation<Port, Pin>, Pull> {
                using pinConfig
                  = decltype(action(Kvasir::Io::Action::PinFunction<1,
                                                                    Io::OutputType::PushPull,
                                                                    Io::OutputSpeed::Low,
                                                                    Io::OutputInit::Low,
                                                                    Pull>{},
                                    Register::PinLocation<Port, Pin>{}));
            };
            template<typename CSPIN, Io::DriveStrength Drive, bool SlewFast>
            struct GetCSPinConfig;

            template<typename dummy, Io::DriveStrength Drive, bool SlewFast>
            struct GetCSPinConfig<Io::NotUsed<dummy>, Drive, SlewFast> {
                using pinConfig = brigand::list<>;
            };

            template<int Port, int Pin, Io::DriveStrength Drive, bool SlewFast>
            struct GetCSPinConfig<Kvasir::Register::PinLocation<Port, Pin>, Drive, SlewFast> {
                using pinConfig
                  = decltype(action(Kvasir::Io::Action::PinFunctionDrive<1, Drive, SlewFast>{},
                                    Register::PinLocation<Port, Pin>{}));
            };

            template<typename MOSIPIN, Io::DriveStrength Drive, bool SlewFast>
            struct GetMOSIPinConfig;

            template<typename dummy, Io::DriveStrength Drive, bool SlewFast>
            struct GetMOSIPinConfig<Io::NotUsed<dummy>, Drive, SlewFast> {
                using pinConfig = brigand::list<>;
            };

            template<int Port, int Pin, Io::DriveStrength Drive, bool SlewFast>
            struct GetMOSIPinConfig<Kvasir::Register::PinLocation<Port, Pin>, Drive, SlewFast> {
                using pinConfig
                  = decltype(action(Kvasir::Io::Action::PinFunctionDrive<1, Drive, SlewFast>{},
                                    Register::PinLocation<Port, Pin>{}));
            };

            template<typename SCLKPIN, Io::DriveStrength Drive, bool SlewFast>
            struct GetSCLKPinConfig;

            template<int Port, int Pin, Io::DriveStrength Drive, bool SlewFast>
            struct GetSCLKPinConfig<Kvasir::Register::PinLocation<Port, Pin>, Drive, SlewFast> {
                using pinConfig
                  = decltype(action(Kvasir::Io::Action::PinFunctionDrive<1, Drive, SlewFast>{},
                                    Register::PinLocation<Port, Pin>{}));
            };

            template<Mode mode>
            struct GetModeConfig {
                static constexpr auto config_ = []() {
                    if constexpr(mode == Mode::_0) {
                        return brigand::list<
                          decltype(write(Regs::SSPCR0::spo, Kvasir::Register::value<0>())),
                          decltype(write(Regs::SSPCR0::sph, Kvasir::Register::value<0>()))>{};
                    } else if constexpr(mode == Mode::_1) {
                        return brigand::list<
                          decltype(write(Regs::SSPCR0::spo, Kvasir::Register::value<0>())),
                          decltype(write(Regs::SSPCR0::sph, Kvasir::Register::value<1>()))>{};
                    } else if constexpr(mode == Mode::_2) {
                        return brigand::list<
                          decltype(write(Regs::SSPCR0::spo, Kvasir::Register::value<1>())),
                          decltype(write(Regs::SSPCR0::sph, Kvasir::Register::value<0>()))>{};
                    } else if constexpr(mode == Mode::_3) {
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
            static constexpr auto baudDivider(std::uint32_t f_clockSpeed,
                                              std::uint32_t f_baud) {
                auto const candidates
                  = std::views::iota(0U, 127U * 2U) | std::views::transform([=](std::uint32_t i) {
                        std::uint64_t const cps  = 2 * (i / 2 + 1);
                        std::uint64_t const den  = cps * f_baud;
                        std::uint64_t const q    = f_clockSpeed / den;   // 1 + SCR, floor
                        std::uint64_t const ceil = q + (f_clockSpeed % den != 0 ? 1 : 0);
                        std::uint64_t const n    = i % 2 == 0 ? q : ceil;
                        auto const          scr
                          = static_cast<std::uint32_t>(std::clamp<std::uint64_t>(n, 1, 256) - 1);
                        return (i / 2) * 256 + scr;
                    });
                return Prescaler::search(
                  f_clockSpeed,
                  f_baud,
                  candidates,
                  [](std::uint32_t i) {
                      auto const [scr, cps] = decodeDivider(i);
                      return Prescaler::Rational{1, std::uint64_t{cps} * (1 + scr)};
                  },
                  Prescaler::Pick::notAbove);
            }

            static constexpr std::pair<std::uint8_t,
                                       std::uint8_t>
            calcBaudRegs(std::uint32_t f_clockSpeed,
                         std::uint32_t f_baud) {
                auto const [scr, cps] = decodeDivider(baudDivider(f_clockSpeed, f_baud).setting);
                return {static_cast<std::uint8_t>(scr), static_cast<std::uint8_t>(cps)};
            }

            template<std::uint32_t f_clockSpeed,
                     std::uint32_t f_baud>
            static constexpr auto getBaudConfig() {
                constexpr auto baudRegs = calcBaudRegs(f_clockSpeed, f_baud);
                return list(
                  write(Regs::SSPCR0::scr, Register::value<std::get<0>(baudRegs)>()),
                  write(Regs::SSPCPSR::cpsdvsr, Register::value<std::get<1>(baudRegs)>()));
            }
        };
    }   // namespace Detail

    namespace Traits { namespace SPI {
        template<unsigned Instance>
        static constexpr auto getIsrIndexs() {
            static_assert(Instance < 2, "invalid SPI instance");
            if constexpr(Instance == 0) {
                return brigand::list<decltype(Kvasir::Interrupt::spi0)>{};
            } else if constexpr(Instance == 1) {
                return brigand::list<decltype(Kvasir::Interrupt::spi1)>{};
            }
        }

        template<unsigned Instance>
        static constexpr auto getEnable() {
            static_assert(Instance < 2, "invalid SPI instance");
            if constexpr(Instance == 0) {
                return clear(Peripheral::RESETS::Registers<>::RESET::spi0);
            } else if constexpr(Instance == 1) {
                return clear(Peripheral::RESETS::Registers<>::RESET::spi1);
            }
        }

        template<unsigned Instance>
        static constexpr auto DmaRX_Trigger() {
            static_assert(Instance < 2, "invalid SPI instance");
            if constexpr(Instance == 0) {
                return DMA::TriggerSource::spi0_rx;
            } else if constexpr(Instance == 1) {
                return DMA::TriggerSource::spi1_rx;
            }
        }

        template<unsigned Instance>
        static constexpr auto DmaTX_Trigger() {
            static_assert(Instance < 2, "invalid SPI instance");
            if constexpr(Instance == 0) {
                return DMA::TriggerSource::spi0_tx;
            } else if constexpr(Instance == 1) {
                return DMA::TriggerSource::spi1_tx;
            }
        }

    }}   // namespace Traits::SPI

    template<typename SPIConfig_>
    struct SPIBase {
        struct SPIConfig : SPIConfig_ {
            static constexpr auto userConfigOverride = [] {
                if constexpr(requires { SPIConfig_::userConfigOverride; }) {
                    return SPIConfig_::userConfigOverride;
                } else {
                    return brigand::list<>{};
                }
            }();

            static constexpr auto maxBaudRateError = [] {
                if constexpr(requires { SPIConfig_::maxBaudRateError; }) {
                    return SPIConfig_::maxBaudRateError;
                } else {
                    return std::ratio<1, 100>{};
                }
            }();
            static constexpr auto misoPinLocation = [] {
                if constexpr(requires { SPIConfig_::misoPinLocation; }) {
                    return SPIConfig_::misoPinLocation;
                } else {
                    return Io::NotUsed<>{};
                }
            }();
            static constexpr auto mosiPinLocation = [] {
                if constexpr(requires { SPIConfig_::mosiPinLocation; }) {
                    return SPIConfig_::mosiPinLocation;
                } else {
                    return Io::NotUsed<>{};
                }
            }();
            static constexpr auto csPinLocation = [] {
                if constexpr(requires { SPIConfig_::csPinLocation; }) {
                    return SPIConfig_::csPinLocation;
                } else {
                    return Io::NotUsed<>{};
                }
            }();
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
        static constexpr auto Instance = SPIConfig::instance;
        static_assert(Instance < 2,
                      "invalid SPI instance");
        using Regs = Kvasir::Peripheral::SPI::Registers<Instance>;

        using InterruptIndexs = decltype(Traits::SPI::getIsrIndexs<Instance>());

        // Startup: this block, and the clock the baud divisor is computed from (the PL022
        // counts clk_peri).
        using Provides = brigand::list<Startup::Resource<InstanceTag, Instance>>;
        using Claims   = Clocks::Claim<Clocks::ClkPeri, SPIConfig::clockSpeed>;

        using Config = Detail::Config<Instance>;

        static constexpr auto RxDmaTrigger = Traits::SPI::DmaRX_Trigger<Instance>();
        static constexpr auto TxDmaTrigger = Traits::SPI::DmaTX_Trigger<Instance>();

        // the achieved SCK against maxBaudRateError; a failure prints wanted, got and ppm
        static constexpr bool BaudInTolerance = [] {
            Prescaler::assertInTolerance<
              Config::baudDivider(SPIConfig::clockSpeed, SPIConfig::baudRate).achieved,
              SPIConfig::baudRate,
              Prescaler::Tolerance{SPIConfig::maxBaudRateError},
              "SPI SCK">();
            return true;
        }();
        // a static data member of a class template is initialised only when used: this use is
        // what runs the check
        static_assert(BaudInTolerance);
        static_assert(Config::isValidPinLocationMISO(SPIConfig::misoPinLocation),
                      "invalid MISOPin");
        static_assert(Config::isValidPinLocationMOSI(SPIConfig::mosiPinLocation),
                      "invalid MOSIPin");
        static_assert(Config::isValidPinLocationSCLK(SPIConfig::sclkPinLocation),
                      "invalid SCLKPin");
        static_assert(Config::isValidPinLocationCS(SPIConfig::csPinLocation),
                      "invalid CSPin");

        static_assert(!Io::Detail::PinLocationEqual(SPIConfig::misoPinLocation,
                                                    SPIConfig::mosiPinLocation),
                      "MISO and MOSI are the same pin");
        static_assert(!Io::Detail::PinLocationEqual(SPIConfig::misoPinLocation,
                                                    SPIConfig::sclkPinLocation),
                      "MISO and SCLK are the same pin");
        static_assert(!Io::Detail::PinLocationEqual(SPIConfig::misoPinLocation,
                                                    SPIConfig::csPinLocation),
                      "MISO and CS are the same pin");
        static_assert(!Io::Detail::PinLocationEqual(SPIConfig::mosiPinLocation,
                                                    SPIConfig::sclkPinLocation),
                      "MOSI and SCLK are the same pin");
        static_assert(!Io::Detail::PinLocationEqual(SPIConfig::mosiPinLocation,
                                                    SPIConfig::csPinLocation),
                      "MOSI and CS are the same pin");
        static_assert(!Io::Detail::PinLocationEqual(SPIConfig::sclkPinLocation,
                                                    SPIConfig::csPinLocation),
                      "SCLK and CS are the same pin");

        static constexpr auto powerClockEnable = list(Traits::SPI::getEnable<Instance>());

        // Pad drive/slew for the output pins: optional `driveStrength` /
        // `slewFast` members override the rate-tiered default from
        // Detail::Config::spiDrive.
        static constexpr Io::DriveStrength pinDrive = [] {
            if constexpr(requires { SPIConfig::driveStrength; }) {
                return SPIConfig::driveStrength;
            } else {
                return Config::spiDrive(SPIConfig::baudRate);
            }
        }();
        static constexpr bool pinSlewFast = [] {
            if constexpr(requires { SPIConfig::slewFast; }) {
                return SPIConfig::slewFast;
            } else {
                return Config::spiSlewFast(SPIConfig::baudRate);
            }
        }();

        static constexpr Io::PullConfiguration misoPull = [] {
            if constexpr(requires { SPIConfig::misoPull; }) {
                return SPIConfig::misoPull;
            } else {
                return Io::PullConfiguration::PullNone;
            }
        }();

        static constexpr auto initStepPinConfig = list(
          typename Config::template GetMISOPinConfig<
            std::decay_t<decltype(SPIConfig::misoPinLocation)>,
            misoPull>::pinConfig{},
          typename Config::template GetMOSIPinConfig<
            std::decay_t<decltype(SPIConfig::mosiPinLocation)>,
            pinDrive,
            pinSlewFast>::pinConfig{},
          typename Config::template GetSCLKPinConfig<
            std::decay_t<decltype(SPIConfig::sclkPinLocation)>,
            pinDrive,
            pinSlewFast>::pinConfig{},
          typename Config::template GetCSPinConfig<std::decay_t<decltype(SPIConfig::csPinLocation)>,
                                                   pinDrive,
                                                   pinSlewFast>::pinConfig{});

        static constexpr auto initStepPeripheryConfig
          = list(Config::template getBaudConfig<SPIConfig::clockSpeed, SPIConfig::baudRate>(),

                 typename Config::template GetModeConfig<SPIConfig::mode>::config{},

                 write(Regs::SSPCR0::frf, Register::value<0>()),
                 write(Regs::SSPCR0::dss, Register::value<7>()),
                 clear(Regs::SSPCR1::sod),
                 clear(Regs::SSPCR1::ms),
                 clear(Regs::SSPCR1::sse),
                 clear(Regs::SSPCR1::lbm),

                 /*clear(Regs::SSPIMSC::txim),
          clear(Regs::SSPIMSC::rxim),
          clear(Regs::SSPIMSC::rtim),
          clear(Regs::SSPIMSC::rorim),*/

                 set(Regs::SSPDMACR::txdmae),
                 set(Regs::SSPDMACR::rxdmae),
                 SPIConfig::userConfigOverride);

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<SPIConfig::isrPriority>(InterruptIndexs{}),
                 Nvic::makeClearPending(InterruptIndexs{}));

        static constexpr auto initStepPeripheryEnable = list(set(Regs::SSPCR1::sse)
                                                             //, Nvic::makeEnable(InterruptIndexs{})
        );
    };

    template<typename SPIConfig, typename Dma, typename DMAConfig>
    struct SPIBehavior : SPIBase<SPIConfig> {
        using base = SPIBase<SPIConfig>;
        using Regs = typename base::Regs;

        static constexpr auto DmaChannelA = DMAConfig::ChannelA;
        static constexpr auto DmaChannelB = DMAConfig::ChannelB;
        static constexpr auto DmaPriority = DMAConfig::Priority;

        using Claims = brigand::append<typename base::Claims,
                                       Kvasir::DMA::Claims<Dma, DmaChannelA, DmaChannelB>>;

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
                    .txDmaRemaining = Dma::template remaining<DmaChannelA>(),
                    .rxDmaRemaining = Dma::template remaining<DmaChannelB>(),
                    .txDmaBusy      = !Dma::template ready<DmaChannelA>(),
                    .rxDmaBusy      = !Dma::template ready<DmaChannelB>()};
        }

        // Abandon an in-flight transfer and leave the bus usable. Without it a
        // caller that gives up cannot stop the DMA: the channel keeps running,
        // `busy` is never cleared by a completion nobody is listening for, and
        // every later transfer inherits a channel that is still live.
        static void abortTransfer() {
            Dma::template abort<DmaChannelA>();
            Dma::template abort<DmaChannelB>();
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

        static void send_nocopy(std::span<std::byte const> inData) {
            send_nocopy_impl<true>(inData.data(), inData.size(), std::nullopt);
        }

        template<typename F>
        static void send_nocopy(std::span<std::byte const> inData,
                                F                          f) {
            send_nocopy_impl<true>(inData.data(), inData.size(), f);
        }

        static void send_nocopy_static(std::byte const* staticValue,
                                       std::size_t      size) {
            send_nocopy_impl<false>(staticValue, size, std::nullopt);
        }

        template<typename F>
        static void send_nocopy_static(std::byte const* staticValue,
                                       std::size_t      size,
                                       F                f) {
            send_nocopy_impl<false>(staticValue, size, f);
        }

        static void send_receive_nocopy(std::span<std::byte> inOutData) {
            send_receive_nocopy(inOutData, inOutData, std::nullopt);
        }

        template<typename F>
        static void send_receive_nocopy(std::span<std::byte> inOutData,
                                        F                    f) {
            send_receive_nocopy(inOutData, inOutData, f);
        }

        static void send_receive_nocopy_static(std::byte const*     staticValue,
                                               std::span<std::byte> outData) {
            send_receive_nocopy_impl<false>(staticValue,
                                            outData.data(),
                                            outData.size(),
                                            std::nullopt);
        }

        template<typename F>
        static void send_receive_nocopy_static(std::byte const*     staticValue,
                                               std::span<std::byte> outData,
                                               F                    f) {
            send_receive_nocopy_impl<false>(staticValue, outData.data(), outData.size(), f);
        }

        static void send_receive_nocopy(std::span<std::byte const> inData,
                                        std::span<std::byte>       outData) {
            assert(inData.size() == outData.size());
            send_receive_nocopy_impl<true>(inData.data(),
                                           outData.data(),
                                           inData.size(),
                                           std::nullopt);
        }

        template<typename F>
        static void send_receive_nocopy(std::span<std::byte const> inData,
                                        std::span<std::byte>       outData,
                                        F                          f) {
            assert(inData.size() == outData.size());
            send_receive_nocopy_impl<true>(inData.data(), outData.data(), inData.size(), f);
        }

    private:
        template<bool increment,
                 typename F>
        static void send_nocopy_impl(std::byte const* first,
                                     std::size_t      size,
                                     F                f) {
            assert(!busy);

            // Drain any stale data left in the RX FIFO. The RX FIFO keeps filling during a
            // transmit-only transfer (rxdmae is enabled but no RX channel is armed), so clearing
            // it here keeps a later send_receive from reading garbage.
            while(apply(read(Regs::SSPSR::rne))) { apply(read(Regs::SSPDR::data)); }

            error.store(false, std::memory_order_relaxed);

            busy = true;

            std::atomic_signal_fence(std::memory_order_release);

            Dma::template start<DmaChannelA,
                                DmaPriority,
                                base::TxDmaTrigger,
                                Dma::TransferSize::_8,
                                false,
                                increment>(Regs::SSPDR::Addr::value,
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

        template<bool increment,
                 typename F>
        static void send_receive_nocopy_impl(std::byte const* first,
                                             std::byte*       firstOut,
                                             std::size_t      size,
                                             F                f) {
            assert(!busy);

            while(apply(read(Regs::SSPSR::rne))) { apply(read(Regs::SSPDR::data)); }

            error.store(false, std::memory_order_relaxed);
            apply(set(Regs::SSPICR::roric));   // clear any stale receive-overrun flag

            busy = true;

            std::atomic_signal_fence(std::memory_order_release);

            // Arm the RX channel (B) before the TX channel (A) so the receiver is ready before
            // the SSP starts clocking out data and filling the RX FIFO.
            Dma::template start<DmaChannelB,
                                DmaPriority,
                                base::RxDmaTrigger,
                                Dma::TransferSize::_8,
                                true,
                                false>(reinterpret_cast<std::uint32_t>(firstOut),
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

            Dma::template start<DmaChannelA,
                                DmaPriority,
                                base::TxDmaTrigger,
                                Dma::TransferSize::_8,
                                false,
                                increment>(Regs::SSPDR::Addr::value,
                                           reinterpret_cast<std::uint32_t>(first),
                                           size);
        }

        /*        static void onIsr() {
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }
        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));*/
    };
}}   // namespace Kvasir::SPI
