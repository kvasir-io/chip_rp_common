#pragma once

#include "DMA.hpp"
#include "Io.hpp"
#include "PinConfig.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Io/Types.hpp"
#include "peripherals/SPI.hpp"

#include <array>
#include <cassert>
#include <span>

namespace Kvasir { namespace SPI {

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
            // default) is plenty up to ~8 MHz; faster clocks get 8 mA with fast slew.
            static constexpr Io::DriveStrength spiDrive(std::uint32_t f_baud) {
                return f_baud <= 8'000'000 ? Io::DriveStrength::mA_4 : Io::DriveStrength::mA_8;
            }

            static constexpr bool spiSlewFast(std::uint32_t f_baud) { return f_baud > 8'000'000; }

            template<typename MISOPIN>
            struct GetMISOPinConfig;

            template<typename dummy>
            struct GetMISOPinConfig<Io::NotUsed<dummy>> {
                using pinConfig = brigand::list<>;
            };

            template<int Port, int Pin>
            struct GetMISOPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using pinConfig = decltype(action(Kvasir::Io::Action::PinFunction<1>{},
                                                  Register::PinLocation<Port, Pin>{}));
            };
            template<typename CSPIN, std::uint32_t f_baud>
            struct GetCSPinConfig;

            template<typename dummy, std::uint32_t f_baud>
            struct GetCSPinConfig<Io::NotUsed<dummy>, f_baud> {
                using pinConfig = brigand::list<>;
            };

            template<int Port, int Pin, std::uint32_t f_baud>
            struct GetCSPinConfig<Kvasir::Register::PinLocation<Port, Pin>, f_baud> {
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunctionDrive<1, spiDrive(f_baud), spiSlewFast(f_baud)>{},
                  Register::PinLocation<Port, Pin>{}));
            };

            template<typename MOSIPIN, std::uint32_t f_baud>
            struct GetMOSIPinConfig;

            template<typename dummy, std::uint32_t f_baud>
            struct GetMOSIPinConfig<Io::NotUsed<dummy>, f_baud> {
                using pinConfig = brigand::list<>;
            };

            template<int Port, int Pin, std::uint32_t f_baud>
            struct GetMOSIPinConfig<Kvasir::Register::PinLocation<Port, Pin>, f_baud> {
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunctionDrive<1, spiDrive(f_baud), spiSlewFast(f_baud)>{},
                  Register::PinLocation<Port, Pin>{}));
            };

            template<typename SCLKPIN, std::uint32_t f_baud>
            struct GetSCLKPinConfig;

            template<int Port, int Pin, std::uint32_t f_baud>
            struct GetSCLKPinConfig<Kvasir::Register::PinLocation<Port, Pin>, f_baud> {
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunctionDrive<1, spiDrive(f_baud), spiSlewFast(f_baud)>{},
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

            static constexpr double calcf_Baud(std::uint32_t f_clockSpeed,
                                               std::uint32_t scr,
                                               std::uint32_t cpsdvsr) {
                return (double(f_clockSpeed) / (double(cpsdvsr) * (1.0 + double(scr))));
            }

            static constexpr std::pair<std::uint8_t,
                                       std::uint8_t>
            calcBaudRegs(std::uint32_t f_clockSpeed,
                         std::uint32_t f_baud) {
                std::pair<std::uint8_t, std::uint8_t> ret{};
                std::pair<std::uint8_t, std::uint8_t> retClosest{};
                double bestUnder = std::numeric_limits<double>::max();
                double bestAbs   = std::numeric_limits<double>::max();
                bool   haveUnder = false;

                for(std::uint32_t cpsdvsr = 2; cpsdvsr < 255; cpsdvsr += 2) {
                    for(std::uint32_t scr = 0; scr < 256; ++scr) {
                        double f_div     = f_baud - calcf_Baud(f_clockSpeed, scr, cpsdvsr);
                        double abs_f_div = f_div > 0.0 ? f_div : -f_div;

                        if(bestAbs > abs_f_div) {
                            bestAbs           = abs_f_div;
                            retClosest.first  = static_cast<std::uint8_t>(scr);
                            retClosest.second = static_cast<std::uint8_t>(cpsdvsr);
                        }

                        // Prefer configurations whose rate does not exceed the requested baud,
                        // so the configured clock never overshoots a peripheral's maximum.
                        if(f_div >= 0.0 && bestUnder > f_div) {
                            haveUnder  = true;
                            bestUnder  = f_div;
                            ret.first  = static_cast<std::uint8_t>(scr);
                            ret.second = static_cast<std::uint8_t>(cpsdvsr);
                            if(f_div == 0.0) { return ret; }
                        }
                    }
                }
                return haveUnder ? ret : retClosest;
            }

            template<std::uint32_t f_clockSpeed,
                     std::uint32_t f_baud,
                     std::intmax_t Num,
                     std::intmax_t Denom>
            static constexpr bool isValidBaudConfig(std::ratio<Num,
                                                               Denom>) {
                constexpr auto baudRegs     = calcBaudRegs(f_clockSpeed, f_baud);
                constexpr auto scr          = std::get<0>(baudRegs);
                constexpr auto cpsdvsr      = std::get<1>(baudRegs);
                constexpr auto f_baudCalced = calcf_Baud(f_clockSpeed, scr, cpsdvsr);
                constexpr auto err          = f_baudCalced - double(f_baud);
                constexpr auto absErr       = err > 0.0 ? err : -err;
                constexpr auto ret = absErr <= (double(f_baud) * (double(Num) / (double(Denom))));
                return (cpsdvsr >= 2) && (cpsdvsr % 2 == 0) && ret;
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

        using Config = Detail::Config<Instance>;

        static constexpr auto RxDmaTrigger = Traits::SPI::DmaRX_Trigger<Instance>();
        static constexpr auto TxDmaTrigger = Traits::SPI::DmaTX_Trigger<Instance>();

        static_assert(
          (Config::template isValidBaudConfig<SPIConfig::clockSpeed,
                                              SPIConfig::baudRate>(SPIConfig::maxBaudRateError)),
          "invalid baud configuration baudRate error too big");
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

        static constexpr auto initStepPinConfig = list(
          typename Config::template GetMISOPinConfig<
            std::decay_t<decltype(SPIConfig::misoPinLocation)>>::pinConfig{},
          typename Config::template GetMOSIPinConfig<
            std::decay_t<decltype(SPIConfig::mosiPinLocation)>,
            SPIConfig::baudRate>::pinConfig{},
          typename Config::template GetSCLKPinConfig<
            std::decay_t<decltype(SPIConfig::sclkPinLocation)>,
            SPIConfig::baudRate>::pinConfig{},
          typename Config::template GetCSPinConfig<std::decay_t<decltype(SPIConfig::csPinLocation)>,
                                                   SPIConfig::baudRate>::pinConfig{});

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
