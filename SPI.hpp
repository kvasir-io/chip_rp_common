#pragma once

#include "Clocks.hpp"
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
#include <cstdint>
#include <ranges>
#include <ratio>
#include <type_traits>
#include <utility>

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
}}   // namespace Kvasir::SPI
