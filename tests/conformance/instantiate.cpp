// The drivers' register writes, instantiated once each (one representative instance and configuration per driver
// template): an explicit instantiation compiles every member function, so every dependent apply() of the driver is
// checked against its register's write-only rules (Register::Semantics: keys, reads with side effects) as the
// conformance tree converts the SVD (guard error, registers derived, mask on). Nothing here is called; what needs a
// board's configuration to be instantiated at all is in not_instantiated.txt.

// the config header first: the chip below needs its chip variant, log backend and log clock (uc_log.hpp would read it
// too, but the chip headers before it need the same)
#include "kvasir_config.hpp"

// then the chip and the drivers
#include "chip/chip.hpp"
#include "chip/rp_common/ADC.hpp"
#include "chip/rp_common/ClockOutput.hpp"
#include "chip/rp_common/DmaCrc.hpp"
#include "chip/rp_common/GpioIrq.hpp"
#include "chip/rp_common/I2CQueued.hpp"
#include "chip/rp_common/ImageCheck.hpp"
#include "chip/rp_common/PioIrq.hpp"
#include "chip/rp_common/PioStateMachine.hpp"
#ifdef KVASIR_CHIP_RP2040
    #include "chip/rp_common/Rtc.hpp"
#else
    #include "chip/rp_common/Powman.hpp"
#endif
#include "chip/rp_common/SPIQueued.hpp"
#ifdef KVASIR_CHIP_RP2350
    #include "chip/rp_common/Sha256.hpp"
    #include "chip/rp_common/Trng.hpp"
#endif
#include "chip/rp_common/UARTStream.hpp"
#include "chip/rp_common/pwm.hpp"
#include "chip/rp_common/watchdog.hpp"
#ifdef KVASIR_CHIP_RP2350
    #include "core/Mpu.hpp"
    #include "peripherals/PPB.hpp"
#endif

#include <chrono>
#include <ratio>

namespace Conformance {
using Kvasir::Io::PinLocation;

inline constexpr auto crystalSpeed = 12'000'000;

struct DmaConfig {
    static constexpr auto numberOfChannels     = 4;
    static constexpr auto callbackFunctionSize = 8;
    static constexpr auto isrPriority          = 3;
};

using Dma = Kvasir::DMA::DmaBase<DmaConfig>;

struct UartConfig {
    static constexpr auto clockSpeed    = clockSpeedValue;
    static constexpr auto instance      = 0;
    static constexpr auto txPinLocation = PinLocation<0, 0>{};
    static constexpr auto rxPinLocation = PinLocation<0, 1>{};
    static constexpr auto baudRate      = 115'200;
    static constexpr auto dataBits      = Kvasir::Hal::Uart::DataBits::bits8;
    static constexpr auto parity        = Kvasir::Hal::Uart::Parity::none;
    static constexpr auto stopBits      = Kvasir::Hal::Uart::StopBits::stop1;
    static constexpr auto isrPriority   = 3;
};

struct SpiConfig {
    static constexpr auto clockSpeed       = clockSpeedValue;
    static constexpr auto instance         = 0;
    static constexpr auto sclkPinLocation  = PinLocation<0, 18>{};
    static constexpr auto mosiPinLocation  = PinLocation<0, 19>{};
    static constexpr auto misoPinLocation  = PinLocation<0, 16>{};
    static constexpr auto baudRate         = 1'000'000;
    static constexpr auto maxBaudRateError = std::ratio<1, 100>{};
    static constexpr auto mode             = Kvasir::Hal::ClockMode::mode0;
    static constexpr auto isrPriority      = 3;
};

struct SpiDmaConfig {
    static constexpr auto channelA = Dma::Channel::ch0;
    static constexpr auto channelB = Dma::Channel::ch1;
    static constexpr auto priority = Dma::Priority::low;
};

struct I2cConfig {
    static constexpr auto clockSpeed     = clockSpeedValue;
    static constexpr auto instance       = 1;
    static constexpr auto sdaPinLocation = PinLocation<0, 2>{};
    static constexpr auto sclPinLocation = PinLocation<0, 3>{};
    static constexpr auto baudRate       = 400'000;
    static constexpr auto isrPriority    = 3;
};

struct PwmConfig {
    static constexpr auto clockSpeed = clockSpeedValue;
    static constexpr auto frequency  = 1'000;
    static constexpr auto top        = 4095;
    static constexpr auto initDuty   = 0;
};

// the first ADC input of the part: GPIO26 / ADC0 on the RP2040 (RP2040 datasheet 5.5.2.2, md l.28814) and the
// RP2350A, GPIO40_ADC0 on the RP2350B (RP2350 data sheet 1.2.1.1 and 1.2.1.2, the QFN-60 and QFN-80 pin locations)
#ifdef KVASIR_CHIP_RP2350B
using AdcPin = PinLocation<0, 40>;
#else
using AdcPin = PinLocation<0, 26>;
#endif

struct AdcConfig {
    static constexpr auto clockSpeed       = 48'000'000;
    static constexpr auto pins             = brigand::list<AdcPin>{};
    static constexpr auto enableTempSensor = true;
    static constexpr auto sampleRate       = 1'000;
    static constexpr auto isrPriority      = 3;
};

struct WatchdogConfig {
    static constexpr auto clockSpeed  = crystalSpeed;
    static constexpr auto overrunTime = std::chrono::seconds{1};
};

struct TimerClockConfig {
    static constexpr unsigned instance    = 0;
    static constexpr auto     tickClockHz = crystalSpeed;
};

using TimerClock = Kvasir::Timer::TimerClockBase<TimerClockConfig>;

inline void onAlarm() {}

inline void onEdge() {}

// one instruction, enough to instantiate the state machine's set-pin path
struct Program : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.wrapTarget();
    a.set(Kvasir::Pio::Set::pins, 1);
    a.wrap();
})> {};

struct SmConfig {
    static constexpr auto clockSpeed  = Conformance::clockSpeedValue;
    static constexpr auto pioInstance = 0;
    static constexpr auto smInstance  = 0;
    static constexpr auto setPins     = brigand::list<PinLocation<0, 6>>{};
    static constexpr auto clockDiv    = 1.0;
};
}   // namespace Conformance

// The image-engine hook as an image whose store chains its CRC on the sniffer names it (Runtime/CrcImageEngine.hpp):
// DmaCrc::Crc32Chain compiles only on the set the hook names (its static_assert), so with it here the chain's body is
// instantiated below. Before every explicit instantiation: one of them naming the hook first would instantiate
// the primary.
template<>
struct Kvasir::Crc::ImageEngines<Kvasir::Tag::User> {
    using type = Kvasir::DmaCrc::Sniffer<Conformance::Dma, Conformance::Dma::Channel::ch1>;
};

template struct Kvasir::DMA::DmaBase<Conformance::DmaConfig>;
template struct Kvasir::UART::UartBehavior<Conformance::UartConfig,
                                           Conformance::SystickClock,
                                           Conformance::Dma,
                                           Conformance::Dma::Channel::ch0,
                                           Conformance::Dma::Priority::low,
                                           256>;
template struct Kvasir::SPI::
  SPIBehavior<Conformance::SpiConfig, Conformance::Dma, Conformance::SpiDmaConfig>;
template struct Kvasir::SPI::SPIQueued<Conformance::SpiConfig,
                                       Conformance::SystickClock,
                                       Conformance::Dma,
                                       Kvasir::SPI::QueuedDma<Conformance::Dma,
                                                              Conformance::Dma::Channel::ch0,
                                                              Conformance::Dma::Channel::ch1,
                                                              Conformance::Dma::Priority::low>>;
// SPIQueued is its hardware half and the front (kvasir_devices SPI/QueuedFront.hpp); an explicit instantiation of
// SPIQueued does not reach inherited members, so the half (Hw: the register writes) and the front are named here
template struct Kvasir::SPI::Detail::SPIQueuedHalf<
  Conformance::SpiConfig,
  Conformance::SystickClock,
  Conformance::Dma,
  Kvasir::SPI::QueuedDma<Conformance::Dma,
                         Conformance::Dma::Channel::ch0,
                         Conformance::Dma::Channel::ch1,
                         Conformance::Dma::Priority::low>,
  8,
  16,
  Kvasir::SPI::QueueCoreDefaults>;
template struct Kvasir::SPI::QueuedFront<
  Kvasir::SPI::SPIQueued<Conformance::SpiConfig,
                         Conformance::SystickClock,
                         Conformance::Dma,
                         Kvasir::SPI::QueuedDma<Conformance::Dma,
                                                Conformance::Dma::Channel::ch0,
                                                Conformance::Dma::Channel::ch1,
                                                Conformance::Dma::Priority::low>>::Core>;
template struct Kvasir::I2C::
  I2CBehaviorQueued<Conformance::I2cConfig, Conformance::SystickClock, 4, 16>;
// I2CBehaviorQueued is its hardware half and the front (kvasir_devices I2C/Controller/QueuedFront.hpp), named here as SPIQueued's
template struct Kvasir::I2C::Detail::
  I2CQueuedHalf<Conformance::I2cConfig, Conformance::SystickClock, 4, 16>;
template struct Kvasir::I2C::QueuedFront<
  Kvasir::I2C::I2CBehaviorQueued<Conformance::I2cConfig, Conformance::SystickClock, 4, 16>::Core>;
template struct Kvasir::PWM::PWM<Kvasir::Io::PinLocation<0, 7>, Conformance::PwmConfig>;
template struct Kvasir::ADC::ADCBase<Conformance::AdcConfig>;
template struct Kvasir::Watchdog<Conformance::WatchdogConfig>;
template struct Kvasir::Timer::TimerClockBase<Conformance::TimerClockConfig>;
template struct Kvasir::Timer::Alarm<Conformance::TimerClock, 0, &Conformance::onAlarm>;
#ifdef KVASIR_CHIP_RP2040
template struct Kvasir::Rtc::Rtc<Conformance::crystalSpeed>;
template struct Kvasir::Rtc::Alarm<Kvasir::Rtc::Rtc<Conformance::crystalSpeed>,
                                   &Conformance::onAlarm>;
#else
template struct Kvasir::Powman::AonTimer<Conformance::crystalSpeed>;
template struct Kvasir::Powman::Alarm<Kvasir::Powman::AonTimer<Conformance::crystalSpeed>,
                                      &Conformance::onAlarm>;
template struct Kvasir::Trng::Trng<>;
template struct Kvasir::Sha256::Sha256<>;
#endif
template struct Kvasir::Clocks::FrequencyCounter<Conformance::crystalSpeed>;
template struct Kvasir::DefaultClockSettings::Settings<Conformance::clockSpeedValue,
                                                       Conformance::crystalSpeed>;
template struct Kvasir::UART::UartStream<
  Conformance::UartConfig,
  Conformance::SystickClock,
  Conformance::Dma,
  Kvasir::UART::TxStream<Conformance::Dma::Channel::ch2, 256>,
  Kvasir::UART::RxRing<Conformance::Dma::Channel::ch3, 512, 4>>;
// one-sided streams: their own static_asserts and claims (a TxNone / RxNone side has no channel)
template struct Kvasir::UART::UartStream<
  Conformance::UartConfig,
  Conformance::SystickClock,
  Conformance::Dma,
  Kvasir::UART::TxStream<Conformance::Dma::Channel::ch2, 256>,
  Kvasir::UART::RxNone>;
template struct Kvasir::UART::UartStream<
  Conformance::UartConfig,
  Conformance::SystickClock,
  Conformance::Dma,
  Kvasir::UART::TxNone,
  Kvasir::UART::RxRing<Conformance::Dma::Channel::ch3, 512, 4>>;
// the chip's own Startup entry (DegradedBoot.hpp) for the default tree
template struct Kvasir::Clocks::DegradedBoot<
  Kvasir::DefaultClockSettings::Settings<Conformance::clockSpeedValue, Conformance::crystalSpeed>>;
template struct Kvasir::Io::
  GpioPinEdgesOn<0, &Conformance::onEdge, &Conformance::onEdge, Kvasir::Io::PinLocation<0, 5>>;
template struct Kvasir::Pio::StateMachine<Conformance::Program, Conformance::SmConfig>;
// GPOUT0 on GPIO21 (RP2040 data sheet 1.4.3, RP2350 data sheet 1.2.3: the GPIO function tables)
template struct Kvasir::Clocks::
  Output<Kvasir::Io::PinLocation<0, 21>, Kvasir::Clocks::Source::clkSys, 10>;
template struct Kvasir::Pio::
  IrqSource<0, 0, Kvasir::Pio::IrqSourceKind::rxNotEmpty, 0, &Conformance::onEdge>;
template struct Kvasir::DmaCrc::
  SnifferCalc<Kvasir::Crc::Presets::crc32IsoHdlc, Conformance::Dma, Conformance::Dma::Channel::ch1>;
template class Kvasir::DmaCrc::SnifferEngine<Kvasir::Crc::Presets::crc16Xmodem,
                                             Conformance::Dma,
                                             Conformance::Dma::Channel::ch1>;
template struct Kvasir::DmaCrc::Sniffer<Conformance::Dma, Conformance::Dma::Channel::ch1>;
template struct Kvasir::DmaCrc::Crc32Chain<Conformance::Dma, Conformance::Dma::Channel::ch1>;
template struct Kvasir::ImageCheck::RpStream<Conformance::Dma, Conformance::Dma::Channel::ch1>;
template struct Kvasir::I2C::LineRecovery<Kvasir::I2C::Detail::I2CBase<Conformance::I2cConfig>,
                                          Conformance::SystickClock>;
#ifdef KVASIR_CHIP_RP2350

// Two SVDs describe the Cortex-M33's MPU: the core's (core_cortex_m33 core.svd, what core/Mpu.hpp and chip/Mpu.hpp
// name) and the chip's PPB copy (the vendor's chip.svd, kept as it is). Both at the same addresses:
namespace Conformance {
using CoreMpu = Kvasir::Core::Mpu::Regs;
using Ppb     = Kvasir::Peripheral::PPB::Registers<0>;
static_assert(CoreMpu::TYPE::Addr::value == Ppb::MPU_TYPE::Addr::value);
static_assert(CoreMpu::CTRL::Addr::value == Ppb::MPU_CTRL::Addr::value);
static_assert(CoreMpu::RNR::Addr::value == Ppb::MPU_RNR::Addr::value);
static_assert(CoreMpu::RBAR::Addr::value == Ppb::MPU_RBAR::Addr::value);
static_assert(CoreMpu::RLAR::Addr::value == Ppb::MPU_RLAR::Addr::value);
static_assert(CoreMpu::RBAR_A3::Addr::value == Ppb::MPU_RBAR_A3::Addr::value);
static_assert(CoreMpu::RLAR_A3::Addr::value == Ppb::MPU_RLAR_A3::Addr::value);
static_assert(CoreMpu::MAIR0::Addr::value == Ppb::MPU_MAIR0::Addr::value);
static_assert(CoreMpu::MAIR1::Addr::value == Ppb::MPU_MAIR1::Addr::value);
}   // namespace Conformance
#endif
