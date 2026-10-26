// The chip contract of chip_rp2040 and chip_rp2350 as it is today (Kvasir_SDK tests/conformance/ChipContract.hpp), the
// chip the tree is configured for (CONFORMANCE_CHIP, CMakeLists.txt; its package defines KVASIR_CHIP_RP2040 or
// KVASIR_CHIP_RP2350): every row the chip has is REQUIRE, every row it lacks is GAP - a GAP that the chip fills fails
// here and moves to REQUIRE.

// the config header first: the chip below needs its chip variant, log backend and log clock (uc_log.hpp would read it
// too, but the chip headers before it need the same)
#include "kvasir_config.hpp"

// then the chip and the drivers
#include "chip/chip.hpp"
#include "chip/rp_common/SerialNumber.hpp"
#ifdef KVASIR_CHIP_RP2350
    #include "chip/picobin.hpp"
#endif
#include "chip/rp_common/FlashRegion.hpp"
#include "chip/rp_common/ImageCheck.hpp"
#include "chip/rp_common/WaitBounds.hpp"
#include "chip/rp_common/watchdog.hpp"
// the core's debug query and feature macros
#include "core/Features.hpp"
#include "cortex_common/Debug.hpp"

// last: the contract rows see only what is declared before them
#include "ChipContract.hpp"

#include <chrono>

namespace Conformance {
struct WatchdogConfig {
    static constexpr auto clockSpeed  = 12'000'000;
    static constexpr auto overrunTime = std::chrono::seconds{1};
    static constexpr bool gatedFeed   = true;
};

struct DmaConfig {
    static constexpr auto numberOfChannels     = 1;
    static constexpr auto callbackFunctionSize = 4;
    static constexpr auto isrPriority          = 3;
};

using Dma = Kvasir::DMA::DmaBase<DmaConfig>;
}   // namespace Conformance

// Startup hooks the SDK's Startup calls
KVASIR_CONFORMANCE_REQUIRE(
  "Startup::FirstInitStep<Tag::User>",
  requires { Kvasir::Startup::FirstInitStep<Kvasir::Tag::User>{}(); });
KVASIR_CONFORMANCE_REQUIRE(
  "Startup::SecondaryCoreInit<Tag::User>::cpacrEnable",
  requires { Kvasir::Startup::SecondaryCoreInit<Kvasir::Tag::User>::cpacrEnable; });
KVASIR_CONFORMANCE_REQUIRE(
  "Startup::StackGuardEntropy<Tag::User>",
  requires { Kvasir::Startup::StackGuardEntropy<Kvasir::Tag::User>{}(); });

// the interrupt table and the pins
KVASIR_CONFORMANCE_REQUIRE("Nvic::InterruptOffsetTraits<Tag::User>::end",
                           Kvasir::Nvic::InterruptOffsetTraits<Kvasir::Tag::User>::end > 0);
KVASIR_CONFORMANCE_REQUIRE("Io::PinLocationTraits<Chip::part>",
                           Kvasir::Io::PinLocationTraits<Kvasir::Chip::part>::pinEnd > 0);

// reset cause, wait bounds
KVASIR_CONFORMANCE_REQUIRE("PM::resetCause()",
                           Kvasir::Conformance::Row::resetCause);
KVASIR_CONFORMANCE_REQUIRE("PM::resetKind()",
                           Kvasir::Conformance::Row::resetKind);
KVASIR_CONFORMANCE_REQUIRE("Register::CpuHzCeiling<Tag::User>",
                           Kvasir::Conformance::Row::cpuHzCeiling);
KVASIR_CONFORMANCE_REQUIRE("Chip::part / partName, the identity constants",
                           Kvasir::Conformance::Row::chipPart);

// the health supervisor's watchdog, the CellStore's flash, the image check's backend
KVASIR_CONFORMANCE_REQUIRE("Kvasir::Watchdog",
                           Kvasir::Conformance::Row::kvasirWatchdog);
// the serial number, as bytes and as hex text (every chip's two functions)
KVASIR_CONFORMANCE_REQUIRE("Kvasir::serialNumber() / serialNumberString()",
                           Kvasir::Conformance::Row::serialNumber);
KVASIR_CONFORMANCE_REQUIRE(
  "health watchdog (gated feed/arm/disarm, Timeout)",
  Kvasir::Conformance::HealthWatchdog<Kvasir::Watchdog<Conformance::WatchdogConfig>>);
KVASIR_CONFORMANCE_REQUIRE("Hal::NorRegion (Flash::Region)",
                           Kvasir::Hal::NorRegion<Kvasir::Flash::Region<0x100000,
                                                                        2>>);
KVASIR_CONFORMANCE_REQUIRE(
  "ImageCheck::Backend (ImageCheck::RpStream)",
  Kvasir::ImageCheck::Backend<Kvasir::ImageCheck::RpStream<Conformance::Dma,
                                                           Conformance::Dma::Channel::ch0>>);

// a reboot every chip spells the same, "is a debugger there" (always true here: DebuggerPresent.hpp)
KVASIR_CONFORMANCE_REQUIRE("Chip::reboot()",
                           Kvasir::Conformance::Row::chipReboot);
KVASIR_CONFORMANCE_REQUIRE(
  "Port::Reboot<Tag::User>, the reboot hook",
  requires { Kvasir::Port::Reboot<Kvasir::Tag::User>{}(); });
KVASIR_CONFORMANCE_REQUIRE("Chip::debuggerPresent()",
                           Kvasir::Conformance::Row::chipDebuggerPresent);
// "is a debugger connected", read by the core itself: the M33 (core_cortex_m33) reads DHCSR; known gap on the M0+,
// which cannot (its cortex_common/Debug.hpp has haltingDebug() only, `unknown`) - the RP2040's one gap (a new contract
// row starts as a GAP and moves to REQUIRE when the chip gets it)
#ifdef KVASIR_CHIP_RP2040
KVASIR_CORE_CONFORMANCE_GAP("Core::Debug::isDebuggerConnected()",
                            Kvasir::Conformance::Row::coreDebugIsDebuggerConnected);
#else
KVASIR_CORE_CONFORMANCE_REQUIRE("Core::Debug::isDebuggerConnected()",
                                Kvasir::Conformance::Row::coreDebugIsDebuggerConnected);
#endif
// "is halting debug on": off / on / unknown on every core (cortex_common/Debug.hpp)
KVASIR_CORE_CONFORMANCE_REQUIRE("Core::haltingDebug()",
                                Kvasir::Conformance::Row::coreHaltingDebug);

// chip/Blocks.hpp names each block's base as a number from the address map: each row against the base of the
// generated register set it stands for (in table order), and the PIO rows' per-state-machine requests
// (RP2040 datasheet 2.5.3.1 Table 119, RP2350 datasheet 12.6.4.1)
#include <peripherals/ADC.hpp>
#include <peripherals/BUSCTRL.hpp>
#ifdef KVASIR_CHIP_RP2350
    #include <peripherals/HSTX_CTRL.hpp>
    #include <peripherals/HSTX_FIFO.hpp>
#endif
#include <peripherals/I2C.hpp>
#include <peripherals/PADS_BANK0.hpp>
#include <peripherals/PIO.hpp>
#include <peripherals/PWM.hpp>
#ifdef KVASIR_CHIP_RP2040
    #include <peripherals/RTC.hpp>
#else
    #include <peripherals/SHA256.hpp>
#endif
#include <peripherals/SPI.hpp>
#include <peripherals/TBMAN.hpp>
#ifdef KVASIR_CHIP_RP2350
    #include <peripherals/TRNG.hpp>
#endif
#include <peripherals/UART.hpp>
#include <peripherals/USB_DPRAM.hpp>

namespace Conformance::BlockBases {
namespace P = Kvasir::Peripheral;
using RegisterSets
#ifdef KVASIR_CHIP_RP2040
  = brigand::list<P::ADC::Registers<>,
                  P::BUSCTRL::Registers<>,
                  P::DMA::Registers<>,
                  P::I2C::Registers<0>,
                  P::I2C::Registers<1>,
                  P::IO_BANK0::Registers<>,
                  P::PADS_BANK0::Registers<>,
                  P::PIO::Registers<0>,
                  P::PIO::Registers<1>,
                  P::PLL::Registers<1>,
                  P::PWM::Registers<>,
                  P::RTC::Registers<>,
                  P::SPI::Registers<0>,
                  P::SPI::Registers<1>,
                  P::TBMAN::Registers<>,
                  P::TIMER::Registers<>,
                  P::UART::Registers<0>,
                  P::UART::Registers<1>,
                  P::USB_DPRAM::Registers<>>;
#else
  = brigand::list<P::ADC::Registers<>,
                  P::BUSCTRL::Registers<>,
                  P::DMA::Registers<>,
                  P::HSTX_CTRL::Registers<>,
                  P::HSTX_FIFO::Registers<>,
                  P::I2C::Registers<0>,
                  P::I2C::Registers<1>,
                  P::IO_BANK0::Registers<>,
                  P::PADS_BANK0::Registers<>,
                  P::PIO::Registers<0>,
                  P::PIO::Registers<1>,
                  P::PIO::Registers<2>,
                  P::PLL::Registers<1>,
                  P::PWM::Registers<>,
                  P::SHA256::Registers<>,
                  P::SPI::Registers<0>,
                  P::SPI::Registers<1>,
                  P::TBMAN::Registers<>,
                  P::TIMER::Registers<0>,
                  P::TIMER::Registers<1>,
                  P::TRNG::Registers<>,
                  P::UART::Registers<0>,
                  P::UART::Registers<1>,
                  P::USB_DPRAM::Registers<>>;
#endif

template<typename Row,
         typename Regs>
consteval bool sameBase() {
    static_assert(Row::base == Regs::baseAddr,
                  "chip/Blocks.hpp: a row's base is not its register set's baseAddr");
    return true;
}

template<typename... Rows,
         typename... Regs>
consteval bool everyRow(Kvasir::Hal::BlockTable<Rows...>*,
                        brigand::list<Regs...>*) {
    static_assert(sizeof...(Rows) == sizeof...(Regs),
                  "one register set per row of chip/Blocks.hpp");
    return (sameBase<Rows, Regs>() && ...);
}

static_assert(everyRow(static_cast<Kvasir::Chip::Blocks::Table*>(nullptr),
                       static_cast<RegisterSets*>(nullptr)));

using D = Kvasir::Rp::Dreq;
#ifdef KVASIR_CHIP_RP2040
static_assert(Kvasir::Rp::Block<P::PIO::Registers<1>>::dreqTxOf<3>() == D::pio1_tx3
              && Kvasir::Rp::Block<P::PIO::Registers<1>>::dreqRxOf<3>() == D::pio1_rx3
              && Kvasir::Rp::Block<P::PIO::Registers<0>>::dreqRxOf<0>() == D::pio0_rx0);
#else
static_assert(Kvasir::Rp::Block<P::PIO::Registers<2>>::dreqTxOf<3>() == D::pio2_tx3
              && Kvasir::Rp::Block<P::PIO::Registers<2>>::dreqRxOf<3>() == D::pio2_rx3
              && Kvasir::Rp::Block<P::PIO::Registers<1>>::dreqRxOf<1>() == D::pio1_rx1);
#endif
}   // namespace Conformance::BlockBases
