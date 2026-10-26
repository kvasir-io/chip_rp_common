#pragma once

// The clock tree as Startup resources (kvasir/StartUp/Resources.hpp). Every driver config
// carries a `clockSpeed`, and nothing used to relate it to the clock its block counts: a
// config with 150 MHz on a 200 MHz build gives a wrong baud rate or PIO divider and
// compiles. The clock settings *provide* each clock at the frequency they program, every
// driver *claims* the clock its block runs from at the speed its config says, and a number
// nothing provides is refused by Startup.
//
// Which clock a block counts (RP2350 datasheet, clock tree):
//   clk_sys   PIO, I2C, PWM, DMA, the cores (SysTick), USB's bus side
//   clk_peri  SPI and UART (DefaultClockSettings leaves its aux source at clk_sys)
//   clk_ref   the TICKS generators: TIMER0/1 and the watchdog
//   clk_adc   the ADC, 48 MHz from pll_usb
//   clk_usb   the USB PHY, 48 MHz from pll_usb
//
// The mechanism (Clk, Claim, hzOf, providersOf) is the SDK's, kvasir/Hal/Clocks.hpp. Every
// claim needs a provider: a ClockSettings states what it programs, for the default tree
// `using Provides = Kvasir::DefaultClockSettings::Provides<ClockSpeed, CrystalSpeed>;` (or the
// whole settings as `DefaultClockSettings::Settings<ClockSpeed, CrystalSpeed>`); one without
// `Provides` is refused at the first driver that claims a clock.

#include "kvasir/Hal/Clocks.hpp"

namespace Kvasir { namespace Clocks {
    struct ClkSys : Hal::ClockTag {};

    struct ClkPeri : Hal::ClockTag {};

    struct ClkRef : Hal::ClockTag {};

    struct ClkAdc : Hal::ClockTag {};

    struct ClkUsb : Hal::ClockTag {};

    // What DefaultClockSettings::coreClockInit programs: clk_sys and clk_peri at the
    // requested speed, clk_ref from the crystal, the ADC and USB clocks from the 48 MHz
    // USB PLL, and the processor clock the chip-agnostic core layer names.
    template<auto ClockSpeed, auto CrystalSpeed>
    using DefaultProvides = brigand::list<Hal::Clk<ClkSys, ClockSpeed>,
                                          Hal::Clk<ClkPeri, ClockSpeed>,
                                          Hal::Clk<ClkRef, CrystalSpeed>,
                                          Hal::Clk<ClkAdc, 48'000'000>,
                                          Hal::Clk<ClkUsb, 48'000'000>,
                                          Startup::ProcessorClock<ClockSpeed>>;
}}   // namespace Kvasir::Clocks
