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
// The tags are optionalProvider: a ClockSettings without `Provides` keeps building,
// unchecked. Opting in: `using Provides = Kvasir::DefaultClockSettings::Provides<ClockSpeed,
// CrystalSpeed>;` in the settings struct.

#include "kvasir/StartUp/Resources.hpp"

namespace Kvasir { namespace Clocks {
    struct ClockTag {
        static constexpr bool sharedClaim      = true;
        static constexpr bool coreLocal        = false;
        static constexpr bool optionalProvider = true;
    };

    struct ClkSys : ClockTag {};

    struct ClkPeri : ClockTag {};

    struct ClkRef : ClockTag {};

    struct ClkAdc : ClockTag {};

    struct ClkUsb : ClockTag {};

    template<typename Which, auto Hz>
    using Clk = Startup::Resource<Which, Hz>;

    // `using Claims = Kvasir::Clocks::Claim<Kvasir::Clocks::ClkPeri, Config::clockSpeed>;`
    template<typename Which, auto Hz>
    using Claim = brigand::list<Clk<Which, Hz>>;

    // What DefaultClockSettings::coreClockInit programs: clk_sys and clk_peri at the
    // requested speed, clk_ref from the crystal, the ADC and USB clocks from the 48 MHz
    // USB PLL, and the processor clock the chip-agnostic core layer names.
    template<auto ClockSpeed, auto CrystalSpeed>
    using DefaultProvides = brigand::list<Clk<ClkSys, ClockSpeed>,
                                          Clk<ClkPeri, ClockSpeed>,
                                          Clk<ClkRef, CrystalSpeed>,
                                          Clk<ClkAdc, 48'000'000>,
                                          Clk<ClkUsb, 48'000'000>,
                                          Startup::ProcessorClock<ClockSpeed>>;
}}   // namespace Kvasir::Clocks
