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

    // ---- reading a rate back off a Provides list -----------------------------------------
    //
    // A config that says "this block counts clk_ref" wants the number the tree programs
    // clk_ref to, not a copy of it: `hzOf<ClkRef>(Provides{})` is the crystal on a
    // DefaultProvides. A clock the list does not provide (or provides twice) is a build
    // error, which is what keeps that config from silently becoming a number.
    namespace Detail {
        template<typename Which, typename Resource>
        struct ClockHz {
            static constexpr bool               found = false;
            static constexpr unsigned long long value = 0;
        };

        template<typename Which, unsigned long long Hz>
        struct ClockHz<Which, Startup::Detail::ResourceT<Which, Hz>> {
            static constexpr bool               found = true;
            static constexpr unsigned long long value = Hz;
        };
    }   // namespace Detail

    /// How many entries of the list provide `Which` (1 on a well-formed tree).
    template<typename Which,
             typename... Resources>
    consteval std::size_t providersOf(brigand::list<Resources...>) {
        return ((Detail::ClockHz<Which, Resources>::found ? std::size_t{1} : std::size_t{0}) + ...
                + std::size_t{0});
    }

    /// The rate the list provides `Which` at.
    template<typename Which,
             typename... Resources>
    consteval unsigned long long hzOf(brigand::list<Resources...> provides) {
        static_assert(providersOf<Which>(provides) == 1,
                      "the clock tree provides no such clock (or more than one)");
        return (Detail::ClockHz<Which, Resources>::value + ... + 0ULL);
    }

    /// The same with the list as a type: `hzOf<ClkRef, Provides>()`.
    template<typename Which,
             typename Provides>
    consteval unsigned long long hzOf() {
        return hzOf<Which>(Provides{});
    }
}}   // namespace Kvasir::Clocks
