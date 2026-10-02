#pragma once
#include "chip/rp_common/PioStateMachine.hpp"
#include "chip/rp_common/pio/ClockOutProgram.hpp"
#include "kvasir/Util/Prescaler.hpp"

#include <cstdint>

namespace Kvasir { namespace Pio {
    /// A square wave on any GPIO, from a PIO state machine.
    ///
    /// `Kvasir::Clocks::Output` (ClockOutput.hpp) is preferable where it reaches -- GPOUT is
    /// limited to GPIO 13, 15, 21, 23, 24 and 25. This costs a state machine and a coarser
    /// divider but works on any pin.
    ///
    /// Config:
    ///   ClockSpeed  (required) clk_sys, the divider's base
    ///   PioInstance (required) 0..2
    ///   SmInstance  (required) 0..3
    ///   OutputHz    (required) the wave on the pin
    ///   ToleranceHz (default: 0.1% of OutputHz) how far the achievable divider may miss by
    ///   ProgramOffset, GpioBase  as PioStateMachine.hpp
    ///
    /// The 16.8 divider cannot hit most frequencies exactly; the achievable one is checked
    /// against the tolerance at compile time and reported by `AchievedHz`.
    template<typename Pin, typename Config_>
    struct ClockOut {
        struct Config : Config_ {
            static constexpr auto ProgramOffset = [] {
                if constexpr(requires { Config_::ProgramOffset; }) {
                    return Config_::ProgramOffset;
                } else {
                    return 0;
                }
            }();

            static constexpr double ToleranceHz = [] {
                if constexpr(requires { Config_::ToleranceHz; }) {
                    return double{Config_::ToleranceHz};
                } else {
                    return double{Config_::OutputHz} / 1000.0;
                }
            }();

            // Two cycles a period (ClockOutProgram), so the machine runs at twice the output.
            static constexpr double clockDiv
              = double{Config_::ClockSpeed} / (2.0 * double{Config_::OutputHz});

            static constexpr auto setPins = brigand::list<Pin>{};
        };

        /// The 16.8 divider the state machine is given (Kvasir::Pio::getDiv truncates the
        /// fraction), and what the pin therefore does.
        static constexpr double ActualDiv = [] {
            auto const [integer, fraction] = Kvasir::Pio::getDiv(Config::clockDiv);
            return double(integer) + double(fraction) / 256.0;
        }();

        static constexpr double AchievedHz = double{Config::ClockSpeed} / (2.0 * ActualDiv);

        static constexpr double ErrorHz = AchievedHz - double{Config::OutputHz};

        static constexpr double ErrorPpm = ErrorHz * 1.0e6 / double{Config::OutputHz};

        static_assert(Config::clockDiv >= 1.0 && Config::clockDiv < 65536.0,
                      "OutputHz is not reachable from ClockSpeed with a 16.8 PIO divider");

        /// The same, exact: ClockSpeed x 256 / (2 x (256 INT + FRAC)) (RP2350 datasheet 11.5.5
        /// and Table 995, SMx_CLKDIV: frequency = clock / (INT + FRAC / 256)).
        static constexpr Prescaler::Rational Achieved = [] {
            auto const [integer, fraction] = Kvasir::Pio::getDiv(Config::clockDiv);
            return Prescaler::Rational{std::uint64_t{Config::ClockSpeed} * 256U,
                                       2U * (std::uint64_t{integer} * 256U + fraction)};
        }();

        /// ToleranceHz relative to OutputHz, in mHz; the default 1/1000 exactly.
        static constexpr Prescaler::Tolerance AllowedError = [] {
            if constexpr(requires { Config_::ToleranceHz; }) {
                return Prescaler::Tolerance{
                  static_cast<std::uint64_t>(double{Config_::ToleranceHz} * 1000.0 + 0.5),
                  std::uint64_t{Config_::OutputHz} * 1000U};
            } else {
                return Prescaler::Tolerance{1, 1000};
            }
        }();

        // the 16.8 divider misses OutputHz by more than ToleranceHz: pick another frequency,
        // another ClockSpeed, or widen the tolerance deliberately. The message has the numbers.
        static constexpr bool OutputInTolerance = [] {
            Prescaler::assertInTolerance<Achieved,
                                         std::uint64_t{Config::OutputHz},
                                         AllowedError,
                                         "PIO clock output">();
            return true;
        }();
        // a static data member of a class template is initialised only when used: this use is
        // what runs the check
        static_assert(OutputInTolerance);

        using Sm = Kvasir::Pio::StateMachine<ClockOutProgram, Config>;

        using Provides = typename Sm::Provides;
        using Claims   = typename Sm::Claims;

        static constexpr auto powerClockEnable        = Sm::powerClockEnable;
        static constexpr auto initStepPinConfig       = Sm::initStepPinConfig;
        static constexpr auto initStepPeripheryConfig = Sm::initStepPeripheryConfig;
        static constexpr auto initStepPeripheryEnable = Sm::initStepPeripheryEnable;

        static void preEnableRuntimeInit() { Sm::preEnableRuntimeInit(); }

        static void runtimeInit() { Sm::runtimeInit(); }

        /// The wave runs from startup; this gates it afterwards.
        static void setEnabled(bool on) { Sm::setEnabled(on); }
    };

}}   // namespace Kvasir::Pio
