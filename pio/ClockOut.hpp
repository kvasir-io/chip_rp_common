#pragma once
#include "chip/rp_common/PioStateMachine.hpp"
#include "chip/rp_common/pio/ClockOutProgram.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Prescaler.hpp"

#include <cstdint>

namespace Kvasir { namespace Pio {
    namespace detail {
        // ClockOut's configuration with its defaults, outside the class so that its state machine
        // can be ClockOut's base (StartupOf).
        template<typename Pin, typename Config_>
        struct ClockOutConfig : Config_ {
            static constexpr auto programOffset = KVASIR_CONFIG_OR(Config_, programOffset, 0);

            static constexpr double toleranceHz
              = double{KVASIR_CONFIG_OR(Config_, toleranceHz, double{Config_::outputHz} / 1000.0)};

            // Two cycles a period (ClockOutProgram), so the machine runs at twice the output.
            static constexpr double clockDiv
              = double{Config_::clockSpeed} / (2.0 * double{Config_::outputHz});

            static constexpr auto setPins = brigand::list<Pin>{};
        };
    }   // namespace detail

    /// A square wave on any GPIO, from a PIO state machine.
    ///
    /// `Kvasir::Clocks::Output` (ClockOutput.hpp) is preferable where it reaches -- GPOUT is
    /// limited to GPIO 13, 15, 21, 23, 24 and 25. This costs a state machine and a coarser
    /// divider but works on any pin.
    ///
    /// Config:
    ///   clockSpeed  (required) clk_sys, the divider's base
    ///   pioInstance (required) 0..2
    ///   smInstance  (required) 0..3
    ///   outputHz    (required) the wave on the pin
    ///   toleranceHz (default: 0.1% of outputHz) how far the achievable divider may miss by
    ///   programOffset, gpioBase  as PioStateMachine.hpp
    ///
    /// The 16.8 divider cannot hit most frequencies exactly; the achievable one is checked
    /// against the tolerance at compile time and reported by `AchievedHz`.
    template<typename Pin, typename Config_>
    struct ClockOut
      : StartupOf<
          Kvasir::Pio::StateMachine<ClockOutProgram, detail::ClockOutConfig<Pin, Config_>>> {
        using Config = detail::ClockOutConfig<Pin, Config_>;

        /// The 16.8 divider the state machine is given (Kvasir::Pio::getDiv truncates the
        /// fraction), and what the pin therefore does.
        static constexpr double actualDiv = [] {
            auto const [integer, fraction] = Kvasir::Pio::getDiv(Config::clockDiv);
            return double(integer) + double(fraction) / 256.0;
        }();

        static constexpr double achievedHz = double{Config::clockSpeed} / (2.0 * actualDiv);

        static constexpr double errorHz = achievedHz - double{Config::outputHz};

        static constexpr double errorPpm = errorHz * 1.0e6 / double{Config::outputHz};

        static_assert(Config::clockDiv >= 1.0 && Config::clockDiv < 65536.0,
                      "outputHz is not reachable from clockSpeed with a 16.8 PIO divider");

        /// The same, exact: clockSpeed x 256 / (2 x (256 INT + FRAC)) (RP2350 datasheet 11.5.5
        /// and Table 995, SMx_CLKDIV: frequency = clock / (INT + FRAC / 256)).
        static constexpr Prescaler::Rational achieved = [] {
            auto const [integer, fraction] = Kvasir::Pio::getDiv(Config::clockDiv);
            return Prescaler::Rational{std::uint64_t{Config::clockSpeed} * 256U,
                                       2U * (std::uint64_t{integer} * 256U + fraction)};
        }();

        /// toleranceHz relative to outputHz, in mHz; the default 1/1000 exactly.
        static constexpr Prescaler::Tolerance allowedError = [] {
            if constexpr(requires { Config_::toleranceHz; }) {
                // to the nearest mHz (std::llround is not constexpr in libc++)
                double const mHz   = double{Config_::toleranceHz} * 1000.0;
                auto const   whole = static_cast<std::uint64_t>(mHz);
                auto const   num   = whole + (mHz - static_cast<double>(whole) >= 0.5 ? 1U : 0U);
                return Prescaler::Tolerance{num, std::uint64_t{Config_::outputHz} * 1000U};
            } else {
                return Prescaler::Tolerance{1, 1000};
            }
        }();

        // the 16.8 divider misses outputHz by more than toleranceHz: pick another frequency,
        // another clockSpeed, or widen the tolerance deliberately. The message has the numbers.
        static constexpr bool outputInTolerance = [] {
            Prescaler::assertInTolerance<achieved,
                                         std::uint64_t{Config::outputHz},
                                         allowedError,
                                         "PIO clock output">();
            return true;
        }();
        // a static data member of a class template is initialised only when used: this use is
        // what runs the check
        static_assert(outputInTolerance);

        using Sm = Kvasir::Pio::StateMachine<ClockOutProgram, Config>;

        /// The wave runs from startup; this gates it afterwards.
        static void setEnabled(bool on) { Sm::setEnabled(on); }
    };

}}   // namespace Kvasir::Pio
