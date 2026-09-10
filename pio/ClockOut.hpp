#pragma once
#include "chip/rp_common/PioStateMachine.hpp"
#include "clockoutPio/clockout.hpp"

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

            // Two cycles a period (clockout.pio), so the machine runs at twice the output.
            static constexpr double clockDiv
              = double{Config_::ClockSpeed} / (2.0 * double{Config_::OutputHz});

            static constexpr auto setPins = brigand::list<Pin>{};
        };

        /// What the 16.8 divider actually rounds to, and what the pin therefore does.
        static constexpr double ActualDiv = [] {
            auto const scaled = static_cast<std::uint32_t>(Config::clockDiv * 256.0 + 0.5);
            return double(scaled) / 256.0;
        }();

        static constexpr double AchievedHz = double{Config::ClockSpeed} / (2.0 * ActualDiv);

        static constexpr double ErrorHz = AchievedHz - double{Config::OutputHz};

        static constexpr double ErrorPpm = ErrorHz * 1.0e6 / double{Config::OutputHz};

        static_assert(Config::clockDiv >= 1.0 && Config::clockDiv < 65536.0,
                      "OutputHz is not reachable from ClockSpeed with a 16.8 PIO divider");

        static_assert((ErrorHz < 0 ? -ErrorHz : ErrorHz) <= Config::ToleranceHz,
                      "the nearest 16.8 divider misses OutputHz by more than ToleranceHz: pick "
                      "another frequency, another ClockSpeed, or widen the tolerance deliberately");

        using Sm = Kvasir::Pio::StateMachine<Kvasir::Pio::clockoutProgramm, Config>;

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
