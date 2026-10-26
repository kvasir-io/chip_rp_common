#pragma once
#include "Clocks.hpp"
#include "PinConfig.hpp"
#include "PwmMath.hpp"
#include "chip/Blocks.hpp"   // the chip's block table (Rp::Block)
#include "chip/PinConfig.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Port/Extends.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Result.hpp"
#include "peripherals/PWM.hpp"
#include "peripherals/RESETS.hpp"

#include <cstdint>
#include <optional>
#include <uc_log/uc_log.hpp>
#include <utility>

namespace Kvasir { namespace PWM {
    // Startup resources (kvasir/StartUp/Resources.hpp). A slice has one counter (DIV, TOP)
    // and two outputs; two peripherals on one slice are fine if they agree on the counter
    // (pins 0 and 1 at one frequency) and a build error if they do not, and each output
    // belongs to one peripheral.
    struct OutputTag {};

    struct SliceTag {
        static constexpr bool mergeIdentical = true;
    };

    template<unsigned Slice, bool A>
    using OutputResource = Startup::Resource<OutputTag, Slice * 2 + (A ? 0 : 1)>;

    template<unsigned Slice, unsigned Div16, unsigned Top>
    using SliceResource = Startup::Resource<SliceTag, Slice, Div16, Top>;

    /// Why setFrequency() refused a frequency (nothing was written): a run-time value is class E (Kvasir_SDK
    /// Runtime/Check.hpp), the caller decides.
    enum class PwmError : std::uint8_t {
        frequencyTooHigh,    // the divider would be below 1.0
        frequencyTooLow,     // the divider would be 256.0 or more
        minTopUnreachable,   // no TOP of at least the config's minTop reaches it
    };

    namespace detail {
        // The search at run time, as a Result: the divider and TOP, or why not.
        template<std::uint32_t ClockSpeed>
        Kvasir::Result<std::pair<std::uint16_t,
                                 std::uint16_t>,
                       PwmError>
        divAndTopFor(std::uint32_t frequency,
                     std::uint32_t topMin) {
            auto const search = searchDivAndTop<ClockSpeed>(frequency, topMin);
            if(search.value) { return *search.value; }
            switch(search.refusal) {
            case DivTopRefusal::frequencyTooHigh:
                return std::unexpected{PwmError::frequencyTooHigh};
            case DivTopRefusal::frequencyTooLow: return std::unexpected{PwmError::frequencyTooLow};
            case DivTopRefusal::none:            // never without a value
            case DivTopRefusal::minTopUnreachable: break;
            }
            return std::unexpected{PwmError::minTopUnreachable};
        }

        // The search at run time: a refusal is logged with its reason.
        template<std::uint32_t ClockSpeed>
        std::optional<std::pair<std::uint16_t,
                                std::uint16_t>>
        checkedDivAndTop(std::uint32_t frequency,
                         std::uint32_t topMin) {
            auto const search = searchDivAndTop<ClockSpeed>(frequency, topMin);
            switch(search.refusal) {
            case DivTopRefusal::none:              break;
            case DivTopRefusal::frequencyTooHigh:  UC_LOG_W("freq too large"); break;
            case DivTopRefusal::frequencyTooLow:   UC_LOG_W("freq too small"); break;
            case DivTopRefusal::minTopUnreachable: UC_LOG_W("cant reach min top"); break;
            }
            return search.value;
        }

        template<int Port,
                 int Pin>
        constexpr std::uint32_t getChannel(Kvasir::Io::PinLocation<Port,
                                                                   Pin>) {
            return PinConfig::getPwmChannelInfo<Chip::part, Pin>().channel;
        }

        template<int Port,
                 int Pin>
        constexpr bool isChannelA(Kvasir::Io::PinLocation<Port,
                                                          Pin>) {
            return PinConfig::getPwmChannelInfo<Chip::part, Pin>().isChannelA;
        }

        template<std::uint16_t Duty,
                 std::uint32_t Channel,
                 bool          AOrB>
        constexpr auto setDuty() {
            using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<Channel>;
            if constexpr(AOrB) {
                return write(Regs::CC::a, Kvasir::Register::value<Duty>());
            } else {
                return write(Regs::CC::b, Kvasir::Register::value<Duty>());
            }
        }

        template<std::uint16_t Duty,
                 int           Port,
                 int           Pin>
        constexpr auto setDuty(Kvasir::Io::PinLocation<Port,
                                                       Pin> x) {
            return setDuty<Duty, getChannel(x), isChannelA(x)>();
        }

        template<bool Invert,
                 int  Port,
                 int  Pin>
        constexpr auto setInvert(Kvasir::Io::PinLocation<Port,
                                                         Pin> x) {
            using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<getChannel(x)>;
            if constexpr(isChannelA(x)) {
                return write(Regs::CSR::a_inv, Kvasir::Register::value<Invert>());
            } else {
                return write(Regs::CSR::b_inv, Kvasir::Register::value<Invert>());
            }
        }

        // Slice Ch's bit of INTR / INTE / INTS: bit Ch (RP2040 datasheet Table 523, md l.24894; RP2350 datasheet
        // Table 1138, md l.53356)
        template<typename Reg,
                 std::size_t Ch>
        constexpr auto getChField() {
            return Register::fieldAt<static_cast<unsigned>(Ch)>(Reg{});
        }

        // The slices' interrupt enable and status: the RP2040 has one INTE / INTS, the RP2350 two of each
        // (IRQ0_*, IRQ1_*: two interrupt lines), of which the driver uses line 0
#ifdef KVASIR_CHIP_RP2040
        using IrqEnable = Kvasir::Peripheral::PWM::Registers<>::INTE;
        using IrqStatus = Kvasir::Peripheral::PWM::Registers<>::INTS;
#else
        using IrqEnable = Kvasir::Peripheral::PWM::Registers<>::IRQ0_INTE;
        using IrqStatus = Kvasir::Peripheral::PWM::Registers<>::IRQ0_INTS;
#endif
    }   // namespace detail

    template<typename Pin, typename Config_>
    struct PWM {
        struct Config : Config_ {
            static constexpr auto invert   = KVASIR_CONFIG_OR(Config_, invert, false);
            static constexpr auto initDuty = KVASIR_CONFIG_OR(Config_, initDuty, 0);
            // top, minTop: Config_'s own, if given (minTopGiven / topGiven below decide)
        };

        // Which of the two the config gives. `minTop` bounds the TOP the frequency search may
        // pick (also at run time, setFrequency/isValidFrequency); with `top` given there is no
        // search.
        static constexpr bool minTopGiven = requires { Config_::minTop; };
        static constexpr bool topGiven    = requires { Config_::top; };

        static_assert(!(minTopGiven && topGiven),
                      "minTop only applies when top is not given: the driver searches TOP for the "
                      "frequency");

        using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<detail::getChannel(Pin{})>;

        /// The slice the pin belongs to and whether it is its A output (else B).
        static constexpr unsigned slice    = detail::getChannel(Pin{});
        static constexpr bool     channelA = detail::isChannelA(Pin{});

        // `minTop` if given, else 0: any TOP the search finds. (A default of `top` or 256 would
        // refuse run-time frequencies that work today; a possible later change.)
        static constexpr std::uint16_t minTop = []() {
            if constexpr(minTopGiven) {
                static_assert(Config_::minTop >= 0 && Config_::minTop <= 65535,
                              "minTop must fit the 16-bit TOP register");
                return static_cast<std::uint16_t>(Config_::minTop);
            } else {
                return std::uint16_t{0};
            }
        }();

        // The frequency has to be reachable from clk_sys with a 16-bit TOP and an 8.4-bit
        // divider, or with the given TOP a divider of at least 1.0. A static_assert, not an
        // assert inside the constexpr lambda: that is a no-op under NDEBUG, and the `->` on
        // an empty optional then fails constant evaluation with a useless message.
        static_assert(
          [] {
              if constexpr(!topGiven) {
                  return detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, minTop)
                    .has_value();
              } else {
                  return detail::div16Reachable<Config::clockSpeed>(Config::top, Config::frequency);
              }
          }(),
          "PWM frequency unreachable from clk_sys: with a 16-bit TOP and an 8.4-bit divider "
          "(frequency given), or the divider would be below 1.0 (top given)");

        static constexpr std::uint16_t initialTop = []() {
            if constexpr(!topGiven) {
                return detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, minTop)->second;
            } else {
                return Config::top;
            }
        }();

        static constexpr std::uint16_t initialDiv16 = []() {
            if constexpr(!topGiven) {
                return detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, minTop)->first;
            } else {
                return detail::calcDiv16<Config::clockSpeed>(Config::top, Config::frequency);
            }
        }();

        static constexpr std::uint16_t initialDuty = Config::initDuty;

        // Startup: this slice's counter as configured, this output, and the clock the
        // divider is computed from (clk_sys).
        using Provides
          = brigand::list<OutputResource<detail::getChannel(Pin{}), detail::isChannelA(Pin{})>,
                          SliceResource<detail::getChannel(Pin{}), initialDiv16, initialTop>>;
        using Claims = Hal::Claim<Clocks::ClkSys, Config::clockSpeed>;

        static constexpr auto powerClockEnable
          = list(clear(Rp::Block<Kvasir::Peripheral::PWM::Registers<>>::reset));

        static constexpr auto initStepPinConfig
          = list(action(Kvasir::Io::Action::PinFunction<4>{}, Pin{}));

        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::DIV::div_16, Kvasir::Register::value<initialDiv16>()),
                 write(Regs::TOP::top, Kvasir::Register::value<initialTop>()),
                 detail::setInvert<Config::invert>(Pin{}),
                 detail::setDuty<initialDuty>(Pin{}));

        static constexpr auto initStepPeripheryEnable = list(set(Regs::CSR::en));

        static void reset() {
            apply(initStepPinConfig, initStepPeripheryConfig, initStepPeripheryEnable);
        }

        // Opt-in `safeDuty`: the output's compare value on every fatal path (Hook::SafeState). No member, no entry.
        // Not "duty 0" by default: off is 0 for a plain output, the TOP for an inverted one, full speed for a fan.
        static constexpr bool safeDutyGiven = requires { Config_::safeDuty; };

        static void safeState() {
            // A fatal path may come before this block is out of reset (a panic in the clock set-up): its registers
            // are not ready until RESETS.RESET_DONE.PWM is set (RP2040 DS 2.14.3 Table 204, bit 14; RP2350 DS
            // 7.5.3 Table 537, bit 16), and an output still in reset drives nothing.
            if(get<0>(apply(read(Rp::Block<Kvasir::Peripheral::PWM::Registers<>>::resetDone))) == 0)
            {
                return;
            }
            if constexpr(safeDutyGiven) {
                static_assert(Config_::safeDuty >= 0 && Config_::safeDuty <= 65535,
                              "safeDuty must fit the 16-bit compare register");
                setDuty(static_cast<std::uint16_t>(Config_::safeDuty));
            }
        }

        using Extends
          = std::conditional_t<safeDutyGiven,
                               Kvasir::Startup::Extend<Kvasir::Hook::SafeState, &PWM::safeState>,
                               brigand::list<>>;

        static std::uint16_t getTop() {
            return static_cast<std::uint16_t>(get<0>(apply(read(Regs::TOP::top))));
        }

        static void setDuty(std::uint16_t duty) {
            if constexpr(detail::isChannelA(Pin{})) {
                apply(write(Regs::CC::a, duty));
            } else {
                apply(write(Regs::CC::b, duty));
            }
        }

        static bool isValidFrequency(std::uint32_t frequency) {
            auto const divTopO = detail::checkedDivAndTop<Config::clockSpeed>(frequency, minTop);
            return divTopO.has_value();
        }

        /// The divider and TOP for `frequency`, or why it cannot be reached - then nothing is written and the
        /// slice runs on as it was.
        [[nodiscard]] static Kvasir::Result<void,
                                            PwmError>
        setFrequency(std::uint32_t frequency) {
            auto const divTop = detail::divAndTopFor<Config::clockSpeed>(frequency, minTop);
            if(!divTop) { return std::unexpected{divTop.error()}; }
            auto const& [div16, top] = *divTop;
            apply(write(Regs::DIV::div_16, div16), write(Regs::TOP::top, top));
            return {};
        }
    };

    // What a slice counts in counter mode (CSR.DIVMODE): the divider's ticks while the B
    // input is high, or one per rising or falling edge on it. `free` is the ordinary PWM.
    enum class CountMode { level, rising, falling };

    // A slice as a counter: its B pin is an input and the 16-bit counter advances by
    // Config::mode on it (pico-examples' measure_duty_cycle as a Startup-list peripheral).
    //
    //   struct DutyCounterConfig {
    //       static constexpr auto clockSpeed = HW::ClockSpeed;              // clk_sys
    //       static constexpr auto mode       = Kvasir::PWM::CountMode::level;
    //       static constexpr auto clockDiv   = 100.0;   // one count per 100 cycles high
    //   };
    //   using DutyCounter = Kvasir::PWM::Counter<HW::Pin::pwm_in, DutyCounterConfig>;
    //
    // Config (required): clockSpeed, mode. Optional: clockDiv (1.0; 8.4 fixed point), top
    // (65535), enabled (true). The pin must be a B channel (an odd GPIO): only the B input
    // reaches the divider. The slice's counter is provided, so an ordinary PWM on the
    // slice's A pin is a build error.
    template<typename Pin, typename Config_>
    struct Counter {
        struct Config : Config_ {
            static constexpr double clockDiv
              = static_cast<double>(KVASIR_CONFIG_OR(Config_, clockDiv, 1.0));
            static constexpr std::uint16_t top
              = static_cast<std::uint16_t>(KVASIR_CONFIG_OR(Config_, top, std::uint16_t{65535}));
            static constexpr bool enabled
              = static_cast<bool>(KVASIR_CONFIG_OR(Config_, enabled, true));
        };

        static_assert(!detail::isChannelA(Pin{}),
                      "a PWM counter's input is the slice's B pin (an odd GPIO)");
        static_assert(Config::clockDiv >= 1.0 && Config::clockDiv < 256.0,
                      "the PWM divider is 8.4 fixed point: 1.0 to 255.9375");

        static constexpr unsigned slice = detail::getChannel(Pin{});
        using Regs                      = Kvasir::Peripheral::PWM::Registers<>::CH<slice>;

        /// The divider as the hardware holds it, 8.4 fixed point: clockDiv rounded down
        /// to a sixteenth. DivExact says whether that rounding lost anything.
        static constexpr std::uint16_t div16 = static_cast<std::uint16_t>(Config::clockDiv * 16.0);
        static constexpr bool divExact     = static_cast<double>(div16) == Config::clockDiv * 16.0;
        static constexpr std::uint16_t top = Config::top;

        using Provides
          = brigand::list<OutputResource<slice, false>, SliceResource<slice, div16, top>>;
        using Claims = Hal::Claim<Clocks::ClkSys, Config::clockSpeed>;

        static constexpr auto powerClockEnable
          = list(clear(Rp::Block<Kvasir::Peripheral::PWM::Registers<>>::reset));

        // F4 is the PWM function; the pad's input buffer is on by default, and in a gated
        // mode the slice reads the pin rather than driving it.
        static constexpr auto initStepPinConfig
          = list(action(Kvasir::Io::Action::PinFunction<4>{}, Pin{}));

        static constexpr auto modeConfig = [] {
            if constexpr(Config::mode == CountMode::level) {
                return write(Regs::CSR::DIVMODEValC::level);
            } else if constexpr(Config::mode == CountMode::rising) {
                return write(Regs::CSR::DIVMODEValC::rise);
            } else {
                return write(Regs::CSR::DIVMODEValC::fall);
            }
        }();

        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::DIV::div_16, Kvasir::Register::value<div16>()),
                 write(Regs::TOP::top, Kvasir::Register::value<top>()),
                 write(Regs::CTR::ctr, Kvasir::Register::value<0>()),
                 modeConfig);

        static constexpr auto initStepPeripheryEnable = [] {
            if constexpr(Config::enabled) {
                return list(set(Regs::CSR::en));
            } else {
                return list(clear(Regs::CSR::en));
            }
        }();

        /// The counter, 0..top.
        [[nodiscard]] static std::uint16_t count() {
            return static_cast<std::uint16_t>(get<0>(apply(read(Regs::CTR::ctr))));
        }

        /// Back to zero. The counter keeps running if it is enabled.
        static void resetCount() { apply(write(Regs::CTR::ctr, Kvasir::Register::value<0>())); }

        /// Start or stop counting; the count is kept either way.
        static void setEnabled(bool on) {
            if(on) {
                apply(set(Regs::CSR::en));
            } else {
                apply(clear(Regs::CSR::en));
            }
        }

        /// The rate the counter advances at while its input is high (level mode): clk_sys
        /// over the divider the hardware runs, Div16 / 16, not Config::clockDiv. For edge
        /// modes the count is the number of edges.
        static constexpr std::uint32_t countRate = static_cast<std::uint32_t>(
          static_cast<std::uint64_t>(Config::clockSpeed) * 16U / div16);
    };

    template<std::size_t Channel, typename Config, typename Callback>
    struct PwmTimer {
        using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<Channel>;

        using InterruptIndexs = decltype(PinConfig::PwmTraits<Chip::part>::interrupts);

        static constexpr auto getIsrSetEnable() {
            return set(detail::getChField<detail::IrqEnable, Channel>());
        }

        static constexpr auto getIsrIsEnable() {
            return read(detail::getChField<detail::IrqStatus, Channel>());
        }

        static constexpr std::uint16_t minTop = 1;

        static_assert(
          detail::calcDivAndTop<Config::clockSpeed>(Config::frequency,
                                                    minTop)
            .has_value(),
          "PWM_Timer frequency unreachable from clk_sys with a 16-bit TOP and an 8.4-bit "
          "divider");

        static constexpr std::uint16_t initialTop
          = detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, minTop)->second;

        static constexpr std::uint16_t initialDiv16
          = detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, minTop)->first;

        static constexpr std::uint16_t initialDuty = initialTop;

        // Startup: this slice's counter as configured, output A (whose compare it sets),
        // and the clock the divider is computed from.
        using Provides = brigand::list<OutputResource<Channel, true>,
                                       SliceResource<Channel, initialDiv16, initialTop>>;
        using Claims   = Hal::Claim<Clocks::ClkSys, Config::clockSpeed>;

        static constexpr auto powerClockEnable
          = list(clear(Rp::Block<Kvasir::Peripheral::PWM::Registers<>>::reset));

        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::DIV::div_16, Kvasir::Register::value<initialDiv16>()),
                 write(Regs::TOP::top, Kvasir::Register::value<initialTop>()),
                 detail::setDuty<initialDuty, Channel, true>());

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<Config::isrPriority>(InterruptIndexs{}),
                 Nvic::makeClearPending(InterruptIndexs{}));

        static constexpr auto initStepPeripheryEnable
          = list(set(Regs::CSR::en), getIsrSetEnable(), Nvic::makeEnable(InterruptIndexs{}));

        static void reset() {
            apply(initStepPeripheryConfig, initStepInterruptConfig, initStepPeripheryEnable);
        }

        static bool isValidFrequency(std::uint32_t frequency) {
            auto const divTopO = detail::checkedDivAndTop<Config::clockSpeed>(frequency, minTop);
            return divTopO.has_value();
        }

        /// The divider and TOP for `frequency`, or why it cannot be reached - then nothing is written and the
        /// slice runs on as it was.
        [[nodiscard]] static Kvasir::Result<void,
                                            PwmError>
        setFrequency(std::uint32_t frequency) {
            auto const divTop = detail::divAndTopFor<Config::clockSpeed>(frequency, minTop);
            if(!divTop) { return std::unexpected{divTop.error()}; }
            auto const& [div16, top] = *divTop;
            apply(write(Regs::DIV::div_16, div16), write(Regs::TOP::top, top));
            return {};
        }

        static void onIsr() {
            auto const state = apply(getIsrIsEnable());
            if(state) { Callback{}(); }
            using IntrReg = Kvasir::Peripheral::PWM::Registers<>::INTR;
            apply(set(detail::getChField<IntrReg, Channel>()));
        }

        using Isr = Nvic::IsrsFor<std::addressof(onIsr), InterruptIndexs>;
    };

}}   // namespace Kvasir::PWM
