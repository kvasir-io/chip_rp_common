#pragma once
#include "Clocks.hpp"
#include "PinConfig.hpp"
#include "kvasir/StartUp/Resources.hpp"

#include <cstdint>
#include <optional>
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

    namespace detail {

        template<std::uint32_t ClockSpeed>
        static constexpr std::uint32_t calcFrequency(std::uint16_t div16,
                                                     std::uint16_t top) {
            return ((ClockSpeed * 16) / (div16)) / (top + 1);
        }

        template<std::uint32_t ClockSpeed>
        static constexpr std::uint32_t calcTop(std::uint16_t div16,
                                               std::uint32_t frequency) {
            return ((ClockSpeed * 16) / (div16)) / frequency - 1;
        }

        // Whether a given TOP and frequency need a divider of at least 1.0 (16 in 8.4).
        template<std::uint32_t ClockSpeed>
        static constexpr bool div16Reachable(std::uint16_t top,
                                             std::uint32_t frequency) {
            return (ClockSpeed * 16) / ((top + 1) * frequency) >= 16;
        }

        template<std::uint32_t ClockSpeed>
        static constexpr std::uint32_t calcDiv16(std::uint16_t top,
                                                 std::uint32_t frequency) {
            auto const ret = (ClockSpeed * 16) / ((top + 1) * frequency);
            assert(ret >= 16);
            return ret;
        }

        template<std::uint32_t ClockSpeed>
        static constexpr std::optional<std::pair<std::uint16_t,
                                                 std::uint16_t>>
        calcDivAndTop(std::uint32_t frequency,
                      std::uint32_t topMin) {
            constexpr std::uint32_t TopMax    = 65536 - 1;
            constexpr std::uint32_t PeriodMax = TopMax + 1;
            constexpr std::uint32_t DivMax    = (256 * 16) - 1;
            constexpr std::uint32_t Div       = 16;

            std::uint32_t div16_top = Div * ClockSpeed / frequency;
            std::uint32_t top       = 1;
            std::uint32_t div_tmp   = DivMax;

            std::uint32_t div_best{};
            std::uint32_t top_best{};
            while(div_tmp > Div) {
                std::uint32_t tmpTop
                  = calcTop<ClockSpeed>(static_cast<std::uint16_t>(div_tmp), frequency);
                if(TopMax >= tmpTop && tmpTop > topMin) {
                    if(tmpTop > top_best
                       && frequency
                            == calcFrequency<ClockSpeed>(static_cast<std::uint16_t>(div_tmp),
                                                         static_cast<std::uint16_t>(tmpTop)))
                    {
                        top_best = tmpTop;
                        div_best = div_tmp;
                    }
                }
                --div_tmp;
            }
            if(top_best == 0) {
                // Factorization fallback: factor div16_top into div16 * (TOP + 1)
                // Here we use 'period' to track (TOP + 1), then convert to TOP at the end
                div16_top            = Div * ClockSpeed / frequency;
                std::uint32_t period = 1;   // Period is (TOP + 1)
                while(true) {
                    // Try a few small prime factors to get close to the desired frequency.
                    if(div16_top >= Div * 7 && div16_top % 7 == 0 && period * 7 <= PeriodMax) {
                        div16_top /= 7;
                        period *= 7;
                    } else if(div16_top >= Div * 5 && div16_top % 5 == 0 && period * 5 <= PeriodMax)
                    {
                        div16_top /= 5;
                        period *= 5;
                    } else if(div16_top >= Div * 3 && div16_top % 3 == 0 && period * 3 <= PeriodMax)
                    {
                        div16_top /= 3;
                        period *= 3;
                    } else if(div16_top >= Div * 2 && period * 2 <= PeriodMax) {
                        div16_top /= 2;
                        period *= 2;
                    } else {
                        break;
                    }
                }
                top = period - 1;   // Convert period to TOP register value
            } else {
                div16_top = div_best;
                top       = top_best;
            }

            if(div16_top < Div) {
                UC_LOG_W("freq too large");
                return std::nullopt;
            } else if(div16_top >= DivMax) {
                UC_LOG_W("freq too small");
                return std::nullopt;
            } else if(topMin > top) {
                UC_LOG_W("cant reach min top");
                return std::nullopt;
            }

            return std::make_pair(static_cast<std::uint16_t>(div16_top),
                                  static_cast<std::uint16_t>(top));
        }

        template<int Port,
                 int Pin>
        constexpr std::uint32_t getChannel(Kvasir::Register::PinLocation<Port,
                                                                         Pin>) {
            return PinConfig::getPwmChannelInfo<PinConfig::CurrentChip, Pin>().channel;
        }

        template<int Port,
                 int Pin>
        constexpr bool isChannelA(Kvasir::Register::PinLocation<Port,
                                                                Pin>) {
            return PinConfig::getPwmChannelInfo<PinConfig::CurrentChip, Pin>().isChannelA;
        }

        template<std::uint16_t Duty,
                 std::uint32_t Channel,
                 bool          a_or_b>
        constexpr auto setDuty() {
            using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<Channel>;
            if constexpr(a_or_b) {
                return write(Regs::CC::a, Kvasir::Register::value<Duty>());
            } else {
                return write(Regs::CC::b, Kvasir::Register::value<Duty>());
            }
        }

        template<std::uint16_t Duty,
                 int           Port,
                 int           Pin>
        constexpr auto setDuty(Kvasir::Register::PinLocation<Port,
                                                             Pin> x) {
            return setDuty<Duty, getChannel(x), isChannelA(x)>();
        }

        template<bool Invert,
                 int  Port,
                 int  Pin>
        constexpr auto setInvert(Kvasir::Register::PinLocation<Port,
                                                               Pin> x) {
            using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<getChannel(x)>;
            if constexpr(isChannelA(x)) {
                return write(Regs::CSR::a_inv, Kvasir::Register::value<Invert>());
            } else {
                return write(Regs::CSR::b_inv, Kvasir::Register::value<Invert>());
            }
        }

        template<typename Reg,
                 std::size_t Ch>
        constexpr auto getChField() {
            if constexpr(Ch == 0) {
                return Reg::ch0;
            } else if constexpr(Ch == 1) {
                return Reg::ch1;
            } else if constexpr(Ch == 2) {
                return Reg::ch2;
            } else if constexpr(Ch == 3) {
                return Reg::ch3;
            } else if constexpr(Ch == 4) {
                return Reg::ch4;
            } else if constexpr(Ch == 5) {
                return Reg::ch5;
            } else if constexpr(Ch == 6) {
                return Reg::ch6;
            } else if constexpr(Ch == 7) {
                return Reg::ch7;
            } else if constexpr(Ch == 8) {
                return Reg::ch8;
            } else if constexpr(Ch == 9) {
                return Reg::ch9;
            } else if constexpr(Ch == 10) {
                return Reg::ch10;
            } else if constexpr(Ch == 11) {
                return Reg::ch11;
            }
        }
    }   // namespace detail

    template<typename Pin, typename Config_>
    struct PWM {
        struct Config : Config_ {
            static constexpr auto invert = [] {
                if constexpr(requires { Config_::invert; }) {
                    return Config_::invert;
                } else {
                    return false;
                }
            }();
            static constexpr auto top = [] {
                if constexpr(requires { Config_::top; }) {
                    return Config_::top;
                } else {
                    return false;
                }
            }();
            static constexpr auto minTop = [] {
                if constexpr(requires { Config_::minTop; }) {
                    return Config_::minTop;
                } else {
                    return false;
                }
            }();
            static constexpr auto initDuty = [] {
                if constexpr(requires { Config_::initDuty; }) {
                    return Config_::initDuty;
                } else {
                    return 0;
                }
            }();
        };

        static constexpr bool hasMinTop
          = std::is_same_v<bool, std::remove_cvref_t<decltype(Config::minTop)>>;
        static constexpr bool hasTop
          = std::is_same_v<bool, std::remove_cvref_t<decltype(Config::top)>>;

        using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<detail::getChannel(Pin{})>;

        static constexpr std::uint16_t MinTop = []() {
            if constexpr(hasMinTop) {
                return Config::minTop;
            } else {
                if constexpr(hasTop) {
                    return Config::top;
                } else {
                    return 256;
                }
            }
        }();

        // The frequency has to be reachable from clk_sys with a 16-bit TOP and an 8.4-bit
        // divider, or with the given TOP a divider of at least 1.0. A static_assert, not an
        // assert inside the constexpr lambda: that is a no-op under NDEBUG, and the `->` on
        // an empty optional then fails constant evaluation with a useless message.
        static_assert(
          [] {
              if constexpr(hasTop) {
                  return detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, MinTop)
                    .has_value();
              } else {
                  return detail::div16Reachable<Config::clockSpeed>(Config::top, Config::frequency);
              }
          }(),
          "PWM frequency unreachable from clk_sys: with a 16-bit TOP and an 8.4-bit divider "
          "(frequency given), or the divider would be below 1.0 (top given)");

        static constexpr std::uint16_t InitialTop = []() {
            if constexpr(hasTop) {
                return detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, MinTop)->second;
            } else {
                return Config::top;
            }
        }();

        static constexpr std::uint16_t InitialDiv16 = []() {
            if constexpr(hasTop) {
                return detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, MinTop)->first;
            } else {
                return detail::calcDiv16<Config::clockSpeed>(Config::top, Config::frequency);
            }
        }();

        static constexpr std::uint16_t InitialDuty = Config::initDuty;

        // Startup: this slice's counter as configured, this output, and the clock the
        // divider is computed from (clk_sys).
        using Provides
          = brigand::list<OutputResource<detail::getChannel(Pin{}), detail::isChannelA(Pin{})>,
                          SliceResource<detail::getChannel(Pin{}), InitialDiv16, InitialTop>>;
        using Claims = Clocks::Claim<Clocks::ClkSys, Config::clockSpeed>;

        static constexpr auto powerClockEnable
          = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::pwm));

        static constexpr auto initStepPinConfig
          = list(action(Kvasir::Io::Action::PinFunction<4>{}, Pin{}));

        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::DIV::div_16, Kvasir::Register::value<InitialDiv16>()),
                 write(Regs::TOP::top, Kvasir::Register::value<InitialTop>()),
                 detail::setInvert<Config::invert>(Pin{}),
                 detail::setDuty<InitialDuty>(Pin{}));

        static constexpr auto initStepPeripheryEnable = list(set(Regs::CSR::en));

        static void reset() {
            apply(initStepPinConfig, initStepPeripheryConfig, initStepPeripheryEnable);
        }

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

        static void setFrequency(std::uint32_t frequency) {
            auto const result = setFrequencyChecked(frequency);
            assert(result);
        }

        static bool isValidFrequency(std::uint32_t frequency) {
            auto const divTopO = detail::calcDivAndTop<Config::clockSpeed>(frequency, MinTop);
            return divTopO.has_value();
        }

        static bool setFrequencyChecked(std::uint32_t frequency) {
            auto const divTopO = detail::calcDivAndTop<Config::clockSpeed>(frequency, MinTop);
            if(!divTopO.has_value()) { return false; }
            auto const& [div16, top] = *divTopO;
            apply(write(Regs::DIV::div_16, div16), write(Regs::TOP::top, top));

            return true;
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
            static constexpr double clockDiv = [] {
                if constexpr(requires { Config_::clockDiv; }) {
                    return static_cast<double>(Config_::clockDiv);
                } else {
                    return 1.0;
                }
            }();
            static constexpr std::uint16_t top = [] {
                if constexpr(requires { Config_::top; }) {
                    return static_cast<std::uint16_t>(Config_::top);
                } else {
                    return std::uint16_t{65535};
                }
            }();
            static constexpr bool enabled = [] {
                if constexpr(requires { Config_::enabled; }) {
                    return static_cast<bool>(Config_::enabled);
                } else {
                    return true;
                }
            }();
        };

        static_assert(!detail::isChannelA(Pin{}),
                      "a PWM counter's input is the slice's B pin (an odd GPIO)");
        static_assert(Config::clockDiv >= 1.0 && Config::clockDiv < 256.0,
                      "the PWM divider is 8.4 fixed point: 1.0 to 255.9375");

        static constexpr unsigned Slice = detail::getChannel(Pin{});
        using Regs                      = Kvasir::Peripheral::PWM::Registers<>::CH<Slice>;

        static constexpr std::uint16_t Div16 = static_cast<std::uint16_t>(Config::clockDiv * 16.0);
        static constexpr std::uint16_t Top   = Config::top;

        using Provides
          = brigand::list<OutputResource<Slice, false>, SliceResource<Slice, Div16, Top>>;
        using Claims = Clocks::Claim<Clocks::ClkSys, Config::clockSpeed>;

        static constexpr auto powerClockEnable
          = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::pwm));

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
          = list(write(Regs::DIV::div_16, Kvasir::Register::value<Div16>()),
                 write(Regs::TOP::top, Kvasir::Register::value<Top>()),
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
        /// over the divider. For edge modes the count is the number of edges.
        static constexpr std::uint32_t countRate
          = static_cast<std::uint32_t>(Config::clockSpeed / Config::clockDiv);
    };

    template<std::size_t Channel, typename Config, typename Callback>
    struct PWM_Timer {
        using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<Channel>;

        using InterruptIndexs = decltype(PinConfig::PwmTraits<PinConfig::CurrentChip>::Interrupts);

        template<typename R = Regs>
        static constexpr auto getIsrSetEnable() {
#if __has_include("chip/rp2040.hpp")
            using InteReg = Kvasir::Peripheral::PWM::Registers<>::INTE;
            return set(detail::getChField<InteReg, Channel>());
#else
            if constexpr(requires { typename R::IRQ0_INTE; }) {
                return set(detail::getChField<typename R::IRQ0_INTE, Channel>());
            } else if constexpr(requires { typename R::IRQ_INTE; }) {
                return set(detail::getChField<typename R::IRQ_INTE, Channel>());
            } else {
                using PwmRegs = Kvasir::Peripheral::PWM::Registers<>;
                return set(detail::getChField<typename PwmRegs::IRQ0_INTE, Channel>());
            }
#endif
        }

        template<typename R = Regs>
        static constexpr auto getIsrIsEnable() {
#if __has_include("chip/rp2040.hpp")
            using IntsReg = Kvasir::Peripheral::PWM::Registers<>::INTS;
            return read(detail::getChField<IntsReg, Channel>());
#else
            if constexpr(requires { typename R::IRQ0_INTS; }) {
                return read(detail::getChField<typename R::IRQ0_INTS, Channel>());
            } else if constexpr(requires { typename R::IRQ_INTS; }) {
                return read(detail::getChField<typename R::IRQ_INTS, Channel>());
            } else {
                using PwmRegs = Kvasir::Peripheral::PWM::Registers<>;
                return read(detail::getChField<typename PwmRegs::IRQ0_INTS, Channel>());
            }
#endif
        }

        static constexpr std::uint16_t MinTop = 1;

        static_assert(
          detail::calcDivAndTop<Config::clockSpeed>(Config::frequency,
                                                    MinTop)
            .has_value(),
          "PWM_Timer frequency unreachable from clk_sys with a 16-bit TOP and an 8.4-bit "
          "divider");

        static constexpr std::uint16_t InitialTop
          = detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, MinTop)->second;

        static constexpr std::uint16_t InitialDiv16
          = detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, MinTop)->first;

        static constexpr std::uint16_t InitialDuty = InitialTop;

        // Startup: this slice's counter as configured, output A (whose compare it sets),
        // and the clock the divider is computed from.
        using Provides = brigand::list<OutputResource<Channel, true>,
                                       SliceResource<Channel, InitialDiv16, InitialTop>>;
        using Claims   = Clocks::Claim<Clocks::ClkSys, Config::clockSpeed>;

        static constexpr auto powerClockEnable
          = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::pwm));

        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::DIV::div_16, Kvasir::Register::value<InitialDiv16>()),
                 write(Regs::TOP::top, Kvasir::Register::value<InitialTop>()),
                 detail::setDuty<InitialDuty, Channel, true>());

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<Config::isrPriority>(InterruptIndexs{}),
                 Nvic::makeClearPending(InterruptIndexs{}));

        static constexpr auto initStepPeripheryEnable
          = list(set(Regs::CSR::en), getIsrSetEnable<Regs>(), Nvic::makeEnable(InterruptIndexs{}));

        static void reset() {
            apply(initStepPeripheryConfig, initStepInterruptConfig, initStepPeripheryEnable);
        }

        static void setFrequency(std::uint32_t frequency) {
            auto const result = setFrequencyChecked(frequency);
            assert(result);
        }

        static bool isValidFrequency(std::uint32_t frequency) {
            auto const divTopO = detail::calcDivAndTop<Config::clockSpeed>(frequency, MinTop);
            return divTopO.has_value();
        }

        static bool setFrequencyChecked(std::uint32_t frequency) {
            auto const divTopO = detail::calcDivAndTop<Config::clockSpeed>(frequency, MinTop);
            if(!divTopO.has_value()) { return false; }
            auto const& [div16, top] = *divTopO;
            apply(write(Regs::DIV::div_16, div16), write(Regs::TOP::top, top));

            return true;
        }

        static void onIsr() {
            auto state = apply(getIsrIsEnable<Regs>());
            if(state) { Callback{}(); }
            using IntrReg = Kvasir::Peripheral::PWM::Registers<>::INTR;
            apply(set(detail::getChField<IntrReg, Channel>()));
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }

        using Isr = decltype(makeIsr(InterruptIndexs{}));
    };

}}   // namespace Kvasir::PWM
