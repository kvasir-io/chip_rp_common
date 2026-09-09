#pragma once
#include "Clocks.hpp"
#include "Io.hpp"
#include "PIO.hpp"
#include "kvasir/Register/Register.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <peripherals/PIO.hpp>

// A PIO state machine running one program, as a Startup-list peripheral (the generic form
// of ws2812.hpp): the Startup list loads the program, sets divider, wrap, shift registers
// and pin mapping, gives the pins the PIO function and their directions, and starts the
// machine. The application talks to it through the FIFOs.
//
//   pioasm_generate(blinkPio INPUT_FILE examples/pio_blink/blink.pio)   // CMake: the program
//
//   struct BlinkSmConfig {
//       static constexpr auto ClockSpeed  = HW::ClockSpeed;         // clk_sys (the divider's base)
//       static constexpr auto PioInstance = 0;                      // PIO0..2
//       static constexpr auto SmInstance  = 0;                      // 0..3
//       static constexpr auto setPins     = brigand::list<HW::Pin::led>{};   // `set pins`
//   };
//   using BlinkSm = Kvasir::Pio::StateMachine<Kvasir::Pio::blinkProgramm, BlinkSmConfig>;
//
// Optional config, with defaults:
//   ProgramOffset (0), clockDiv (1.0; 16.8, the machine runs at ClockSpeed / clockDiv)
//   GpioBase      (derived)    the instance's GPIO window, 0 or 16 (PIO.hpp): a machine
//                              names 32 pins counted from it, so the RP2350B's GPIO 32..47
//                              need 16. Derived from this machine's pins; name it only to
//                              agree with another driver on the same PIO instance
//   setPins, outPins, sidesetPins, inPins (empty brigand::lists): consecutive GPIOs, the
//                              first is the mapping's base; set/out/side-set pins become
//                              outputs, in pins inputs, all get this PIO's function
//   sidesetOptional (false)    `.side_set N opt`: the enable bit is part of the count
//   sidesetPindirs (false)     side-set drives pin directions instead of levels
//   autopull (false), pullThreshold (32), outShiftRight (true)
//   autopush (false), pushThreshold (32), inShiftRight (true)
//   joinTx, joinRx (false)     an 8-deep FIFO in one direction
//   jmpPin (none)              the pin `jmp pin` tests (a brigand::list of one pin)
//   inPullUp (false)           pull-ups on the in pins and the jmp pin
//   pinsInitialHigh (empty)    output pins driven high before the machine starts
//   outSticky (false)          EXECCTRL.OUT_STICKY
//   movStatus (none), movStatusN (0)   what `mov x, status` reports (MovStatus)
//
// Two state machines may run one program at one offset (the slots are shared); different
// programs on overlapping slots are a build error.
namespace Kvasir { namespace Pio {

    enum class MovStatus { none, txFifoLessThan, rxFifoLessThan, irqSet };

    namespace detail {
        template<int Port,
                 int Pin>
        constexpr int pinNumber(Register::PinLocation<Port,
                                                      Pin>) {
            return Pin;
        }

        template<typename List>
        struct PinRange;

        template<>
        struct PinRange<brigand::list<>> {
            static constexpr int  base  = 0;
            static constexpr int  count = 0;
            static constexpr bool ok    = true;
        };

        template<typename First, typename... Rest>
        struct PinRange<brigand::list<First, Rest...>> {
            static constexpr int base  = pinNumber(First{});
            static constexpr int count = 1 + static_cast<int>(sizeof...(Rest));

            static constexpr bool consecutive() {
                constexpr int pins[] = {pinNumber(First{}), pinNumber(Rest{})...};
                for(int i = 1; i < count; ++i) {
                    if(pins[i] != pins[0] + i) { return false; }
                }
                return true;
            }

            static constexpr bool ok = consecutive();
        };

        // Every pin of a mapping list, as GPIO numbers, for the window checks.
        template<typename List>
        struct PinNumbers;

        template<typename... Ps>
        struct PinNumbers<brigand::list<Ps...>> {
            static constexpr std::array<unsigned, sizeof...(Ps)> value{
              static_cast<unsigned>(pinNumber(Ps{}))...};
        };

        // A mapping's base as the state machine names it. An empty mapping keeps 0: its
        // count is 0, so the base is never used, and subtracting the window from a base
        // that is not a pin would underflow.
        template<typename Range>
        constexpr unsigned mappedBase(unsigned gpioBase) {
            return Range::count == 0 ? 0U : static_cast<unsigned>(Range::base) - gpioBase;
        }

        // The elements of List that are not in Other.
        template<typename List, typename Other>
        struct Difference;

        template<typename Other>
        struct Difference<brigand::list<>, Other> {
            using type = brigand::list<>;
        };

        template<typename T, typename... Ts, typename Other>
        struct Difference<brigand::list<T, Ts...>, Other> {
            using rest = typename Difference<brigand::list<Ts...>, Other>::type;
            using type
              = std::conditional_t<brigand::any<Other, std::is_same<brigand::_1, T>>::value,
                                   rest,
                                   brigand::push_front<rest, T>>;
        };

        // A list without repeats, so a pin in two mappings (set and side-set on one GPIO) is
        // configured once: Startup refuses a GPIO with two providers.
        template<typename List, typename Out = brigand::list<>>
        struct Unique;

        template<typename Out>
        struct Unique<brigand::list<>, Out> {
            using type = Out;
        };

        template<typename T, typename... Ts, typename Out>
        struct Unique<brigand::list<T, Ts...>, Out> {
            using next = std::conditional_t<brigand::any<Out, std::is_same<brigand::_1, T>>::value,
                                            Out,
                                            brigand::push_back<Out, T>>;
            using type = typename Unique<brigand::list<Ts...>, next>::type;
        };
    }   // namespace detail

    template<typename Program, typename Config_>
    struct StateMachine {
        struct Config : Config_ {
            static constexpr auto ProgramOffset = [] {
                if constexpr(requires { Config_::ProgramOffset; }) {
                    return Config_::ProgramOffset;
                } else {
                    return 0;
                }
            }();
            static constexpr double clockDiv = [] {
                if constexpr(requires { Config_::clockDiv; }) {
                    return static_cast<double>(Config_::clockDiv);
                } else {
                    return 1.0;
                }
            }();
            static constexpr auto setPins = [] {
                if constexpr(requires { Config_::setPins; }) {
                    return Config_::setPins;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr auto outPins = [] {
                if constexpr(requires { Config_::outPins; }) {
                    return Config_::outPins;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr auto sidesetPins = [] {
                if constexpr(requires { Config_::sidesetPins; }) {
                    return Config_::sidesetPins;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr auto inPins = [] {
                if constexpr(requires { Config_::inPins; }) {
                    return Config_::inPins;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr bool sidesetOptional = [] {
                if constexpr(requires { Config_::sidesetOptional; }) {
                    return Config_::sidesetOptional;
                } else {
                    return false;
                }
            }();
            static constexpr bool sidesetPindirs = [] {
                if constexpr(requires { Config_::sidesetPindirs; }) {
                    return Config_::sidesetPindirs;
                } else {
                    return false;
                }
            }();
            static constexpr bool autopull = [] {
                if constexpr(requires { Config_::autopull; }) {
                    return Config_::autopull;
                } else {
                    return false;
                }
            }();
            static constexpr unsigned pullThreshold = [] {
                if constexpr(requires { Config_::pullThreshold; }) {
                    return static_cast<unsigned>(Config_::pullThreshold);
                } else {
                    return 32U;
                }
            }();
            static constexpr bool outShiftRight = [] {
                if constexpr(requires { Config_::outShiftRight; }) {
                    return Config_::outShiftRight;
                } else {
                    return true;
                }
            }();
            static constexpr bool autopush = [] {
                if constexpr(requires { Config_::autopush; }) {
                    return Config_::autopush;
                } else {
                    return false;
                }
            }();
            static constexpr unsigned pushThreshold = [] {
                if constexpr(requires { Config_::pushThreshold; }) {
                    return static_cast<unsigned>(Config_::pushThreshold);
                } else {
                    return 32U;
                }
            }();
            static constexpr bool inShiftRight = [] {
                if constexpr(requires { Config_::inShiftRight; }) {
                    return Config_::inShiftRight;
                } else {
                    return true;
                }
            }();
            static constexpr bool joinTx = [] {
                if constexpr(requires { Config_::joinTx; }) {
                    return Config_::joinTx;
                } else {
                    return false;
                }
            }();
            static constexpr bool joinRx = [] {
                if constexpr(requires { Config_::joinRx; }) {
                    return Config_::joinRx;
                } else {
                    return false;
                }
            }();
            static constexpr auto jmpPin = [] {
                if constexpr(requires { Config_::jmpPin; }) {
                    return Config_::jmpPin;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr bool inPullUp = [] {
                if constexpr(requires { Config_::inPullUp; }) {
                    return Config_::inPullUp;
                } else {
                    return false;
                }
            }();
            static constexpr auto pinsInitialHigh = [] {
                if constexpr(requires { Config_::pinsInitialHigh; }) {
                    return Config_::pinsInitialHigh;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr bool outSticky = [] {
                if constexpr(requires { Config_::outSticky; }) {
                    return Config_::outSticky;
                } else {
                    return false;
                }
            }();
            static constexpr MovStatus movStatus = [] {
                if constexpr(requires { Config_::movStatus; }) {
                    return Config_::movStatus;
                } else {
                    return MovStatus::none;
                }
            }();
            static constexpr unsigned movStatusN = [] {
                if constexpr(requires { Config_::movStatusN; }) {
                    return static_cast<unsigned>(Config_::movStatusN);
                } else {
                    return 0U;
                }
            }();
        };

        static constexpr unsigned Instance = Config::PioInstance;
        static constexpr unsigned Sm       = Config::SmInstance;
        static constexpr unsigned Offset   = Config::ProgramOffset;

        static_assert(Instance < PinConfig::pioCount(PinConfig::CurrentChip),
                      "the RP2350 has PIO0..PIO2, the RP2040 PIO0 and PIO1");
        static_assert(Sm < 4,
                      "a PIO instance has four state machines");
        static_assert(!Config::joinTx || !Config::joinRx,
                      "a FIFO can be joined in one direction only");
        static_assert(Config::pullThreshold >= 1 && Config::pullThreshold <= 32,
                      "pull threshold is 1..32");
        static_assert(Config::pushThreshold >= 1 && Config::pushThreshold <= 32,
                      "push threshold is 1..32");

        using SetPins  = detail::PinRange<std::remove_cvref_t<decltype(Config::setPins)>>;
        using OutPins  = detail::PinRange<std::remove_cvref_t<decltype(Config::outPins)>>;
        using SidePins = detail::PinRange<std::remove_cvref_t<decltype(Config::sidesetPins)>>;
        using InPins   = detail::PinRange<std::remove_cvref_t<decltype(Config::inPins)>>;

        static_assert(SetPins::ok && OutPins::ok && SidePins::ok && InPins::ok,
                      "a PIO pin mapping is a range of consecutive GPIOs");
        static_assert(SetPins::count <= 5,
                      "`set` drives at most five pins");
        static_assert(SidePins::count + (Config::sidesetOptional ? 1 : 0) <= 5,
                      "side-set has five bits, one of them the enable when optional");

        using JmpPin = detail::PinRange<std::remove_cvref_t<decltype(Config::jmpPin)>>;
        static_assert(JmpPin::count <= 1,
                      "jmpPin is one pin");
        static_assert(Config::movStatusN < 32,
                      "movStatusN is a FIFO level (0..15 on this chip) or an IRQ flag (0..7)");

        // The pins that are outputs, the pins that are inputs (the jmp pin included), each
        // without repeats, and everything together: a pin may be in several mappings (out
        // and side-set on one GPIO) and is configured once.
        using OutputPins = typename detail::Unique<
          brigand::append<std::remove_cvref_t<decltype(Config::setPins)>,
                          std::remove_cvref_t<decltype(Config::outPins)>,
                          std::remove_cvref_t<decltype(Config::sidesetPins)>>>::type;
        using InputPins = typename detail::Unique<
          brigand::append<std::remove_cvref_t<decltype(Config::inPins)>,
                          std::remove_cvref_t<decltype(Config::jmpPin)>>>::type;
        using AllPins = typename detail::Unique<brigand::append<OutputPins, InputPins>>::type;

        // The instance's GPIO window: a machine names 32 pins counted from it, so a pin above
        // 31 needs 16 (PIO.hpp). Derived from this machine's own pins unless the config pins
        // it, which is how two drivers on one instance are made to agree.
        static constexpr unsigned GpioBase = [] {
            if constexpr(requires { Config::GpioBase; }) {
                return static_cast<unsigned>(Config::GpioBase);
            } else {
                return Kvasir::Pio::gpioBaseFor(detail::PinNumbers<AllPins>::value);
            }
        }();

        static_assert(Kvasir::Pio::pinsInWindow(detail::PinNumbers<AllPins>::value,
                                                GpioBase),
                      "a pin of this state machine is not reachable from the PIO instance's "
                      "GPIO window: a machine sees 32 pins from GpioBase (0 or 16), so one "
                      "below 16 and one above 31 cannot be driven by the same instance");

        using PioRegs = Kvasir::Peripheral::PIO::Registers<Instance>;
        using SmRegs  = typename PioRegs::template SM<Sm>;

        // EXECCTRL.STATUS_N: an enumerated field on the RP2350 (its upper bits select the
        // IRQ-flag source), a plain count on the RP2040.
        static constexpr auto statusN() {
            using E = typename SmRegs::EXECCTRL;
            if constexpr(requires { typename E::STATUS_NVal; }) {
                return Register::value<typename E::STATUS_NVal,
                                       static_cast<typename E::STATUS_NVal>(Config::movStatusN)>();
            } else {
                return Register::value<static_cast<std::uint32_t>(Config::movStatusN)>();
            }
        }

        using Fifo = typename PioRegs::template FIFO<Sm>;

        static constexpr std::uint32_t SmMask = 1U << Sm;

        // Startup: the state machine, the instruction slots (tagged with the program, so two
        // machines on one program share them), the instance's GPIO window, the pins, the
        // clock.
        using Provides = Kvasir::Pio::Provides<Instance, Sm, Offset, Program, GpioBase>;
        using Claims   = Clocks::Claim<Clocks::ClkSys, Config::ClockSpeed>;

        static constexpr auto powerClockEnable = list(Kvasir::Pio::getEnable<Instance>());

        template<typename... Pins>
        static constexpr auto pinConfigs(brigand::list<Pins...>) {
            if constexpr(sizeof...(Pins) == 0) {
                return brigand::list<>{};
            } else {
                return list(Kvasir::Pio::getPinConfig<Instance>(Pins{})...);
            }
        }

        template<typename... Pins>
        static constexpr auto inputPinConfigs(brigand::list<Pins...>) {
            if constexpr(sizeof...(Pins) == 0) {
                return brigand::list<>{};
            } else if constexpr(Config::inPullUp) {
                return list(Kvasir::Pio::getPinConfigPullUp<Instance>(Pins{})...);
            } else {
                return list(Kvasir::Pio::getPinConfig<Instance>(Pins{})...);
            }
        }

        // An input that is also an output (a bidirectional line) is configured as an output.
        using PureInputPins = typename detail::Difference<InputPins, OutputPins>::type;

        static constexpr auto initStepPinConfig
          = brigand::append<decltype(pinConfigs(OutputPins{})),
                            decltype(inputPinConfigs(PureInputPins{}))>{};

        static constexpr auto initStepPeripheryConfig = list(
          Kvasir::Pio::getDivConfig<SmRegs>([]() { return Config::clockDiv; }),
          SmRegs::EXECCTRL::overrideDefaults(
            write(SmRegs::EXECCTRL::wrap_bottom, Register::value<Program::WrapTarget + Offset>()),
            write(SmRegs::EXECCTRL::wrap_top, Register::value<Program::Wrap + Offset>()),
            write(SmRegs::EXECCTRL::side_en, Register::value<Config::sidesetOptional ? 1 : 0>()),
            write(SmRegs::EXECCTRL::side_pindir, Register::value<Config::sidesetPindirs ? 1 : 0>()),
            write(SmRegs::EXECCTRL::jmp_pin,
                  Register::value<detail::mappedBase<JmpPin>(GpioBase)>()),
            write(SmRegs::EXECCTRL::out_sticky, Register::value<Config::outSticky ? 1 : 0>()),
            write(SmRegs::EXECCTRL::status_sel,
                  Register::value<typename SmRegs::EXECCTRL::STATUS_SELVal,
                                  static_cast<typename SmRegs::EXECCTRL::STATUS_SELVal>(
                                    Config::movStatus == MovStatus::rxFifoLessThan ? 1
                                    : Config::movStatus == MovStatus::irqSet       ? 2
                                                                                   : 0)>()),
            write(SmRegs::EXECCTRL::status_n, statusN())),
          SmRegs::SHIFTCTRL::overrideDefaults(
            write(SmRegs::SHIFTCTRL::fjoin_rx, Register::value<Config::joinRx ? 1 : 0>()),
            write(SmRegs::SHIFTCTRL::fjoin_tx, Register::value<Config::joinTx ? 1 : 0>()),
            write(SmRegs::SHIFTCTRL::pull_thresh, Register::value<Config::pullThreshold % 32U>()),
            write(SmRegs::SHIFTCTRL::push_thresh, Register::value<Config::pushThreshold % 32U>()),
            write(SmRegs::SHIFTCTRL::out_shiftdir,
                  Register::value<Config::outShiftRight ? 1 : 0>()),
            write(SmRegs::SHIFTCTRL::in_shiftdir, Register::value<Config::inShiftRight ? 1 : 0>()),
            write(SmRegs::SHIFTCTRL::autopull, Register::value<Config::autopull ? 1 : 0>()),
            write(SmRegs::SHIFTCTRL::autopush, Register::value<Config::autopush ? 1 : 0>())));

    private:
        // `set pindirs, v` / `set pins, v` for one pin: point the set mapping at it, execute
        // the instruction. 0xE080 is SET PINDIRS with data 0, 0xE000 SET PINS with data 0;
        // bit 0 of the data is the direction or the level.
        template<typename Pin>
        static void setPinDir(Pin,
                              bool out) {
            constexpr auto n
              = Kvasir::Pio::pinIndex(static_cast<unsigned>(detail::pinNumber(Pin{})), GpioBase);
            apply(SmRegs::PINCTRL::overrideDefaults(
              write(SmRegs::PINCTRL::set_base, Register::value<n>()),
              write(SmRegs::PINCTRL::set_count, Register::value<1>())));
            apply(write(SmRegs::INSTR::instr, static_cast<std::uint32_t>(out ? 0xE081U : 0xE080U)));
        }

        template<typename Pin>
        static void setPinLevel(Pin,
                                bool high) {
            constexpr auto n
              = Kvasir::Pio::pinIndex(static_cast<unsigned>(detail::pinNumber(Pin{})), GpioBase);
            apply(SmRegs::PINCTRL::overrideDefaults(
              write(SmRegs::PINCTRL::set_base, Register::value<n>()),
              write(SmRegs::PINCTRL::set_count, Register::value<1>())));
            apply(
              write(SmRegs::INSTR::instr, static_cast<std::uint32_t>(high ? 0xE001U : 0xE000U)));
        }

        template<typename... Pins>
        static void setPinDirs(brigand::list<Pins...>,
                               bool out) {
            (setPinDir(Pins{}, out), ...);
        }

        template<typename... Pins>
        static void setPinLevels(brigand::list<Pins...>,
                                 bool high) {
            (setPinLevel(Pins{}, high), ...);
        }

    public:
        // Load the program and fix the pin levels and directions, before the machine is
        // enabled. The level first, so a pin that idles high never shows a low.
        static void preEnableRuntimeInit() {
            // Before any mapping is written and while no machine on the instance runs: every
            // base below counts from this window.
            Kvasir::Pio::applyGpioBase<Instance, GpioBase>();

            auto* addr = reinterpret_cast<std::uint16_t volatile*>(
              PioRegs::template INSTR_MEM<Offset>::Addr::value);
            for(auto const v : Program::Instructions) {
                *addr = v;
                addr += 2;   // one 32-bit register per instruction slot
            }

            setPinLevels(std::remove_cvref_t<decltype(Config::pinsInitialHigh)>{}, true);
            setPinDirs(OutputPins{}, true);
            setPinDirs(PureInputPins{}, false);

            apply(SmRegs::PINCTRL::overrideDefaults(
              write(SmRegs::PINCTRL::set_base,
                    Register::value<detail::mappedBase<SetPins>(GpioBase)>()),
              write(SmRegs::PINCTRL::set_count,
                    Register::value<static_cast<unsigned>(SetPins::count)>()),
              write(SmRegs::PINCTRL::out_base,
                    Register::value<detail::mappedBase<OutPins>(GpioBase)>()),
              write(SmRegs::PINCTRL::out_count,
                    Register::value<static_cast<unsigned>(OutPins::count)>()),
              write(SmRegs::PINCTRL::sideset_base,
                    Register::value<detail::mappedBase<SidePins>(GpioBase)>()),
              write(SmRegs::PINCTRL::sideset_count,
                    Register::value<static_cast<unsigned>(SidePins::count)
                                    + (Config::sidesetOptional ? 1U : 0U)>()),
              write(SmRegs::PINCTRL::in_base,
                    Register::value<detail::mappedBase<InPins>(GpioBase)>())));
        }

        // Jump to the program's first instruction. The restart bits and sm_enable live in
        // the instance's CTRL register, shared with the other machines, so they are written
        // at run time in runtimeInit() (after every peripheral's enable step, see
        // ws2812.hpp): four machines' literal writes to one field would otherwise be one
        // step with four different values, which Startup refuses.
        static constexpr auto initStepPeripheryEnable
          = list(write(SmRegs::INSTR::instr, Register::value<Offset>()));

        static void runtimeInit() {
            apply(write(PioRegs::CTRL::sm_restart, Register::value<SmMask>()));
            apply(write(PioRegs::CTRL::clkdiv_restart, Register::value<SmMask>()));
            apply(write(SmRegs::INSTR::instr, Register::value<Offset>()));
            setEnabled(true);
        }

        // -- the run-time interface --------------------------------------------------

        static void setEnabled(bool on) {
            auto const enabled = get<0>(apply(read(PioRegs::CTRL::sm_enable)));
            apply(write(PioRegs::CTRL::sm_enable, on ? (enabled | SmMask) : (enabled & ~SmMask)));
        }

        /// Restart: registers cleared, program counter at the program's start.
        static void restart() {
            apply(write(PioRegs::CTRL::sm_restart, Register::value<SmMask>()));
            apply(write(SmRegs::INSTR::instr, Register::value<Offset>()));
        }

        /// Execute one instruction out of band (a `set`, a `jmp`, a `pull`).
        static void exec(std::uint16_t instruction) {
            apply(write(SmRegs::INSTR::instr, static_cast<std::uint32_t>(instruction)));
        }

        [[nodiscard]] static bool txFull() {
            return (get<0>(apply(read(PioRegs::FSTAT::txfull))) & SmMask) != 0;
        }

        [[nodiscard]] static bool txEmpty() {
            return (get<0>(apply(read(PioRegs::FSTAT::txempty))) & SmMask) != 0;
        }

        [[nodiscard]] static bool rxEmpty() {
            return (get<0>(apply(read(PioRegs::FSTAT::rxempty))) & SmMask) != 0;
        }

        /// One word into the TX FIFO if there is room.
        static bool tryPush(std::uint32_t v) {
            if(txFull()) { return false; }
            apply(write(Fifo::TXF::fifo, v));
            return true;
        }

        /// One word out of the RX FIFO if one is waiting.
        [[nodiscard]] static std::optional<std::uint32_t> tryPop() {
            if(rxEmpty()) { return std::nullopt; }
            return get<0>(apply(read(Fifo::RXF::fifo)));
        }

        /// How many words the RX / TX FIFO holds.
        [[nodiscard]] static unsigned rxLevel() { return levelField<true>(); }

        [[nodiscard]] static unsigned txLevel() { return levelField<false>(); }

        /// This machine's program-relative IRQ flags: `irq n rel` sets flag (n + Sm) mod 4
        /// in the upper two bits' group; the whole 8-flag register is returned and flags are
        /// cleared with clearIrqFlags(mask). Pio::Irq routes chosen flags to the NVIC.
        [[nodiscard]] static std::uint32_t irqFlags() {
            return get<0>(apply(read(PioRegs::IRQ::irq)));
        }

        static void clearIrqFlags(std::uint32_t mask) { apply(write(PioRegs::IRQ::irq, mask)); }

        /// Which flag `irq N rel` of this machine sets: bits 1:0 of N plus Sm, modulo 4,
        /// within N's upper half.
        static constexpr std::uint32_t relativeFlag(unsigned n) {
            return (n & 0x4U) | ((n + Sm) & 0x3U);
        }

    private:
        template<bool Rx>
        static unsigned levelField() {
            if constexpr(Sm == 0) {
                return get<0>(apply(read(std::conditional_t<Rx,
                                                            decltype(PioRegs::FLEVEL::rx0),
                                                            decltype(PioRegs::FLEVEL::tx0)>{})));
            } else if constexpr(Sm == 1) {
                return get<0>(apply(read(std::conditional_t<Rx,
                                                            decltype(PioRegs::FLEVEL::rx1),
                                                            decltype(PioRegs::FLEVEL::tx1)>{})));
            } else if constexpr(Sm == 2) {
                return get<0>(apply(read(std::conditional_t<Rx,
                                                            decltype(PioRegs::FLEVEL::rx2),
                                                            decltype(PioRegs::FLEVEL::tx2)>{})));
            } else {
                return get<0>(apply(read(std::conditional_t<Rx,
                                                            decltype(PioRegs::FLEVEL::rx3),
                                                            decltype(PioRegs::FLEVEL::tx3)>{})));
            }
        }

    public:
        /// The TX FIFO stalled (the program waited on an empty FIFO) since the last clear.
        [[nodiscard]] static bool txStalled() {
            return (get<0>(apply(read(PioRegs::FDEBUG::txstall))) & SmMask) != 0;
        }

        static void clearTxStall() {
            apply(write(PioRegs::FDEBUG::txstall, Register::value<SmMask>()));
        }

        /// The address of the TX FIFO, and the DMA request that pairs with it, for a DMA-fed
        /// program (ws2812.hpp does exactly this).
        static constexpr std::uint32_t txFifoAddress = Fifo::TXF::Addr::value;
        static constexpr std::uint32_t rxFifoAddress = Fifo::RXF::Addr::value;

        template<typename Dma>
        static constexpr typename Dma::TriggerSource txDmaTrigger() {
            return Kvasir::Pio::getTxDmaTrigger<Dma, Instance, Sm>();
        }

        template<typename Dma>
        static constexpr typename Dma::TriggerSource rxDmaTrigger() {
            return Kvasir::Pio::getRxDmaTrigger<Dma, Instance, Sm>();
        }
    };

}}   // namespace Kvasir::Pio
