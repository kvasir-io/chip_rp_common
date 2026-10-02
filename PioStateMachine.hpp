#pragma once
#include "Clocks.hpp"
#include "Io.hpp"
#include "PIO.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/RESETS.hpp"
#include "pio/Asm.hpp"

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
//   struct BlinkProgram : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
//       a.set(Kvasir::Pio::Set::pins, 1);     // the program, built by the compiler (pio/Asm.hpp;
//       a.set(Kvasir::Pio::Set::pins, 0);     // pio/AsmParse.hpp reads pioasm text instead)
//   })> {};
//
//   struct BlinkSmConfig {
//       static constexpr auto ClockSpeed  = HW::ClockSpeed;         // clk_sys (the divider's base)
//       static constexpr auto PioInstance = 0;                      // PIO0..2
//       static constexpr auto SmInstance  = 0;                      // 0..3
//       static constexpr auto setPins     = brigand::list<HW::Pin::led>{};   // `set pins`
//   };
//   using BlinkSm = Kvasir::Pio::StateMachine<BlinkProgram, BlinkSmConfig>;
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
//   sidesetOptional, sidesetPindirs   `.side_set N opt` (the enable bit is part of the count)
//                              and `.side_set N pindirs` (side-set drives directions): taken
//                              from the program when it carries them (Asm.hpp's Program; a
//                              pioasm header since kvasir_output emits them), else false.
//                              Named here they must agree with the program's .side_set
//   autopull (false), pullThreshold (32), outShiftRight (true)
//   autopush (false), pushThreshold (32), inShiftRight (true)
//   joinTx, joinRx (false)     an 8-deep FIFO in one direction
//                              These, and movStatus below, come from the program's .out, .in,
//                              .fifo and .mov_status when it gives them (a builder program:
//                              a.outConfig(), a.inConfig(), a.fifo(), a.movStatus()); named here
//                              they must agree with it, and outPins / inPins / setPins must be
//                              as many pins as its .out / .in / .set
//   jmpPin (none)              the pin `jmp pin` tests (a brigand::list of one pin)
//   inPullUp (false)           pull-ups on the in pins and the jmp pin
//   pinsInitialHigh (empty)    output pins driven high before the machine starts
//   outSticky (false)          EXECCTRL.OUT_STICKY
//   movStatus (none), movStatusN (0)   what `mov x, status` reports (MovStatus)
//   driveStrength (4 mA), slewFast (false)   the machine's pads
//   pullUpPins, pullDownPins (empty)   pulls on any of the machine's pins (inPullUp: on
//                              every input pin)
//   outputInverted, oeInverted (empty)   GPIO OUTOVER / OEOVER = invert for these pins: a
//                              clock idling high, an open-drain line driven through
//                              `set pindirs` (pico-sdk gpio_set_outover / _oeover)
//   inputSyncBypass (empty)    pins whose 2-flop input synchroniser is bypassed (INPUT_SYNC_
//                              BYPASS): fast synchronous inputs, a counter above ~37 MHz
//   foreignPads (empty)        input pins (in pins, the jmp pin) whose pads belong to someone
//                              else: the machine maps and reads them but leaves their function,
//                              pull, overrides and direction alone. A PIO reads a pad's input
//                              whatever its function ("the input is always connected, so the
//                              PIOs can always see the state of all pins": RP2350 datasheet 9.4,
//                              RP2040 datasheet 2.19.2), as long as the owner keeps the pad's
//                              input enabled (PADS IE; every Io pin action does). So a clock
//                              on a GPOUT pin, the clock block's GPIN, or a pin the board's pin
//                              table sets up can be counted without the two fighting over it.
//                              Claimed rather than provided, so their owner has to be in the
//                              Startup list. Never an output; no pullUpPins / pullDownPins /
//                              outputInverted / oeInverted (inPullUp leaves them out); the
//                              synchroniser bypass is the PIO's own and stays allowed
//   rxFifoPut, rxFifoGet (from the program's .fifo txput / txget / putget, RP2350)
//   startEnabled (true)        false: loaded and configured but left stopped; the driver
//                              starts it itself (restartAt(), setEnabled())
//
// Two state machines may run one program at one offset (the slots are shared); different
// programs on overlapping slots are a build error.
namespace Kvasir { namespace Pio {

    enum class MovStatus { none, txFifoLessThan, rxFifoLessThan, irqSet };

    /// The shift and FIFO settings (SHIFTCTRL) as one value: what the config sets up, and
    /// what StateMachine::setShift<>() switches to at run time (a driver with phases that
    /// shift differently, like PioQspi's write and read).
    struct Shift {
        bool     autopull{false};
        unsigned pullThreshold{32};
        bool     outShiftRight{true};
        bool     autopush{false};
        unsigned pushThreshold{32};
        bool     inShiftRight{true};
        bool     joinTx{false};
        bool     joinRx{false};
        // RP2350: the RX FIFO's four entries as registers - put: the machine writes them
        // (`mov rxfifo[i], isr`), get: it reads them (`mov osr, rxfifo[i]`); the processor
        // reaches them (rxEntry / setRxEntry) when exactly one of the two is set
        bool rxPut{false};
        bool rxGet{false};
    };

    namespace detail {
        using Kvasir::Io::pinNumber;

        // A program that says what its .side_set is (Asm.hpp's Program, and pioasm headers
        // since kvasir_output.cpp emits it); headers from an older pioasm do not.
        // A program that says what its other directives are: .fifo, .in, .out, .set,
        // .mov_status (Asm.hpp's Program and every pioasm kvasir header; -1 = not given, and
        // FifoMode txrx, pioasm's default, is taken as not given).
        template<typename P>
        concept ProgramWithDirectives = requires {
            P::FifoMode;
            P::InPinCount;
            P::InRight;
            P::InAutoP;
            P::InThreshold;
            P::OutPinCount;
            P::OutRight;
            P::OutAutoP;
            P::OutThreshold;
            P::SetCount;
            P::MovStatusType;
            P::MovStatusN;
        };

        template<typename P>
        constexpr bool programSaysOut() {
            if constexpr(ProgramWithDirectives<P>) { return P::OutPinCount >= 0; }
            return false;
        }

        template<typename P>
        constexpr bool programSaysIn() {
            if constexpr(ProgramWithDirectives<P>) { return P::InPinCount >= 0; }
            return false;
        }

        template<typename P>
        constexpr int programFifo() {
            if constexpr(ProgramWithDirectives<P>) { return P::FifoMode; }
            return 0;
        }

        template<typename P>
        constexpr bool programSaysMovStatus() {
            if constexpr(ProgramWithDirectives<P>) { return P::MovStatusType >= 0; }
            return false;
        }

        template<typename P>
        concept ProgramWithSideset = requires {
            P::SidesetCount;
            P::SidesetOptional;
            P::SidesetPindirs;
        };

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
                } else if constexpr(detail::ProgramWithSideset<Program>) {
                    return Program::SidesetOptional;
                } else {
                    return false;
                }
            }();
            static constexpr bool sidesetPindirs = [] {
                if constexpr(requires { Config_::sidesetPindirs; }) {
                    return Config_::sidesetPindirs;
                } else if constexpr(detail::ProgramWithSideset<Program>) {
                    return Program::SidesetPindirs;
                } else {
                    return false;
                }
            }();
            static constexpr bool autopull = [] {
                if constexpr(requires { Config_::autopull; }) {
                    return Config_::autopull;
                } else {
                    if constexpr(detail::programSaysOut<Program>()) {
                        return Program::OutAutoP;
                    }   // from the program
                    return false;
                }
            }();
            static constexpr unsigned pullThreshold = [] {
                if constexpr(requires { Config_::pullThreshold; }) {
                    return static_cast<unsigned>(Config_::pullThreshold);
                } else {
                    if constexpr(detail::programSaysOut<Program>()) {
                        return static_cast<unsigned>(Program::OutThreshold);
                    }   // from the program
                    return 32U;
                }
            }();
            static constexpr bool outShiftRight = [] {
                if constexpr(requires { Config_::outShiftRight; }) {
                    return Config_::outShiftRight;
                } else {
                    if constexpr(detail::programSaysOut<Program>()) {
                        return Program::OutRight;
                    }   // from the program
                    return true;
                }
            }();
            static constexpr bool autopush = [] {
                if constexpr(requires { Config_::autopush; }) {
                    return Config_::autopush;
                } else {
                    if constexpr(detail::programSaysIn<Program>()) {
                        return Program::InAutoP;
                    }   // from the program
                    return false;
                }
            }();
            static constexpr unsigned pushThreshold = [] {
                if constexpr(requires { Config_::pushThreshold; }) {
                    return static_cast<unsigned>(Config_::pushThreshold);
                } else {
                    if constexpr(detail::programSaysIn<Program>()) {
                        return static_cast<unsigned>(Program::InThreshold);
                    }   // from the program
                    return 32U;
                }
            }();
            static constexpr bool inShiftRight = [] {
                if constexpr(requires { Config_::inShiftRight; }) {
                    return Config_::inShiftRight;
                } else {
                    if constexpr(detail::programSaysIn<Program>()) {
                        return Program::InRight;
                    }   // from the program
                    return true;
                }
            }();
            static constexpr bool joinTx = [] {
                if constexpr(requires { Config_::joinTx; }) {
                    return Config_::joinTx;
                } else {
                    if constexpr(detail::programFifo<Program>() != 0) {
                        return detail::programFifo<Program>() == 1;
                    }   // from the program
                    return false;
                }
            }();
            static constexpr bool joinRx = [] {
                if constexpr(requires { Config_::joinRx; }) {
                    return Config_::joinRx;
                } else {
                    if constexpr(detail::programFifo<Program>() != 0) {
                        return detail::programFifo<Program>() == 2;
                    }   // from the program
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
                    if constexpr(detail::programSaysMovStatus<Program>()) {
                        return static_cast<MovStatus>(Program::MovStatusType + 1);
                    }   // from the program
                    return MovStatus::none;
                }
            }();
            // .fifo txput / txget / putget (RP2350): the RX FIFO as random-access registers
            static constexpr bool rxFifoPut = [] {
                if constexpr(requires { Config_::rxFifoPut; }) {
                    return Config_::rxFifoPut;
                } else if constexpr(detail::programFifo<Program>() != 0) {
                    return detail::programFifo<Program>() == 4
                        || detail::programFifo<Program>() == 5;
                } else {
                    return false;
                }
            }();
            static constexpr bool rxFifoGet = [] {
                if constexpr(requires { Config_::rxFifoGet; }) {
                    return Config_::rxFifoGet;
                } else if constexpr(detail::programFifo<Program>() != 0) {
                    return detail::programFifo<Program>() == 3
                        || detail::programFifo<Program>() == 5;
                } else {
                    return false;
                }
            }();
            // pins: pulls beyond inPullUp, GPIO overrides, the input synchroniser
            static constexpr auto pullUpPins = [] {
                if constexpr(requires { Config_::pullUpPins; }) {
                    return Config_::pullUpPins;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr auto pullDownPins = [] {
                if constexpr(requires { Config_::pullDownPins; }) {
                    return Config_::pullDownPins;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr auto outputInverted = [] {
                if constexpr(requires { Config_::outputInverted; }) {
                    return Config_::outputInverted;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr auto oeInverted = [] {
                if constexpr(requires { Config_::oeInverted; }) {
                    return Config_::oeInverted;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr auto inputSyncBypass = [] {
                if constexpr(requires { Config_::inputSyncBypass; }) {
                    return Config_::inputSyncBypass;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr auto foreignPads = [] {
                if constexpr(requires { Config_::foreignPads; }) {
                    return Config_::foreignPads;
                } else {
                    return brigand::list<>{};
                }
            }();
            static constexpr auto driveStrength = [] {
                if constexpr(requires { Config_::driveStrength; }) {
                    return Config_::driveStrength;
                } else {
                    return Kvasir::Io::DriveStrength::mA_4;   // what the plain pin function writes
                }
            }();
            static constexpr bool slewFast = [] {
                if constexpr(requires { Config_::slewFast; }) {
                    return Config_::slewFast;
                } else {
                    return false;
                }
            }();
            // false: the Startup list loads and configures the machine but leaves it stopped, for
            // a driver that starts it itself at an entry point (PioQspi)
            static constexpr bool startEnabled = [] {
                if constexpr(requires { Config_::startEnabled; }) {
                    return Config_::startEnabled;
                } else {
                    return true;
                }
            }();
            static constexpr unsigned movStatusN = [] {
                if constexpr(requires { Config_::movStatusN; }) {
                    return static_cast<unsigned>(Config_::movStatusN);
                } else {
                    if constexpr(detail::programSaysMovStatus<Program>()) {
                        return static_cast<unsigned>(Program::MovStatusN);
                    }   // from the program
                    return 0U;
                }
            }();
        };

        static constexpr unsigned Instance = Config::PioInstance;
        static constexpr unsigned Sm       = Config::SmInstance;
        static constexpr unsigned Offset   = Config::ProgramOffset;

        // The program's other directives (.fifo, .in, .out, .set, .mov_status) against what the
        // config names: a config value the program contradicts is the same misread machine as
        // a wrong side-set - an autopull the program does not expect, a shift the wrong way.
        template<typename C>
        static constexpr bool directivesAgree() {
            if constexpr(!detail::ProgramWithDirectives<Program>) {
                return true;
            } else {
                bool ok = true;
                if constexpr(detail::programSaysOut<Program>()) {
                    if constexpr(requires { C::autopull; }) {
                        ok = ok && C::autopull == Program::OutAutoP;
                    }
                    if constexpr(requires { C::pullThreshold; }) {
                        ok = ok && static_cast<int>(C::pullThreshold) == Program::OutThreshold;
                    }
                    if constexpr(requires { C::outShiftRight; }) {
                        ok = ok && C::outShiftRight == Program::OutRight;
                    }
                }
                if constexpr(detail::programSaysIn<Program>()) {
                    if constexpr(requires { C::autopush; }) {
                        ok = ok && C::autopush == Program::InAutoP;
                    }
                    if constexpr(requires { C::pushThreshold; }) {
                        ok = ok && static_cast<int>(C::pushThreshold) == Program::InThreshold;
                    }
                    if constexpr(requires { C::inShiftRight; }) {
                        ok = ok && C::inShiftRight == Program::InRight;
                    }
                }
                if constexpr(detail::programFifo<Program>() != 0) {
                    if constexpr(requires { C::rxFifoPut; }) {
                        ok = ok
                          && C::rxFifoPut == (Program::FifoMode == 4 || Program::FifoMode == 5);
                    }
                    if constexpr(requires { C::rxFifoGet; }) {
                        ok = ok
                          && C::rxFifoGet == (Program::FifoMode == 3 || Program::FifoMode == 5);
                    }
                    if constexpr(requires { C::joinTx; }) {
                        ok = ok && C::joinTx == (Program::FifoMode == 1);
                    }
                    if constexpr(requires { C::joinRx; }) {
                        ok = ok && C::joinRx == (Program::FifoMode == 2);
                    }
                }
                if constexpr(detail::programSaysMovStatus<Program>()) {
                    if constexpr(requires { C::movStatus; }) {
                        ok = ok
                          && C::movStatus == static_cast<MovStatus>(Program::MovStatusType + 1);
                    }
                    if constexpr(requires { C::movStatusN; }) {
                        ok = ok && static_cast<int>(C::movStatusN) == Program::MovStatusN;
                    }
                }
                return ok;
            }
        }

        static_assert(directivesAgree<Config_>(),
                      "the config's autopull/autopush, thresholds, shift directions, FIFO join or "
                      "movStatus disagree with the program's .out/.in/.fifo/.mov_status");
        static_assert(!(Config::rxFifoPut || Config::rxFifoGet)
                        || !PinConfig::isRp2040(PinConfig::CurrentChip),
                      "RX FIFO random access (.fifo txput / txget / putget) is an RP2350 feature");
        static_assert(!(Config::rxFifoPut || Config::rxFifoGet) || !Config::joinRx,
                      "the RX FIFO is either joined or random-access registers, not both");

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

        // PINCTRL.SIDESET_COUNT and EXECCTRL.SIDE_EN / SIDE_PINDIR are how the machine splits
        // bits 12:8 of every instruction (RP2350 datasheet 11.4.1, 11.5.1): set differently from
        // the program's .side_set, it reads side-set bits as delay and the other way round.
        static constexpr bool sidesetAgrees = [] {
            if constexpr(detail::ProgramWithSideset<Program>) {
                return Config::sidesetOptional == Program::SidesetOptional
                    && Config::sidesetPindirs == Program::SidesetPindirs;
            } else {
                return true;
            }
        }();
        static_assert(sidesetAgrees,
                      "sidesetOptional / sidesetPindirs disagree with the program's .side_set");
        static constexpr bool sidesetPinsAgree = [] {
            if constexpr(detail::ProgramWithSideset<Program>) {
                return SidePins::count == Program::SidesetCount;
            } else {
                return true;
            }
        }();
        static_assert(sidesetPinsAgree,
                      "sidesetPins must be as many pins as the program's .side_set count");

        // `.out N` / `.in N` / `.set N` are the program's pin counts: the mappings must match
        static constexpr bool pinCountsAgree = [] {
            if constexpr(!detail::ProgramWithDirectives<Program>) {
                return true;
            } else {
                return (Program::OutPinCount < 0 || OutPins::count == Program::OutPinCount)
                    && (Program::InPinCount < 0 || Program::InPinCount == 32
                        || InPins::count == Program::InPinCount)
                    && (Program::SetCount < 0 || SetPins::count == Program::SetCount);
            }
        }();
        static_assert(
          pinCountsAgree,
          "outPins / inPins / setPins must be as many pins as the program's .out / .in / .set");

        static constexpr bool originAgrees = [] {
            if constexpr(requires { Program::Origin; }) {
                return Program::Origin < 0
                    || static_cast<unsigned>(Program::Origin) == Config::ProgramOffset;
            } else {
                return true;
            }
        }();
        static_assert(originAgrees,
                      "the program has .origin: ProgramOffset must be that slot");

        // Decided by the words (Asm.hpp isPioV1Only), not by the version the program declares
        static_assert(!PinConfig::isRp2040(PinConfig::CurrentChip)
                        || !Kvasir::Pio::usesPioV1(Program::Instructions),
                      "the program uses a PIO version 1 instruction (RP2350 only: wait jmppin, "
                      "irq prev/next, mov rxfifo[], mov pindirs); the RP2040 has PIO version 0");

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
        using AllPins     = typename detail::Unique<brigand::append<OutputPins, InputPins>>::type;
        using ForeignPads = std::remove_cvref_t<decltype(Config::foreignPads)>;

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

        /// The shift settings the config (and the program) gave.
        static constexpr Shift ConfiguredShift{Config::autopull,
                                               Config::pullThreshold,
                                               Config::outShiftRight,
                                               Config::autopush,
                                               Config::pushThreshold,
                                               Config::inShiftRight,
                                               Config::joinTx,
                                               Config::joinRx,
                                               Config::rxFifoPut,
                                               Config::rxFifoGet};

        // IN_COUNT (RP2350 only): how many IN pins are not masked to 0 - the program's `.in N`,
        // as pico-sdk sets it; left at its reset value 0 (= 32) when the program does not say.
        static constexpr unsigned InCount = [] {
            if constexpr(detail::programSaysIn<Program>()) {
                return static_cast<unsigned>(Program::InPinCount) % 32U;
            } else {
                return 0U;
            }
        }();

        template<Shift S>
        static constexpr auto shiftCtrl() {
            static_assert(!S.joinTx || !S.joinRx, "a FIFO can be joined in one direction only");
            static_assert(S.pullThreshold >= 1 && S.pullThreshold <= 32, "pull threshold is 1..32");
            static_assert(S.pushThreshold >= 1 && S.pushThreshold <= 32, "push threshold is 1..32");
            auto const common = [](auto... more) {
                return SmRegs::SHIFTCTRL::overrideDefaults(
                  write(SmRegs::SHIFTCTRL::fjoin_rx, Register::value<S.joinRx ? 1 : 0>()),
                  write(SmRegs::SHIFTCTRL::fjoin_tx, Register::value<S.joinTx ? 1 : 0>()),
                  write(SmRegs::SHIFTCTRL::pull_thresh, Register::value<S.pullThreshold % 32U>()),
                  write(SmRegs::SHIFTCTRL::push_thresh, Register::value<S.pushThreshold % 32U>()),
                  write(SmRegs::SHIFTCTRL::out_shiftdir,
                        Register::value<S.outShiftRight ? 1 : 0>()),
                  write(SmRegs::SHIFTCTRL::in_shiftdir, Register::value<S.inShiftRight ? 1 : 0>()),
                  write(SmRegs::SHIFTCTRL::autopull, Register::value<S.autopull ? 1 : 0>()),
                  write(SmRegs::SHIFTCTRL::autopush, Register::value<S.autopush ? 1 : 0>()),
                  more...);
            };
            static_assert(!(S.rxPut || S.rxGet) || !S.joinRx,
                          "the RX FIFO is either joined or random-access registers, not both");
            if constexpr(requires { SmRegs::SHIFTCTRL::in_count; }) {
                return common(
                  write(SmRegs::SHIFTCTRL::in_count, Register::value<InCount>()),
                  write(SmRegs::SHIFTCTRL::fjoin_rx_put, Register::value<S.rxPut ? 1 : 0>()),
                  write(SmRegs::SHIFTCTRL::fjoin_rx_get, Register::value<S.rxGet ? 1 : 0>()));
            } else {
                static_assert(!(S.rxPut || S.rxGet), "RX FIFO random access is an RP2350 feature");
                return common();
            }
        }

        static constexpr std::uint32_t SmMask = 1U << Sm;

        // Startup: the state machine, the instruction slots (tagged with the program, so two
        // machines on one program share them), the instance's GPIO window, the pins, the
        // clock.
        using Provides = Kvasir::Pio::Provides<Instance, Sm, Offset, Program, GpioBase>;
        // foreign pads are claimed: their owner has to be in the list and configure them
        using Claims = brigand::append<Clocks::Claim<Clocks::ClkSys, Config::ClockSpeed>,
                                       brigand::wrap<ForeignPads, Kvasir::Io::PinClaims>>;

        static constexpr auto powerClockEnable = list(Kvasir::Pio::getEnable<Instance>());

        template<typename List, typename Pin>
        static constexpr bool listed = brigand::any<List, std::is_same<brigand::_1, Pin>>::value;

        template<typename C,
                 typename Pin,
                 bool Input>
        static constexpr Kvasir::Io::PullConfiguration pullOf() {
            if constexpr(listed<std::remove_cvref_t<decltype(C::pullUpPins)>, Pin>) {
                return Kvasir::Io::PullConfiguration::PullUp;
            } else if constexpr(listed<std::remove_cvref_t<decltype(C::pullDownPins)>, Pin>) {
                return Kvasir::Io::PullConfiguration::PullDown;
            } else if constexpr(Input && C::inPullUp) {
                return Kvasir::Io::PullConfiguration::PullUp;
            } else {
                return Kvasir::Io::PullConfiguration::PullNone;
            }
        }

        template<typename List,
                 typename Pin>
        static constexpr Kvasir::Io::PinOverride overrideOf() {
            return listed<List, Pin> ? Kvasir::Io::PinOverride::invert
                                     : Kvasir::Io::PinOverride::normal;
        }

        // One action per pin: the PIO function, the pad's drive and slew, its pull, and the
        // GPIO overrides - all of the pin's settings in the one write Startup allows.
        template<bool Input,
                 typename... Pins>
        static constexpr auto pinConfigs(brigand::list<Pins...>) {
            if constexpr(sizeof...(Pins) == 0) {
                return brigand::list<>{};
            } else {
                return list(action(
                  Kvasir::Io::Action::PinFunctionDrive<
                    Kvasir::Pio::pinFunction<Instance>,
                    Config::driveStrength,
                    Config::slewFast,
                    pullOf<Config, Pins, Input>(),
                    Kvasir::Io::OutputInit::Low,
                    overrideOf<std::remove_cvref_t<decltype(Config::outputInverted)>, Pins>(),
                    overrideOf<std::remove_cvref_t<decltype(Config::oeInverted)>, Pins>()>{},
                  Pins{})...);
            }
        }

        // An input that is also an output (a bidirectional line) is configured as an output.
        using PureInputPins = typename detail::Difference<InputPins, OutputPins>::type;

        // The inputs whose pads are this machine's: all but the foreign ones.
        using OwnInputPins = typename detail::Difference<PureInputPins, ForeignPads>::type;

        static constexpr auto initStepPinConfig
          = brigand::append<decltype(pinConfigs<false>(OutputPins{})),
                            decltype(pinConfigs<true>(OwnInputPins{}))>{};

        // Every pin a pin option names is one of the machine's pins.
        template<typename... Pins>
        static constexpr bool allMine(brigand::list<Pins...>) {
            return (listed<AllPins, Pins> && ...);
        }

        static_assert(allMine(std::remove_cvref_t<decltype(Config::pullUpPins)>{})
                        && allMine(std::remove_cvref_t<decltype(Config::pullDownPins)>{})
                        && allMine(std::remove_cvref_t<decltype(Config::outputInverted)>{})
                        && allMine(std::remove_cvref_t<decltype(Config::oeInverted)>{})
                        && allMine(std::remove_cvref_t<decltype(Config::inputSyncBypass)>{}),
                      "pullUpPins / pullDownPins / outputInverted / oeInverted / inputSyncBypass "
                      "name a pin that is not one of this machine's (set, out, side-set, in, jmp)");

        template<typename... Pins>
        static constexpr bool noneOf(brigand::list<Pins...>,
                                     auto list) {
            return (!listed<decltype(list), Pins> && ...);
        }

        static_assert(allMine(ForeignPads{})
                        && noneOf(ForeignPads{},
                                  OutputPins{}),
                      "foreignPads names a pin that is not an input of this machine (in, jmp): "
                      "an output needs its pad set to the PIO's function");
        // Pulls and overrides are written by the pad's pin action, which a foreign pad does not
        // get: named for one they would be dropped without a word. inPullUp reaches the
        // configured pads only, so it needs no rule here.
        static_assert(noneOf(ForeignPads{},
                             std::remove_cvref_t<decltype(Config::pullUpPins)>{})
                        && noneOf(ForeignPads{},
                                  std::remove_cvref_t<decltype(Config::pullDownPins)>{})
                        && noneOf(ForeignPads{},
                                  std::remove_cvref_t<decltype(Config::outputInverted)>{})
                        && noneOf(ForeignPads{},
                                  std::remove_cvref_t<decltype(Config::oeInverted)>{}),
                      "pullUpPins / pullDownPins / outputInverted / oeInverted name a foreign "
                      "pad: its pad and GPIO settings are its owner's");

        // INPUT_SYNC_BYPASS: one bit per GPIO, counted from the instance's GPIO window (as
        // pico-sdk pio_set_input_sync_bypass_with_mask64 shifts it by the GPIO base)
        template<typename... Pins>
        static constexpr std::uint32_t syncBypassMask(brigand::list<Pins...>) {
            return (0U | ...
                    | (1U << Kvasir::Pio::pinIndex(static_cast<unsigned>(detail::pinNumber(Pins{})),
                                                   GpioBase)));
        }

        static constexpr std::uint32_t SyncBypassMask
          = syncBypassMask(std::remove_cvref_t<decltype(Config::inputSyncBypass)>{});

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
          shiftCtrl<ConfiguredShift>());

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
        // JMP targets are absolute instruction addresses, so a program loaded at a non-zero
        // offset needs them relocated. JMP is opcode 000 in bits 15:13, target in bits 4:0;
        // no other instruction carries an address.
        static constexpr auto RelocatedInstructions = [] {
            auto instructions = Program::Instructions;
            for(auto& i : instructions) {
                if((i >> 13U) == 0U) {
                    auto const target = static_cast<std::uint16_t>(((i & 0x1FU) + Offset) & 0x1FU);
                    i = static_cast<std::uint16_t>((i & static_cast<std::uint16_t>(~0x1FU))
                                                   | target);
                }
            }
            return instructions;
        }();

        static_assert(Offset + Program::Instructions.size() <= 32,
                      "the program does not fit in instruction memory at this ProgramOffset: a "
                      "PIO instance has 32 slots and a JMP target is five bits");

        // The reset release (powerClockEnable) is not immediate: the instance is usable once
        // RESET_DONE says so. PioQspi's CYW43 predecessor failed intermittently without the wait.
        static bool resetDone() {
            using Done = typename Peripheral::RESETS::Registers<Instance * 0>::RESET_DONE;
            if constexpr(Instance == 0) {
                return get<0>(apply(read(Done::pio0))) != 0;
            } else if constexpr(Instance == 1) {
                return get<0>(apply(read(Done::pio1))) != 0;
            } else {
                return get<0>(apply(read(Done::pio2))) != 0;
            }
        }

        static void preEnableRuntimeInit() {
            while(!resetDone()) {}
            if constexpr(SyncBypassMask != 0) {
                // the register is the instance's, shared with the other machines: set our bits
                auto const bypass = get<0>(apply(read(PioRegs::INPUT_SYNC_BYPASS::FULLREGISTER)));
                apply(write(PioRegs::INPUT_SYNC_BYPASS::FULLREGISTER, bypass | SyncBypassMask));
            }
            // Before any mapping is written and while no machine on the instance runs: every
            // base below counts from this window.
            Kvasir::Pio::applyGpioBase<Instance, GpioBase>();

            auto* addr = reinterpret_cast<std::uint16_t volatile*>(
              PioRegs::template INSTR_MEM<Offset>::Addr::value);
            for(auto const v : RelocatedInstructions) {
                *addr = v;
                addr += 2;   // one 32-bit register per instruction slot
            }

            setPinLevels(std::remove_cvref_t<decltype(Config::pinsInitialHigh)>{}, true);
            setPinDirs(OutputPins{}, true);
            setPinDirs(OwnInputPins{}, false);

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
            if constexpr(Config::startEnabled) { setEnabled(true); }
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

        /// Restart at one of the program's entry points (an offset in the program, as
        /// Program::offset("label") gives it): registers cleared, the clock divider's phase
        /// too, then a jmp there. The machine keeps its enable; stop it first to start clean.
        static void restartAt(unsigned entry) {
            apply(write(PioRegs::CTRL::sm_restart, Register::value<SmMask>()),
                  write(PioRegs::CTRL::clkdiv_restart, Register::value<SmMask>()));
            exec(static_cast<std::uint16_t>((entry + Offset) & 0x1FU));   // JMP (always) to it
        }

        /// Another clock divider at run time (a second bus speed), computed at compile time
        /// like the config's clockDiv.
        template<double Div>
        static void setClockDiv() {
            static_assert(
              Kvasir::Pio::divInRange<SmRegs>(Div),
              "clock divider out of range: at least 1.0, below 65536 (16.8 fixed point)");
            constexpr auto d = Kvasir::Pio::getDiv(Div);
            apply(write(SmRegs::CLKDIV::_int, Register::value<std::get<0>(d)>()),
                  write(SmRegs::CLKDIV::frac, Register::value<std::get<1>(d)>()));
        }

        /// Other shift and FIFO settings at run time, all fields at once (the IN_COUNT of the
        /// program's `.in` kept). Changing a FIFO join flushes both FIFOs.
        template<Shift S>
        static void setShift() {
            apply(shiftCtrl<S>());
        }

        /// Empty both FIFOs: a change of FJOIN_RX flushes them (RP2350 datasheet SMx_SHIFTCTRL,
        /// RP2040 datasheet 3.7 Table 383), so it is flipped and put back.
        static void clearFifos() {
            auto const joined = get<0>(apply(read(SmRegs::SHIFTCTRL::fjoin_rx)));
            apply(write(SmRegs::SHIFTCTRL::fjoin_rx, joined ^ 1U));
            apply(write(SmRegs::SHIFTCTRL::fjoin_rx, joined));
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

        /// One word into the TX FIFO, waiting while it is full (pico-sdk pio_sm_put_blocking).
        static void push(std::uint32_t v) {
            while(txFull()) {}
            apply(write(Fifo::TXF::fifo, v));
        }

        /// One word out of the RX FIFO, waiting while it is empty (pio_sm_get_blocking).
        [[nodiscard]] static std::uint32_t pop() {
            while(rxEmpty()) {}
            return get<0>(apply(read(Fifo::RXF::fifo)));
        }

        /// Empty the TX FIFO through the machine: `out null, 32` with autopull on, `pull
        /// noblock` without (pio_sm_drain_tx_fifo). The OSR ends up with the last word.
        static void drainTxFifo() {
            bool const autopull = get<0>(apply(read(SmRegs::SHIFTCTRL::autopull))) != 0;
            auto const instr    = autopull ? Enc::out(3, 32) : Enc::pull(false, false);
            while(!txEmpty()) { exec(instr); }
        }

        /// exec() and wait until the instruction has completed (a `wait`, a blocking `pull`
        /// stalls it: EXECCTRL.EXEC_STALLED; pio_sm_exec_wait_blocking).
        static void execWait(std::uint16_t instruction) {
            exec(instruction);
            while(get<0>(apply(read(SmRegs::EXECCTRL::exec_stalled))) != 0) {}
        }

        /// The RX FIFO's entry I as a register (RP2350, .fifo txput or txget): with put the
        /// machine writes it and this reads it, with get this writes it and the machine reads.
        template<unsigned I>
        [[nodiscard]] static std::uint32_t rxEntry() {
            static_assert(I < 4, "the RX FIFO has four entries");
            static_assert(Config::rxFifoPut != Config::rxFifoGet,
                          "the processor reaches the entries with exactly one of put / get set");
            using E = typename PioRegs::template RXF_PUTGET<Sm>::template ENTRY<I>;
            return get<0>(apply(read(E::FULLREGISTER)));
        }

        template<unsigned I>
        static void setRxEntry(std::uint32_t v) {
            static_assert(I < 4, "the RX FIFO has four entries");
            static_assert(Config::rxFifoPut != Config::rxFifoGet,
                          "the processor reaches the entries with exactly one of put / get set");
            using E = typename PioRegs::template RXF_PUTGET<Sm>::template ENTRY<I>;
            apply(write(E::FULLREGISTER, v));
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

    namespace detail {
        template<typename... Sms>
        constexpr std::uint32_t syncMask(unsigned instance) {
            return (0U | ... | (Sms::Instance == instance ? Sms::SmMask : 0U));
        }
    }   // namespace detail

    /// Start state machines in step: their clock dividers restarted and their enable set in
    /// one register write, so they run in lockstep from the same cycle (pico-sdk
    /// pio_enable_sm_mask_in_sync; across PIO blocks pio_enable_sm_multi_mask_in_sync). On one
    /// PIO block anywhere; across blocks on the RP2350 only, through the first one's CTRL and
    /// its NEXT/PREV masks (RP2350 datasheet PIO CTRL: NEXTPREV_SM_ENABLE,
    /// NEXTPREV_CLKDIV_RESTART). Give the machines startEnabled = false.
    template<typename... Sms>
    void startInSync() {
        static_assert(sizeof...(Sms) >= 1, "which state machines?");
        static_assert((!Sms::Config::startEnabled && ...),
                      "a machine started in step must not start by itself: startEnabled = false");
        using First                   = brigand::front<brigand::list<Sms...>>;
        constexpr unsigned      home  = First::Instance;
        constexpr unsigned      count = PinConfig::pioCount(PinConfig::CurrentChip);
        constexpr unsigned      next  = (home + 1) % count;
        constexpr unsigned      prev  = (home + count - 1) % count;
        constexpr std::uint32_t own   = detail::syncMask<Sms...>(home);
        constexpr std::uint32_t nm    = detail::syncMask<Sms...>(next);
        constexpr std::uint32_t pm    = detail::syncMask<Sms...>(prev);
        static_assert(
          ((Sms::Instance == home || Sms::Instance == next || Sms::Instance == prev) && ...),
          "every machine is on the first one's PIO block or a neighbouring one");
        using Ctrl         = typename Kvasir::Peripheral::PIO::Registers<home>::CTRL;
        auto const enabled = get<0>(apply(read(Ctrl::sm_enable)));
        if constexpr(nm == 0 && pm == 0) {
            apply(write(Ctrl::sm_enable, enabled | own),
                  write(Ctrl::clkdiv_restart, Register::value<own>()));
        } else {
            static_assert(!PinConfig::isRp2040(PinConfig::CurrentChip),
                          "the RP2040 starts machines in step on one PIO block only");
            apply(write(Ctrl::sm_enable, enabled | own),
                  write(Ctrl::clkdiv_restart, Register::value<own>()),
                  write(Ctrl::next_pio_mask, Register::value<nm>()),
                  write(Ctrl::prev_pio_mask, Register::value<pm>()),
                  write(Ctrl::nextprev_sm_enable, Register::value<1>()),
                  write(Ctrl::nextprev_clkdiv_restart, Register::value<1>()));
        }
    }

}}   // namespace Kvasir::Pio
