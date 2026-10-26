#pragma once
#include "Clocks.hpp"
#include "Io.hpp"
#include "PIO.hpp"
#include "WaitBounds.hpp"
#include "kvasir/Mpl/List.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Config.hpp"
#include "peripherals/RESETS.hpp"
#include "pio/Asm.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <peripherals/PIO.hpp>
#include <utility>

// A PIO state machine running one program, as a Startup-list peripheral (the generic form
// of ws2812.hpp): the Startup list loads the program, sets divider, wrap, shift registers
// and pin mapping, gives the pins the PIO function and their directions, and starts the
// machine. The application talks to it through the FIFOs.
//
//   struct BlinkProgram : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
//       a.set(Kvasir::Pio::Set::pins, 1);     // the program, built by the compiler
//       a.set(Kvasir::Pio::Set::pins, 0);     // (pio/Asm.hpp)
//   })> {};
//
//   struct BlinkSmConfig {
//       static constexpr auto clockSpeed  = HW::ClockSpeed;         // clk_sys (the divider's base)
//       static constexpr auto pioInstance = 0;                      // PIO0..2
//       static constexpr auto smInstance  = 0;                      // 0..3
//       static constexpr auto setPins     = brigand::list<HW::Pin::led>{};   // `set pins`
//   };
//   using BlinkSm = Kvasir::Pio::StateMachine<BlinkProgram, BlinkSmConfig>;
//
// Optional config, with defaults:
//   programOffset (0), clockDiv (1.0; 16.8, the machine runs at clockSpeed / clockDiv)
//   gpioBase      (derived)    the instance's GPIO window, 0 or 16 (PIO.hpp): a machine
//                              names 32 pins counted from it, so the RP2350B's GPIO 32..47
//                              need 16. Derived from this machine's pins; name it only to
//                              agree with another driver on the same PIO instance
//   setPins, outPins, sidesetPins, inPins (empty brigand::lists): consecutive GPIOs, the
//                              first is the mapping's base; set/out/side-set pins become
//                              outputs, in pins inputs, all get this PIO's function
//   sidesetOptional, sidesetPindirs   `.side_set N opt` (the enable bit is part of the count)
//                              and `.side_set N pindirs` (side-set drives directions): taken
//                              from the program's .side_set (every program carries it:
//                              Asm.hpp's Program). Named here they must agree with it
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

        // The n-th set bit of Mask (n from 0).
        template<std::uint32_t Mask>
        consteval unsigned nthSetBit(std::size_t n) {
            std::uint32_t m = Mask;
            for(; n != 0; --n) { m &= m - 1U; }
            return static_cast<unsigned>(std::countr_zero(m));
        }

        template<std::uint32_t Mask,
                 typename F,
                 std::size_t... I>
        consteval auto setEachBit(std::index_sequence<I...>) {
            return brigand::list<decltype(Register::set(
              Register::bit<nthSetBit<Mask>(I)>(F{})))...>{};
        }

        // set() of each bit of Mask in field F, one bit<n> field each: under Register::atomic the actions of one
        // register merge into one store to its SET alias (pico-sdk's hw_set_bits). A machine's bit of PIO CTRL's
        // SM_ENABLE / SM_RESTART / CLKDIV_RESTART is shared with three other machines and the other core.
        template<std::uint32_t Mask,
                 typename F>
        consteval auto setBits(F) {
            static_assert(Mask != 0, "setBits: no bit");
            return setEachBit<Mask, F>(
              std::make_index_sequence<static_cast<std::size_t>(std::popcount(Mask))>{});
        }

        // What a StateMachine's program must say (Asm.hpp's Program does): its directives - .fifo,
        // .in, .out, .set, .mov_status (-1 = not given, and FifoMode txrx, the default, is taken
        // as not given), its .origin (-1 = none) - and its .side_set (ProgramWithSideset below).
        template<typename P>
        concept ProgramWithDirectives = requires {
            P::fifoMode;
            P::inPinCount;
            P::inRight;
            P::inAutoP;
            P::inThreshold;
            P::outPinCount;
            P::outRight;
            P::outAutoP;
            P::outThreshold;
            P::setCount;
            P::movStatusType;
            P::movStatusN;
            P::origin;
        };

        // What the program says for each config member, or the member's own default where the program does
        // not give the directive (.out/.in/.mov_status count -1, .fifo txrx). A config member overrides it
        // (and must agree with it where the program does say: directivesAgree).
        template<typename P>
        consteval bool programSaysOut() {
            return P::outPinCount >= 0;
        }

        template<typename P>
        consteval bool programSaysIn() {
            return P::inPinCount >= 0;
        }

        template<typename P>
        consteval bool programSaysFifo() {
            return P::fifoMode != 0;
        }

        template<typename P>
        consteval bool programSaysMovStatus() {
            return P::movStatusType >= 0;
        }

        template<typename P>
        consteval bool programAutopull() {
            return programSaysOut<P>() && P::outAutoP;
        }

        template<typename P>
        consteval unsigned programPullThreshold() {
            return programSaysOut<P>() ? static_cast<unsigned>(P::outThreshold) : 32U;
        }

        template<typename P>
        consteval bool programOutShiftRight() {
            return !programSaysOut<P>() || P::outRight;
        }

        template<typename P>
        consteval bool programAutopush() {
            return programSaysIn<P>() && P::inAutoP;
        }

        template<typename P>
        consteval unsigned programPushThreshold() {
            return programSaysIn<P>() ? static_cast<unsigned>(P::inThreshold) : 32U;
        }

        template<typename P>
        consteval bool programInShiftRight() {
            return !programSaysIn<P>() || P::inRight;
        }

        // .fifo: 1 tx, 2 rx, 3 txget, 4 txput, 5 putget (pioasm's pio_enums.h; 0 txrx = not given)
        template<typename P>
        consteval bool programJoinTx() {
            return P::fifoMode == 1;
        }

        template<typename P>
        consteval bool programJoinRx() {
            return P::fifoMode == 2;
        }

        template<typename P>
        consteval bool programRxFifoPut() {
            return P::fifoMode == 4 || P::fifoMode == 5;
        }

        template<typename P>
        consteval bool programRxFifoGet() {
            return P::fifoMode == 3 || P::fifoMode == 5;
        }

        template<typename P>
        consteval MovStatus programMovStatus() {
            return programSaysMovStatus<P>() ? static_cast<MovStatus>(P::movStatusType + 1)
                                             : MovStatus::none;
        }

        template<typename P>
        consteval unsigned programMovStatusN() {
            return programSaysMovStatus<P>() ? static_cast<unsigned>(P::movStatusN) : 0U;
        }

        template<typename P>
        concept ProgramWithSideset = requires {
            P::sidesetCount;
            P::sidesetOptional;
            P::sidesetPindirs;
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
            using type = std::
              conditional_t<Kvasir::MPL::contains<T, Other>, rest, brigand::push_front<rest, T>>;
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
            using next
              = std::conditional_t<Kvasir::MPL::contains<T, Out>, Out, brigand::push_back<Out, T>>;
            using type = typename Unique<brigand::list<Ts...>, next>::type;
        };
    }   // namespace detail

    template<typename Program, typename Config_>
        requires detail::ProgramWithDirectives<Program> && detail::ProgramWithSideset<Program>
    struct StateMachine {
        struct Config : Config_ {
            static constexpr auto   programOffset = KVASIR_CONFIG_OR(Config_, programOffset, 0);
            static constexpr double clockDiv
              = static_cast<double>(KVASIR_CONFIG_OR(Config_, clockDiv, 1.0));
            static constexpr auto setPins = KVASIR_CONFIG_OR(Config_, setPins, brigand::list<>{});
            static constexpr auto outPins = KVASIR_CONFIG_OR(Config_, outPins, brigand::list<>{});
            static constexpr auto sidesetPins
              = KVASIR_CONFIG_OR(Config_, sidesetPins, brigand::list<>{});
            static constexpr auto inPins = KVASIR_CONFIG_OR(Config_, inPins, brigand::list<>{});
            static constexpr bool sidesetOptional
              = KVASIR_CONFIG_OR(Config_, sidesetOptional, Program::sidesetOptional);
            static constexpr bool sidesetPindirs
              = KVASIR_CONFIG_OR(Config_, sidesetPindirs, Program::sidesetPindirs);
            static constexpr bool autopull
              = KVASIR_CONFIG_OR(Config_, autopull, detail::programAutopull<Program>());
            static constexpr unsigned pullThreshold = static_cast<unsigned>(
              KVASIR_CONFIG_OR(Config_, pullThreshold, detail::programPullThreshold<Program>()));
            static constexpr bool outShiftRight
              = KVASIR_CONFIG_OR(Config_, outShiftRight, detail::programOutShiftRight<Program>());
            static constexpr bool autopush
              = KVASIR_CONFIG_OR(Config_, autopush, detail::programAutopush<Program>());
            static constexpr unsigned pushThreshold = static_cast<unsigned>(
              KVASIR_CONFIG_OR(Config_, pushThreshold, detail::programPushThreshold<Program>()));
            static constexpr bool inShiftRight
              = KVASIR_CONFIG_OR(Config_, inShiftRight, detail::programInShiftRight<Program>());
            static constexpr bool joinTx
              = KVASIR_CONFIG_OR(Config_, joinTx, detail::programJoinTx<Program>());
            static constexpr bool joinRx
              = KVASIR_CONFIG_OR(Config_, joinRx, detail::programJoinRx<Program>());
            static constexpr auto jmpPin   = KVASIR_CONFIG_OR(Config_, jmpPin, brigand::list<>{});
            static constexpr bool inPullUp = KVASIR_CONFIG_OR(Config_, inPullUp, false);
            static constexpr auto pinsInitialHigh
              = KVASIR_CONFIG_OR(Config_, pinsInitialHigh, brigand::list<>{});
            static constexpr bool      outSticky = KVASIR_CONFIG_OR(Config_, outSticky, false);
            static constexpr MovStatus movStatus
              = KVASIR_CONFIG_OR(Config_, movStatus, detail::programMovStatus<Program>());
            // .fifo txput / txget / putget (RP2350): the RX FIFO as random-access registers
            static constexpr bool rxFifoPut
              = KVASIR_CONFIG_OR(Config_, rxFifoPut, detail::programRxFifoPut<Program>());
            static constexpr bool rxFifoGet
              = KVASIR_CONFIG_OR(Config_, rxFifoGet, detail::programRxFifoGet<Program>());
            // pins: pulls beyond inPullUp, GPIO overrides, the input synchroniser
            static constexpr auto pullUpPins
              = KVASIR_CONFIG_OR(Config_, pullUpPins, brigand::list<>{});
            static constexpr auto pullDownPins
              = KVASIR_CONFIG_OR(Config_, pullDownPins, brigand::list<>{});
            static constexpr auto outputInverted
              = KVASIR_CONFIG_OR(Config_, outputInverted, brigand::list<>{});
            static constexpr auto oeInverted
              = KVASIR_CONFIG_OR(Config_, oeInverted, brigand::list<>{});
            static constexpr auto inputSyncBypass
              = KVASIR_CONFIG_OR(Config_, inputSyncBypass, brigand::list<>{});
            static constexpr auto foreignPads
              = KVASIR_CONFIG_OR(Config_, foreignPads, brigand::list<>{});
            // what the plain pin function writes
            static constexpr auto driveStrength
              = KVASIR_CONFIG_OR(Config_, driveStrength, Kvasir::Io::DriveStrength::mA4);
            static constexpr bool slewFast = KVASIR_CONFIG_OR(Config_, slewFast, false);
            // false: the Startup list loads and configures the machine but leaves it stopped, for
            // a driver that starts it itself at an entry point (PioQspi)
            static constexpr bool     startEnabled = KVASIR_CONFIG_OR(Config_, startEnabled, true);
            static constexpr unsigned movStatusN   = static_cast<unsigned>(
              KVASIR_CONFIG_OR(Config_, movStatusN, detail::programMovStatusN<Program>()));
        };

        static constexpr unsigned instance = Config::pioInstance;
        static constexpr unsigned sm       = Config::smInstance;
        static constexpr unsigned offset   = Config::programOffset;

        // The program's other directives (.fifo, .in, .out, .set, .mov_status) against what the
        // config names: a config value the program contradicts is the same misread machine as
        // a wrong side-set - an autopull the program does not expect, a shift the wrong way.
        // Config's member is the config's own where it names one, else the program's: so it equals the
        // program's exactly when the config agrees or is silent. A directive the program does not give
        // (.out/.in not stated, .fifo txrx, no .mov_status) leaves the config free.
        static constexpr bool directivesAgree() {
            using P             = Program;
            using C             = Config;
            constexpr bool out  = detail::programSaysOut<P>();
            constexpr bool in   = detail::programSaysIn<P>();
            constexpr bool fifo = detail::programSaysFifo<P>();
            constexpr bool mov  = detail::programSaysMovStatus<P>();
            return (!out
                    || (C::autopull == detail::programAutopull<P>()
                        && C::pullThreshold == detail::programPullThreshold<P>()
                        && C::outShiftRight == detail::programOutShiftRight<P>()))
                && (!in
                    || (C::autopush == detail::programAutopush<P>()
                        && C::pushThreshold == detail::programPushThreshold<P>()
                        && C::inShiftRight == detail::programInShiftRight<P>()))
                && (!fifo
                    || (C::rxFifoPut == detail::programRxFifoPut<P>()
                        && C::rxFifoGet == detail::programRxFifoGet<P>()
                        && C::joinTx == detail::programJoinTx<P>()
                        && C::joinRx == detail::programJoinRx<P>()))
                && (!mov
                    || (C::movStatus == detail::programMovStatus<P>()
                        && C::movStatusN == detail::programMovStatusN<P>()));
        }

        static_assert(directivesAgree(),
                      "the config's autopull/autopush, thresholds, shift directions, FIFO join or "
                      "movStatus disagree with the program's .out/.in/.fifo/.mov_status");
        static_assert(!(Config::rxFifoPut || Config::rxFifoGet) || !PinConfig::isRp2040(Chip::part),
                      "RX FIFO random access (.fifo txput / txget / putget) is an RP2350 feature");
        static_assert(!(Config::rxFifoPut || Config::rxFifoGet) || !Config::joinRx,
                      "the RX FIFO is either joined or random-access registers, not both");

        static_assert(instance < PinConfig::pioCount(Chip::part),
                      "the RP2350 has PIO0..PIO2, the RP2040 PIO0 and PIO1");
        static_assert(sm < 4,
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
        static constexpr bool sidesetAgrees = Config::sidesetOptional == Program::sidesetOptional
                                           && Config::sidesetPindirs == Program::sidesetPindirs;
        static_assert(sidesetAgrees,
                      "sidesetOptional / sidesetPindirs disagree with the program's .side_set");
        static constexpr bool sidesetPinsAgree = SidePins::count == Program::sidesetCount;
        static_assert(sidesetPinsAgree,
                      "sidesetPins must be as many pins as the program's .side_set count");

        // `.out N` / `.in N` / `.set N` are the program's pin counts: the mappings must match
        static constexpr bool pinCountsAgree
          = (Program::outPinCount < 0 || OutPins::count == Program::outPinCount)
         && (Program::inPinCount < 0 || Program::inPinCount == 32
             || InPins::count == Program::inPinCount)
         && (Program::setCount < 0 || SetPins::count == Program::setCount);
        static_assert(
          pinCountsAgree,
          "outPins / inPins / setPins must be as many pins as the program's .out / .in / .set");

        static constexpr bool originAgrees
          = Program::origin < 0 || static_cast<unsigned>(Program::origin) == Config::programOffset;
        static_assert(originAgrees,
                      "the program has .origin: programOffset must be that slot");

        // Decided by the words (Asm.hpp isPioV1Only), not by the version the program declares
        static_assert(!PinConfig::isRp2040(Chip::part)
                        || !Kvasir::Pio::usesPioV1(Program::instructions),
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
        static constexpr unsigned gpioBase = static_cast<unsigned>(
          KVASIR_CONFIG_OR(Config,
                           gpioBase,
                           Kvasir::Pio::gpioBaseFor(detail::PinNumbers<AllPins>::value)));

        static_assert(Kvasir::Pio::pinsInWindow(detail::PinNumbers<AllPins>::value,
                                                gpioBase),
                      "a pin of this state machine is not reachable from the PIO instance's "
                      "GPIO window: a machine sees 32 pins from gpioBase (0 or 16), so one "
                      "below 16 and one above 31 cannot be driven by the same instance");

        using PioRegs = Kvasir::Peripheral::PIO::Registers<instance>;
        using SmRegs  = typename PioRegs::template SM<sm>;

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

        using Fifo = typename PioRegs::template FIFO<sm>;

        /// The shift settings the config (and the program) gave.
        static constexpr Shift configuredShift{Config::autopull,
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
        static constexpr unsigned inCount = [] {
            if constexpr(detail::programSaysIn<Program>()) {
                return static_cast<unsigned>(Program::inPinCount) % 32U;
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
                  write(SmRegs::SHIFTCTRL::in_count, Register::value<inCount>()),
                  write(SmRegs::SHIFTCTRL::fjoin_rx_put, Register::value<S.rxPut ? 1 : 0>()),
                  write(SmRegs::SHIFTCTRL::fjoin_rx_get, Register::value<S.rxGet ? 1 : 0>()));
            } else {
                static_assert(!(S.rxPut || S.rxGet), "RX FIFO random access is an RP2350 feature");
                return common();
            }
        }

        static constexpr std::uint32_t smMask = 1U << sm;

        // Startup: the state machine, the instruction slots (tagged with the program, so two
        // machines on one program share them), the instance's GPIO window, the pins, the
        // clock.
        using Provides = Kvasir::Pio::Provides<instance, sm, offset, Program, gpioBase>;
        // foreign pads are claimed: their owner has to be in the list and configure them
        using Claims = brigand::append<Hal::Claim<Clocks::ClkSys, Config::clockSpeed>,
                                       brigand::wrap<ForeignPads, Kvasir::Io::PinClaims>>;

        static constexpr auto powerClockEnable
          = list(clear(Rp::Block<Kvasir::Peripheral::PIO::Registers<instance>>::reset));

        template<typename List, typename Pin>
        static constexpr bool listed = Kvasir::MPL::contains<Pin, List>;

        template<typename C,
                 typename Pin,
                 bool Input>
        static constexpr Kvasir::Io::PullConfiguration pullOf() {
            // NOLINTNEXTLINE(bugprone-branch-clone) // the order decides: a listed pull-down beats inPullUp
            if constexpr(listed<std::remove_cvref_t<decltype(C::pullUpPins)>, Pin>) {
                return Kvasir::Io::PullConfiguration::pullUp;
            } else if constexpr(listed<std::remove_cvref_t<decltype(C::pullDownPins)>, Pin>) {
                return Kvasir::Io::PullConfiguration::pullDown;
            } else if constexpr(Input && C::inPullUp) {
                return Kvasir::Io::PullConfiguration::pullUp;
            } else {
                return Kvasir::Io::PullConfiguration::pullNone;
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
                    Kvasir::Pio::pinFunction<instance>,
                    Config::driveStrength,
                    Config::slewFast,
                    pullOf<Config, Pins, Input>(),
                    Kvasir::Io::OutputInit::low,
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
        static constexpr std::uint32_t syncBypassMaskOf(brigand::list<Pins...>) {
            return (0U | ...
                    | (1U << Kvasir::Pio::pinIndex(static_cast<unsigned>(detail::pinNumber(Pins{})),
                                                   gpioBase)));
        }

        static constexpr std::uint32_t syncBypassMask
          = syncBypassMaskOf(std::remove_cvref_t<decltype(Config::inputSyncBypass)>{});

        static constexpr auto initStepPeripheryConfig = list(
          Kvasir::Pio::getDivConfig<SmRegs>([]() { return Config::clockDiv; }),
          SmRegs::EXECCTRL::overrideDefaults(
            write(SmRegs::EXECCTRL::wrap_bottom, Register::value<Program::wrapTarget + offset>()),
            write(SmRegs::EXECCTRL::wrap_top, Register::value<Program::wrap + offset>()),
            write(SmRegs::EXECCTRL::side_en, Register::value<Config::sidesetOptional ? 1 : 0>()),
            write(SmRegs::EXECCTRL::side_pindir, Register::value<Config::sidesetPindirs ? 1 : 0>()),
            write(SmRegs::EXECCTRL::jmp_pin,
                  Register::value<detail::mappedBase<JmpPin>(gpioBase)>()),
            write(SmRegs::EXECCTRL::out_sticky, Register::value<Config::outSticky ? 1 : 0>()),
            write(SmRegs::EXECCTRL::status_sel,
                  Register::value<typename SmRegs::EXECCTRL::STATUS_SELVal,
                                  static_cast<typename SmRegs::EXECCTRL::STATUS_SELVal>(
                                    Config::movStatus == MovStatus::rxFifoLessThan ? 1
                                    : Config::movStatus == MovStatus::irqSet       ? 2
                                                                                   : 0)>()),
            write(SmRegs::EXECCTRL::status_n, statusN())),
          shiftCtrl<configuredShift>());

    private:
        // `set pindirs, v` / `set pins, v` for one pin: point the set mapping at it, execute
        // the instruction (SET destination 4 = PINDIRS, 0 = PINS); bit 0 of the data is the
        // direction or the level.
        static_assert(Kvasir::Pio::Enc::set(4,
                                            1)
                        == 0xE081U
                      && Kvasir::Pio::Enc::set(0,
                                               0)
                           == 0xE000U);

        template<typename Pin>
        static void setPinDir(Pin,
                              bool out) {
            constexpr auto n
              = Kvasir::Pio::pinIndex(static_cast<unsigned>(detail::pinNumber(Pin{})), gpioBase);
            apply(SmRegs::PINCTRL::overrideDefaults(
              write(SmRegs::PINCTRL::set_base, Register::value<n>()),
              write(SmRegs::PINCTRL::set_count, Register::value<1>())));
            apply(
              write(SmRegs::INSTR::instr, std::uint32_t{Kvasir::Pio::Enc::set(4, out ? 1U : 0U)}));
        }

        template<typename Pin>
        static void setPinLevel(Pin,
                                bool high) {
            constexpr auto n
              = Kvasir::Pio::pinIndex(static_cast<unsigned>(detail::pinNumber(Pin{})), gpioBase);
            apply(SmRegs::PINCTRL::overrideDefaults(
              write(SmRegs::PINCTRL::set_base, Register::value<n>()),
              write(SmRegs::PINCTRL::set_count, Register::value<1>())));
            apply(
              write(SmRegs::INSTR::instr, std::uint32_t{Kvasir::Pio::Enc::set(0, high ? 1U : 0U)}));
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
        static constexpr auto relocatedInstructions = [] {
            auto instructions = Program::instructions;
            for(auto& i : instructions) {
                if((i >> 13U) == 0U) {
                    auto const target = static_cast<std::uint16_t>(((i & 0x1FU) + offset) & 0x1FU);
                    i = static_cast<std::uint16_t>((i & static_cast<std::uint16_t>(~0x1FU))
                                                   | target);
                }
            }
            return instructions;
        }();

        static_assert(offset + Program::instructions.size() <= 32,
                      "the program does not fit in instruction memory at this programOffset: a "
                      "PIO instance has 32 slots and a JMP target is five bits");

        // The reset release (powerClockEnable) is not immediate: the instance is usable once
        // RESET_DONE says so. PioQspi's CYW43 predecessor failed intermittently without the wait.
        static bool resetDone() {
            return get<0>(apply(
                     read(Rp::Block<Kvasir::Peripheral::PIO::Registers<instance>>::resetDone)))
                != 0;
        }

        static void preEnableRuntimeInit() {
            Kvasir::Register::waitUntil<Kvasir::Chip::ResetDoneBound>([] { return resetDone(); });
            if constexpr(syncBypassMask != 0) {
                // the register is the instance's, shared with the other machines (and their cores): our bits
                // through the SET alias, one store (the RP2040's SVD has no field for it, only FULLREGISTER)
                apply(Register::atomic(
                  detail::setBits<syncBypassMask>(PioRegs::INPUT_SYNC_BYPASS::FULLREGISTER)));
            }
            // Before any mapping is written and while no machine on the instance runs: every
            // base below counts from this window.
            Kvasir::Pio::applyGpioBase<instance, gpioBase>();

            // one 32-bit register per instruction slot, the instruction in bits 15:0
            using InstrMem = Kvasir::Register::Array<PioRegs::template INSTR_MEM, 32>;
            for(unsigned i = 0; i < relocatedInstructions.size(); ++i) {
                InstrMem::writeWhole(offset + i, relocatedInstructions[i]);
            }

            setPinLevels(std::remove_cvref_t<decltype(Config::pinsInitialHigh)>{}, true);
            setPinDirs(OutputPins{}, true);
            setPinDirs(OwnInputPins{}, false);

            apply(SmRegs::PINCTRL::overrideDefaults(
              write(SmRegs::PINCTRL::set_base,
                    Register::value<detail::mappedBase<SetPins>(gpioBase)>()),
              write(SmRegs::PINCTRL::set_count,
                    Register::value<static_cast<unsigned>(SetPins::count)>()),
              write(SmRegs::PINCTRL::out_base,
                    Register::value<detail::mappedBase<OutPins>(gpioBase)>()),
              write(SmRegs::PINCTRL::out_count,
                    Register::value<static_cast<unsigned>(OutPins::count)>()),
              write(SmRegs::PINCTRL::sideset_base,
                    Register::value<detail::mappedBase<SidePins>(gpioBase)>()),
              write(SmRegs::PINCTRL::sideset_count,
                    Register::value<static_cast<unsigned>(SidePins::count)
                                    + (Config::sidesetOptional ? 1U : 0U)>()),
              write(SmRegs::PINCTRL::in_base,
                    Register::value<detail::mappedBase<InPins>(gpioBase)>())));
        }

        // Jump to the program's first instruction. The restart bits and sm_enable live in
        // the instance's CTRL register, shared with the other machines, so they are written
        // at run time in runtimeInit() (after every peripheral's enable step, see
        // ws2812.hpp): four machines' literal writes to one field would otherwise be one
        // step with four different values, which Startup refuses.
        static constexpr auto initStepPeripheryEnable
          = list(write(SmRegs::INSTR::instr, Register::value<offset>()));

        // This machine's bit of the instance's CTRL fields. CTRL is shared by four machines (and both cores), so every
        // write of it is Register::atomic: one store to its SET or CLR alias, no read-modify-write that could lose
        // another machine's update (RP2040 datasheet 2.1.2, md l.830; RP2350 2.1.3, md l.1225-1244 - PIO is not behind
        // the interposer, the block applies the alias itself). SM_RESTART / CLKDIV_RESTART are self-clearing
        // triggers, oneToSet in the SVDs (RP2350 CTRL, md l.46557-46572): a SET-alias store of the machine's bit is
        // exactly "write 1" to it.
        static constexpr auto smEnable      = Register::bit<sm>(PioRegs::CTRL::sm_enable);
        static constexpr auto smRestart     = Register::bit<sm>(PioRegs::CTRL::sm_restart);
        static constexpr auto clkdivRestart = Register::bit<sm>(PioRegs::CTRL::clkdiv_restart);

        static void runtimeInit() {
            apply(Register::atomic(set(smRestart)));
            apply(Register::atomic(set(clkdivRestart)));
            apply(write(SmRegs::INSTR::instr, Register::value<offset>()));
            if constexpr(Config::startEnabled) { setEnabled(true); }
        }

        // -- the run-time interface --------------------------------------------------

        static void setEnabled(bool on) {
            if(on) {
                apply(Register::atomic(set(smEnable)));
            } else {
                apply(Register::atomic(clear(smEnable)));
            }
        }

        /// Restart: registers cleared, program counter at the program's start.
        static void restart() {
            apply(Register::atomic(set(smRestart)));
            apply(write(SmRegs::INSTR::instr, Register::value<offset>()));
        }

        /// Restart the machine's state and nothing else (pico-sdk's pio_sm_restart and
        /// pio_sm_clkdiv_restart): SM_RESTART and CLKDIV_RESTART of this machine in one store to
        /// CTRL's SET alias. SM_RESTART clears the shift counters, the ISR, the delay counter, a
        /// wait on an IRQ and a stalled INSTR/OUT EXEC instruction; the OSR and X/Y stay, and the
        /// program counter too (CTRL, RP2040 datasheet md l.17316 says so; the RP2350's row, md
        /// l.46572, leaves the PC out of both lists).
        /// No jmp follows, unlike restart() and restartAt(): with autopull on, a forced jmp on a
        /// running machine pulls from the TX FIFO once the OSR is exhausted, which SM_RESTART
        /// makes it.
        static void restartState() {
            apply(Register::atomic(set(smRestart), set(clkdivRestart)));   // one store
        }

        /// Restart at one of the program's entry points (an offset in the program, as
        /// Program::offset("label") gives it): registers cleared, the clock divider's phase
        /// too, then a jmp there. The machine keeps its enable; stop it first to start clean.
        static void restartAt(unsigned entry) {
            restartState();
            exec(static_cast<std::uint16_t>((entry + offset) & 0x1FU));   // JMP (always) to it
        }

        /// Another wrap at run time (pico-sdk's pio_sm_set_wrap): after the instruction at
        /// program offset Top the machine continues at Bottom (EXECCTRL WRAP_TOP / WRAP_BOTTOM,
        /// RP2040 datasheet 3.5.2 md l.16367 and l.17586/17598, RP2350 11.5.2 md l.45555 and
        /// l.46850/46862), both relative to the program like Program::offset("label") and
        /// relocated by its offset here. The program's .wrap_target / .wrap are what the
        /// Startup sets up. EXECCTRL is this machine's own register.
        template<unsigned Bottom,
                 unsigned Top>
        static void setWrap() {
            static_assert(Bottom < Program::instructions.size()
                            && Top < Program::instructions.size(),
                          "setWrap<Bottom, Top>: both are offsets inside the program");
            apply(write(SmRegs::EXECCTRL::wrap_bottom, Register::value<Bottom + offset>()),
                  write(SmRegs::EXECCTRL::wrap_top, Register::value<Top + offset>()));
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
            return get<0>(apply(read(Register::bit<sm>(PioRegs::FSTAT::txfull)))) != 0;
        }

        [[nodiscard]] static bool txEmpty() {
            return get<0>(apply(read(Register::bit<sm>(PioRegs::FSTAT::txempty)))) != 0;
        }

        [[nodiscard]] static bool rxEmpty() {
            return get<0>(apply(read(Register::bit<sm>(PioRegs::FSTAT::rxempty)))) != 0;
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

        /// exec() and wait until the instruction has completed (a `wait`, a blocking `pull` stalls it:
        /// EXECCTRL.EXEC_STALLED, RP2040 datasheet 3.5.7 / RP2350 11.5.7; pio_sm_exec_wait_blocking).
        /// Bounded by Bound under the image's wait policy (Register/Wait.hpp) or the Policy given. A wait
        /// that ends before the instruction did writes a NOP to INSTR, which drops the latched instruction
        /// (same sections), so the machine runs its program again. Not with OUT EXEC / MOV EXEC programs: a
        /// stalled INSTR, and that NOP, overwrite their latch (RP2350 11.5.7 CAUTION).
        template<typename Bound  = Kvasir::Chip::PioExecBound,
                 typename Policy = Kvasir::Register::ImagePolicy>
        static Kvasir::Register::WaitResult execWait(std::uint16_t instruction) {
            exec(instruction);
            auto const r = Kvasir::Register::waitUntil<Bound, Policy>(
              Kvasir::Register::isClear(SmRegs::EXECCTRL::exec_stalled));
            if(r.timedOut) { exec(Enc::nop); }
            return r;
        }

        /// The RX FIFO's entry I as a register (RP2350, .fifo txput or txget): with put the
        /// machine writes it and this reads it, with get this writes it and the machine reads.
        template<unsigned I>
        [[nodiscard]] static std::uint32_t rxEntry() {
            static_assert(I < 4, "the RX FIFO has four entries");
            static_assert(Config::rxFifoPut != Config::rxFifoGet,
                          "the processor reaches the entries with exactly one of put / get set");
            using E = typename PioRegs::template RXF_PUTGET<sm>::template ENTRY<I>;
            return get<0>(apply(read(E::FULLREGISTER)));
        }

        template<unsigned I>
        static void setRxEntry(std::uint32_t v) {
            static_assert(I < 4, "the RX FIFO has four entries");
            static_assert(Config::rxFifoPut != Config::rxFifoGet,
                          "the processor reaches the entries with exactly one of put / get set");
            using E = typename PioRegs::template RXF_PUTGET<sm>::template ENTRY<I>;
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
            return (n & 0x4U) | ((n + sm) & 0x3U);
        }

    private:
        // FLEVEL: TXn in bits 8n+3:8n, RXn in 8n+7:8n+4 (RP2040 datasheet Table 371, md l.17386; RP2350
        // datasheet Table 985, md l.46645)
        template<bool Rx>
        static unsigned levelField() {
            return get<0>(
              apply(read(Register::fieldAt<8U * sm + (Rx ? 4U : 0U)>(typename PioRegs::FLEVEL{}))));
        }

    public:
        /// The instruction this machine sits on (SMx_ADDR, an absolute instruction memory address):
        /// subtract the program offset for its index in the program. Diagnostic: a machine that
        /// stays on one address is waiting on a FIFO, a pin or an IRQ.
        [[nodiscard]] static std::uint32_t pc() {
            return static_cast<std::uint32_t>(get<0>(apply(read(SmRegs::ADDR::addr))));
        }

        /// An instruction written to INSTR is stalled and latched (EXECCTRL.EXEC_STALLED, RP2040
        /// datasheet 3.5.7 / RP2350 11.5.7): execWait() waits for this to read false.
        [[nodiscard]] static bool execStalled() {
            return get<0>(apply(read(SmRegs::EXECCTRL::exec_stalled))) != 0;
        }

        /// Which state machines of this instance are enabled (CTRL.SM_ENABLE, one bit each).
        [[nodiscard]] static std::uint32_t enabledMask() {
            return static_cast<std::uint32_t>(get<0>(apply(read(PioRegs::CTRL::sm_enable))));
        }

        /// This machine's PINCTRL as one word: a snapshot of its pin mapping for a log line.
        [[nodiscard]] static std::uint32_t pinCtrl() {
            return Kvasir::Register::readWhole(SmRegs::PINCTRL::FULLREGISTER);
        }

        /// The TX FIFO stalled (the program waited on an empty FIFO) since the last clear.
        [[nodiscard]] static bool txStalled() {
            return get<0>(apply(read(Register::bit<sm>(PioRegs::FDEBUG::txstall)))) != 0;
        }

        static void clearTxStall() {
            apply(Register::reset(Register::bit<sm>(PioRegs::FDEBUG::txstall)));   // W1C: one store
        }

        /// The address of the TX FIFO, and the DMA request that pairs with it, for a DMA-fed
        /// program (ws2812.hpp does exactly this).
        static constexpr std::uint32_t txFifoAddress = Fifo::TXF::Addr::value;
        static constexpr std::uint32_t rxFifoAddress = Fifo::RXF::Addr::value;

        // The names tied to the datasheet arithmetic (DREQ PIO<n>_TX<m> = 8n + m, PIO<n>_RX<m> =
        // 8n + 4 + m; RP2040 datasheet 2.5.3.1, RP2350 datasheet 12.6.4.1), so a wrong row
        // cannot come back.
        template<typename Dma>
        static constexpr typename Dma::TriggerSource txDmaTrigger() {
            constexpr typename Dma::TriggerSource trigger
              = Rp::Block<Kvasir::Peripheral::PIO::Registers<instance>>::template dreqTxOf<sm>();
            static_assert(static_cast<unsigned>(trigger) == 8 * instance + sm,
                          "PIO TX DREQ does not match the datasheet's 8n + m");
            return trigger;
        }

        template<typename Dma>
        static constexpr typename Dma::TriggerSource rxDmaTrigger() {
            constexpr typename Dma::TriggerSource trigger
              = Rp::Block<Kvasir::Peripheral::PIO::Registers<instance>>::template dreqRxOf<sm>();
            static_assert(static_cast<unsigned>(trigger) == 8 * instance + 4 + sm,
                          "PIO RX DREQ does not match the datasheet's 8n + 4 + m");
            return trigger;
        }
    };

    /// The Startup entry of one state machine, for a driver that owns it: the eight members Startup
    /// reads (kvasir/StartUp/StartUp.hpp), taken from `Sm`. A driver derives from it instead of
    /// forwarding them by hand, and redeclares the one it extends (a DMA claim, a pin, a disable
    /// after the start), which hides the base's. Nothing else of `Sm` becomes the driver's API.
    template<typename Sm>
    struct StartupOf {
        using Provides = typename Sm::Provides;
        using Claims   = typename Sm::Claims;

        static constexpr auto powerClockEnable        = Sm::powerClockEnable;
        static constexpr auto initStepPinConfig       = Sm::initStepPinConfig;
        static constexpr auto initStepPeripheryConfig = Sm::initStepPeripheryConfig;
        static constexpr auto initStepPeripheryEnable = Sm::initStepPeripheryEnable;

        static void preEnableRuntimeInit() { Sm::preEnableRuntimeInit(); }

        static void runtimeInit() { Sm::runtimeInit(); }
    };

    namespace detail {
        template<typename... Sms>
        constexpr std::uint32_t syncMask(unsigned instance) {
            return (0U | ... | (Sms::instance == instance ? Sms::smMask : 0U));
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
        constexpr unsigned      home  = First::instance;
        constexpr unsigned      count = PinConfig::pioCount(Chip::part);
        constexpr unsigned      next  = (home + 1) % count;
        constexpr unsigned      prev  = (home + count - 1) % count;
        constexpr std::uint32_t own   = detail::syncMask<Sms...>(home);
        constexpr std::uint32_t nm    = detail::syncMask<Sms...>(next);
        constexpr std::uint32_t pm    = detail::syncMask<Sms...>(prev);
        static_assert(
          ((Sms::instance == home || Sms::instance == next || Sms::instance == prev) && ...),
          "every machine is on the first one's PIO block or a neighbouring one");
        using Ctrl = typename Kvasir::Peripheral::PIO::Registers<home>::CTRL;
        // One store to CTRL's SET alias - pico-sdk 2.3.1 pio_enable_sm_mask_in_sync /
        // pio_enable_sm_multi_mask_in_sync (hardware/pio.h) do the same hw_set_bits: the enable bits of the
        // other machines stay as they are without a read, and the NEXT/PREV masks and NEXTPREV_* bits are
        // self-clearing (SC, RP2350 datasheet PIO CTRL, md l.46543-46558), so writing their 1s is all they need.
        if constexpr(nm == 0 && pm == 0) {
            apply(Register::atomic(detail::setBits<own>(Ctrl::sm_enable),
                                   detail::setBits<own>(Ctrl::clkdiv_restart)));
        } else {
            static_assert(!PinConfig::isRp2040(Chip::part),
                          "the RP2040 starts machines in step on one PIO block only");
            constexpr auto across = [] {
                if constexpr(nm != 0 && pm != 0) {
                    return brigand::append<decltype(detail::setBits<nm>(Ctrl::next_pio_mask)),
                                           decltype(detail::setBits<pm>(Ctrl::prev_pio_mask))>{};
                } else if constexpr(nm != 0) {
                    return detail::setBits<nm>(Ctrl::next_pio_mask);
                } else {
                    return detail::setBits<pm>(Ctrl::prev_pio_mask);
                }
            }();
            if constexpr(own != 0) {
                apply(Register::atomic(detail::setBits<own>(Ctrl::sm_enable),
                                       detail::setBits<own>(Ctrl::clkdiv_restart),
                                       across,
                                       set(Ctrl::nextprev_sm_enable),
                                       set(Ctrl::nextprev_clkdiv_restart)));
            } else {
                apply(Register::atomic(across,
                                       set(Ctrl::nextprev_sm_enable),
                                       set(Ctrl::nextprev_clkdiv_restart)));
            }
        }
    }

}}   // namespace Kvasir::Pio
