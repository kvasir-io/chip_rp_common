#pragma once
#include "Io.hpp"
#include "PinConfig.hpp"
#include "chip/Blocks.hpp"   // the chip's block table (Rp::Block)
#include "kvasir/StartUp/Resources.hpp"
#include "peripherals/RESETS.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <peripherals/PIO.hpp>
#include <utility>

namespace Kvasir { namespace Pio {
    // A PIO instance's state machines and instruction slots as Startup resources
    // (kvasir/StartUp/Resources.hpp). A driver that loads a program provides the state
    // machine it runs it on and every instruction slot the program occupies, each slot
    // tagged with the program's identity: two drivers on one state machine are a build
    // error, and so are two *different* programs overlapping in one instance's memory (the
    // second silently overwriting the first), while two state machines running the *same*
    // program at the same offset share the slots (mergeIdentical: same key, same payload),
    // which is how a program is shared. The state machine is driven from one core;
    // instruction memory is chip-wide.
    struct SmTag {};

    struct InstructionTag {
        static constexpr bool coreLocal      = false;
        static constexpr bool mergeIdentical = true;
    };

    template<unsigned Instance, unsigned Sm>
    using SmResource = Startup::Resource<SmTag, Instance * 4 + Sm>;

    template<unsigned Instance, unsigned Slot, unsigned long long ProgramId>
    using InstructionResource = Startup::Resource<InstructionTag, Instance * 32 + Slot, ProgramId>;

    // ---- the instance's GPIO window (GPIOBASE) -------------------------------------------
    // A state machine names a pin in five bits - PINCTRL's four bases and EXECCTRL's jmp_pin
    // - so it reaches 32 GPIOs, and GPIOBASE says which 32: it relocates the instance's GPIO
    // 0 in the system numbering. Only 0 and 16 are supported, so the RP2350B's GPIO 32..47
    // are reachable only from window 16, and a pin below 16 is not reachable from it at all.
    //
    // Without this the pin number was written into the field as it stood and a GPIO above 31
    // silently lost its top bits, pointing the machine at a pad 32 lower. Register::write now
    // refuses the literal, and a driver maps every pin through `pinIndex` instead.
    //
    // GPIOBASE belongs to the whole instance - all four machines, every mapping - so a driver
    // *provides* the window it needs (GpioBaseResource, appended by `Provides` below). Two
    // drivers that share an instance and disagree are then a build error rather than one of
    // them driving the wrong pad. A driver whose pins all sit in 16..31 fits either window
    // and derives 0; sharing an instance with a driver that needs 16 is what
    // `static constexpr unsigned gpioBase = 16;` in its config is for.
    struct GpioBaseTag {
        static constexpr bool coreLocal      = false;
        static constexpr bool mergeIdentical = true;
    };

    template<unsigned Instance, unsigned Base>
    using GpioBaseResource = Startup::Resource<GpioBaseTag, Instance, Base>;

    /// The window a set of GPIOs needs: 16 as soon as one of them is above 31, else 0.
    template<std::size_t N>
    constexpr unsigned gpioBaseFor(std::array<unsigned,
                                              N> const& pins) {
        for(auto const p : pins) {
            if(p >= 32) { return 16U; }
        }
        return 0U;
    }

    /// Whether every one of `pins` is reachable from `base`.
    template<std::size_t N>
    constexpr bool pinsInWindow(std::array<unsigned,
                                           N> const& pins,
                                unsigned             base) {
        for(auto const p : pins) {
            if(p < base || p - base >= 32) { return false; }
        }
        return true;
    }

    /// A GPIO as the state machine names it.
    constexpr unsigned pinIndex(unsigned pin,
                                unsigned base) {
        return pin - base;
    }

    /// Program the instance's window. Called from a driver's `preEnableRuntimeInit`, which is
    /// after `initStepPeripheryConfig` and before `initStepPeripheryEnable` (StartUp.hpp), so
    /// no machine on the instance is running yet - that, not the write order, is what makes
    /// this safe. EXECCTRL's jmp_pin is in fact written one phase *earlier*, and PINCTRL's
    /// bases in this one after this call: a base is a latched number the machine reads only
    /// while it executes, so when it is written relative to this does not matter, only that
    /// nothing has executed in between. Every driver on the instance writes the same value
    /// here, which the resource above is what guarantees.
    // Not `static`: internal linkage would let clang prove the definition dead in every
    // translation unit that includes this header without a PIO driver in it, which is
    // -Wunused-template on almost every one of them (`gpioBaseFor` and `pinsInWindow` above
    // are non-static for the same reason).
    template<unsigned Instance,
             unsigned Base>
    void applyGpioBase() {
        static_assert(Base == 0 || Base == 16,
                      "GPIOBASE is 0 or 16: only bit 4 of the register is writable");
        using Regs = Kvasir::Peripheral::PIO::Registers<Instance>;
        if constexpr(requires { Regs::GPIOBASE::gpiobase; }) {
            apply(write(Regs::GPIOBASE::gpiobase, Kvasir::Register::value<Base / 16U>()));
        } else {
            static_assert(Base == 0,
                          "this chip's PIO has no GPIOBASE: a state machine reaches GPIO "
                          "0..31 only");
        }
    }

    // A program's identity for the resource check: FNV-1a over its instructions and wrap
    // points. Two programs with the same words at the same offset are the same program.
    template<typename Program>
    constexpr unsigned long long programId() {
        unsigned long long h   = 0xcbf29ce484222325ULL;
        auto const         mix = [&](unsigned long long v) {
            h ^= v;
            h *= 0x100000001b3ULL;
        };
        for(auto const w : Program::instructions) { mix(w); }
        mix(static_cast<unsigned long long>(Program::wrapTarget));
        mix(static_cast<unsigned long long>(Program::wrap));
        return h;
    }

    namespace Detail {
        template<unsigned Instance, unsigned Offset, unsigned long long Id, typename Seq>
        struct Instructions;

        template<unsigned Instance, unsigned Offset, unsigned long long Id, std::size_t... Is>
        struct Instructions<Instance, Offset, Id, std::index_sequence<Is...>> {
            using type = brigand::list<
              InstructionResource<Instance, Offset + static_cast<unsigned>(Is), Id>...>;
        };
    }   // namespace Detail

    template<unsigned Instance,
             unsigned Sm,
             unsigned Offset,
             typename Program,
             unsigned GpioBase = 0>
    struct ProvidesFor {
        static constexpr std::size_t length = Program::instructions.size();
        static_assert(Instance < PinConfig::pioCount(Chip::part),
                      "the RP2350 has PIO0..PIO2, the RP2040 PIO0 and PIO1");
        static_assert(Sm < 4,
                      "a PIO instance has four state machines");
        static_assert(Offset + length <= 32,
                      "a PIO program must fit the instance's 32 instruction slots from its "
                      "offset");
        using type = brigand::append<
          brigand::list<SmResource<Instance, Sm>, GpioBaseResource<Instance, GpioBase>>,
          typename Detail::Instructions<Instance,
                                        Offset,
                                        programId<Program>(),
                                        std::make_index_sequence<length>>::type>;
    };

    // `using Provides = Kvasir::Pio::Provides<Instance, Sm, Offset, Program, GpioBase>;`
    template<unsigned Instance,
             unsigned Sm,
             unsigned Offset,
             typename Program,
             unsigned GpioBase = 0>
    using Provides = typename ProvidesFor<Instance, Sm, Offset, Program, GpioBase>::type;

    // One of an instance's two interrupt lines (PIO0_IRQ_0 / _1 ...), for Pio::Irq.
    struct LineTag {};

    template<unsigned Instance, unsigned Line>
    using LineResource = Startup::Resource<LineTag, Instance * 2 + Line>;

    // GPIO function select of each instance: F6, F7, F8.
    template<unsigned Instance>
    constexpr int pinFunction = Instance == 0 ? 6
                              : Instance == 1 ? 7
                                              : 8;

    template<unsigned Instance,
             typename Pin>
    static constexpr auto getPinConfig(Pin) {
        return action(Kvasir::Io::Action::PinFunction<pinFunction<Instance>>{}, Pin{});
    }

    // CLKDIV is 16.8 fixed point: INT in [31:16] (0 means 65536), FRAC in [15:8]. The
    // integer part is not narrowed below the field's width, so a divider of 256 or more
    // (a slow bus from a fast clk_sys) keeps its value instead of wrapping.
    static constexpr auto getDiv(double div) {
        std::uint16_t divInt  = static_cast<std::uint16_t>(div);
        std::uint8_t  divFrac = static_cast<std::uint8_t>((div - divInt) * 256.0);
        return std::pair<std::uint16_t, std::uint8_t>(divInt, divFrac);
    }

    // The range every PIO driver's divider has to be in: at least 1.0, below what the INT
    // field holds. Checked here so a driver need not repeat it (ws2812 keeps its own,
    // tighter, timing asserts).
    template<typename SM>
    static constexpr bool divInRange(double div) {
        constexpr unsigned intBits = static_cast<unsigned>(
          std::popcount(std::remove_cvref_t<decltype(SM::CLKDIV::_int)>::mask));
        return div >= 1.0 && div < static_cast<double>(1ULL << intBits);
    }

    template<typename SM,
             typename Div>
    static constexpr auto getDivConfig(Div) {
        static_assert(divInRange<SM>(Div{}()),
                      "PIO clock divider out of range: at least 1.0 and below 65536 (16.8 "
                      "fixed point) - check clockSpeed against the bus speed asked for");
        constexpr auto divs = getDiv(Div{}());

        return list(write(SM::CLKDIV::_int, Kvasir::Register::value<std::get<0>(divs)>()),
                    write(SM::CLKDIV::frac, Kvasir::Register::value<std::get<1>(divs)>()));
    }

}}   // namespace Kvasir::Pio
