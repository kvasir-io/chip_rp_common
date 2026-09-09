#pragma once
#include "PinConfig.hpp"
#include "chip/Interrupt.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"

#include <cstdint>
#include <peripherals/IO_BANK0.hpp>
#include <tuple>
#include <type_traits>

// GPIO edge interrupts through IO_BANK0. One instance watches a set of pins in one group of
// eight (one INTR/INTE/INTS register triple) for the configured edges and calls F(pending)
// with the INTR bits that fired, restricted to the configured ones; the bits are cleared
// before F runs, so an edge during F fires the line again. Edges only: the level bits are
// read-only mirrors of the pad, so an ISR could not acknowledge them.
//
// The enable register and the NVIC line are per core, so an instance belongs to one core:
// GpioIrq<F, E, Pins...> is core 0's, GpioIrqOn<1, F, E, Pins...> goes in a SecondaryCore's
// list, and `startupCore` lets Startup refuse the wrong list (the interrupt would be enabled
// in one core's mask and the other core's NVIC). Clearing INTR is a write of ones to our
// own bits, so it is safe from either core after launch.
//
// The pad (input, pull) is configured in the application's pin configuration like every
// other pin; the pins are claimed, so a missing one is a build error.
namespace Kvasir::Io {

enum class Edge : std::uint8_t { falling = 1, rising = 2, both = 3 };

// The resource a GpioIrq owns: one group of eight on one core (kvasir/StartUp/Resources.hpp).
struct GpioIrqGroupTag {
    static constexpr unsigned    keyArity = 2;
    static constexpr char const* message
      = "one GpioIrq per group of eight pins per core: list every pin of the group in it";
};

namespace detail {
    template<typename T>
    struct GpioNumber;

    template<int Port, int Pin>
    struct GpioNumber<Register::PinLocation<Port, Pin>> {
        static_assert(Port == 0,
                      "IO_BANK0 has one port");
        static constexpr unsigned value = static_cast<unsigned>(Pin);
    };

    // Each GPIO owns four bits in its group's registers: level_low, level_high, edge_low,
    // edge_high, in that order, at 4 * (gpio % 8). The generated header spells them out per
    // pin (gpio22_edge_low is bit 26 of the *2 registers); computing them is what lets one
    // template cover any pin.
    inline constexpr unsigned EdgeLow  = 2;
    inline constexpr unsigned EdgeHigh = 3;

    constexpr std::uint32_t irqBit(unsigned gpio,
                                   unsigned kind) {
        return 1U << (4U * (gpio % 8U) + kind);
    }

    template<unsigned Group, unsigned Core>
    struct IoBankIrqRegs {
        // Dependent on Group so the groups 4 and 5 (RP2350 only) are not looked up on the
        // RP2040's registers.
        using R = Kvasir::Peripheral::IO_BANK0::Registers<Group * 0>;
        static_assert(Group < (PinConfig::ChipTraits<PinConfig::CurrentChip>::pinCount + 7U) / 8U,
                      "IO_BANK0 has one interrupt group per eight GPIOs");
        static_assert(Core < 2,
                      "two cores");

        static constexpr auto pick() {
            if constexpr(Core == 0) {
                if constexpr(Group == 0) {
                    return std::type_identity<std::tuple<typename R::INTR0,
                                                         typename R::PROC0_INTE0,
                                                         typename R::PROC0_INTS0>>{};
                } else if constexpr(Group == 1) {
                    return std::type_identity<std::tuple<typename R::INTR1,
                                                         typename R::PROC0_INTE1,
                                                         typename R::PROC0_INTS1>>{};
                } else if constexpr(Group == 2) {
                    return std::type_identity<std::tuple<typename R::INTR2,
                                                         typename R::PROC0_INTE2,
                                                         typename R::PROC0_INTS2>>{};
                } else if constexpr(Group == 3) {
                    return std::type_identity<std::tuple<typename R::INTR3,
                                                         typename R::PROC0_INTE3,
                                                         typename R::PROC0_INTS3>>{};
                } else if constexpr(Group == 4) {
                    return std::type_identity<std::tuple<typename R::INTR4,
                                                         typename R::PROC0_INTE4,
                                                         typename R::PROC0_INTS4>>{};
                } else {
                    return std::type_identity<std::tuple<typename R::INTR5,
                                                         typename R::PROC0_INTE5,
                                                         typename R::PROC0_INTS5>>{};
                }
            } else {
                if constexpr(Group == 0) {
                    return std::type_identity<std::tuple<typename R::INTR0,
                                                         typename R::PROC1_INTE0,
                                                         typename R::PROC1_INTS0>>{};
                } else if constexpr(Group == 1) {
                    return std::type_identity<std::tuple<typename R::INTR1,
                                                         typename R::PROC1_INTE1,
                                                         typename R::PROC1_INTS1>>{};
                } else if constexpr(Group == 2) {
                    return std::type_identity<std::tuple<typename R::INTR2,
                                                         typename R::PROC1_INTE2,
                                                         typename R::PROC1_INTS2>>{};
                } else if constexpr(Group == 3) {
                    return std::type_identity<std::tuple<typename R::INTR3,
                                                         typename R::PROC1_INTE3,
                                                         typename R::PROC1_INTS3>>{};
                } else if constexpr(Group == 4) {
                    return std::type_identity<std::tuple<typename R::INTR4,
                                                         typename R::PROC1_INTE4,
                                                         typename R::PROC1_INTS4>>{};
                } else {
                    return std::type_identity<std::tuple<typename R::INTR5,
                                                         typename R::PROC1_INTE5,
                                                         typename R::PROC1_INTS5>>{};
                }
            }
        }

        using Tuple = typename decltype(pick())::type;
        using Intr  = std::tuple_element_t<0, Tuple>;   // raw status, edge bits write-1-to-clear
        using Inte  = std::tuple_element_t<1, Tuple>;   // this core's enable
        using Ints  = std::tuple_element_t<2, Tuple>;   // this core's masked status
    };
}   // namespace detail

template<unsigned Core_, void (*F)(std::uint32_t pending), Edge E, typename... Pins>
struct GpioIrqOn {
    static_assert(sizeof...(Pins) > 0,
                  "GpioIrq needs at least one pin");
    static_assert(Core_ < 2,
                  "two cores");

    static constexpr unsigned Core = Core_;

    // Startup: this belongs in core `Core`'s list, and uses pins it does not configure.
    static constexpr unsigned startupCore = Core_;
    using Claims                          = Io::PinClaims<Pins...>;

    using Irq = std::decay_t<decltype(Kvasir::Interrupt::io_bank0)>;

    static constexpr unsigned Group = [] {
        constexpr unsigned groups[] = {(detail::GpioNumber<Pins>::value / 8U)...};
        return groups[0];
    }();
    static_assert((((detail::GpioNumber<Pins>::value / 8U) == Group) && ...),
                  "all pins of one GpioIrq must share a group of eight (one register triple)");

    // Startup: one instance per (core, group) - two on one group would each install the
    // io_bank0 vector and mask each other's INTE bits. Keyed by both ids.
    using Provides = brigand::list<Startup::Resource<GpioIrqGroupTag, Core_, Group>>;

    static constexpr bool WantFalling
      = (static_cast<std::uint8_t>(E) & static_cast<std::uint8_t>(Edge::falling)) != 0;
    static constexpr bool WantRising
      = (static_cast<std::uint8_t>(E) & static_cast<std::uint8_t>(Edge::rising)) != 0;

    // The bits F sees: which pin, and which way it went.
    static constexpr std::uint32_t fallingMask
      = WantFalling ? (detail::irqBit(detail::GpioNumber<Pins>::value, detail::EdgeLow) | ...) : 0U;
    static constexpr std::uint32_t risingMask
      = WantRising ? (detail::irqBit(detail::GpioNumber<Pins>::value, detail::EdgeHigh) | ...) : 0U;
    static constexpr std::uint32_t mask = fallingMask | risingMask;

    template<typename Pin>
    static constexpr std::uint32_t fallingBit
      = WantFalling ? detail::irqBit(detail::GpioNumber<Pin>::value, detail::EdgeLow) : 0U;
    template<typename Pin>
    static constexpr std::uint32_t risingBit
      = WantRising ? detail::irqBit(detail::GpioNumber<Pin>::value, detail::EdgeHigh) : 0U;

    using Regs = detail::IoBankIrqRegs<Group, Core>;
    using Intr = typename Regs::Intr;
    using Inte = typename Regs::Inte;
    using Ints = typename Regs::Ints;

    // Our bits of the enable register as one field: a literal write of `mask` into a field
    // whose mask is `mask` is a read-modify-write that sets exactly those bits and leaves
    // the rest of the register alone (Register::set would do the same, but only for one bit).
    using EnableField = Register::
      FieldLocation<typename Inte::Addr, mask, Register::ReadWriteAccess, std::uint32_t>;
    static constexpr auto enable
      = Register::Action<EnableField, Register::WriteLiteralAction<mask>>{};

    // A literal full-register write of our bits to INTR is a plain store: zeros are ignored
    // by the hardware, so it clears exactly the edges we own -- stale ones from before boot,
    // or from before the pad was configured.
    static constexpr auto clearPending
      = Register::write(Intr::FULLREGISTER, Register::value<std::uint32_t, mask>());

    static constexpr auto initStepPeripheryConfig = list(clearPending, enable);

    static constexpr auto initStepInterruptConfig
      = list(Nvic::makeSetPriority<3>(Irq{}), Nvic::makeClearPending(Irq{}));
    static constexpr auto initStepPeripheryEnable = list(Nvic::makeEnable(Irq{}));

    static void onIsr() {
        auto const pending = get<0>(apply(Register::read(Ints::FULLREGISTER))) & mask;
        apply(Register::write(Intr::FULLREGISTER, pending));
        F(pending);
    }

    static constexpr Nvic::Isr<std::addressof(onIsr), Irq> isr{};
};

// The boot core's, which is what a single-core chip and most instances want.
template<void (*F)(std::uint32_t pending), Edge E, typename... Pins>
using GpioIrq = GpioIrqOn<0, F, E, Pins...>;

}   // namespace Kvasir::Io
