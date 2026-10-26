#pragma once
#include "PinConfig.hpp"
#include "chip/Interrupt.hpp"
#include "chip/PinConfig.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Io/Claims.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "kvasir/StartUp/SharedIsr.hpp"

#include <cstdint>
#include <peripherals/IO_BANK0.hpp>
#include <type_traits>

// GPIO edge interrupts through IO_BANK0, one Startup entry per pin (GpioPinEdges / GpioPinIrq,
// below): each declares its edges as sub-interrupts of the io_bank0 vector, and Startup generates
// the one ISR all of them share (kvasir/StartUp/SharedIsr.hpp). Edges only: the level bits are
// read-only mirrors of the pad, so an ISR could not acknowledge them.
//
// The enable register and the NVIC line are per core, so an entry belongs to one core: the plain
// names are core 0's, the ...On<1, ...> forms go in a SecondaryCore's list, and `startupCore` lets
// Startup refuse the wrong list. Clearing INTR is a write of ones to the edge's own bit, so it is
// safe from either core.
//
// The pad (input, pull) is configured in the application's pin configuration like every
// other pin; the pins are claimed, so a missing one is a build error.
namespace Kvasir::Io {

enum class Edge : std::uint8_t { falling = 1, rising = 2, both = 3 };

namespace detail {
    template<typename T>
    struct GpioNumber;

    template<int Port, int Pin>
    struct GpioNumber<PinLocation<Port, Pin>> {
        static_assert(Port == 0,
                      "IO_BANK0 has one port");
        static constexpr unsigned value = static_cast<unsigned>(Pin);
    };

    // Each GPIO owns four bits in its group's registers: level_low, level_high, edge_low,
    // edge_high, in that order, at 4 * (gpio % 8). The generated header spells them out per
    // pin (gpio22_edge_low is bit 26 of the *2 registers); computing them is what lets one
    // template cover any pin.
    inline constexpr unsigned edgeLow  = 2;
    inline constexpr unsigned edgeHigh = 3;

    constexpr std::uint32_t irqBit(unsigned gpio,
                                   unsigned kind) {
        return 1U << (4U * (gpio % 8U) + kind);
    }

    // IO_BANK0's interrupt registers, one per group of eight GPIOs: the raw status (shared), and each core's
    // enable and masked status (RP2040 four groups, RP2350 six).
    struct IoBankIrqGroups {
        using R = Kvasir::Peripheral::IO_BANK0::Registers<>;
#ifdef KVASIR_CHIP_RP2040
        using Intr = brigand::list<R::INTR0, R::INTR1, R::INTR2, R::INTR3>;
        using Inte = brigand::list<
          brigand::list<R::PROC0_INTE0, R::PROC0_INTE1, R::PROC0_INTE2, R::PROC0_INTE3>,
          brigand::list<R::PROC1_INTE0, R::PROC1_INTE1, R::PROC1_INTE2, R::PROC1_INTE3>>;
        using Ints = brigand::list<
          brigand::list<R::PROC0_INTS0, R::PROC0_INTS1, R::PROC0_INTS2, R::PROC0_INTS3>,
          brigand::list<R::PROC1_INTS0, R::PROC1_INTS1, R::PROC1_INTS2, R::PROC1_INTS3>>;
#else
        using Intr = brigand::list<R::INTR0, R::INTR1, R::INTR2, R::INTR3, R::INTR4, R::INTR5>;
        using Inte = brigand::list<brigand::list<R::PROC0_INTE0,
                                                 R::PROC0_INTE1,
                                                 R::PROC0_INTE2,
                                                 R::PROC0_INTE3,
                                                 R::PROC0_INTE4,
                                                 R::PROC0_INTE5>,
                                   brigand::list<R::PROC1_INTE0,
                                                 R::PROC1_INTE1,
                                                 R::PROC1_INTE2,
                                                 R::PROC1_INTE3,
                                                 R::PROC1_INTE4,
                                                 R::PROC1_INTE5>>;
        using Ints = brigand::list<brigand::list<R::PROC0_INTS0,
                                                 R::PROC0_INTS1,
                                                 R::PROC0_INTS2,
                                                 R::PROC0_INTS3,
                                                 R::PROC0_INTS4,
                                                 R::PROC0_INTS5>,
                                   brigand::list<R::PROC1_INTS0,
                                                 R::PROC1_INTS1,
                                                 R::PROC1_INTS2,
                                                 R::PROC1_INTS3,
                                                 R::PROC1_INTS4,
                                                 R::PROC1_INTS5>>;
#endif
    };

    template<unsigned Group, unsigned Core>
    struct IoBankIrqRegs {
        static_assert(Group < (PinConfig::ChipTraits<Chip::part>::pinCount + 7U) / 8U,
                      "IO_BANK0 has one interrupt group per eight GPIOs");
        static_assert(Core < 2,
                      "two cores");

        using Intr
          = brigand::at_c<IoBankIrqGroups::Intr, Group>;   // raw status, edge bits write-1-to-clear
        using Inte = brigand::at_c<brigand::at_c<IoBankIrqGroups::Inte, Core>,
                                   Group>;   // this core's enable
        using Ints = brigand::at_c<brigand::at_c<IoBankIrqGroups::Ints, Core>,
                                   Group>;   // this core's masked status
    };
}   // namespace detail

// One pin's edges as sub-interrupts of io_bank0 (kvasir/StartUp/SharedIsr.hpp): any number of
// these, from any drivers, share the vector, and Startup generates its ISR - one read of each
// INTS register, then per pending edge: clear its INTR bit, call its handler. Each edge is a child
// of its own, so clearing one never drops the other's pending bit; when both are pending, the
// falling edge's handler runs first. A handler of nullptr leaves that edge off. The enable bits
// of a group's INTE are set in one write at init. The pad is configured in the pin
// configuration, as for every input; the pin is claimed.
template<unsigned Core_, void (*OnFalling)(), void (*OnRising)(), typename Pin, int Priority = 3>
struct GpioPinEdgesOn {
    static_assert(Core_ < 2,
                  "two cores");
    static_assert(!Nvic::isNullHandler<OnFalling> || !Nvic::isNullHandler<OnRising>,
                  "a GPIO edge interrupt with neither edge");

    static constexpr unsigned startupCore = Core_;
    using Claims                          = Io::PinClaims<Pin>;

    using Irq = std::decay_t<decltype(Kvasir::Interrupt::io_bank0)>;

    static constexpr unsigned gpio = detail::GpioNumber<Pin>::value;
    using Regs                     = detail::IoBankIrqRegs<gpio / 8U, Core_>;

    template<unsigned Kind, void (*F)()>
    struct EdgeIrq {
        static constexpr std::uint32_t mask = detail::irqBit(gpio, Kind);

        // this core's masked status, the shared raw status (edges write-one-to-clear), this
        // core's enable: the pin's bit in each
        static constexpr Register::
          FieldLocation<typename Regs::Ints::Addr, mask, Register::ReadOnlyAccess, std::uint32_t>
            status{};
        static constexpr Register::
          FieldLocation<typename Regs::Intr::Addr, mask, Register::ROneToClearAccess, std::uint32_t>
            clear{};
        static constexpr Register::
          FieldLocation<typename Regs::Inte::Addr, mask, Register::ReadWriteAccess, std::uint32_t>
            enable{};

        using Sub = Nvic::SubIsr<Irq,
                                 F,
                                 Nvic::Status<status>,
                                 Nvic::ClearFirst<clear>,
                                 Nvic::Enable<enable>,
                                 Priority>;
    };

    using Falling = EdgeIrq<detail::edgeLow, OnFalling>;
    using Rising  = EdgeIrq<detail::edgeHigh, OnRising>;

    using SubIsrs = brigand::append<std::conditional_t<!Nvic::isNullHandler<OnFalling>,
                                                       brigand::list<typename Falling::Sub>,
                                                       brigand::list<>>,
                                    std::conditional_t<!Nvic::isNullHandler<OnRising>,
                                                       brigand::list<typename Rising::Sub>,
                                                       brigand::list<>>>;

    // Stale edges from before boot or before the pad was configured. INTR's register type marks
    // every bit write-ignored-if-zero (edges WC, levels RO: RP2040 datasheet Table 286, RP2350
    // Table 758), so the reset is one plain store of the pin's edge bits: no read, no other pin's
    // pending edge written back.
    static constexpr std::uint32_t edgeBits = (Nvic::isNullHandler<OnFalling> ? 0U : Falling::mask)
                                            | (Nvic::isNullHandler<OnRising> ? 0U : Rising::mask);
    static constexpr Register::
      FieldLocation<typename Regs::Intr::Addr, edgeBits, Register::ROneToClearAccess, std::uint32_t>
                          staleEdges{};
    static constexpr auto initStepPeripheryConfig = list(Kvasir::Register::reset(staleEdges));
};

template<void (*OnFalling)(), void (*OnRising)(), typename Pin, int Priority = 3>
using GpioPinEdges = GpioPinEdgesOn<0, OnFalling, OnRising, Pin, Priority>;

// One handler for the edges E names.
template<unsigned Core_, void (*F)(), Edge E, typename Pin, int Priority = 3>
using GpioPinIrqOn = GpioPinEdgesOn<
  Core_,
  ((static_cast<std::uint8_t>(E) & static_cast<std::uint8_t>(Edge::falling)) != 0 ? F : nullptr),
  ((static_cast<std::uint8_t>(E) & static_cast<std::uint8_t>(Edge::rising)) != 0 ? F : nullptr),
  Pin,
  Priority>;

template<void (*F)(), Edge E, typename Pin, int Priority = 3>
using GpioPinIrq = GpioPinIrqOn<0, F, E, Pin, Priority>;

}   // namespace Kvasir::Io
