#pragma once
#include "PIO.hpp"
#include "chip/Interrupt.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/SharedIsr.hpp"

#include <cstdint>
#include <peripherals/PIO.hpp>
#include <type_traits>

// A PIO instance's two interrupt lines to the NVIC, one Startup entry per source
// (kvasir/StartUp/SharedIsr.hpp): RX FIFO not empty or TX FIFO not full of one state machine, or
// one IRQ flag a program raises with `irq n` (0..7 on the RP2350, 0..3 reach the NVIC on the
// RP2040). Any number of sources share a line; Startup generates its ISR - one read of INTS, then
// each pending source's handler. A flag is cleared after its handler; a FIFO source clears when
// the handler drains or fills the FIFO. The enable bits of the line's INTE are set in one write.
//
//   void onRx();       // drain RxSm's FIFO
//   void onBreak();    // the program's `irq 3`
//   using RxIrq    = Kvasir::Pio::RxNotEmptyIrq<0, 0, RxSm::Sm, &onRx>;    // PIO0, line 0
//   using BreakIrq = Kvasir::Pio::FlagIrq<0, 0, 3, &onBreak>;
//
// A handler of nullptr makes the entry declare nothing (a source a chip variant lacks).
namespace Kvasir { namespace Pio {

    template<unsigned... Sms>
    constexpr std::uint32_t smMask = ((1U << Sms) | ...);

    template<unsigned... Flags>
    constexpr std::uint32_t flagMask = ((1U << Flags) | ...);

    enum class IrqSourceKind : std::uint8_t { rxNotEmpty, txNotFull, flag };

    template<unsigned      Instance,
             unsigned      Line,
             IrqSourceKind Kind,
             unsigned      N,
             void (*F)(),
             int Priority = 3>
    struct IrqSource {
        static_assert(Instance < PinConfig::pioCount(PinConfig::CurrentChip),
                      "the RP2350 has PIO0..PIO2, the RP2040 PIO0 and PIO1");
        static_assert(Line < 2,
                      "a PIO instance has two interrupt lines");
        static_assert(Kind == IrqSourceKind::flag ? N < 8 : N < 4,
                      "four state machines, eight IRQ flags");
        static_assert(F == nullptr || Kind != IrqSourceKind::flag
                        || !PinConfig::isRp2040(PinConfig::CurrentChip) || N < 4,
                      "the RP2040 routes IRQ flags 0..3 to the NVIC only");

        // The INTE / INTS bit layout: sm0..3_rxnempty in 3:0, sm0..3_txnfull in 7:4, the
        // flags in 15:8.
        static constexpr std::uint32_t bit = 1U << (Kind == IrqSourceKind::rxNotEmpty  ? N
                                                    : Kind == IrqSourceKind::txNotFull ? 4U + N
                                                                                       : 8U + N);

        using PioRegs = Kvasir::Peripheral::PIO::Registers<Instance>;
        using IrqRegs = typename PioRegs::template IRQS<Line>;

        // The NVIC line, by the chip's name for it (PIO0_IRQ_0 is 15 on the RP2350, 7 on the
        // RP2040).
        static constexpr auto nvicLine() {
            if constexpr(Instance == 0) {
                if constexpr(Line == 0) {
                    return Kvasir::Interrupt::pio0_0;
                } else {
                    return Kvasir::Interrupt::pio0_1;
                }
            } else if constexpr(Instance == 1) {
                if constexpr(Line == 0) {
                    return Kvasir::Interrupt::pio1_0;
                } else {
                    return Kvasir::Interrupt::pio1_1;
                }
            } else {
                return nvicLinePio2();
            }
        }

        template<typename I = Kvasir::Interrupt>
        static constexpr auto nvicLinePio2() {
            if constexpr(Line == 0) {
                return I::pio2_0;
            } else {
                return I::pio2_1;
            }
        }

        using NvicIndex = Nvic::Index<decltype(nvicLine())::value>;

        static constexpr Register::
          FieldLocation<typename IrqRegs::INTS::Addr, bit, Register::ReadOnlyAccess, std::uint32_t>
            status{};
        static constexpr Register::
          FieldLocation<typename IrqRegs::INTE::Addr, bit, Register::ReadWriteAccess, std::uint32_t>
            enable{};
        // a flag is sticky until written: the IRQ register's bit, write-one-to-clear
        static constexpr Register::FieldLocation<typename PioRegs::IRQ::Addr,
                                                 (1U << N),
                                                 Register::ROneToClearAccess,
                                                 std::uint32_t>
          flag{};

        using ClearT
          = std::conditional_t<Kind == IrqSourceKind::flag, Nvic::ClearLast<flag>, Nvic::NoClear>;

        using SubIsrs = std::conditional_t<
          F == nullptr,
          brigand::list<>,
          brigand::list<
            Nvic::
              SubIsr<NvicIndex, F, Nvic::Status<status>, ClearT, Nvic::Enable<enable>, Priority>>>;

        static constexpr auto powerClockEnable = list(Kvasir::Pio::getEnable<Instance>());
    };

    template<unsigned Instance, unsigned Line, unsigned Sm, void (*F)(), int Priority = 3>
    using RxNotEmptyIrq = IrqSource<Instance, Line, IrqSourceKind::rxNotEmpty, Sm, F, Priority>;

    template<unsigned Instance, unsigned Line, unsigned Sm, void (*F)(), int Priority = 3>
    using TxNotFullIrq = IrqSource<Instance, Line, IrqSourceKind::txNotFull, Sm, F, Priority>;

    template<unsigned Instance, unsigned Line, unsigned Flag, void (*F)(), int Priority = 3>
    using FlagIrq = IrqSource<Instance, Line, IrqSourceKind::flag, Flag, F, Priority>;

}}   // namespace Kvasir::Pio
