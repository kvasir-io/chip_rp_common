#pragma once
#include "PIO.hpp"
#include "chip/Interrupt.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Register/Register.hpp"

#include <cstdint>
#include <peripherals/PIO.hpp>

// One of a PIO instance's two interrupt lines to the NVIC, as a Startup-list peripheral.
// Sources: RX FIFO not empty and TX FIFO not full per state machine, and the IRQ flags a
// program raises with `irq n` (0..7 on the RP2350, 0..3 on the RP2040). The callback runs
// with the pending sources; the flags among them are cleared afterwards, the FIFO sources
// clear when the callback drains or fills the FIFO.
//
//   struct RxIrqConfig {
//       static constexpr auto isrPriority = 3;
//       static constexpr auto rxNotEmpty  = Kvasir::Pio::smMask<0>;       // SM0's RX FIFO
//       static constexpr auto flags       = Kvasir::Pio::flagMask<3>;     // `irq 3` from any SM
//   };
//   void onRx(std::uint32_t pending);   // test with RxIrq::RxNotEmpty<0>, RxIrq::IrqFlag<3>
//   using RxIrq = Kvasir::Pio::Irq<0, 0, RxIrqConfig, &onRx>;
//
// Config: isrPriority (3), rxNotEmpty (mask of state machines, 0), txNotFull (mask, 0),
// flags (mask of IRQ flags, 0). Two Irqs on one line are a build error.
namespace Kvasir { namespace Pio {

    template<unsigned... Sms>
    constexpr std::uint32_t smMask = ((1U << Sms) | ...);

    template<unsigned... Flags>
    constexpr std::uint32_t flagMask = ((1U << Flags) | ...);

    template<unsigned Instance, unsigned Line, typename Config_, void (*Callback)(std::uint32_t)>
    struct Irq {
        struct Config : Config_ {
            static constexpr int isrPriority = [] {
                if constexpr(requires { Config_::isrPriority; }) {
                    return static_cast<int>(Config_::isrPriority);
                } else {
                    return 3;
                }
            }();
            static constexpr std::uint32_t rxNotEmpty = [] {
                if constexpr(requires { Config_::rxNotEmpty; }) {
                    return static_cast<std::uint32_t>(Config_::rxNotEmpty);
                } else {
                    return 0U;
                }
            }();
            static constexpr std::uint32_t txNotFull = [] {
                if constexpr(requires { Config_::txNotFull; }) {
                    return static_cast<std::uint32_t>(Config_::txNotFull);
                } else {
                    return 0U;
                }
            }();
            static constexpr std::uint32_t flags = [] {
                if constexpr(requires { Config_::flags; }) {
                    return static_cast<std::uint32_t>(Config_::flags);
                } else {
                    return 0U;
                }
            }();
        };

        static_assert(Instance < PinConfig::pioCount(PinConfig::CurrentChip),
                      "the RP2350 has PIO0..PIO2, the RP2040 PIO0 and PIO1");
        static_assert(Line < 2,
                      "a PIO instance has two interrupt lines");
        static_assert(Config::rxNotEmpty < 16 && Config::txNotFull < 16,
                      "rxNotEmpty / txNotFull are masks over the four state machines");
        static_assert(Config::flags < 256,
                      "flags is a mask over the eight IRQ flags");
        static_assert(!PinConfig::isRp2040(PinConfig::CurrentChip) || Config::flags < 16,
                      "the RP2040 routes IRQ flags 0..3 to the NVIC only");
        static_assert((Config::rxNotEmpty | Config::txNotFull | Config::flags) != 0,
                      "a Pio::Irq with no source enabled would never fire");

        // The INTE / INTS bit layout: sm0..3_rxnempty in 3:0, sm0..3_txnfull in 7:4, the
        // flags in 15:8.
        static constexpr std::uint32_t RxNotEmptyShift = 0;
        static constexpr std::uint32_t TxNotFullShift  = 4;
        static constexpr std::uint32_t FlagShift       = 8;

        template<unsigned Sm>
        static constexpr std::uint32_t RxNotEmpty = 1U << (RxNotEmptyShift + Sm);
        template<unsigned Sm>
        static constexpr std::uint32_t TxNotFull = 1U << (TxNotFullShift + Sm);
        template<unsigned N>
        static constexpr std::uint32_t IrqFlag = 1U << (FlagShift + N);

        static constexpr std::uint32_t Mask = (Config::rxNotEmpty << RxNotEmptyShift)
                                            | (Config::txNotFull << TxNotFullShift)
                                            | (Config::flags << FlagShift);

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

        using Provides = brigand::list<LineResource<Instance, Line>>;

        static constexpr auto powerClockEnable = list(Kvasir::Pio::getEnable<Instance>());

        static constexpr auto initStepPeripheryConfig
          = list(write(IrqRegs::INTE::FULLREGISTER, Register::value<Mask>()));

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<Config::isrPriority>(NvicIndex{}),
                 Nvic::makeClearPending(NvicIndex{}));

        static constexpr auto initStepPeripheryEnable = list(Nvic::makeEnable(NvicIndex{}));

        static void onIsr() {
            auto const pending = get<0>(apply(read(IrqRegs::INTS::FULLREGISTER))) & Mask;
            Callback(pending);
            // The flags are sticky until written; the FIFO sources follow the FIFOs.
            if constexpr(Config::flags != 0) {
                apply(write(PioRegs::IRQ::irq, (pending >> FlagShift) & Config::flags));
            }
        }

        static constexpr Nvic::Isr<std::addressof(onIsr), NvicIndex> isr{};
    };

}}   // namespace Kvasir::Pio
