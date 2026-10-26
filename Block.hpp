#pragma once

// The RP block facts as one table per chip (chip/Blocks.hpp, kvasir/Hal/Block.hpp's hook): for every block with a
// RESETS bit, its address range, its RESET and RESET_DONE fields, its interrupt lines and its DMA requests. A driver
// asks `Rp::Block<Regs>` (its generated register set) instead of keeping a per-instance `if constexpr` ladder:
//
//     static constexpr auto powerClockEnable = list(clear(Rp::Block<Regs>::reset));
//     using InterruptIndexs = Rp::Block<Regs>::Irqs;
//     Rp::Block<Regs>::waitResetDone();
//
// FirstInitStep puts every block of the table into reset at boot; `clear(reset)` in a powerClockEnable provides the
// gate, and every later-phase write into the block claims it (Hal/Block.hpp). Blocks FirstInitStep keeps running
// (SYSINFO, SYSCFG, PLL_SYS, the QSPI pads and IO) are not in the table, and neither are blocks without a RESET bit
// (SIO, the PPB, CLOCKS, XIP, ...): they claim nothing. RESET and RESET_DONE have the same field at the same bit
// (RP2040 datasheet 2.14.3, Tables 202 and 204; RP2350 datasheet 7.5.3, RESET and RESET_DONE), so a row names both.
//
// This header is the mechanism; the table is the chip's `chip/Blocks.hpp`, which includes it. A user of
// `Rp::Block<Regs>` includes `chip/Blocks.hpp`: the table is a late hook, read where `Regs` is complete, so inside a
// template the order does not matter, but a non-template use of a concrete register set needs the table above it.

#include "WaitBounds.hpp"
#include "kvasir/Hal/Block.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Wait.hpp"

#include <bit>
#include <cstdint>
#include <peripherals/DMA.hpp>
#include <peripherals/RESETS.hpp>
#include <type_traits>

namespace Kvasir { namespace Rp {
    using Resets = Peripheral::RESETS::Registers<>;
    /// The DMA request a block raises (the generated TREQ_SEL enum); `permanent` = none.
    using Dreq = Peripheral::DMA::Registers<>::CH<0>::CTRL_TRIG::TREQ_SELVal;

    template<auto const& Field>
    constexpr unsigned bitOf() {
        constexpr auto mask = std::remove_cvref_t<decltype(Field)>::mask;
        static_assert(std::has_single_bit(mask), "a RESETS field is one bit");
        return static_cast<unsigned>(std::countr_zero(mask));
    }

    /// One row of a chip's table. `Base`/`Size`: the block's range (`Regs::baseAddr` of the generated register set).
    /// `DreqTx`/`DreqRx`: the block's requests; a PIO row names state machine 0's, the others follow (`dreqTx(sm)`).
    template<std::uint32_t Base,
             std::uint32_t Size,
             auto const&   Reset,
             auto const&   Done,
             typename Interrupts = brigand::list<>,
             Dreq DreqTx         = Dreq::permanent,
             Dreq DreqRx         = Dreq::permanent>
    struct BlockRow : Hal::BlockRow<Base, Size, bitOf<Reset>(), decltype(clear(Reset))> {
        static_assert(bitOf<Reset>() == bitOf<Done>(),
                      "RESET and RESET_DONE name one block at one bit");
        static constexpr auto const& reset     = Reset;
        static constexpr auto const& resetDone = Done;
        using Irqs                             = Interrupts;
        static constexpr Dreq dreqTx           = DreqTx;
        static constexpr Dreq dreqRx           = DreqRx;
    };

    namespace Detail {
        template<std::uint32_t Base, typename Table>
        struct RowAt;

        template<std::uint32_t Base, typename... Rows>
        struct RowAt<Base, Hal::BlockTable<Rows...>> {
            static constexpr std::size_t n
              = (std::size_t{0} + ... + std::size_t{Rows::base == Base});
            static_assert(
              n == 1,
              "this register set is no block of the chip's table (chip/Blocks.hpp): no RESETS bit, "
              "or released by FirstInitStep");
            using type = brigand::flatten<brigand::list<
              std::conditional_t<Rows::base == Base, brigand::list<Rows>, brigand::list<>>...>>;
        };
    }   // namespace Detail

    /// The table row of a generated register set `Regs` (its `baseAddr`), from the chip's `Hal::BlockGates`.
    template<typename Regs>
    struct Block
      : brigand::front<
          typename Detail::RowAt<Regs::baseAddr, typename Hal::BlockGatesOf<Regs>::type>::type> {
        using Row = brigand::front<
          typename Detail::RowAt<Regs::baseAddr, typename Hal::BlockGatesOf<Regs>::type>::type>;

        /// RESET_DONE: registers written before it is set are dropped, so a driver that released the block at run
        /// time waits for it.
        static void waitResetDone() {
            Register::waitUntil<Chip::ResetDoneBound>(Register::isSet(Row::resetDone));
        }

        /// A PIO block's requests per state machine: TX m = TX 0 + m, RX m = RX 0 + m (RP2040 datasheet 2.5.3.1
        /// Table 119, RP2350 datasheet 12.6.4.1: DREQ_PIOn_TXm = 8n + m, DREQ_PIOn_RXm = 8n + 4 + m).
        template<unsigned Sm>
        static constexpr Dreq dreqTxOf() {
            static_assert(Sm < 4, "a PIO block has four state machines");
            return static_cast<Dreq>(static_cast<std::uint32_t>(Row::dreqTx) + Sm);
        }

        template<unsigned Sm>
        static constexpr Dreq dreqRxOf() {
            static_assert(Sm < 4, "a PIO block has four state machines");
            return static_cast<Dreq>(static_cast<std::uint32_t>(Row::dreqRx) + Sm);
        }
    };
}}   // namespace Kvasir::Rp
