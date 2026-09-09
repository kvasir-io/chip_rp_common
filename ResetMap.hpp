#pragma once

// The RESETS block as Startup resources (kvasir/StartUp/Resources.hpp). FirstInitStep puts
// every block into reset at boot except the few it needs itself (SYSINFO, SYSCFG, PLL_SYS,
// the QSPI pads and IO); a driver that writes a block no listed powerClockEnable releases
// writes into a held block, where the write is lost or bus-faults. So `clear(RESETS::RESET::x)`
// in a powerClockEnable *provides* Resource<ResetTag, bit> (ResourceOfAction), and every
// later-phase action whose address falls into block x *claims* it (ClaimOfAction), by the
// table of block base ranges below (from the generated peripherals/*.hpp base addresses).
// The blocks FirstInitStep releases are not in the table; blocks without a RESET bit (SIO,
// the PPB, CLOCKS, XIP, ...) claim nothing.

#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"

#include <array>
#include <cstdint>
#include <peripherals/RESETS.hpp>
#include <peripherals/UART.hpp>

namespace Kvasir { namespace Resets {
    // mergeIdentical: two instances on one block (two DmaBase, two PWM slices, two PIO
    // programs on one PIO) each release it, which is one release.
    struct ResetTag {
        static constexpr bool        sharedClaim    = true;
        static constexpr bool        coreLocal      = false;
        static constexpr bool        mergeIdentical = true;
        static constexpr char const* message
          = "a peripheral writes a block that no listed powerClockEnable takes out of reset "
            "(the id is the RESETS::RESET bit)";
    };

    template<unsigned Bit>
    using ResetResource = Kvasir::Startup::Resource<ResetTag, Bit>;

    namespace Detail {
        using Reset = Kvasir::Peripheral::RESETS::Registers<>::RESET;

        struct Block {
            std::uint32_t base;
            std::uint32_t size;
            unsigned      bit;
        };

        template<typename Field>
        constexpr unsigned bitOf() {
            return Kvasir::Register::Detail::positionOfFirstSetBit(
              Kvasir::Register::Detail::GetMask<std::remove_cvref_t<Field>>::value);
        }

#if __has_include("chip/rp2040.hpp")
        // RP2040: APB blocks 16 KiB apart, AHB blocks 1 MiB.
        inline constexpr std::array<Block, 19> blocks{
          {
           {0x4004C000, 0x4000, bitOf<decltype(Reset::adc)>()},
           {0x40030000, 0x4000, bitOf<decltype(Reset::busctrl)>()},
           {0x50000000, 0x100000, bitOf<decltype(Reset::dma)>()},
           {0x40044000, 0x4000, bitOf<decltype(Reset::i2c0)>()},
           {0x40048000, 0x4000, bitOf<decltype(Reset::i2c1)>()},
           {0x40014000, 0x4000, bitOf<decltype(Reset::io_bank0)>()},
           {0x4001C000, 0x4000, bitOf<decltype(Reset::pads_bank0)>()},
           {0x50200000, 0x100000, bitOf<decltype(Reset::pio0)>()},
           {0x50300000, 0x100000, bitOf<decltype(Reset::pio1)>()},
           {0x4002C000, 0x4000, bitOf<decltype(Reset::pll_usb)>()},
           {0x40050000, 0x4000, bitOf<decltype(Reset::pwm)>()},
           {0x4005C000, 0x4000, bitOf<decltype(Reset::rtc)>()},
           {0x4003C000, 0x4000, bitOf<decltype(Reset::spi0)>()},
           {0x40040000, 0x4000, bitOf<decltype(Reset::spi1)>()},
           {0x4006C000, 0x4000, bitOf<decltype(Reset::tbman)>()},
           {0x40054000, 0x4000, bitOf<decltype(Reset::timer)>()},
           {0x40034000, 0x4000, bitOf<decltype(Reset::uart0)>()},
           {0x40038000, 0x4000, bitOf<decltype(Reset::uart1)>()},
           {0x50100000, 0x100000, bitOf<decltype(Reset::usbctrl)>()},
           }
        };
#else
        // APB blocks are 32 KiB apart, AHB blocks 1 MiB. Two entries for hstx (control on
        // APB, FIFO on AHB) and usbctrl (registers and DPRAM).
        inline constexpr std::array<Block, 24> blocks{
          {
           {0x400A0000, 0x8000, bitOf<decltype(Reset::adc)>()},
           {0x40068000, 0x8000, bitOf<decltype(Reset::busctrl)>()},
           {0x50000000, 0x100000, bitOf<decltype(Reset::dma)>()},
           {0x400C0000, 0x8000, bitOf<decltype(Reset::hstx)>()},
           {0x50600000, 0x100000, bitOf<decltype(Reset::hstx)>()},
           {0x40090000, 0x8000, bitOf<decltype(Reset::i2c0)>()},
           {0x40098000, 0x8000, bitOf<decltype(Reset::i2c1)>()},
           {0x40028000, 0x8000, bitOf<decltype(Reset::io_bank0)>()},
           {0x40038000, 0x8000, bitOf<decltype(Reset::pads_bank0)>()},
           {0x50200000, 0x100000, bitOf<decltype(Reset::pio0)>()},
           {0x50300000, 0x100000, bitOf<decltype(Reset::pio1)>()},
           {0x50400000, 0x100000, bitOf<decltype(Reset::pio2)>()},
           {0x40058000, 0x8000, bitOf<decltype(Reset::pll_usb)>()},
           {0x400A8000, 0x8000, bitOf<decltype(Reset::pwm)>()},
           {0x400F8000, 0x8000, bitOf<decltype(Reset::sha256)>()},
           {0x40080000, 0x8000, bitOf<decltype(Reset::spi0)>()},
           {0x40088000, 0x8000, bitOf<decltype(Reset::spi1)>()},
           {0x40160000, 0x8000, bitOf<decltype(Reset::tbman)>()},
           {0x400B0000, 0x8000, bitOf<decltype(Reset::timer0)>()},
           {0x400B8000, 0x8000, bitOf<decltype(Reset::timer1)>()},
           {0x400F0000, 0x8000, bitOf<decltype(Reset::trng)>()},
           {0x40070000, 0x8000, bitOf<decltype(Reset::uart0)>()},
           {0x40078000, 0x8000, bitOf<decltype(Reset::uart1)>()},
           {0x50100000, 0x100000, bitOf<decltype(Reset::usbctrl)>()},
           }
        };
#endif

        constexpr int resetBitOf(std::uint32_t address) {
            for(auto const& b : blocks) {
                if(address >= b.base && address < b.base + b.size) {
                    return static_cast<int>(b.bit);
                }
            }
            return -1;
        }

        static_assert(resetBitOf(Kvasir::Peripheral::UART::Registers<0>::baseAddr)
                      == bitOf<decltype(Reset::uart0)>());
        static_assert(resetBitOf(Kvasir::Peripheral::RESETS::Registers<>::baseAddr)
                      == -1);                          // RESETS itself
        static_assert(resetBitOf(0xD0000000) == -1);   // SIO
        static_assert(resetBitOf(0x40000000) == -1);   // SYSINFO: released at boot
    }   // namespace Detail
}}   // namespace Kvasir::Resets

namespace Kvasir { namespace Startup {
    // clear(RESET::x): the block is out of reset from the powerClockEnable phase on.
    template<unsigned Mask, typename Access, typename FieldType>
        requires(Kvasir::Register::Detail::onlyOneBitSet(Mask))
    struct ResourceOfAction<Register::Action<
      Register::FieldLocation<Resets::Detail::Reset::Addr, Mask, Access, FieldType>,
      Register::WriteLiteralAction<0>>> {
        using type = brigand::list<
          Resets::ResetResource<Kvasir::Register::Detail::positionOfFirstSetBit(Mask)>>;
    };

    // Any access to a register inside a block with a RESET bit needs that bit released.
    template<unsigned Addr,
             unsigned Z,
             unsigned O,
             typename RegType,
             typename Mode,
             unsigned Mask,
             typename Access,
             typename FieldType,
             typename TAction>
        requires(Resets::Detail::resetBitOf(Addr) >= 0)
    struct ClaimOfAction<Register::Action<
      Register::
        FieldLocation<Register::Address<Addr, Z, O, RegType, Mode>, Mask, Access, FieldType>,
      TAction>> {
        using type = brigand::list<
          Resets::ResetResource<static_cast<unsigned>(Resets::Detail::resetBitOf(Addr))>>;
    };
}}   // namespace Kvasir::Startup
