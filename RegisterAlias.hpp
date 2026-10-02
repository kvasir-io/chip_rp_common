#pragma once
// Register::atomic() on the RP chips: every register block has XOR / SET / CLR aliases at +0x1000 /
// +0x2000 / +0x3000 (RP2040 datasheet 2.1.2, RP2350 2.1.3 "Atomic register access"), so a literal
// write that must not lose a concurrent update of another field is one store - no read, no
// interrupt mask, safe against the other core.
//
//     apply(Register::atomic(set(DMA::INTE0::inte0<3>), clear(DMA::INTE0::inte0<5>)));
//     // two stores: 0x08 to INTE0 + 0x2000, 0x20 to INTE0 + 0x3000
//
// An alias write acts on its mask's bits only: a pending write-one-to-clear flag of the same
// register survives unless its own bit is in the mask. So atomic() is refused for a one-to-* field
// (Register::atomic, AtomicFactories.hpp) but fine beside one. Behind the bus interposer (I2C, UART, SPI, SSI) an alias write is a
// read-modify-write: a register with a read side effect never gets one (RmwHazard).
#include "kvasir/Register/Exec.hpp"

#include <array>
#include <cstdint>

namespace Kvasir::Rp {
inline constexpr unsigned aliasXor = 0x1000;
inline constexpr unsigned aliasSet = 0x2000;
inline constexpr unsigned aliasClr = 0x3000;

namespace Detail {
    // The register blocks with aliases, from the address maps, minus what the datasheet excludes
    // (SIO, the CoreSight window, the PPB; RP2350 also the OTP block, whose SBPI programming
    // registers have none) and what is memory or a FIFO window rather than registers (USB DPRAM,
    // XIP_AUX, HSTX_FIFO, OTP data, BOOTRAM).
#if __has_include("chip/rp2040.hpp")
    // RP2040 datasheet 2.2.2 (APB, AHB-Lite)
    inline constexpr auto AliasBlocks = std::to_array<unsigned>(
      {0x40000000, 0x40004000, 0x40008000, 0x4000c000, 0x40010000, 0x40014000, 0x40018000,
       0x4001c000, 0x40020000, 0x40024000, 0x40028000, 0x4002c000, 0x40030000, 0x40034000,
       0x40038000, 0x4003c000, 0x40040000, 0x40044000, 0x40048000, 0x4004c000, 0x40050000,
       0x40054000, 0x40058000, 0x4005c000, 0x40060000, 0x40064000, 0x4006c000, 0x50000000,
       0x50110000, 0x50200000, 0x50300000});
#else
    // RP2350 datasheet 2.2.4 Table 13 (APB) and 2.2.5 Table 14 (AHB)
    inline constexpr auto AliasBlocks = std::to_array<unsigned>(
      {0x40000000, 0x40008000, 0x40010000, 0x40018000, 0x40020000, 0x40028000, 0x40030000,
       0x40038000, 0x40040000, 0x40048000, 0x40050000, 0x40058000, 0x40060000, 0x40068000,
       0x40070000, 0x40078000, 0x40080000, 0x40088000, 0x40090000, 0x40098000, 0x400a0000,
       0x400a8000, 0x400b0000, 0x400b8000, 0x400c0000, 0x400c8000, 0x400d0000, 0x400d8000,
       0x400e8000, 0x400f0000, 0x400f8000, 0x40100000, 0x40108000, 0x40158000, 0x40160000,
       0x50000000, 0x50110000, 0x50200000, 0x50300000, 0x50400000});
#endif
}   // namespace Detail

// address lies in the first 4 KB of an aliased block (the normal-access window)
constexpr bool hasAtomicAlias(unsigned address) {
    for(auto const base : Detail::AliasBlocks) {
        if(address >= base && address < base + aliasXor) { return true; }
    }
    return false;
}
}   // namespace Kvasir::Rp

namespace Kvasir::Register {
template<unsigned A,
         unsigned Z,
         unsigned O,
         typename T,
         typename M,
         unsigned Mask,
         typename Acc,
         typename FT,
         unsigned Data>
struct ExecuteSeam<
  Action<FieldLocation<Address<A, Z, O, T, M>, Mask, Acc, FT>, AtomicWriteLiteralAction<Data>>,
  ::Kvasir::Tag::User> {
    static_assert(
      Rp::hasAtomicAlias(A),
      "Register::atomic: this register has no atomic alias (SIO, PPB, a memory window): "
      "use its own SET/CLR/XOR registers, or a critical section");
    static_assert(sizeof(T) == 4,
                  "RP registers are 32 bits wide");
    static_assert(
      !M::readHasSideEffect,
      "Register::atomic: the register's read has a side effect, and an alias write behind "
      "the bus interposer reads it (RP2350 datasheet 2.1.3)");

    static constexpr unsigned setBits = Data;           // bits to end up 1
    static constexpr unsigned clrBits = Mask & ~Data;   // bits to end up 0

    template<unsigned Alias>
    static void store(unsigned v) {
#ifdef KVASIR_REGISTER_MOCK
        ::Kvasir::Test::write<T, A + Alias>(static_cast<T>(v));
#else
        *reinterpret_cast<T volatile*>(A + Alias) = static_cast<T>(v);
#endif
    }

    unsigned operator()(unsigned = 0) {
        if constexpr(clrBits == 0) {
            store<Rp::aliasSet>(setBits);   // one store, no read
        } else if constexpr(setBits == 0) {
            store<Rp::aliasClr>(clrBits);   // one store, no read
        } else {
            // a field that needs both: flip exactly the bits that differ (pico-sdk hw_write_masked)
            unsigned const old = Detail::GetAddress<Address<A, Z, O, T, M>>::read();
            store<Rp::aliasXor>((old ^ Data) & Mask);
        }
        return 0;
    }
};
}   // namespace Kvasir::Register
