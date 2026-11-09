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
// read-modify-write: a register with a read side effect never gets one (Register::Semantics::readSideEffect).
//
// The access is the Register::ExecuteSeam hook (Kvasir_SDK Register/Exec.hpp) for Kvasir::Tag::User. chip_rp2040 and
// chip_rp2350 name this header in KVASIR_CHIP_REGISTER_SEAM (cmake/chip.cmake), and Exec.hpp includes it at its end:
// every TU of an image that reaches apply() has it before its first atomic(). Only the SDK, Tags and Mmio here - it is
// read in the middle of the register layer.
//
// The same header names the SIO's shared states (Register::SharedState, at its end): a pin's output level is written
// through GPIO_OUT, GPIO_OUT_SET, _CLR and _XOR, its output enable through GPIO_OE, _SET, _CLR and _XOR (the same for
// GPIO_HI_*). One step of an apply() runs by register address, highest first (Register/Apply.hpp), not in the order
// written, and GPIO_OUT_CLR sits above GPIO_OUT_SET on both chips: `makeOutputInitHigh(pin)` (GPIO_OUT_SET, Io.hpp) with
// `clear(pin)` (GPIO_OUT_CLR) in one list ended HIGH, `makeOutput(pin)` with `toggle(pin)` low (GPIO_OUT_XOR first). With
// the states apply() refuses such a list and names the pin; a Startup phase (every entry's initStepPinConfig) is one
// apply(), so two entries that write one pin's level are refused too. Io.hpp checks the literals against the generated
// SIO registers.
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Register/Exec.hpp"
#include "kvasir/Register/Mmio.hpp"

#include <array>
#include <cstdint>
#include <string_view>

namespace Kvasir::Rp {
inline constexpr unsigned aliasXor = 0x1000;
inline constexpr unsigned aliasSet = 0x2000;
inline constexpr unsigned aliasClr = 0x3000;

namespace Detail {
    // The register blocks with aliases, from the address maps, minus what the datasheet excludes
    // (SIO, the CoreSight window, the PPB; RP2350 also the OTP block, whose SBPI programming
    // registers have none) and what is memory or a FIFO window rather than registers (USB DPRAM,
    // XIP_AUX, HSTX_FIFO, OTP data, BOOTRAM).
#ifdef KVASIR_CHIP_RP2040
    // RP2040 datasheet 2.2.2 (APB, AHB-Lite)
    inline constexpr auto aliasBlocks = std::to_array<unsigned>(
      {0x40000000, 0x40004000, 0x40008000, 0x4000c000, 0x40010000, 0x40014000, 0x40018000,
       0x4001c000, 0x40020000, 0x40024000, 0x40028000, 0x4002c000, 0x40030000, 0x40034000,
       0x40038000, 0x4003c000, 0x40040000, 0x40044000, 0x40048000, 0x4004c000, 0x40050000,
       0x40054000, 0x40058000, 0x4005c000, 0x40060000, 0x40064000, 0x4006c000, 0x50000000,
       0x50110000, 0x50200000, 0x50300000});
#else
    // RP2350 datasheet 2.2.4 Table 13 (APB) and 2.2.5 Table 14 (AHB)
    inline constexpr auto aliasBlocks = std::to_array<unsigned>(
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
    for(auto const base : Detail::aliasBlocks) {
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
  ::Kvasir::Tag::User,
  Action<FieldLocation<Address<A, Z, O, T, M>, Mask, Acc, FT>, AtomicWriteLiteralAction<Data>>> {
    static_assert(
      Rp::hasAtomicAlias(A),
      "Register::atomic: this register has no atomic alias (SIO, PPB, a memory window): "
      "use its own SET/CLR/XOR registers, or a critical section");
    static_assert(sizeof(T) == 4,
                  "RP registers are 32 bits wide");
    static_assert(
      !M::semantics.readSideEffect,
      "Register::atomic: the register's read has a side effect, and an alias write behind "
      "the bus interposer reads it (RP2350 datasheet 2.1.3)");

    static constexpr unsigned setBits = Data;           // bits to end up 1
    static constexpr unsigned clrBits = Mask & ~Data;   // bits to end up 0

    template<unsigned Alias>
    [[gnu::always_inline]] static void store(unsigned v) {
        Mmio<T>::store(A + Alias, static_cast<T>(v));
    }

    [[gnu::always_inline]] unsigned operator()(unsigned = 0) {
        if constexpr(clrBits == 0) {
            store<Rp::aliasSet>(setBits);   // one store, no read
        } else if constexpr(setBits == 0) {
            store<Rp::aliasClr>(clrBits);   // one store, no read
        } else {
            // a field that needs both: flip exactly the bits that differ (pico-sdk hw_write_masked)
            unsigned const old = RegisterTraits<Address<A, Z, O, T, M>>::read();
            store<Rp::aliasXor>((old ^ Data) & Mask);
        }
        return 0;
    }
};
}   // namespace Kvasir::Register

// ---- the SIO's shared states (Register::SharedState; the top of this file says why) ----
namespace Kvasir::Rp::SioSeam {
// RP2040 datasheet 2.3.1.7 Table 16, RP2350 datasheet 3.1.11 Table 17: the SIO at 0xd0000000 (SIO_BASE); its GPIO
// registers from 0x010 to 0x04c, each state's own register (GPIO_OUT, GPIO_OE, GPIO_HI_OUT, GPIO_HI_OE) with its
// SET, CLR and XOR registers above it. A state is its own register's address, the lowest of its four (SharedState's
// contract, Register/Exec.hpp)
inline constexpr unsigned baseAddress = 0xd0000000;
inline constexpr unsigned firstState  = 0x010;
inline constexpr unsigned statesEnd   = 0x050;

#ifdef KVASIR_CHIP_RP2040
// RP2040 Table 16: GPIO_OUT 0x010, _SET 0x014, _CLR 0x018, _XOR 0x01c; GPIO_OE 0x020..0x02c; GPIO_HI_OUT (the QSPI
// pins) 0x030..0x03c; GPIO_HI_OE 0x040..0x04c - a state's four registers next to each other
constexpr unsigned stateOf(unsigned address) {
    if(address < baseAddress + firstState || address >= baseAddress + statesEnd || address % 4 != 0)
    {
        return 0;
    }
    return address - (address - baseAddress) % 0x10;
}

constexpr bool isLevel(unsigned state) {
    return state - baseAddress == 0x010 || state - baseAddress == 0x030;
}

constexpr bool isHighBank(unsigned state) { return state - baseAddress >= 0x030; }

// a GPIO_HI_* bit: QSPI SCLK, SSn, SD0..SD3 in bits 0..5 (RP2040 Table 19, GPIO_HI_IN; Table 28, GPIO_HI_OUT)
consteval std::string_view highPinName(unsigned bit) {
    constexpr std::array<std::string_view, 6> names{"QSPI_SCLK",
                                                    "QSPI_SS",
                                                    "QSPI_SD0",
                                                    "QSPI_SD1",
                                                    "QSPI_SD2",
                                                    "QSPI_SD3"};
    return bit < names.size() ? names[bit] : std::string_view{};
}
#else
// RP2350 Table 17: the two banks interleaved - GPIO_OUT 0x010, GPIO_HI_OUT 0x014, GPIO_OUT_SET 0x018, GPIO_HI_OUT_SET
// 0x01c, _CLR 0x020 / 0x024, _XOR 0x028 / 0x02c; GPIO_OE 0x030, GPIO_HI_OE 0x034, ... 0x048 / 0x04c - a state's
// registers 8 apart. The Non-secure SIO at 0xd0020000 (SIO_NONSEC_BASE, 3.1.1) is another view of the same GPIO
// registers (3.1.3): its addresses write the Secure view's states.
inline constexpr unsigned nonSecureBase = 0xd0020000;

constexpr unsigned stateOf(unsigned address) {
    unsigned const a = address >= nonSecureBase && address < nonSecureBase + statesEnd
                       ? address - (nonSecureBase - baseAddress)
                       : address;
    if(a < baseAddress + firstState || a >= baseAddress + statesEnd || a % 4 != 0) { return 0; }
    unsigned const offset = a - baseAddress - firstState;
    return baseAddress + firstState + (offset & 0x20U) + (offset & 0x4U);
}

constexpr bool isLevel(unsigned state) { return state - baseAddress < 0x030; }

constexpr bool isHighBank(unsigned state) { return (state - baseAddress) % 8 == 4; }

// a GPIO_HI_* bit: GPIO32..47 in bits 15:0, USB_DP 24, USB_DM 25, QSPI_SCK 26, QSPI_CSN 27, QSPI_SD0..3 in 31:28
// (RP2350 Table 22, GPIO_HI_OUT); the GPIOs are named by the caller
consteval std::string_view highPinName(unsigned bit) {
    constexpr std::array<std::string_view, 8> names{"USB_DP",
                                                    "USB_DM",
                                                    "QSPI_SCK",
                                                    "QSPI_CSN",
                                                    "QSPI_SD0",
                                                    "QSPI_SD1",
                                                    "QSPI_SD2",
                                                    "QSPI_SD3"};
    return bit >= 24 ? names[bit - 24] : std::string_view{};
}
#endif

// "GPIO17, GPIO18": the pins of `mask` in the bank of `state`
consteval void pinNames(Kvasir::Diagnostic::Text& t,
                        unsigned                  state,
                        unsigned                  mask) {
    bool first = true;
    for(unsigned bit = 0; bit < 32; ++bit) {
        if(((mask >> bit) & 1U) == 0) { continue; }
        t << (first ? "" : ", ");
        first = false;
        if(!isHighBank(state)) {
            t << "GPIO";
            t.dec(bit);
        } else if(std::string_view const name = highPinName(bit); !name.empty()) {
            t << name;
        } else {
#ifdef KVASIR_CHIP_RP2040
            t << "GPIO_HI bit ";
            t.dec(bit);
#else
            t << "GPIO";
            t.dec(32 + bit);
#endif
        }
    }
}

// "GPIO_OUT_CLR", "GPIO_HI_OE_SET": the register of `state` and its alias `suffix` ("" for the state's own)
consteval void registerName(Kvasir::Diagnostic::Text& t,
                            unsigned                  state,
                            std::string_view          suffix) {
    t << (isHighBank(state) ? "GPIO_HI_" : "GPIO_") << (isLevel(state) ? "OUT" : "OE") << suffix;
}
}   // namespace Kvasir::Rp::SioSeam

namespace Kvasir::Register {
template<unsigned A, unsigned Z, unsigned O, typename T, typename M>
    requires(Rp::SioSeam::stateOf(A) != 0)
struct SharedState<::Kvasir::Tag::User, Address<A, Z, O, T, M>> {
    static constexpr unsigned value = Rp::SioSeam::stateOf(A);

    // the rest of apply()'s message (Register/Diagnostic.hpp OneStateTwoRegisters): the pins and the fix. Short: the
    // whole message is cut at Kvasir::Diagnostic::Text's 768 characters (about 300 are the SDK's, 330 these)
    static consteval void describe(::Kvasir::Diagnostic::Text& t,
                                   unsigned                    mask) {
        namespace S = Rp::SioSeam;
        t << "RP SIO ";
        S::pinNames(t, value, mask);
        if(S::isLevel(value)) {
            t << ", its output level: makeOutput(pin) and clear(pin) write it low (";
            S::registerName(t, value, "_CLR");
            t << "), makeOutputInitHigh(pin) and set(pin) high (";
            S::registerName(t, value, "_SET");
            t << "), toggle(pin) flips it (";
            S::registerName(t, value, "_XOR");
            t << "). Give it once: makeOutputInitHigh(pin) for high, makeOutput(pin) for low - or "
                 "drive the pin in an apply() of its own";
        } else {
            t << ", its output enable: makeOutput(pin) and a peripheral pin set it (";
            S::registerName(t, value, "_SET");
            t << "), makeInput(pin) clears it (";
            S::registerName(t, value, "_CLR");
            t << "). Configure the pin once per list";
        }
    }
};
}   // namespace Kvasir::Register
