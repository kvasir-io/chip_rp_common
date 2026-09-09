#pragma once
#if !__has_include("chip/rp2350.hpp")
    #error \
      "the QMI exists on the RP2350 only; the RP2040 has the SSI (flash_do_cmd in bootrom_functions.hpp)"
#endif
#include "bootrom_functions.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/attributes.hpp"
#include "peripherals/QMI.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

// The QMI's direct mode (RP2350 datasheet 12.14.5): the flash controller as a plain SPI
// master, for commands the memory-mapped path cannot express: the JEDEC id, the unique
// id, the status registers, SFDP. While direct mode is on the XIP path is off, so the
// code that runs it lives in RAM and the caller keeps interrupts away (the ISRs would
// fetch from flash): Kvasir::Qmi::direct() does both.
//
//   std::array<std::byte, 4> tx{std::byte{0x9F}}, rx{};
//   Kvasir::Qmi::direct(tx, rx);          // rx[1..3]: the JEDEC id
//
// Full duplex like any SPI: byte i of `rx` is what arrived while byte i of `tx` went out.
namespace Kvasir { namespace Qmi {

    using Regs = Kvasir::Peripheral::QMI::Registers<0>;

    namespace detail {
        // In RAM, interrupts off: the transaction on chip select 0 at clk_sys / clkdiv.
        [[KVASIR_RAM_FUNC_ATTRIBUTES]] inline void transfer(std::span<std::byte const> tx,
                                                            std::span<std::byte>       rx,
                                                            std::uint32_t              clkdiv) {
            Kvasir::detail::FlashXipDisabler xip{};
            {
                Kvasir::detail::XipGuard guard{xip};
                // Direct mode on, CS0 asserted by hand, the divider (2..255 of clk_sys).
                apply(write(Regs::DIRECT_CSR::FULLREGISTER,
                            (clkdiv << 22) | (1U << 2) | 1U));   // clkdiv, assert_cs0n, en
                std::size_t sent = 0;
                std::size_t got  = 0;
                auto const  n    = tx.size();
                while(got < n) {
                    auto const csr     = get<0>(apply(read(Regs::DIRECT_CSR::FULLREGISTER)));
                    bool const txFull  = (csr & (1U << 10)) != 0;
                    bool const rxEmpty = (csr & (1U << 16)) != 0;
                    if(sent < n && !txFull && sent - got < 4) {
                        // One byte: 8-bit data, single-width, output enabled.
                        apply(write(Regs::DIRECT_TX::FULLREGISTER,
                                    (1U << 19) | static_cast<std::uint32_t>(tx[sent])));
                        ++sent;
                    }
                    if(!rxEmpty) {
                        auto const v = get<0>(apply(read(Regs::DIRECT_RX::FULLREGISTER)));
                        if(got < rx.size()) { rx[got] = static_cast<std::byte>(v & 0xFFU); }
                        ++got;
                    }
                }
                while((get<0>(apply(read(Regs::DIRECT_CSR::FULLREGISTER))) & (1U << 1)) != 0) {
                }   // busy
                apply(write(Regs::DIRECT_CSR::FULLREGISTER, 0U));   // CS up, direct mode off
            }
        }
    }   // namespace detail

    /// One transaction on the flash's chip select: the first `rx.size()` bytes that arrive
    /// are stored (byte i is what came in while byte i of `tx` went out), the rest dropped.
    inline void direct(std::span<std::byte const> tx,
                       std::span<std::byte>       rx,
                       std::uint32_t              clkdiv = 6) {
        Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> guard{};
        detail::transfer(tx, rx, clkdiv < 2 ? 2U : (clkdiv > 255 ? 255U : clkdiv));
    }

    struct JedecId {
        std::uint8_t manufacturer{};
        std::uint8_t type{};
        std::uint8_t capacity{};   ///< log2 bytes on most parts

        [[nodiscard]] constexpr std::uint32_t bytes() const {
            return capacity >= 0x10 && capacity <= 0x20 ? (1U << capacity) : 0U;
        }
    };

    [[nodiscard]] inline JedecId jedecId() {
        std::byte tx[4]{std::byte{0x9F}};
        std::byte rx[4]{};
        direct(tx, rx);
        return {static_cast<std::uint8_t>(rx[1]),
                static_cast<std::uint8_t>(rx[2]),
                static_cast<std::uint8_t>(rx[3])};
    }

    /// The 64-bit unique id most Winbond and compatible parts answer to 0x4B (4 dummy bytes).
    [[nodiscard]] inline std::uint64_t uniqueId() {
        std::byte tx[13]{std::byte{0x4B}};
        std::byte rx[13]{};
        direct(tx, rx);
        std::uint64_t id = 0;
        for(std::size_t i = 5; i < 13; ++i) { id = (id << 8) | static_cast<std::uint8_t>(rx[i]); }
        return id;
    }

    /// Status register 1 (0x05): bit 0 busy, bit 1 write-enable latch, the protection bits.
    [[nodiscard]] inline std::uint8_t status() {
        std::byte tx[2]{std::byte{0x05}};
        std::byte rx[2]{};
        direct(tx, rx);
        return static_cast<std::uint8_t>(rx[1]);
    }

}}   // namespace Kvasir::Qmi
