#pragma once
// The RP2350's OTP one row at a time, as Secure code on the chip reads and writes it (RP2350 datasheet 13): through the
// boot ROM's otp_access (5.4.8.21; Bootrom::otpAccess, under the OTP boot lock), which applies the page locks - a row
// the caller may not see is a refused call, not a bus fault -, the lock rows also through the raw read window, and the
// OTP block's soft locks (SW_LOCK0..63). Where they differ:
// - otp_access judges a lock row (pages 62-63) by the soft lock of the page that row controls on an A3/A4 ROM
//   (varm_otp.c:352; the A2 ROM by page 62/63's own): once a stage before the image has soft-locked a page, that page's
//   LOCK1 row no longer reads through it. The raw window (OTP_DATA_RAW, row n at its base + 4 n, the 24 bits in the low
//   bits) is under page 62/63's own locks, and there the lock rows are always readable (pico-sdk otp_data.h):
//   readLockRowsByWindow().
// - while a row is programmed the memory-mapped windows (OTP_DATA, OTP_DATA_RAW) answer with bus errors (13.1:
//   USR.DCTRL is clear meanwhile): the writes here mask interrupts around the ROM call, and nothing else may read a
//   window meanwhile (core 1 neither).
// - a write never clears a bit: the ROM refuses a word that would (UNSUPPORTED_MODIFICATION; NOT_PERMITTED for a lock
//   the caller is under), with what it wrote before already written; an ECC write stores the row inverted (bits 23:22)
//   where the plain codeword does not fit over what the row holds. Nothing is read back here: that is the caller's.
// - a soft lock only tightens: SW_LOCKn ORs what is written into what it holds, until the OTP block's next reset.
#include "BootInfo.hpp"
#include "core/InterruptMask.hpp"
#include "kvasir/Register/Memory.hpp"
#include "kvasir/Register/Register.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#ifdef KVASIR_CHIP_RP2350
    #include "peripherals/OTP.hpp"
    #include "peripherals/OTP_DATA_RAW.hpp"
#endif

namespace Kvasir { namespace Otp {
#ifdef KVASIR_CHIP_RP2350
    /// A row's bits.
    inline constexpr std::uint32_t rowMask = 0x00FF'FFFFU;
    /// The first lock row: PAGE0_LOCK0, page 62's first row (PAGEn_LOCK0 at 0xf80 + 2n, PAGEn_LOCK1 right after it).
    inline constexpr std::uint16_t lockRowsFirst = 0x0F80U;
    /// The raw read window, OTP_DATA_RAW.
    inline constexpr std::uintptr_t rawWindowBase
      = Kvasir::Peripheral::OTP_DATA_RAW::Registers<>::baseAddr;
    static_assert(rawWindowBase == 0x4013'4000U,
                  "OTP_DATA_RAW_BASE (RP2350 datasheet 2.2, the address map)");
    /// otp_access's BOOTROM_ERROR_NOT_PERMITTED (pico-sdk boot/bootrom_constants.h): a lock keeps the caller from
    /// the row. Every other non-zero answer is a failure of another kind.
    inline constexpr int romNotPermitted = -4;

    namespace Detail {
        using Regs = Kvasir::Peripheral::OTP::Registers<>;
        /// SW_LOCK0..63, one register per page in page order (RP2350 datasheet 13.9; one SVD dim), indexed by a page
        /// known at run time.
        using SwLocks = Kvasir::Register::Array<Regs::template SW_LOCK, 64>;
        static_assert(SwLocks::stride == 4U,
                      "OTP SW_LOCKn: one word a page (RP2350 datasheet 13.9, SW_LOCK0..63)");
    }   // namespace Detail

    /// @p row's 24 bits through otp_access; nullopt where the boot ROM refuses it (a lock the caller is under).
    [[nodiscard]] inline std::optional<std::uint32_t> readRaw(std::uint16_t row) {
        alignas(4) std::array<std::uint8_t, 4>
          raw{};   // a raw read: 4 bytes a row, the 24 bits little endian; otp_access refuses a buffer not 4-byte aligned
        if(Bootrom::otpAccess(raw.data(), static_cast<std::uint32_t>(raw.size()), row) != 0) {
            return std::nullopt;
        }
        return std::uint32_t{raw[0]} | (std::uint32_t{raw[1]} << 8U)
             | (std::uint32_t{raw[2]} << 16U);
    }

    /// @p row's 24 bits through the raw read window: under page 62/63's locks, not those of the page it belongs to -
    /// for the lock rows (see the header). A row whose page a lock hides reads all ones there (the unguarded raw alias, 13.1): lock rows only.
    [[nodiscard]] inline std::uint32_t readRawWindow(std::uint16_t row) {
        return Kvasir::Memory::load<std::uint32_t>(rawWindowBase + (std::uintptr_t{row} * 4U))
             & rowMask;
    }

    /// @p row's 24 bits as configuration is read: a lock row (pages 62-63) through the raw window, where it always
    /// reads, any other through otp_access; nullopt where the boot ROM refuses it.
    [[nodiscard]] inline std::optional<std::uint32_t> readLockRowsByWindow(std::uint16_t row) {
        if(row >= lockRowsFirst) { return readRawWindow(row); }
        return readRaw(row);
    }

    /// out.size() / 2 rows from @p row, the 16 data bits of each as the ECC reads them, low byte first, through
    /// otp_access; false where it refuses, for no row, and for a buffer it does not take (not 2-byte aligned).
    [[nodiscard]] inline bool readEcc(std::uint16_t           row,
                                      std::span<std::uint8_t> out) {
        auto const rows = static_cast<std::uint32_t>(out.size() / 2U);
        if(rows == 0) { return false; }
        if((std::bit_cast<std::uintptr_t>(out.data()) & 1U) != 0) { return false; }
        return Bootrom::otpAccess(out.data(), rows * 2U, row | Bootrom::OtpCmd::ecc) == 0;
    }

    /// @p row's 16 data bits as the ECC reads them; nullopt where otp_access refuses.
    [[nodiscard]] inline std::optional<std::uint16_t> readEcc(std::uint16_t row) {
        alignas(4) std::array<std::uint8_t, 2> data{};
        if(!readEcc(row, data)) { return std::nullopt; }
        return static_cast<std::uint16_t>(std::uint32_t{data[0]} | (std::uint32_t{data[1]} << 8U));
    }

    /// Writes @p word's 24 bits raw into @p row, interrupts masked: 0, or otp_access's error (romNotPermitted for a
    /// lock).
    [[nodiscard]] inline int writeRaw(std::uint16_t row,
                                      std::uint32_t word) {
        std::uint32_t const                    bits = word & rowMask;
        alignas(4) std::array<std::uint8_t, 4> raw{static_cast<std::uint8_t>(bits & 0xFFU),
                                                   static_cast<std::uint8_t>((bits >> 8U) & 0xFFU),
                                                   static_cast<std::uint8_t>((bits >> 16U) & 0xFFU),
                                                   0U};
        Kvasir::Core::IrqGuard const           guard{};
        return Bootrom::otpAccess(raw.data(),
                                  static_cast<std::uint32_t>(raw.size()),
                                  row | Bootrom::OtpCmd::write);
    }

    /// Writes @p data's 16 bits into @p row with its ECC, interrupts masked (the ROM stores the row inverted where the
    /// plain codeword does not fit over what it holds): 0, or otp_access's error.
    [[nodiscard]] inline int writeEcc(std::uint16_t row,
                                      std::uint16_t data) {
        alignas(4) std::array<std::uint8_t, 2> bytes{static_cast<std::uint8_t>(data & 0xFFU),
                                                     static_cast<std::uint8_t>(data >> 8U)};
        Kvasir::Core::IrqGuard const           guard{};
        return Bootrom::otpAccess(bytes.data(),
                                  static_cast<std::uint32_t>(bytes.size()),
                                  row | Bootrom::OtpCmd::write | Bootrom::OtpCmd::ecc);
    }

    /// SW_LOCKn.SEC of @p page (0-63) as it is now: 0 read-write, 1 read-only, 3 inaccessible for Secure code.
    [[nodiscard]] inline std::uint8_t softLockSecure(std::size_t page) {
        std::uint32_t const word = Detail::SwLocks::readWhole(static_cast<unsigned>(page));
        return static_cast<std::uint8_t>(
          Kvasir::Register::extract(Detail::Regs::SW_LOCK<0>::sec, word));
    }

    /// Tightens SW_LOCKn of @p page: SEC to @p secure, NSEC to @p nonSecure, each ORed into what it holds by the
    /// hardware (so never looser than it was), until the OTP block's next reset.
    inline void tightenSoftLock(std::size_t  page,
                                std::uint8_t secure,
                                std::uint8_t nonSecure) {
        std::uint32_t const word
          = Kvasir::Register::deposit(Detail::Regs::SW_LOCK<0>::sec, std::uint32_t{secure})
          | Kvasir::Register::deposit(Detail::Regs::SW_LOCK<0>::nsec, std::uint32_t{nonSecure});
        Detail::SwLocks::writeWhole(static_cast<unsigned>(page), word);
    }
#endif
}}   // namespace Kvasir::Otp
