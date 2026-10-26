#pragma once
// The RP2350's boot ROM services under their public names: partition table, address translation, OTP access,
// BOOT_INFO and explicit_buy (TBYB).
#include "Rom.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Bootrom {
#ifdef KVASIR_CHIP_RP2350
    // get_partition_table_info flags (RP2350 datasheet 5.4.8.16): what the words after the
    // echoed flags word describe.
    namespace PtInfoFlags {
        constexpr std::uint32_t ptInfo           = 0x0001;   // count, unpartitioned space
        constexpr std::uint32_t singlePartition  = 0x8000;   // just partition (flags >> 24)
        constexpr std::uint32_t locationAndFlags = 0x0010;
        constexpr std::uint32_t id               = 0x0020;
        constexpr std::uint32_t familyIds        = 0x0040;
        constexpr std::uint32_t name             = 0x0080;
    }   // namespace PtInfoFlags

    // The word a PT_INFO query returns after the echoed flags: the partition count in the
    // low byte, and whether a partition table exists at all.
    namespace PtInfo {
        constexpr std::uint32_t partitionCountMask = 0x00FF;
        constexpr std::uint32_t hasPartitionTable  = 0x0100;
    }   // namespace PtInfo

    // otp_access (5.4.8.19): rows are 24 bits wide; read or written as raw 32-bit words
    // (four bytes per row, ECC off) or as ECC-protected 16-bit halves (two bytes per row).
    namespace OtpCmd {
        constexpr std::uint32_t rowMask = 0x0000FFFF;
        constexpr std::uint32_t write   = 0x00010000;
        constexpr std::uint32_t ecc     = 0x00020000;
    }   // namespace OtpCmd

    /// get_partition_table_info (RP2350 datasheet 5.4.8.16): fills `out_buffer` with the
    /// echoed flags word and what PtInfoFlags asks for; returns the word count or a
    /// negative bootrom error.
    [[nodiscard]] inline int getPartitionTableInfo(std::uint32_t* outBuffer,
                                                   std::uint32_t  outBufferWordSize,
                                                   std::uint32_t  partitionAndFlags) {
        return RomFunctions::getPartitionTableInfo(outBuffer, outBufferWordSize, partitionAndFlags);
    }

    /// flash_runtime_to_storage_addr (5.4.8.13): the QMI's address translation of a runtime
    /// (XIP window) address; negative when the address is not mapped. The answer is in the XIP
    /// space too, not a byte offset: flash byte 0 is 0x10000000, chip select 1 0x11000000
    /// (measured on an RP2350 without a partition table; the data sheet does not say).
    [[nodiscard]] inline int flashRuntimeToStorageAddr(std::uint32_t addr) {
        return RomFunctions::flashRuntimeToStorageAddr(addr);
    }

    /// otp_access (5.4.8.19): read or write `buf_len` bytes of OTP from the row in
    /// `cmd_flags` (OtpCmd::ROW_MASK, WRITE, ECC); 0 on success.
    [[nodiscard]] inline int otpAccess(std::uint8_t* buf,
                                       std::uint32_t bufLen,
                                       std::uint32_t cmdFlags) {
        return RomFunctions::otpAccess(buf, bufLen, cmdFlags);
    }

    // OTP row numbers (RP2350 datasheet 13.6, the OTP row table): CHIPID0..3 are rows
    // 0x000..0x003 (ECC, read together they are the 64-bit chip id); the rows from 0x400
    // (page 16) on are not assigned by the datasheet and are the user area.
    namespace Otp::Rows {
        constexpr std::uint32_t chipId   = 0x000;
        constexpr std::uint32_t userBase = 0x400;
    }   // namespace Otp::Rows

    /// How the boot ROM booted this run: get_sys_info BOOT_INFO (RP2350 datasheet 5.4.8.17, flag
    /// 0x0040, word 0 = 0xttppbbdd). The flag and boot type values are pico-sdk's
    /// boot/bootrom_constants.h (BOOT_TBYB_AND_UPDATE_FLAG_*, BOOT_TYPE_*); the data sheet names
    /// the fields only.
    struct BootInfo {
        static constexpr std::uint8_t buyPending = 0x1;   // a TBYB image launched, not bought yet
        static constexpr std::uint8_t otpVersionApplied = 0x2;
        static constexpr std::uint8_t otherErased       = 0x4;
        static constexpr std::uint8_t noPartition       = 0xff;   // pp -1: none (a board without a
                                                                  // partition table reads 0xfe, -2)

        static constexpr std::uint8_t typeNormal      = 0;
        static constexpr std::uint8_t typeBootsel     = 2;
        static constexpr std::uint8_t typeRamImage    = 3;
        static constexpr std::uint8_t typeFlashUpdate = 4;
        static constexpr std::uint8_t typeChainedFlag
          = 0x80;   // or'ed in when an image chained another

        bool          valid{};   // the boot ROM answered
        std::uint8_t  tbybAndUpdate{};
        std::uint8_t  partition{noPartition};
        std::uint8_t  bootType{};
        std::uint32_t diagnostic{};

        /// tt's BUY_PENDING alone. tt is "updated on regular non BOOTSEL boots" only, so after a
        /// boot through BOOTSEL it is the flag of an earlier boot - an image bought since still reads it
        /// (seen on hardware). trialBoot() is the check to act on.
        [[nodiscard]] constexpr bool isBuyPending() const {
            return valid && (tbybAndUpdate & buyPending) != 0;
        }

        /// This run is a TBYB image on trial: BUY_PENDING on a FLASH_UPDATE boot (chained or not), the
        /// only boot type that launches an unbought TBYB image over the other partition (5.1.17 step
        /// 4). pico-sdk's rom_pick_ab_partition_during_update makes the same two checks.
        [[nodiscard]] constexpr bool trialBoot() const {
            return isBuyPending() && (bootType & ~typeChainedFlag) == typeFlashUpdate;
        }
    };

    /// BOOT_INFO of this boot. Read it early: tt and pp are "updated on regular non BOOTSEL
    /// boots", and the boot ROM's later calls do not change them. A boot ROM call: on an image a
    /// debugger started in RAM only once the RCP salt is valid (chip_rp2350 StartUp.hpp).
    [[nodiscard]] inline BootInfo bootInfo() {
        constexpr std::uint32_t      flag = 0x0040;   // SYS_INFO_BOOT_INFO
        std::array<std::uint32_t, 5> buffer{};        // the supported-flags word, then the 4
        auto const words = RomFunctions::getSysInfo(buffer.data(), buffer.size(), flag);
        if(words < 5 || (buffer[0] & flag) == 0) { return BootInfo{}; }
        return BootInfo{.valid         = true,
                        .tbybAndUpdate = static_cast<std::uint8_t>(buffer[1] >> 24),
                        .partition     = static_cast<std::uint8_t>(buffer[1] >> 16),
                        .bootType      = static_cast<std::uint8_t>(buffer[1] >> 8),
                        .diagnostic    = buffer[2]};
    }

    /// The scratch explicit_buy needs to rewrite the sector holding the TBYB flag (5.4.8.4: "at
    /// least 4 kB", word aligned).
    inline constexpr std::size_t explicitBuyScratchBytes = 4096;

    /// explicit_buy (RP2350 datasheet 5.4.8.4, 5.1.17): make the image a flash update boot
    /// launched on trial (an IMAGE_DEF with the TBYB flag) the one that boots from now on. It
    /// clears the flag in flash and, if the image is a version downgrade, erases the first sector
    /// of the other A/B partition; until then any reset that is not that flash update boot
    /// starts the other partition again. On a secured chip it may also write the rollback
    /// version to OTP, and may reboot doing so (5.4.8.4). Erases and programs flash: interrupts
    /// masked and core 1 off flash around it, as for flashErase() (pico-sdk runs it under
    /// flash_safe_execute). 0 (BOOTROM_OK) or a negative boot ROM error; a call on a bought image
    /// does nothing and returns 0.
    [[nodiscard]] inline int explicitBuy(std::span<std::uint32_t,
                                                   explicitBuyScratchBytes / 4> scratch) {
        return RomFunctions::explicitBuy(reinterpret_cast<std::uint8_t*>(scratch.data()),
                                         static_cast<std::uint32_t>(scratch.size_bytes()));
    }
#endif
}}   // namespace Kvasir::Bootrom
