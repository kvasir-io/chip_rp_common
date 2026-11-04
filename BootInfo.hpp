#pragma once
// The RP2350's boot ROM services under their public names: partition table, address translation, OTP access,
// BOOT_INFO and explicit_buy (TBYB), each under the boot lock the boot ROM checks for it.
#include "FlashOps.hpp"   // explicitBuy(): recorded as a flash operation (Flash::OpInFlight::Kind::buy)
#include "Rom.hpp"
#include "XipControl.hpp"   // explicitBuy(): the ROM's flash writes need XIP off

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#ifdef KVASIR_CHIP_RP2350
    #include "chip/Blocks.hpp"   // the chip's block table (Rp::Block)
    #include "peripherals/BOOTRAM.hpp"
    #include "peripherals/SHA256.hpp"

    #include <kvasir/Register/Register.hpp>
#endif

namespace Kvasir {
namespace Bootrom {
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

    /// The boot locks (RP2350 datasheet 5.4.4; BOOTRAM BOOTLOCK0..7, 4.3 Table 438). With lock checking on (lock 7
    /// claimed), a boot ROM function that uses one of these resources returns BOOTROM_ERROR_LOCK_REQUIRED (-19) unless
    /// its lock is claimed too (pico-bootrom inline_s_lock_check: SHA-256 for get_partition_table_info,
    /// load_partition_table and the other partition scans, OTP for otp_access, FLASH_OP for the checked flash_op).
    /// Kvasir never turns checking on, but a boot stage before it may have and left it so - a pico-sdk bootloader
    /// (picotool's decryption stage) does, and the image it chains into runs with it.
    enum class BootLock : std::uint8_t {
        sha256  = 0,
        flashOp = 1,
        otp     = 2,
        enable  = 7,   ///< claimed = the boot ROM checks the other locks
    };

}   // namespace Bootrom

namespace Detail {
    using BootRam = Kvasir::Peripheral::BOOTRAM::Registers<>;

    template<Bootrom::BootLock Lock>
    constexpr auto bootLockField() {
        if constexpr(Lock == Bootrom::BootLock::sha256) {
            return BootRam::BOOTLOCK0::bootlock0;
        } else if constexpr(Lock == Bootrom::BootLock::flashOp) {
            return BootRam::BOOTLOCK1::bootlock1;
        } else if constexpr(Lock == Bootrom::BootLock::otp) {
            return BootRam::BOOTLOCK2::bootlock2;
        } else {
            static_assert(Lock == Bootrom::BootLock::enable);
            return BootRam::BOOTLOCK7::bootlock7;
        }
    }

    // The boot ROM hashes on the SHA-256 block (pico-bootrom's sweet-b, its RP2350 path writes sha256_hw) - the
    // partition table's words at every get_partition_table_info, which compares them with the hash it took at
    // boot. Kvasir's first init step holds every block nothing claims in reset, and a block held in reset reads
    // its reset values (CSR 0x1206, WDATA_RDY set): the ROM's loop runs through without an engine and the digest
    // is wrong - get_partition_table_info answered BOOTROM_ERROR_MODIFIED_DATA (-15) on every partitioned boot
    // (measured, RESETS.RESET bit 17 set). Released here, before the ROM calls that hash.
    inline void sha256OutOfReset() {
        using Sha = Kvasir::Peripheral::SHA256::Registers<>;
        if(apply(read(Rp::Block<Sha>::reset)) != 0) {
            apply(clear(Rp::Block<Sha>::reset));
            Rp::Block<Sha>::waitResetDone();
        }
    }
}   // namespace Detail

namespace Bootrom {

    /// Whether the boot ROM checks the boot locks now: lock 7 claimed (BOOTLOCK_STAT reads 1 for unclaimed, 4.3).
    [[nodiscard]] inline bool bootLockCheckingOn() {
        auto const stat = get<0>(apply(read(Detail::BootRam::BOOTLOCK_STAT::bootlock_stat)));
        return (stat & (1U << static_cast<unsigned>(BootLock::enable))) == 0U;
    }

    /// Claims a boot lock for its lifetime, if it can (`held()`): a BOOTLOCKn read claims and answers non-zero, a
    /// write releases (4.3 Table 438). One try, no waiting - a lock someone else holds is reported, the ROM call then
    /// answers LOCK_REQUIRED if checking is on, and is not released here. With checking off it costs a read and a
    /// write. Not reentrant: a nested guard of the same lock gets held() == false and leaves the release to the outer.
    template<BootLock Lock>
    class BootLockGuard {
    public:
        BootLockGuard() : held_{get<0>(apply(read(Detail::bootLockField<Lock>()))) != 0U} {}

        ~BootLockGuard() {
            if(held_) {
                apply(write(Detail::bootLockField<Lock>(), Kvasir::Register::value<0U>()));
            }
        }

        BootLockGuard(BootLockGuard const&)            = delete;
        BootLockGuard& operator=(BootLockGuard const&) = delete;
        BootLockGuard(BootLockGuard&&)                 = delete;
        BootLockGuard& operator=(BootLockGuard&&)      = delete;

        [[nodiscard]] bool held() const { return held_; }

    private:
        bool held_;
    };

    /// get_partition_table_info (RP2350 datasheet 5.4.8.16): fills `out_buffer` with the
    /// echoed flags word and what PtInfoFlags asks for; returns the word count or a
    /// negative bootrom error (BOOTROM_ERROR_PRECONDITION_NOT_MET, -14, when the table is not
    /// loaded: a RAM or watchdog boot - loadPartitionTable() first). Under the SHA-256 boot lock,
    /// with the SHA-256 block out of reset (Detail::sha256OutOfReset).
    [[nodiscard]] inline int getPartitionTableInfo(std::uint32_t* outBuffer,
                                                   std::uint32_t  outBufferWordSize,
                                                   std::uint32_t  partitionAndFlags) {
        BootLockGuard<BootLock::sha256> const lock{};
        Detail::sha256OutOfReset();
        return RomFunctions::getPartitionTableInfo(outBuffer, outBufferWordSize, partitionAndFlags);
    }

    /// The work area load_partition_table and the other partition scans need: the boot ROM checks it against
    /// WORKAREA_CHECK_SIZE = MAX(sizeof(scan_workarea_t), sizeof(uf2_target_workarea_t)) = 3264 (pico-bootrom
    /// varm_apis.c, the same in A2, A3 and A4; pico-sdk: "currently required is 3264"). The datasheet's 3064 (5.4.8.20)
    /// is sizeof(scan_workarea_t) alone: the ROM answers BOOTROM_ERROR_BUFFER_TOO_SMALL to that.
    inline constexpr std::size_t partitionWorkareaBytes = 3264;

}   // namespace Bootrom

namespace Detail {
    // load_partition_table rescans the flash the way a boot does, minus the boot: it sets the QMI to serial 03h at
    // BOOTROM_SPI_CLKDIV_NSBOOT (6) itself (s_varm_crit_ram_trash_perform_flash_scan_and_maybe_run_image ->
    // flash_select_xip_read_mode, pico-bootrom varm_flash_boot.c), without taking a flash in continuous read
    // (the image's EBh, XipControl.hpp) out of it first, and leaves that mode behind. Under the XipGuard the flash
    // leaves continuous mode before and the image's mode comes back after. RAM code for the same reason.
    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline int
    loadPartitionTableImpl(Kvasir::Flash::XipDisabler& xipDisabler,
                           int (*load)(std::uint8_t*,
                                       std::uint32_t,
                                       bool),
                           std::uint8_t* workarea,
                           std::uint32_t workareaSize,
                           bool          forceReload) {
        KVASIR_RAM_FUNC_MARK();
        Kvasir::Flash::XipGuard const guard{xipDisabler};
        return load(workarea, workareaSize, forceReload);
    }
}   // namespace Detail

namespace Bootrom {

    /// load_partition_table (RP2350 datasheet 5.4.8.20): reads the partition table from flash into the boot ROM's
    /// state, which a RAM or watchdog boot leaves empty, so getPartitionTableInfo() answers; `forceReload` reads it
    /// again after it changed in flash (without it, a loaded table is kept and the call returns 0 at once). 0 or a
    /// negative boot ROM error. The scan reads flash in serial 03h at clk_sys / 6 (25 MHz at 150 MHz, inside 03h's
    /// rating up to 300 MHz), with XIP off around it (Detail::loadPartitionTableImpl): interrupts masked and core 1 off
    /// flash, as for flashErase(). Under the SHA-256 boot lock.
    [[nodiscard]] inline int loadPartitionTable(std::span<std::uint32_t,
                                                          partitionWorkareaBytes / 4> workarea,
                                                bool forceReload = false) {
        BootLockGuard<BootLock::sha256> const lock{};
        Detail::sha256OutOfReset();
        auto const load = RomFunctions::
          getRomFunctionPointer<'L', 'P', int (*)(std::uint8_t*, std::uint32_t, bool)>();
        Kvasir::Flash::XipDisabler xipDisabler{};
        return Detail::loadPartitionTableImpl(xipDisabler,
                                              load,
                                              reinterpret_cast<std::uint8_t*>(workarea.data()),
                                              static_cast<std::uint32_t>(workarea.size_bytes()),
                                              forceReload);
    }

    /// flash_runtime_to_storage_addr (5.4.8.13): the QMI's address translation of a runtime
    /// (XIP window) address; negative when the address is not mapped. The answer is in the XIP
    /// space too, not a byte offset: flash byte 0 is 0x10000000, chip select 1 0x11000000
    /// (measured on an RP2350 without a partition table; the data sheet does not say).
    [[nodiscard]] inline int flashRuntimeToStorageAddr(std::uint32_t addr) {
        return RomFunctions::flashRuntimeToStorageAddr(addr);
    }

    /// otp_access (5.4.8.19): read or write `buf_len` bytes of OTP from the row in
    /// `cmd_flags` (OtpCmd::ROW_MASK, WRITE, ECC); 0 on success. Under the OTP boot lock.
    [[nodiscard]] inline int otpAccess(std::uint8_t* buf,
                                       std::uint32_t bufLen,
                                       std::uint32_t cmdFlags) {
        BootLockGuard<BootLock::otp> const lock{};
        return RomFunctions::otpAccess(buf, bufLen, cmdFlags);
    }

    // OTP row numbers (RP2350 datasheet 13.6, the OTP row table): CHIPID0..3 are rows
    // 0x000..0x003 (ECC, read together they are the 64-bit chip id); the rows from 0x400
    // (page 16) on are not assigned by the datasheet and are the user area.
    namespace Otp::Rows {
        constexpr std::uint32_t chipId   = 0x000;
        constexpr std::uint32_t userBase = 0x400;
    }   // namespace Otp::Rows

    /// BOOT_DIAGNOSTIC's bits (RP2350 datasheet 5.4.8.17 "Boot Diagnostic", md l.20339-20407; the names pico-sdk's
    /// boot/bootrom_constants.h BOOT_DIAGNOSTIC_*): one 16-bit set per region of the diagnosed pair - slot 0 and slot 1,
    /// or an A partition and its B -, how far the boot ROM's scan got with each.
    namespace Diagnostic {
        inline constexpr std::uint16_t windowSearched
          = 0x0001;   // the data sheet's REGION_SEARCHED
        inline constexpr std::uint16_t invalidBlockLoop
          = 0x0002;   // with validBlockLoop: a table that failed
                      // its full parse after its hash held
        inline constexpr std::uint16_t validBlockLoop        = 0x0004;
        inline constexpr std::uint16_t validImageDef         = 0x0008;
        inline constexpr std::uint16_t hasPartitionTable     = 0x0010;
        inline constexpr std::uint16_t considered            = 0x0020;
        inline constexpr std::uint16_t chosen                = 0x0040;
        inline constexpr std::uint16_t ptMatchingKey         = 0x0080;
        inline constexpr std::uint16_t ptHashForVerify       = 0x0100;
        inline constexpr std::uint16_t ptVerifiedOk          = 0x0200;
        inline constexpr std::uint16_t imageDefMatchingKey   = 0x0400;
        inline constexpr std::uint16_t imageDefHashForVerify = 0x0800;
        inline constexpr std::uint16_t imageDefVerifiedOk    = 0x1000;
        inline constexpr std::uint16_t loadMapEntriesLoaded  = 0x2000;
        inline constexpr std::uint16_t imageLaunched         = 0x4000;
        inline constexpr std::uint16_t imageConditionFailure = 0x8000;
    }   // namespace Diagnostic

    /// The boot diagnostic's "partition" numbers below 0 (5.4.8.17, md l.20323-20333; pico-sdk BOOT_PARTITION_*): what
    /// BootInfo::diagnosticPartition reads and Bootrom::rebootFlashUpdate(windowBase, diagnosticPartition) takes.
    /// 0..15 is a partition: for an A/B pair, its A.
    namespace DiagnosticPartition {
        inline constexpr std::int8_t none = -1;
        inline constexpr std::int8_t slot0
          = -2;   // both slots: slot 0 in the low half, slot 1 in the high half
        inline constexpr std::int8_t slot1 = -3;
        inline constexpr std::int8_t image
          = -4;   // a RAM or OTP image's launch, or a chain_image()
    }   // namespace DiagnosticPartition

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

        bool         valid{};   // the boot ROM answered
        std::uint8_t tbybAndUpdate{};
        std::uint8_t partition{noPartition};
        std::uint8_t bootType{};
        /// dd: the region `diagnostic` is about (DiagnosticPartition, or a partition number; pico-bootrom bootram
        /// always.diagnostic_partition_index). For an A/B pair it is the A partition: `diagnostic` holds A's flags in
        /// its low half, B's in its high half. Set from a regular reboot's p0 (Bootrom::rebootFlashUpdate(windowBase,
        /// diagnosticPartition) sets it for a flash update boot), to `image` by a RAM or OTP image's launch and by a
        /// chain_image() (pico-bootrom varm_launch_image.c), and kept by every reset but a cold one (the RUN pin, a
        /// POWMAN reset: 5.4.8.17), after which the boot ROM starts at slot0 (varm_boot_path.c).
        std::int8_t diagnosticPartition{DiagnosticPartition::none};
        /// BOOT_DIAGNOSTIC: what this boot's scan found for the diagnostic region (Diagnostic:: bits, per half).
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

        /// `diagnostic`'s halves: slot 0 or the A partition (low), slot 1 or the B partition (high).
        [[nodiscard]] constexpr std::uint16_t diagnosticA() const {
            return static_cast<std::uint16_t>(diagnostic & 0xffffU);
        }

        [[nodiscard]] constexpr std::uint16_t diagnosticB() const {
            return static_cast<std::uint16_t>(diagnostic >> 16);
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
        return BootInfo{.valid               = true,
                        .tbybAndUpdate       = static_cast<std::uint8_t>(buffer[1] >> 24),
                        .partition           = static_cast<std::uint8_t>(buffer[1] >> 16),
                        .bootType            = static_cast<std::uint8_t>(buffer[1] >> 8),
                        .diagnosticPartition = static_cast<std::int8_t>(buffer[1] & 0xffU),
                        .diagnostic          = buffer[2]};
    }

    /// The scratch explicit_buy needs to rewrite the sector holding the TBYB flag (5.4.8.4: "at
    /// least 4 kB", word aligned).
    inline constexpr std::size_t explicitBuyScratchBytes = 4096;

}   // namespace Bootrom

namespace Detail {
    // explicit_buy reads the TBYB flag's sector through the XIP window (s_varm_crit_flash_read_data: the
    // uncached untranslated alias, in whatever mode the QMI runs) and erases and programs it in direct mode
    // without leaving XIP first (pico-bootrom varm_launch_image.c s_varm_api_explicit_buy, varm_generic_flash.c).
    // A flash in continuous read (the EBh mode an image runs, XipControl.hpp) takes those commands for address
    // bits. Under the XipGuard the flash is out of continuous mode and the window reads 03h at the boot ROM's own
    // divider (flash_exit_xip -> flash_init_spi), the state the ROM's own buy at boot runs in; afterwards the
    // image's mode comes back. RAM code: the flash is not readable during the ROM's erase and program.
    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline int
    explicitBuyImpl(Kvasir::Flash::XipDisabler& xipDisabler,
                    int (*buy)(std::uint8_t*,
                               std::uint32_t),
                    std::uint8_t* buffer,
                    std::uint32_t bufferSize) {
        KVASIR_RAM_FUNC_MARK();
        Kvasir::Flash::XipGuard const guard{xipDisabler};
        return buy(buffer, bufferSize);
    }
}   // namespace Detail

namespace Bootrom {

    /// explicit_buy (RP2350 datasheet 5.4.8.4, 5.1.17): make the image a flash update boot
    /// launched on trial (an IMAGE_DEF with the TBYB flag) the one that boots from now on. It
    /// clears the flag in flash and, if the image is a version downgrade (the other A/B partition's
    /// IMAGE_DEF has the greater version), erases the other partition's first sector - only then:
    /// with equal versions the other image stays bootable and partition A wins a normal boot
    /// (pico-bootrom varm_flash_boot.c, version_downgrade_erase_flash_addr; 5.1.17's "the first 4 kB
    /// sector of the other partition is erased" is the downgrade case). Until the buy any reset that
    /// is not that flash update boot starts the other partition again. On a secured chip it may also
    /// write the rollback version to OTP, and may reboot doing so (5.4.8.4). Erases and programs
    /// flash with XIP off (Detail::explicitBuyImpl): interrupts masked and core 1 off flash around
    /// it, as for flashErase(). 0 (BOOTROM_OK) or a negative boot ROM error; a call on a bought
    /// image does nothing and returns 0. Under the OTP boot lock: the rollback version goes
    /// through the checked otp_access (pico-bootrom varm_launch_image.c); its flash writes are
    /// unchecked. Recorded and timed like flashErase() / flashWrite() (FlashOps.hpp,
    /// Flash::OpInFlight::Kind::buy, offset and size 0: the boot ROM finds the sector itself).
    [[nodiscard]] inline int explicitBuy(std::span<std::uint32_t,
                                                   explicitBuyScratchBytes / 4> scratch) {
        BootLockGuard<BootLock::otp> const lock{};
        auto const                         buy
          = RomFunctions::getRomFunctionPointer<'E', 'B', int (*)(std::uint8_t*, std::uint32_t)>();
        auto const startUs
          = Kvasir::Flash::Detail::beginOp(Kvasir::Flash::OpInFlight::Kind::buy, 0, 0);
        int result = 0;
        {
            Kvasir::Flash::XipDisabler xipDisabler{};
            result = Detail::explicitBuyImpl(xipDisabler,
                                             buy,
                                             reinterpret_cast<std::uint8_t*>(scratch.data()),
                                             static_cast<std::uint32_t>(scratch.size_bytes()));
        }
        Kvasir::Flash::Detail::endOp(startUs);
        return result;
    }
#endif
}
}   // namespace Kvasir
