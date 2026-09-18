#pragma once
#include "peripherals/PSM.hpp"
#include "peripherals/WATCHDOG.hpp"

#include <cassert>
#include <cstdint>
#include <functional>
#include <kvasir/Util/StaticString.hpp>
#include <kvasir/Util/attributes.hpp>
#include <string_view>
#if __has_include("peripherals/QMI.hpp")
    #include "peripherals/QMI.hpp"
#endif

namespace Kvasir {
namespace RomFunctions {
    constexpr std::uint32_t lookupCode(char c1,
                                       char c2) {
        return static_cast<std::uint32_t>(c1) | (static_cast<std::uint32_t>(c2) << 8);
    }

    // The bootrom's function lookup differs between the chips:
    //   RP2040: rom_table_lookup(table, code) at 0x18, the function table's address at 0x14
    //   RP2350: rom_table_lookup(code, mask) at 0x16, mask 0x0004 selecting Arm secure code
    // Both addresses are 16-bit pointers into the ROM.
    template<typename F>
    [[KVASIR_RAM_FUNC_INLINE_ATTRIBUTES]] inline F lookupRomFunction(std::uint32_t code) {
#if __has_include("chip/rp2040.hpp")
        using RomTableLookupFunction = F (*)(std::uint16_t const* table, std::uint32_t code);

        auto const lookup = reinterpret_cast<RomTableLookupFunction>(
          static_cast<std::uintptr_t>(*reinterpret_cast<std::uint16_t const*>(0x18U)));
        auto const table = reinterpret_cast<std::uint16_t const*>(
          static_cast<std::uintptr_t>(*reinterpret_cast<std::uint16_t const*>(0x14U)));
        return lookup(table, code);
#else
        static constexpr std::uint32_t ArmSecureFunction = 0x0004;

        using RomTableLookupFunction = F (*)(std::uint32_t code, std::uint32_t mask);

        auto const lookup = reinterpret_cast<RomTableLookupFunction>(
          static_cast<std::uintptr_t>(*reinterpret_cast<std::uint16_t const*>(0x16U)));
        return lookup(code, ArmSecureFunction);
#endif
    }

    template<char C1,
             char C2,
             typename F>
    F getRomFunctionPointer() {
        auto const fp = lookupRomFunction<F>(lookupCode(C1, C2));
        assert(fp != nullptr);
        return fp;
    }

    /// getRomFunctionPointer() for code running from RAM while the flash may not be readable:
    /// the lookup routine is in ROM and nothing here is fetched from flash. No assert (its
    /// failure path is flash code): a null result is the caller's to handle.
    template<char C1,
             char C2,
             typename F>
    [[KVASIR_RAM_FUNC_ATTRIBUTES]] F getRomFunctionPointerFromRam() {
        return lookupRomFunction<F>(lookupCode(C1, C2));
    }

    template<char C1,
             char C2,
             typename F,
             typename... Args>
    std::invoke_result_t<F,
                         Args...>
    call(Args... args) {
        return std::invoke(getRomFunctionPointer<C1, C2, F>(), args...);
    }
}   // namespace RomFunctions

#if __has_include("chip/rp2350.hpp")
enum class BootArch : std::uint8_t { Arm = 0, RiscV = 1 };

enum class GlitchDetectorSens : std::uint8_t {
    Default = 0,
    Low     = 1,
    Medium  = 2,
    High    = 3,
};

struct CriticalFlags {
    bool               secureBootEnabled;
    bool               secureDebugDisable;
    bool               debugDisable;
    BootArch           bootArch;
    bool               glitchDetectorEnabled;
    GlitchDetectorSens glitchDetectorSens;
};

struct OtpPageLocks {
    bool page1;    ///< PAGE1_LOCK1  — critical boot flags page
    bool page2;    ///< PAGE2_LOCK1  — boot key hash page
    bool page29;   ///< PAGE29_LOCK1 — AES key page
    bool page30;   ///< PAGE30_LOCK1 — AES key page
    bool page31;   ///< PAGE31_LOCK1 — IV salt page
};
#endif

namespace detail {
#if __has_include("peripherals/QMI.hpp")
    // The XIP read mode every RP2350 project runs in.
    //
    // The bootrom brings the flash up in EBh quad I/O (command on one line, address, mode
    // bits, dummy and data on four) and leaves the mode bits at 0x00, so every fetch that
    // misses the cache spends 8 SCK cycles on the command byte. With mode bits 0xA0 the
    // flash stays in "continuous read": it keeps answering EBh-shaped transfers without
    // the command byte, 28 -> 20 SCK cycles per random word, as pico-sdk's boot2 does.
    // apply() makes the switch when the QMI is in the bootrom's EBh mode and leaves any
    // other mode alone (a flash the bootrom would not run in quad mode is not forced into
    // it). It also writes the timing, because the bootrom's flash_exit_xip resets M0_TIMING
    // to clkdiv 12.
    //
    // Runs from RAM with interrupts off and the QMI idle: while the format changes,
    // nothing may fetch from flash. peripheryClockInit() calls it once at boot,
    // FlashXipDisabler::enable() after every direct-mode access (the BOOTRAM setup
    // function that re-enables XIP restores the bootrom's mode, mode bits 0x00).
    struct XipReadMode {
        using QMI = Kvasir::Peripheral::QMI::Registers<0>;

        static constexpr std::uint32_t XipNoCacheBase = 0x1400'0000U;
        static constexpr std::uint32_t PrefixLenBit   = 1U << 12;
        // prefix 1-bit 8 bits, address/suffix/dummy/data 4-bit, suffix 8 bits, dummy 16
        // bits (4 quad clocks): what the bootrom programs for EBh
        static constexpr std::uint32_t RfmtQuadEBh        = 0x0004'92A8U;
        static constexpr std::uint32_t CommandEBh         = 0xEBU;
        static constexpr std::uint32_t ContinuousModeBits = 0xA0U;

        // M0_TIMING that reads at clk_ref speed and at the full clock: clkdiv 4, rxdelay 2,
        // min_deselect 2, cooldown 1 - the divider flashInit() uses before the PLL switch
        // with the bootrom's sample delay, i.e. what every boot runs on through the switch
        // until peripheryClockInit(). What a clk_sys resus handler writes before it touches
        // flash: at 12 MHz the full-clock timing's rxdelay samples after the bit is gone.
        static constexpr std::uint32_t TimingAtClkRef = (1U << 30) | (2U << 12) | (2U << 8) | 4U;

        // The resus handler reads it before the flash is readable: never an out-of-line call.
        [[nodiscard,
          gnu::always_inline]] static inline std::uint32_t
        currentTiming() {
            return *reinterpret_cast<std::uint32_t const volatile*>(QMI::M0_TIMING::Addr::value);
        }

        [[nodiscard]] static bool isQuadEBh(std::uint32_t rfmt,
                                            std::uint32_t rcmd) {
            return (rfmt | PrefixLenBit) == RfmtQuadEBh && (rcmd & 0xFFU) == CommandEBh;
        }

        [[nodiscard]] static bool isContinuous(std::uint32_t rfmt,
                                               std::uint32_t rcmd) {
            return isQuadEBh(rfmt, rcmd) && (rfmt & PrefixLenBit) == 0
                && ((rcmd >> 8) & 0xFFU) == ContinuousModeBits;
        }

        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static void apply(std::uint32_t timing) {
            auto* const timingReg
              = reinterpret_cast<std::uint32_t volatile*>(QMI::M0_TIMING::Addr::value);
            auto* const rfmtReg
              = reinterpret_cast<std::uint32_t volatile*>(QMI::M0_RFMT::Addr::value);
            auto* const rcmdReg
              = reinterpret_cast<std::uint32_t volatile*>(QMI::M0_RCMD::Addr::value);

            // The fetch that brought us here may still hold the chip select (cooldown,
            // 64 x COOLDOWN clk_sys cycles): let it expire, then the QMI is idle.
            for(std::uint32_t i = 0; i < 256; ++i) { asm volatile("" ::: "memory"); }
            *timingReg = timing;
            asm volatile("dsb" ::: "memory");

            auto const rfmt = *rfmtReg;
            auto const rcmd = *rcmdReg;
            if(!isQuadEBh(rfmt, rcmd) || isContinuous(rfmt, rcmd)) { return; }

            // The one transfer with the command byte and the continuous mode bits: the
            // flash is in continuous mode after it.
            *rcmdReg = CommandEBh | (ContinuousModeBits << 8);
            *rfmtReg = RfmtQuadEBh;
            asm volatile("dsb" ::: "memory");
            (void)*reinterpret_cast<std::uint32_t const volatile*>(XipNoCacheBase);
            asm volatile("dsb" ::: "memory");
            // Let the chip select cooldown (64 x COOLDOWN clk_sys cycles) run out so the
            // format is changed with the QMI idle.
            for(std::uint32_t i = 0; i < 256; ++i) { asm volatile("" ::: "memory"); }
            *rfmtReg = RfmtQuadEBh & ~PrefixLenBit;   // no command byte from now on
            asm volatile("dsb\n isb" ::: "memory");
        }
    };
#endif

    struct FlashXipDisabler {
        using RomConnectInternalFlash = void (*)(void);
        using RomFlashExitXip         = void (*)(void);
        using RomFlashFlushCache      = void (*)(void);
        using XipEnableFunction       = void (*)(void);

        RomConnectInternalFlash const       connectInternalFlash;
        RomFlashExitXip const               flashExitXip;
        RomFlashFlushCache const            flushCache;
        std::array<std::uint32_t, 64> const xipEnableRamCopy;
        XipEnableFunction const             xipEnable;

#if __has_include("peripherals/QMI.hpp")
        std::uint32_t flashTiming;
#endif

        static std::array<std::uint32_t,
                          64> getXipEnable() {
            if constexpr(PinConfig::CurrentChip == Kvasir::PinConfig::ChipVariant::RP2040) {
#if __has_include("chip/rp2040.hpp")
                return Kvasir::Startup::second_stage_bootloader;
#endif
            } else {
                return *reinterpret_cast<std::array<std::uint32_t, 64> const*>(0x400E0000UL);
            }
        }

        FlashXipDisabler()
          : connectInternalFlash{
              Kvasir::RomFunctions::getRomFunctionPointer<'I',
                                                          'F',
                                                          RomConnectInternalFlash>()}
          , flashExitXip{Kvasir::RomFunctions::getRomFunctionPointer<'E',
                                                                     'X',
                                                                     RomFlashExitXip>()}
          , flushCache{Kvasir::RomFunctions::getRomFunctionPointer<'F',
                                                                   'C',
                                                                   RomFlashFlushCache>()}
          , xipEnableRamCopy{getXipEnable()}
          , xipEnable{reinterpret_cast<XipEnableFunction>(
              const_cast<std::byte*>(std::as_bytes(std::span{xipEnableRamCopy}).data() + 1))} {
#if __has_include("peripherals/QMI.hpp")
            using namespace Kvasir::Peripheral::QMI;
            using QMI   = Registers<0>;
            flashTiming = apply(read(QMI::M0_TIMING::FULLREGISTER));
#endif
        }

        [[KVASIR_RAM_FUNC_ATTRIBUTES]] void disable() {
            connectInternalFlash();
            flashExitXip();
        }

        [[KVASIR_RAM_FUNC_ATTRIBUTES]] void enable() {
            flushCache();
            xipEnable();
#if __has_include("peripherals/QMI.hpp")
            // xipEnable() ran the BOOTRAM setup function: bootrom mode, bootrom timing.
            XipReadMode::apply(flashTiming);
#endif
        }
    };

    struct XipGuard {
        FlashXipDisabler& disabler;

        [[KVASIR_RAM_FUNC_INLINE_ATTRIBUTES]] explicit XipGuard(FlashXipDisabler& d) : disabler(d) {
            disabler.disable();
        }

        [[KVASIR_RAM_FUNC_INLINE_ATTRIBUTES]] ~XipGuard() { disabler.enable(); }

        XipGuard(XipGuard const&)            = delete;
        XipGuard& operator=(XipGuard const&) = delete;
    };

    static inline int get_sys_info(std::uint32_t* out_buffer,
                                   std::uint32_t  out_buffer_word_size,
                                   std::uint32_t  flags) {
        using rom_get_sys_info = int (*)(std::uint32_t* out_buffer,
                                         std::uint32_t  out_buffer_word_size,
                                         std::uint32_t  flags);

        return RomFunctions::call<'G', 'S', rom_get_sys_info>(out_buffer,
                                                              out_buffer_word_size,
                                                              flags);
    }

    static inline int reboot(std::uint32_t flags,
                             std::uint32_t delay_ms,
                             std::uint32_t p0,
                             std::uint32_t p1) {
        using rom_reboot = int (*)(std::uint32_t flags,
                                   std::uint32_t delay_ms,
                                   std::uint32_t p0,
                                   std::uint32_t p1);

        return RomFunctions::call<'R', 'B', rom_reboot>(flags, delay_ms, p0, p1);
    }

#if __has_include("chip/rp2350.hpp")
    static inline int flash_runtime_to_storage_addr(std::uint32_t addr) {
        using rom_flash_runtime_to_storage_addr = int (*)(std::uint32_t addr);

        return RomFunctions::call<'F', 'A', rom_flash_runtime_to_storage_addr>(addr);
    }

    // flash_op flags
    namespace FlashOpFlags {
        // Address translation (select one)
        constexpr std::uint32_t ASPACE_STORAGE = 0x00000000;
        constexpr std::uint32_t ASPACE_RUNTIME = 0x00000001;

        // Security level (select one)
        constexpr std::uint32_t SECLEVEL_SECURE     = 0x00000100;
        constexpr std::uint32_t SECLEVEL_NONSECURE  = 0x00000200;
        constexpr std::uint32_t SECLEVEL_BOOTLOADER = 0x00000300;

        // Operation (select one)
        constexpr std::uint32_t OP_ERASE   = 0x00000000;
        constexpr std::uint32_t OP_PROGRAM = 0x00010000;
        constexpr std::uint32_t OP_READ    = 0x00020000;
    }   // namespace FlashOpFlags

    static inline int flash_op(std::uint32_t flags,
                               std::uint32_t addr,
                               std::uint32_t size_bytes,
                               std::uint8_t* buf) {
        using rom_flash_op = int (*)(std::uint32_t, std::uint32_t, std::uint32_t, std::uint8_t*);

        return RomFunctions::call<'F', 'O', rom_flash_op>(flags, addr, size_bytes, buf);
    }

    // get_partition_table_info flags (RP2350 datasheet 5.4.8.16): what the words after the
    // echoed flags word describe.
    namespace PtInfoFlags {
        constexpr std::uint32_t PT_INFO            = 0x0001;   // count, unpartitioned space
        constexpr std::uint32_t SINGLE_PARTITION   = 0x8000;   // just partition (flags >> 24)
        constexpr std::uint32_t LOCATION_AND_FLAGS = 0x0010;
        constexpr std::uint32_t ID                 = 0x0020;
        constexpr std::uint32_t FAMILY_IDS         = 0x0040;
        constexpr std::uint32_t NAME               = 0x0080;
    }   // namespace PtInfoFlags

    // The word a PT_INFO query returns after the echoed flags: the partition count in the
    // low byte, and whether a partition table exists at all.
    namespace PtInfo {
        constexpr std::uint32_t PartitionCountMask = 0x00FF;
        constexpr std::uint32_t HasPartitionTable  = 0x0100;
    }   // namespace PtInfo

    static inline int get_partition_table_info(std::uint32_t* out_buffer,
                                               std::uint32_t  out_buffer_word_size,
                                               std::uint32_t  partition_and_flags) {
        using rom_get_partition_table_info = int (*)(std::uint32_t*, std::uint32_t, std::uint32_t);

        return RomFunctions::call<'G', 'P', rom_get_partition_table_info>(out_buffer,
                                                                          out_buffer_word_size,
                                                                          partition_and_flags);
    }

    // otp_access (5.4.8.19): rows are 24 bits wide; read or written as raw 32-bit words
    // (four bytes per row, ECC off) or as ECC-protected 16-bit halves (two bytes per row).
    namespace OtpCmd {
        constexpr std::uint32_t ROW_MASK = 0x0000FFFF;
        constexpr std::uint32_t WRITE    = 0x00010000;
        constexpr std::uint32_t ECC      = 0x00020000;
    }   // namespace OtpCmd

    static inline int otp_access(std::uint8_t* buf,
                                 std::uint32_t buf_len,
                                 std::uint32_t cmd_flags) {
        using rom_func_otp_access = int (*)(std::uint8_t*, std::uint32_t, std::uint32_t);

        return RomFunctions::call<'O', 'A', rom_func_otp_access>(buf, buf_len, cmd_flags);
    }
#endif

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
    flash_erase_and_write_impl(FlashXipDisabler& xipDisabler,
                               void (*erase)(std::uint32_t,
                                             std::size_t,
                                             std::uint32_t,
                                             std::uint8_t),
                               void (*write)(std::uint32_t,
                                             std::uint8_t const*,
                                             std::size_t),
                               std::uint32_t       offset,
                               std::uint8_t const* data,
                               std::size_t         size) {
        XipGuard guard{xipDisabler};
        erase(offset, 4096, 1 << 16, 0xD8);
        write(offset, data, size);
    }

    static inline void flash_erase_and_write(std::uint32_t       offset,
                                             std::uint8_t const* data,
                                             std::size_t         size) {
        auto erase = RomFunctions::getRomFunctionPointer<
          'R',
          'E',
          void (*)(std::uint32_t, std::size_t, std::uint32_t, std::uint8_t)>();
        auto write = RomFunctions::getRomFunctionPointer<
          'R',
          'P',
          void (*)(std::uint32_t, std::uint8_t const*, std::size_t)>();

        FlashXipDisabler xipDisabler{};
        flash_erase_and_write_impl(xipDisabler, erase, write, offset, data, size);
    }

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
    flash_erase_impl(FlashXipDisabler& xipDisabler,
                     void (*erase)(std::uint32_t,
                                   std::size_t,
                                   std::uint32_t,
                                   std::uint8_t),
                     std::uint32_t offset,
                     std::size_t   blocks) {
        XipGuard guard{xipDisabler};
        erase(offset, blocks * 4096, 1 << 16, 0xD8);
    }

    static inline void flash_erase(std::uint32_t offset,
                                   std::size_t   blocks) {
        auto erase = RomFunctions::getRomFunctionPointer<
          'R',
          'E',
          void (*)(std::uint32_t, std::size_t, std::uint32_t, std::uint8_t)>();

        FlashXipDisabler xipDisabler{};
        flash_erase_impl(xipDisabler, erase, offset, blocks);
    }

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
    flash_write_impl(FlashXipDisabler& xipDisabler,
                     void (*write)(std::uint32_t,
                                   std::uint8_t const*,
                                   std::size_t),
                     std::uint32_t       offset,
                     std::uint8_t const* data,
                     std::size_t         size) {
        XipGuard guard{xipDisabler};
        write(offset, data, size);
    }

    static inline void flash_write(std::uint32_t       offset,
                                   std::uint8_t const* data,
                                   std::size_t         size) {
        auto write = RomFunctions::getRomFunctionPointer<
          'R',
          'P',
          void (*)(std::uint32_t, std::uint8_t const*, std::size_t)>();

        FlashXipDisabler xipDisabler{};
        flash_write_impl(xipDisabler, write, offset, data, size);
    }

#if __has_include("chip/rp2040.hpp")
    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
    flash_do_cmd_impl(FlashXipDisabler&          xipDisabler,
                      std::span<std::byte const> txBuffer,
                      std::span<std::byte>       rxBuffer) {
        using QSPI_CS  = Kvasir::Peripheral::IO_QSPI::Registers<>::GPIO_QSPI_SS_CTRL::OUTOVERValC;
        using SSI_Regs = Kvasir::Peripheral::XIP_SSI::Registers<>;
        XipGuard guard{xipDisabler};

        apply(write(QSPI_CS::low));

        static constexpr std::size_t maxInFlight = 16 - 2;

        std::size_t inFlight{};
        while(!txBuffer.empty() || !rxBuffer.empty()) {
            auto const flags   = apply(read(SSI_Regs::SR::tfnf), read(SSI_Regs::SR::rfne));
            bool const can_put = get<0>(flags);
            bool const can_get = get<1>(flags);
            if(can_put && !txBuffer.empty() && inFlight < maxInFlight) {
                apply(write(SSI_Regs::DR0::dr, static_cast<std::uint32_t>(txBuffer[0])));
                txBuffer = txBuffer.subspan(1);
                ++inFlight;
            }
            if(can_get && !rxBuffer.empty()) {
                auto const v = get<0>(apply(read(SSI_Regs::DR0::dr)));
                rxBuffer[0]  = static_cast<std::byte>(v);
                rxBuffer     = rxBuffer.subspan(1);
                --inFlight;
            }
        }

        apply(write(QSPI_CS::high));
    }

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
    flash_do_cmd(std::span<std::byte const> txBuffer,
                 std::span<std::byte>       rxBuffer) {
        FlashXipDisabler xipDisabler{};
        flash_do_cmd_impl(xipDisabler, txBuffer, rxBuffer);
    }

#endif
    static inline std::array<std::byte,
                             8> read_serial_number() {
        if constexpr(PinConfig::CurrentChip == Kvasir::PinConfig::ChipVariant::RP2040) {
#if __has_include("chip/rp2040.hpp")
            static constexpr std::byte   Cmd{0x4b};
            static constexpr std::size_t DummyBytes = 5;
            static constexpr std::size_t DataBytes  = 8;
            static constexpr std::size_t TotalBytes = DummyBytes + DataBytes;

            std::array<std::byte, TotalBytes> txBuffer{};
            std::array<std::byte, TotalBytes> rxBuffer{};
            txBuffer[0] = Cmd;
            {
                Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> guard{};
                flash_do_cmd(txBuffer, rxBuffer);
            }
            std::array<std::byte, DataBytes> id;
            std::copy(rxBuffer.begin() + DummyBytes, rxBuffer.end(), id.begin());
            return id;
#endif
        } else {
#if __has_include("chip/rp2350.hpp")
            std::array<std::uint32_t, 4> buffer{};

            static constexpr std::uint32_t CHIP_INFO = 0x0001;

            auto const length = detail::get_sys_info(buffer.data(), buffer.size(), CHIP_INFO);

            std::array<std::byte, 8> serial_number{};

            if(length != 4 || buffer[0] != CHIP_INFO) {
                UC_LOG_C("error reading serial_number {}", length);
            } else {
                if constexpr(PinConfig::CurrentChip == Kvasir::PinConfig::ChipVariant::RP2350A) {
                    if(buffer[1] != 1) {
                        UC_LOG_C(
                          "error you probably selected the wrong chip in PinConfig::CurrentChip");
                    }
                } else if(PinConfig::CurrentChip == Kvasir::PinConfig::ChipVariant::RP2350B) {
                    if(buffer[1] != 0) {
                        UC_LOG_C(
                          "error you probably selected the wrong chip in PinConfig::CurrentChip");
                    }
                }

                // Extract bytes in reverse order from each word to match pico-sdk behavior
                // pico-sdk accesses bytes[15:8] which is buffer[3] then buffer[2], both word-reversed
                auto const swapped = std::array{std::byteswap(buffer[3]), std::byteswap(buffer[2])};
                std::ranges::copy(std::as_bytes(std::span{swapped}), serial_number.begin());
            }

            return serial_number;
#endif
        }
    }

#if __has_include("chip/rp2350.hpp")
    // STRDEF-only white-label OTP indices (RP2350 datasheet §13.10).
    // VALUE entries (VID=0x0000, PID=0x0001, BCD=0x0002, LANG_ID=0x0003,
    // CONFIG_ATTRIBUTES=0x0007) are raw 16-bit values, not strings, and are
    // intentionally omitted here.
    enum class WhiteLabelStrIndex : std::uint16_t {
        usb_device_manufacturer  = 0x0004,   // max 30 chars
        usb_device_product       = 0x0005,   // max 30 chars
        usb_device_serial_number = 0x0006,   // max 30 chars
        volume_label             = 0x0008,   // max 11 chars (ASCII only)
        scsi_inquiry_vendor      = 0x0009,   // max  8 chars (ASCII only)
        scsi_inquiry_product     = 0x000a,   // max 16 chars (ASCII only)
        scsi_inquiry_version     = 0x000b,   // max  4 chars (ASCII only)
        index_htm_redirect_url   = 0x000c,   // max 127 chars
        index_htm_redirect_name  = 0x000d,   // max 127 chars
        info_uf2_txt_model       = 0x000e,   // max 127 chars
        info_uf2_txt_board_id    = 0x000f,   // max 127 chars
    };

    template<WhiteLabelStrIndex Index,
             std::size_t        MaxLen>
    static inline std::optional<Kvasir::StaticString<MaxLen>> read_white_label_string() {
        constexpr std::uint32_t OTP_DATA_BASE                     = 0x40130000;
        constexpr std::uint16_t OTP_DATA_USB_WHITE_LABEL_ADDR_ROW = 0x005c;
        constexpr std::uint16_t INDEX_OFFSET = static_cast<std::uint16_t>(Index);

        auto otp_read_ecc = [](std::uint16_t row) -> std::uint16_t {
            return *reinterpret_cast<std::uint16_t const volatile*>(OTP_DATA_BASE
                                                                    + row * sizeof(std::uint16_t));
        };

        std::uint16_t const white_label_addr = otp_read_ecc(OTP_DATA_USB_WHITE_LABEL_ADDR_ROW);

        if(white_label_addr == 0 || white_label_addr == 0xFFFF) { return std::nullopt; }

        std::uint16_t const string_definition = otp_read_ecc(white_label_addr + INDEX_OFFSET);

        if(string_definition == 0x0000 || string_definition == 0xFFFF) { return std::nullopt; }

        constexpr std::uint8_t UNICODE_FLAG_BIT = 0x80;
        constexpr std::uint8_t LENGTH_MASK      = 0x7F;

        bool const         is_unicode      = (string_definition & UNICODE_FLAG_BIT) != 0;
        std::size_t const  character_count = string_definition & LENGTH_MASK;
        std::uint8_t const data_row_offset
          = static_cast<std::uint8_t>((string_definition >> 8) & 0xFF);

        if(is_unicode || character_count == 0 || character_count > MaxLen) { return std::nullopt; }

        std::uint16_t const          string_data_start_row = white_label_addr + data_row_offset;
        Kvasir::StaticString<MaxLen> result;
        result.resize(character_count);

        auto              output_iter = result.begin();
        std::size_t const num_rows    = (character_count + 1) / 2;

        for(std::size_t row_index = 0; row_index < num_rows; ++row_index) {
            std::uint16_t const packed_chars
              = otp_read_ecc(static_cast<std::uint16_t>(string_data_start_row + row_index));

            *output_iter++ = static_cast<char>(packed_chars & 0xFF);

            if(output_iter != result.end()) {
                *output_iter++ = static_cast<char>((packed_chars >> 8) & 0xFF);
            }
        }

        return result;
    }

    static inline CriticalFlags read_critical_flags() {
        constexpr std::uint32_t      CRITICAL = 0x0002;
        std::array<std::uint32_t, 2> buffer{};
        auto const                   length = get_sys_info(buffer.data(), buffer.size(), CRITICAL);
        if(length < 2 || (buffer[0] & CRITICAL) == 0) { return {}; }
        auto const raw = buffer[1];
        return {
          .secureBootEnabled     = (raw & 0x01) != 0,
          .secureDebugDisable    = (raw & 0x02) != 0,
          .debugDisable          = (raw & 0x04) != 0,
          .bootArch              = static_cast<BootArch>((raw >> 3) & 0x01),
          .glitchDetectorEnabled = (raw & 0x10) != 0,
          .glitchDetectorSens    = static_cast<GlitchDetectorSens>((raw >> 5) & 0x03),
        };
    }

    static inline OtpPageLocks read_otp_page_locks() {
        constexpr std::uint32_t OTP_DATA_RAW_BASE = 0x40134000;
        // Offsets verified against SVD: baseAddr + 0x3E00 + page*8 + 4 for LOCK1
        constexpr std::uint32_t PAGE1_LOCK1_OFFSET  = 0x3E0C;
        constexpr std::uint32_t PAGE2_LOCK1_OFFSET  = 0x3E14;
        constexpr std::uint32_t PAGE29_LOCK1_OFFSET = 0x3EEC;
        constexpr std::uint32_t PAGE30_LOCK1_OFFSET = 0x3EF4;
        constexpr std::uint32_t PAGE31_LOCK1_OFFSET = 0x3EFC;

        // Bits [5:0] encode lock_s (1:0), lock_ns (3:2), lock_bl (5:4); any non-zero means locked.
        auto is_locked = [](std::uint32_t offset) -> bool {
            auto const val
              = *reinterpret_cast<std::uint32_t const volatile*>(OTP_DATA_RAW_BASE + offset);
            return (val & 0x3F) != 0;
        };

        return {
          .page1  = is_locked(PAGE1_LOCK1_OFFSET),
          .page2  = is_locked(PAGE2_LOCK1_OFFSET),
          .page29 = is_locked(PAGE29_LOCK1_OFFSET),
          .page30 = is_locked(PAGE30_LOCK1_OFFSET),
          .page31 = is_locked(PAGE31_LOCK1_OFFSET),
        };
    }

    static inline std::optional<Kvasir::StaticString<30>> read_white_label_serial_number() {
        return read_white_label_string<WhiteLabelStrIndex::usb_device_serial_number, 30>();
    }

    static inline std::optional<Kvasir::StaticString<127>> read_white_label_board_id() {
        return read_white_label_string<WhiteLabelStrIndex::info_uf2_txt_board_id, 127>();
    }
#endif

}   // namespace detail

namespace detail {
    // The watchdog's own reboot: what the pico-sdk's watchdog_reboot(0, 0, ..) and the RP2350
    // bootrom's reboot() do underneath, without the timer. CTRL is written as a whole so the
    // PAUSE_DBG0/1/JTAG bits go too: with them set a watchdog does nothing while a probe is
    // attached, and "does not reboot under the debugger" is not a reboot. SCRATCH4 = 0 keeps
    // the bootrom from taking a stale watchdog boot vector (5.2.4). PSM.WDSEL names the
    // stages the reset runs through: the RP2350 bootrom resets everything but the processor
    // cold domain (so the debug halt-on-reset bits survive), the pico-sdk on the RP2040
    // everything but the oscillators; the RESETS stage being in both is what puts every
    // peripheral back into reset, and the PROC stages what puts core 1 back into the
    // bootrom's holding pen.
    [[noreturn]] inline void watchdogReboot() {
        using WD  = Kvasir::Peripheral::WATCHDOG::Registers<>;
        using PSM = Kvasir::Peripheral::PSM::Registers<>;

        apply(WD::CTRL::overrideDefaults(write(WD::CTRL::pause_dbg1, Register::value<0>()),
                                         write(WD::CTRL::pause_dbg0, Register::value<0>()),
                                         write(WD::CTRL::pause_jtag, Register::value<0>())));
        apply(write(WD::SCRATCH4::FULLREGISTER, Register::value<0>()));

#if __has_include("chip/rp2350.hpp")
        apply(set(PSM::WDSEL::proc1),
              set(PSM::WDSEL::proc0),
              set(PSM::WDSEL::accessctrl),
              set(PSM::WDSEL::sio),
              set(PSM::WDSEL::xip),
              set(PSM::WDSEL::sram9),
              set(PSM::WDSEL::sram8),
              set(PSM::WDSEL::sram7),
              set(PSM::WDSEL::sram6),
              set(PSM::WDSEL::sram5),
              set(PSM::WDSEL::sram4),
              set(PSM::WDSEL::sram3),
              set(PSM::WDSEL::sram2),
              set(PSM::WDSEL::sram1),
              set(PSM::WDSEL::sram0),
              set(PSM::WDSEL::bootram),
              set(PSM::WDSEL::rom),
              set(PSM::WDSEL::busfabric),
              set(PSM::WDSEL::ready),
              set(PSM::WDSEL::clocks),
              set(PSM::WDSEL::resets),
              set(PSM::WDSEL::xosc),
              set(PSM::WDSEL::rosc),
              set(PSM::WDSEL::otp),
              clear(PSM::WDSEL::proc_cold));
#else
        apply(set(PSM::WDSEL::proc1),
              set(PSM::WDSEL::proc0),
              set(PSM::WDSEL::sio),
              set(PSM::WDSEL::vreg_and_chip_reset),
              set(PSM::WDSEL::xip),
              set(PSM::WDSEL::sram5),
              set(PSM::WDSEL::sram4),
              set(PSM::WDSEL::sram3),
              set(PSM::WDSEL::sram2),
              set(PSM::WDSEL::sram1),
              set(PSM::WDSEL::sram0),
              set(PSM::WDSEL::rom),
              set(PSM::WDSEL::busfabric),
              set(PSM::WDSEL::resets),
              set(PSM::WDSEL::clocks),
              clear(PSM::WDSEL::xosc),
              clear(PSM::WDSEL::rosc));
#endif
        apply(set(WD::CTRL::trigger));
        while(true) { asm volatile("wfi"); }
    }
}   // namespace detail

[[noreturn]] inline void resetToUsbBoot() {
    if constexpr(PinConfig::CurrentChip == Kvasir::PinConfig::ChipVariant::RP2040) {
        using romResetToUsbBoot
          = void (*)(std::uint32_t gpioActivityPinMask, std::uint32_t disableInterfaceMask);

        RomFunctions::call<'U', 'B', romResetToUsbBoot>(0, 0);
    } else {
        static constexpr std::uint32_t NO_RETURN_ON_SUCCESS = 0x0100;
        static constexpr std::uint32_t REBOOT_TYPE_BOOTSEL  = 0x0002;

        [[maybe_unused]] auto const ret
          = detail::reboot(REBOOT_TYPE_BOOTSEL | NO_RETURN_ON_SUCCESS, 1, 0, 0);
        UC_LOG_C("reboot ret {}", ret);
    }

    UC_LOG_C("This should not happen reboot returned");
    detail::watchdogReboot();   // a chip reset at least, see reboot() below
}

// Reboot the chip: both cores and every peripheral restart from the bootrom, as after a
// power-on. PM::reset_cause() reports watchdog_timer on the RP2350 (the bootrom arms the
// watchdog's timer, 1 ms) and watchdog_force on the raw-trigger path.
//
// Not SystemControl::SystemReset: SYSRESETREQ is a warm reset of the core that asserts it
// and of nothing else (RP2040 datasheet 2.4.2.9, RP2350 datasheet 12.9, pico-feedback #329).
// Issued from core 1 it parks core 1 in the bootrom and leaves core 0 running; from core 0
// it leaves core 1 running and the peripherals configured, and only looks like a reboot
// because FirstInitStep puts the peripherals back into reset.
//
// The RP2350 goes through the bootrom's reboot() (datasheet 5.4.8.24), the pico-sdk's and
// picotool's path: it switches POWMAN off clk_ref before the clock generators reset (a
// clk_pow glitch otherwise) and keeps the boot diagnostics. NO_RETURN_ON_SUCCESS parks this
// core in the ROM until the watchdog fires, 1 ms later. The raw watchdog is the RP2040's
// path and the fallback should the ROM call refuse.
[[noreturn]] inline void reboot() {
    if constexpr(PinConfig::CurrentChip != Kvasir::PinConfig::ChipVariant::RP2040) {
        static constexpr std::uint32_t NO_RETURN_ON_SUCCESS = 0x0100;
        static constexpr std::uint32_t REBOOT_TYPE_NORMAL   = 0x0000;

        [[maybe_unused]] auto const ret
          = detail::reboot(REBOOT_TYPE_NORMAL | NO_RETURN_ON_SUCCESS, 1, 0, 0);
        UC_LOG_C("bootrom reboot returned {}", ret);
    }
    detail::watchdogReboot();
}

inline auto serialNumber() {
    static std::array<std::byte, 8> const serial_number = detail::read_serial_number();
    return serial_number;
}

inline auto serialNumberString() {
    auto const               rawSerialBytes = Kvasir::serialNumber();
    Kvasir::StaticString<16> hexSerialString;
    static constexpr std::array<char, 16>
      hexDigits{'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

    for(auto const& byte : rawSerialBytes) {
        hexSerialString.push_back(hexDigits[(static_cast<int>(byte) & 0xF0) >> 4]);
        hexSerialString.push_back(hexDigits[static_cast<int>(byte) & 0x0F]);
    }

    return hexSerialString;
}

#if __has_include("chip/rp2350.hpp")
inline auto whiteLabelSerialNumber() {
    static std::optional<Kvasir::StaticString<30>> const serial_number
      = detail::read_white_label_serial_number();
    return serial_number;
}

inline auto whiteLabelBoardId() {
    static std::optional<Kvasir::StaticString<127>> const board_id
      = detail::read_white_label_board_id();
    return board_id;
}

inline CriticalFlags criticalFlags() { return detail::read_critical_flags(); }

inline OtpPageLocks otpPageLocks() { return detail::read_otp_page_locks(); }

inline bool isSecureBootEnabled() { return criticalFlags().secureBootEnabled; }
#else
inline bool isSecureBootEnabled() { return false; }
#endif

inline Kvasir::StaticString<30> bootromUSBSerialNumber() {
#if __has_include("chip/rp2350.hpp")
    auto const whiteLabelSerial = Kvasir::whiteLabelSerialNumber();
    if(whiteLabelSerial.has_value()) { return whiteLabelSerial.value(); }
#endif
    return serialNumberString();
}

/// serialNumberString() as a view of one function-local copy: what a string_view member
/// or a USB descriptor can hold on to (the by-value one is a temporary).
inline std::string_view serialNumberView() {
    static Kvasir::StaticString<16> const hexSerialString = serialNumberString();
    return std::string_view{hexSerialString};
}

inline bool isFlashBinary() {
    // Read SCB->VTOR (0xE000ED08): the vector table base address tells us where
    // the binary lives. On RP2350: flash = 0x10000000, RAM = 0x20000000.
    auto const vtor = *reinterpret_cast<std::uint32_t const volatile*>(0xE000ED08U);
    return vtor < 0x20000000U;
}

// The bootrom's flash, partition table and OTP entry points under their public names; the
// detail:: originals they wrap are what the drivers use.
namespace Bootrom {
    /// Erase `blocks` 4096-byte sectors at flash `offset` (from the start of flash, not the
    /// XIP window). Runs with XIP disabled; mask interrupts around it.
    inline void flashErase(std::uint32_t offset,
                           std::size_t   blocks) {
        detail::flash_erase(offset, blocks);
    }

    /// Program `size` bytes (a multiple of 256, page aligned `offset`) from `data`. Runs
    /// with XIP disabled; mask interrupts around it.
    inline void flashWrite(std::uint32_t       offset,
                           std::uint8_t const* data,
                           std::size_t         size) {
        detail::flash_write(offset, data, size);
    }

#if __has_include("chip/rp2350.hpp")
    namespace PtInfoFlags = Kvasir::detail::PtInfoFlags;
    namespace PtInfo      = Kvasir::detail::PtInfo;
    namespace OtpCmd      = Kvasir::detail::OtpCmd;

    /// get_partition_table_info (RP2350 datasheet 5.4.8.16): fills `out_buffer` with the
    /// echoed flags word and what PtInfoFlags asks for; returns the word count or a
    /// negative bootrom error.
    [[nodiscard]] inline int getPartitionTableInfo(std::uint32_t* out_buffer,
                                                   std::uint32_t  out_buffer_word_size,
                                                   std::uint32_t  partition_and_flags) {
        return detail::get_partition_table_info(out_buffer,
                                                out_buffer_word_size,
                                                partition_and_flags);
    }

    /// flash_runtime_to_storage_addr (5.4.8.15): the QMI's address translation of a runtime
    /// (XIP window) address; negative when the address is not mapped.
    [[nodiscard]] inline int flashRuntimeToStorageAddr(std::uint32_t addr) {
        return detail::flash_runtime_to_storage_addr(addr);
    }

    /// otp_access (5.4.8.19): read or write `buf_len` bytes of OTP from the row in
    /// `cmd_flags` (OtpCmd::ROW_MASK, WRITE, ECC); 0 on success.
    [[nodiscard]] inline int otpAccess(std::uint8_t* buf,
                                       std::uint32_t buf_len,
                                       std::uint32_t cmd_flags) {
        return detail::otp_access(buf, buf_len, cmd_flags);
    }

    // OTP row numbers (RP2350 datasheet 13.6, the OTP row table): CHIPID0..3 are rows
    // 0x000..0x003 (ECC, read together they are the 64-bit chip id); the rows from 0x400
    // (page 16) on are not assigned by the datasheet and are the user area.
    namespace Otp::Rows {
        constexpr std::uint32_t ChipId   = 0x000;
        constexpr std::uint32_t UserBase = 0x400;
    }   // namespace Otp::Rows
#endif
}   // namespace Bootrom

}   // namespace Kvasir
