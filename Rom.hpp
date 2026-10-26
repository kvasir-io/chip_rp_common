#pragma once
// The RP boot ROM's function table: the lookup (RomFunctions) and the plain wrappers of the ROM calls the drivers use,
// with their flag words.
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <kvasir/Register/Memory.hpp>
#include <kvasir/Util/attributes.hpp>

namespace Kvasir { namespace RomFunctions {
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
#ifdef KVASIR_CHIP_RP2040
        using RomTableLookupFunction = F (*)(std::uint16_t const* table, std::uint32_t code);

        auto const lookup = reinterpret_cast<RomTableLookupFunction>(
          static_cast<std::uintptr_t>(Kvasir::Memory::load<std::uint16_t>(0x18U)));
        auto const table = reinterpret_cast<std::uint16_t const*>(
          static_cast<std::uintptr_t>(Kvasir::Memory::load<std::uint16_t>(0x14U)));
        return lookup(table, code);
#else
        static constexpr std::uint32_t armSecureFunction = 0x0004;

        using RomTableLookupFunction = F (*)(std::uint32_t code, std::uint32_t mask);

        auto const lookup = reinterpret_cast<RomTableLookupFunction>(
          static_cast<std::uintptr_t>(Kvasir::Memory::load<std::uint16_t>(0x16U)));
        return lookup(code, armSecureFunction);
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
        KVASIR_RAM_FUNC_MARK();
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

    static inline int getSysInfo(std::uint32_t* outBuffer,
                                 std::uint32_t  outBufferWordSize,
                                 std::uint32_t  flags) {
        using rom_get_sys_info
          = int (*)(std::uint32_t* outBuffer, std::uint32_t outBufferWordSize, std::uint32_t flags);

        return RomFunctions::call<'G', 'S', rom_get_sys_info>(outBuffer, outBufferWordSize, flags);
    }

    static inline int reboot(std::uint32_t flags,
                             std::uint32_t delayMs,
                             std::uint32_t p0,
                             std::uint32_t p1) {
        using rom_reboot
          = int (*)(std::uint32_t flags, std::uint32_t delayMs, std::uint32_t p0, std::uint32_t p1);

        return RomFunctions::call<'R', 'B', rom_reboot>(flags, delayMs, p0, p1);
    }

#ifdef KVASIR_CHIP_RP2350
    static inline int flashRuntimeToStorageAddr(std::uint32_t addr) {
        using rom_flash_runtime_to_storage_addr = int (*)(std::uint32_t addr);

        return RomFunctions::call<'F', 'A', rom_flash_runtime_to_storage_addr>(addr);
    }

    static inline int explicitBuy(std::uint8_t* buffer,
                                  std::uint32_t bufferSize) {
        using rom_explicit_buy = int (*)(std::uint8_t* buffer, std::uint32_t bufferSize);

        return RomFunctions::call<'E', 'B', rom_explicit_buy>(buffer, bufferSize);
    }

    // flash_op flags
    namespace FlashOpFlags {
        // Address translation (select one)
        constexpr std::uint32_t aspaceStorage = 0x00000000;
        constexpr std::uint32_t aspaceRuntime = 0x00000001;

        // Security level (select one)
        constexpr std::uint32_t seclevelSecure     = 0x00000100;
        constexpr std::uint32_t seclevelNonsecure  = 0x00000200;
        constexpr std::uint32_t seclevelBootloader = 0x00000300;

        // Operation (select one)
        constexpr std::uint32_t opErase   = 0x00000000;
        constexpr std::uint32_t opProgram = 0x00010000;
        constexpr std::uint32_t opRead    = 0x00020000;
    }   // namespace FlashOpFlags

    static inline int flashOp(std::uint32_t flags,
                              std::uint32_t addr,
                              std::uint32_t sizeBytes,
                              std::uint8_t* buf) {
        using rom_flash_op = int (*)(std::uint32_t, std::uint32_t, std::uint32_t, std::uint8_t*);

        return RomFunctions::call<'F', 'O', rom_flash_op>(flags, addr, sizeBytes, buf);
    }

    // the flags and the answer's masks: Bootrom::PtInfoFlags / Bootrom::PtInfo (BootInfo.hpp)
    static inline int getPartitionTableInfo(std::uint32_t* outBuffer,
                                            std::uint32_t  outBufferWordSize,
                                            std::uint32_t  partitionAndFlags) {
        using rom_get_partition_table_info = int (*)(std::uint32_t*, std::uint32_t, std::uint32_t);

        return RomFunctions::call<'G', 'P', rom_get_partition_table_info>(outBuffer,
                                                                          outBufferWordSize,
                                                                          partitionAndFlags);
    }

    // the command word: Bootrom::OtpCmd (BootInfo.hpp)
    static inline int otpAccess(std::uint8_t* buf,
                                std::uint32_t bufLen,
                                std::uint32_t cmdFlags) {
        using rom_func_otp_access = int (*)(std::uint8_t*, std::uint32_t, std::uint32_t);

        return RomFunctions::call<'O', 'A', rom_func_otp_access>(buf, bufLen, cmdFlags);
    }
#endif
}}   // namespace Kvasir::RomFunctions
