#pragma once
#include <cassert>
#include <cstdint>
#include <functional>

namespace Kvasir {
namespace RomFunctions {
    constexpr std::uint32_t lookupCode(char c1,
                                       char c2) {
        return static_cast<std::uint32_t>(c1) | (static_cast<std::uint32_t>(c2) << 8);
    }

    template<typename T,
             std::uint32_t Address>
    T dereferenceAs() {
        return reinterpret_cast<T>(*reinterpret_cast<std::uint16_t const*>(Address));
    }

    template<char C1,
             char C2,
             typename F>
    F getRomFunctionPointer() {
        static constexpr std::uint32_t LookupFunctionAddress{0x16};
        static constexpr auto          FunctionLookupCode = lookupCode(C1, C2);

        using RomTabelLookupFunction = F (*)(std::uint32_t code, std::uint32_t mask);

        auto const romTableLookupFunction
          = dereferenceAs<RomTabelLookupFunction, LookupFunctionAddress>();

        auto const fp = romTableLookupFunction(FunctionLookupCode, 0x0004);
        assert(fp != nullptr);

        return fp;
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

namespace detail {
    inline int get_sys_info(std::uint32_t* out_buffer,
                            std::uint32_t  out_buffer_word_size,
                            std::uint32_t  flags) {
        using rom_get_sys_info = int (*)(std::uint32_t* out_buffer,
                                         std::uint32_t  out_buffer_word_size,
                                         std::uint32_t  flags);

        return RomFunctions::call<'G', 'S', rom_get_sys_info>(out_buffer,
                                                              out_buffer_word_size,
                                                              flags);
    }

    inline int reboot(std::uint32_t flags,
                      std::uint32_t delay_ms,
                      std::uint32_t p0,
                      std::uint32_t p1) {
        using rom_reboot = int (*)(std::uint32_t flags,
                                   std::uint32_t delay_ms,
                                   std::uint32_t p0,
                                   std::uint32_t p1);

        return RomFunctions::call<'R', 'B', rom_reboot>(flags, delay_ms, p0, p1);
    }

}   // namespace detail

inline void resetToUsbBoot() {
    static constexpr std::uint32_t REBOOT_TYPE_BOOTSEL = 0x0002;

    auto ret = detail::reboot(REBOOT_TYPE_BOOTSEL, 0, 0, 0);
    if(ret != 0) {
        UC_LOG_C("This should not happen reboot returned {}", ret);
    }
    apply(Kvasir::SystemControl::SystemReset{});
}

inline auto serialNumber() {
    std::array<std::uint32_t, 4> buffer{};

    static constexpr std::uint32_t CHIP_INFO = 0x0001;

    auto const length = detail::get_sys_info(buffer.data(), buffer.size(), CHIP_INFO);

    std::array<std::byte, 8> serial_number{};

    if(length != 4 || buffer[0] != CHIP_INFO) {
        UC_LOG_C("error reading serial_number {}", length);
    } else {
        if constexpr(PinConfig::CurrentChip == Kvasir::PinConfig::ChipVariant::RP2350A) {
            if(buffer[1] != 1) {
                UC_LOG_C("error you probably selected the wrong chip in PinConfig::CurrentChip");
            }
        } else if(PinConfig::CurrentChip == Kvasir::PinConfig::ChipVariant::RP2350B) {
            if(buffer[1] != 0) {
                UC_LOG_C("error you probably selected the wrong chip in PinConfig::CurrentChip");
            }
        }

        // Extract bytes in reverse order from each word to match pico-sdk behavior
        // pico-sdk accesses bytes[15:8] which is buffer[3] then buffer[2], both word-reversed
        auto const swapped = std::array{std::byteswap(buffer[3]), std::byteswap(buffer[2])};
        std::ranges::copy(std::as_bytes(std::span{swapped}), serial_number.begin());
    }

    return serial_number;
}
}   // namespace Kvasir
