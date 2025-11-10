#pragma once
#include <cassert>
#include <cstdint>
#include <functional>

#if __has_include("peripherals/QMI.hpp")
    #include "peripherals/QMI.hpp"
#endif

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
    struct FlashXipDisabler {
        using RomConnectInternalFlash = void (*)(void);
        using RomFlashExitXip         = void (*)(void);
        using RomFlashFlushCache      = void (*)(void);
        using XipEnableFunction       = void (*)(void);

        RomFlashFlushCache const            flushCache;
        std::array<std::uint32_t, 64> const xipEnableRamCopy;
        XipEnableFunction const             xipEnable;

#if __has_include("peripherals/QMI.hpp")
        std::uint32_t flashTiming;
#endif

        static std::array<std::uint32_t,
                          64> getXipEnable() {
            if constexpr(PinConfig::CurrentChip == Kvasir::PinConfig::ChipVariant::RP2040) {
#if __has_include("peripherals/XIP_SSI.hpp")
                return Kvasir::Startup::second_stage_bootloader;
#endif
            } else {
                return *reinterpret_cast<std::array<std::uint32_t, 64> const* const>(0x400E0000UL);
            }
        }

        [[KVASIR_RAM_FUNC_ATTRIBUTES]] FlashXipDisabler()
          : flushCache{Kvasir::RomFunctions::getRomFunctionPointer<'F',
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
            Kvasir::RomFunctions::call<'I', 'F', RomConnectInternalFlash>();
            Kvasir::RomFunctions::call<'E', 'X', RomFlashExitXip>();
        }

        [[KVASIR_RAM_FUNC_ATTRIBUTES]] ~FlashXipDisabler() {
            flushCache();
            xipEnable();

#if __has_include("peripherals/QMI.hpp")
            using namespace Kvasir::Peripheral::QMI;
            using QMI = Registers<0>;
            apply(write(QMI::M0_TIMING::FULLREGISTER, flashTiming));
#endif
        }
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

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
    flash_erase_and_write(std::uint32_t       offset,
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
        erase(offset, 4096, 1 << 16, 0xD8);
        write(offset, data, size);
    }

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void flash_erase(std::uint32_t offset,
                                                                  std::size_t   blocks) {
        auto erase = RomFunctions::getRomFunctionPointer<
          'R',
          'E',
          void (*)(std::uint32_t, std::size_t, std::uint32_t, std::uint8_t)>();

        FlashXipDisabler xipDisabler{};
        erase(offset, blocks * 4096, 1 << 16, 0xD8);
    }

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void flash_write(std::uint32_t       offset,
                                                                  std::uint8_t const* data,
                                                                  std::size_t         size) {
        auto write = RomFunctions::getRomFunctionPointer<
          'R',
          'P',
          void (*)(std::uint32_t, std::uint8_t const*, std::size_t)>();

        FlashXipDisabler xipDisabler{};
        write(offset, data, size);
    }

    //rp2040 function
    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
    flash_do_cmd([[maybe_unused]] std::span<std::byte const> txBuffer,
                 [[maybe_unused]] std::span<std::byte>       rxBuffer) {
#if __has_include("peripherals/XIP_SSI.hpp")
        using QSPI_CS  = Kvasir::Peripheral::IO_QSPI::Registers<>::GPIO_QSPI_SS_CTRL::OUTOVERValC;
        using SSI_Regs = Kvasir::Peripheral::XIP_SSI::Registers<>;
        FlashXipDisabler xipDisabler{};

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
#endif
    }

    static inline std::array<std::byte,
                             8>
    read_serial_number() {
        if constexpr(PinConfig::CurrentChip == Kvasir::PinConfig::ChipVariant::RP2040) {
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
        } else {
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
        }
    }

}   // namespace detail

inline void resetToUsbBoot() {
    if constexpr(PinConfig::CurrentChip == Kvasir::PinConfig::ChipVariant::RP2040) {
        using romResetToUsbBoot
          = void (*)(std::uint32_t gpioActivityPinMask, std::uint32_t disableInterfaceMask);

        RomFunctions::call<'U', 'B', romResetToUsbBoot>(0, 0);
    } else {
        static constexpr std::uint32_t REBOOT_TYPE_BOOTSEL = 0x0002;

        [[maybe_unused]] auto const ret = detail::reboot(REBOOT_TYPE_BOOTSEL, 0, 0, 0);
        UC_LOG_C("reboot ret {}", ret);
    }

    UC_LOG_C("This should not happen reboot returned");
    apply(Kvasir::SystemControl::SystemReset{});
}

inline auto serialNumber() {
    static std::array<std::byte, 8> const serial_number = detail::read_serial_number();
    return serial_number;
}
}   // namespace Kvasir

