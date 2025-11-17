#pragma once
#include <cassert>
#include <cstdint>
#include <functional>
#include <kvasir/Util/StaticString.hpp>
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
#if __has_include("peripherals/XIP_SSI.hpp")
                return Kvasir::Startup::second_stage_bootloader;
#endif
            } else {
                return *reinterpret_cast<std::array<std::uint32_t, 64> const* const>(0x400E0000UL);
            }
        }

        FlashXipDisabler()
          : connectInternalFlash{Kvasir::RomFunctions::getRomFunctionPointer<
              'I',
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
            using namespace Kvasir::Peripheral::QMI;
            using QMI = Registers<0>;
            apply(write(QMI::M0_TIMING::FULLREGISTER, flashTiming));
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

#if __has_include("peripherals/XIP_SSI.hpp")
    //rp2040 function
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

    //rp2040 function
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
#if __has_include("peripherals/XIP_SSI.hpp")
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

    static inline std::optional<Kvasir::StaticString<30>> read_white_label_serial_number() {
        constexpr std::uint32_t OTP_DATA_BASE                         = 0x40130000;
        constexpr std::uint16_t OTP_DATA_USB_WHITE_LABEL_ADDR_ROW     = 0x005c;
        constexpr std::uint32_t INDEX_USB_DEVICE_SERIAL_NUMBER_STRDEF = 6;
        constexpr std::size_t   MAX_SERIAL_LENGTH                     = 30;

        auto otp_read_ecc = [](std::uint16_t row) -> std::uint16_t {
            return *reinterpret_cast<std::uint16_t const volatile*>(OTP_DATA_BASE
                                                                    + row * sizeof(std::uint16_t));
        };

        std::uint16_t const white_label_addr = otp_read_ecc(OTP_DATA_USB_WHITE_LABEL_ADDR_ROW);

        if(white_label_addr == 0 || white_label_addr == 0xFFFF) { return std::nullopt; }

        std::uint16_t const string_definition
          = otp_read_ecc(white_label_addr + INDEX_USB_DEVICE_SERIAL_NUMBER_STRDEF);

        if(string_definition == 0x0000 || string_definition == 0xFFFF) { return std::nullopt; }

        constexpr std::uint8_t UNICODE_FLAG_BIT = 0x80;
        constexpr std::uint8_t LENGTH_MASK      = 0x7F;

        bool const         is_unicode      = (string_definition & UNICODE_FLAG_BIT) != 0;
        std::size_t const  character_count = string_definition & LENGTH_MASK;
        std::uint8_t const data_row_offset
          = static_cast<std::uint8_t>((string_definition >> 8) & 0xFF);

        if(is_unicode || character_count == 0 || character_count > MAX_SERIAL_LENGTH) {
            return std::nullopt;
        }

        std::uint16_t const      string_data_start_row = white_label_addr + data_row_offset;
        Kvasir::StaticString<30> result;
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

inline auto whiteLabelSerialNumber() {
    static std::optional<Kvasir::StaticString<30>> const serial_number
      = detail::read_white_label_serial_number();
    return serial_number;
}

inline Kvasir::StaticString<30> bootromUSBSerialNumber() {
    auto const whiteLabelSerial = Kvasir::whiteLabelSerialNumber();
    if(whiteLabelSerial.has_value()) { return whiteLabelSerial.value(); }
    return serialNumberString();
}

}   // namespace Kvasir
