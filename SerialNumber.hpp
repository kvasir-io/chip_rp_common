#pragma once
// The chip's serial number (the RP2040's flash unique id, the RP2350's chip id), the RP2350's OTP reads as public
// functions, and isFlashBinary().
#include "Otp.hpp"
#include "Rom.hpp"
#include "XipControl.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <kvasir/Port/LinkerSymbols.hpp>
#include <kvasir/Util/HexString.hpp>
#include <kvasir/Util/StaticString.hpp>
#include <optional>
#include <span>
#include <string_view>
#include <uc_log/uc_log.hpp>

namespace Kvasir {
// The serial number read, in a namespace of its own for its log module ("bootrom", derived from
// the scope).
namespace Bootrom {
    static inline std::array<std::byte,
                             8>
    readSerialNumber() {
        if constexpr(Chip::part == Kvasir::Chip::Part::rp2040) {
#ifdef KVASIR_CHIP_RP2040
            static constexpr std::byte   cmd{0x4b};
            static constexpr std::size_t dummyBytes = 5;
            static constexpr std::size_t dataBytes  = 8;
            static constexpr std::size_t totalBytes = dummyBytes + dataBytes;

            std::array<std::byte, totalBytes> txBuffer{};
            std::array<std::byte, totalBytes> rxBuffer{};
            txBuffer[0] = cmd;
            {
                Kvasir::Core::IrqGuard const guard{};
                detail::flashDoCmd(txBuffer, rxBuffer);
            }
            std::array<std::byte, dataBytes> id;
            std::copy(rxBuffer.begin() + dummyBytes, rxBuffer.end(), id.begin());
            return id;
#endif
        } else {
#ifdef KVASIR_CHIP_RP2350
            std::array<std::uint32_t, 4> buffer{};

            static constexpr std::uint32_t chipInfo = 0x0001;

            auto const length = RomFunctions::getSysInfo(buffer.data(), buffer.size(), chipInfo);

            std::array<std::byte, 8> serialNumber{};

            if(length != 4 || buffer[0] != chipInfo) {
                UC_LOG_W("error reading serial_number {}", length);   // recovered: a zero serial
            } else {
                if constexpr(Chip::part == Kvasir::Chip::Part::rp2350a) {
                    if(buffer[1] != 1) {
                        UC_LOG_E(   // a configuration mistake the run survives (Runtime/Check.hpp's classes)
                          "error you probably selected the wrong chip in Chip::part");
                    }
                } else if(Chip::part == Kvasir::Chip::Part::rp2350b) {
                    if(buffer[1] != 0) {
                        UC_LOG_E(   // a configuration mistake the run survives (Runtime/Check.hpp's classes)
                          "error you probably selected the wrong chip in Chip::part");
                    }
                }

                // Extract bytes in reverse order from each word to match pico-sdk behavior
                // pico-sdk accesses bytes[15:8] which is buffer[3] then buffer[2], both word-reversed
                auto const swapped = std::array{std::byteswap(buffer[3]), std::byteswap(buffer[2])};
                std::ranges::copy(std::as_bytes(std::span{swapped}), serialNumber.begin());
            }

            return serialNumber;
#endif
        }
    }
}   // namespace Bootrom

inline auto serialNumber() {
    static std::array<std::byte, 8> const serialNumber = Bootrom::readSerialNumber();
    return serialNumber;
}

/// serialNumber() as 16 upper-case hex digits (Util/HexString.hpp, every chip's form).
inline auto serialNumberString() { return Kvasir::hexString(Kvasir::serialNumber()); }

#ifdef KVASIR_CHIP_RP2350
inline auto whiteLabelSerialNumber() {
    static std::optional<Kvasir::StaticString<30>> const serialNumber
      = detail::readWhiteLabelSerialNumber();
    return serialNumber;
}

inline CriticalFlags criticalFlags() { return detail::readCriticalFlags(); }

inline OtpPageLocks otpPageLocks() { return detail::readOtpPageLocks(); }
#endif

/// serialNumberString() as a view of one function-local copy: what a string_view member
/// or a USB descriptor can hold on to (the by-value one is a temporary).
inline std::string_view serialNumberView() {
    static Kvasir::StaticString<16> const hexSerialString = serialNumberString();
    return std::string_view{hexSerialString};
}

inline bool isFlashBinary() {
    // Where the image's own vector table is linked: flash 0x10000000, RAM 0x20000000 (RP2040 and
    // RP2350). Not VTOR: a flash image with Startup::RamVectorTable (Clocks::Resus needs one) runs
    // with VTOR on its RAM copy and would call itself a RAM image.
    return Kvasir::Startup::linkerAddress<_LINKER_vectors_start_>() < 0x20000000U;
}
}   // namespace Kvasir
