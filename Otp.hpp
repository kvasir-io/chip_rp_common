#pragma once
// The RP2350's OTP as the drivers read it: the critical boot flags, the page locks and the USB white-label strings.
#include "Rom.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <kvasir/Register/Memory.hpp>
#include <kvasir/Register/Register.hpp>
#include <kvasir/Util/StaticString.hpp>
#include <optional>
#include <utility>
#ifdef KVASIR_CHIP_RP2350
    #include "peripherals/OTP_DATA.hpp"
    #include "peripherals/OTP_DATA_RAW.hpp"
#endif

namespace Kvasir {
#ifdef KVASIR_CHIP_RP2350
enum class BootArch : std::uint8_t { arm = 0, riscV = 1 };

enum class GlitchDetectorSens : std::uint8_t {
    sensDefault = 0,
    low         = 1,
    medium      = 2,
    high        = 3,
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
#ifdef KVASIR_CHIP_RP2350
    // STRDEF-only white-label OTP indices (RP2350 datasheet §13.10).
    // VALUE entries (VID=0x0000, PID=0x0001, BCD=0x0002, LANG_ID=0x0003,
    // CONFIG_ATTRIBUTES=0x0007) are raw 16-bit values, not strings, and are
    // intentionally omitted here.
    enum class WhiteLabelStrIndex : std::uint16_t {
        usbDeviceManufacturer = 0x0004,   // max 30 chars
        usbDeviceProduct      = 0x0005,   // max 30 chars
        usbDeviceSerialNumber = 0x0006,   // max 30 chars
        volumeLabel           = 0x0008,   // max 11 chars (ASCII only)
        scsiInquiryVendor     = 0x0009,   // max  8 chars (ASCII only)
        scsiInquiryProduct    = 0x000a,   // max 16 chars (ASCII only)
        scsiInquiryVersion    = 0x000b,   // max  4 chars (ASCII only)
        indexHtmRedirectUrl   = 0x000c,   // max 127 chars
        indexHtmRedirectName  = 0x000d,   // max 127 chars
        infoUf2TxtModel       = 0x000e,   // max 127 chars
        infoUf2TxtBoardId     = 0x000f,   // max 127 chars
    };

    template<WhiteLabelStrIndex Index,
             std::size_t        MaxLen>
    static inline std::optional<Kvasir::StaticString<MaxLen>> readWhiteLabelString() {
        using OtpData                       = Kvasir::Peripheral::OTP_DATA::Registers<>;
        constexpr std::uint16_t indexOffset = static_cast<std::uint16_t>(Index);

        // Rows found at run time (the white-label table and its strings): OTP memory, not registers. The ECC alias
        // holds 16 bits per row, row n at OTP_DATA_BASE + 2n ("a 32-bit read returns the ECC-corrected data for two
        // neighbouring rows", RP2350 datasheet 13.1, md l.61886); the base is the SVD's OTP_DATA block.
        auto const otpReadEcc = [](std::uint16_t row) -> std::uint16_t {
            return Kvasir::Memory::load<std::uint16_t>(OtpData::baseAddr
                                                       + row * sizeof(std::uint16_t));
        };

        // The table's own row is a named one (USB_WHITE_LABEL_ADDR, row 0x05c).
        std::uint16_t const whiteLabelAddr = static_cast<std::uint16_t>(
          get<0>(apply(read(OtpData::USB_WHITE_LABEL_ADDR::usb_white_label_addr))));

        if(whiteLabelAddr == 0 || whiteLabelAddr == 0xFFFF) { return std::nullopt; }

        std::uint16_t const stringDefinition = otpReadEcc(whiteLabelAddr + indexOffset);

        if(stringDefinition == 0x0000 || stringDefinition == 0xFFFF) { return std::nullopt; }

        constexpr std::uint8_t unicodeFlagBit = 0x80;
        constexpr std::uint8_t lengthMask     = 0x7F;

        bool const         isUnicode      = (stringDefinition & unicodeFlagBit) != 0;
        std::size_t const  characterCount = stringDefinition & lengthMask;
        std::uint8_t const dataRowOffset
          = static_cast<std::uint8_t>((stringDefinition >> 8) & 0xFF);

        if(isUnicode || characterCount == 0 || characterCount > MaxLen) { return std::nullopt; }

        std::uint16_t const          stringDataStartRow = whiteLabelAddr + dataRowOffset;
        Kvasir::StaticString<MaxLen> result;
        result.resize(characterCount);

        auto              outputIter = result.begin();
        std::size_t const numRows    = (characterCount + 1) / 2;

        for(std::size_t rowIndex = 0; rowIndex < numRows; ++rowIndex) {
            std::uint16_t const packedChars
              = otpReadEcc(static_cast<std::uint16_t>(stringDataStartRow + rowIndex));

            *outputIter++ = static_cast<char>(packedChars & 0xFF);

            if(outputIter != result.end()) {
                *outputIter++ = static_cast<char>((packedChars >> 8) & 0xFF);
            }
        }

        return result;
    }

    static inline CriticalFlags readCriticalFlags() {
        constexpr std::uint32_t      critical = 0x0002;
        std::array<std::uint32_t, 2> buffer{};
        auto const length = RomFunctions::getSysInfo(buffer.data(), buffer.size(), critical);
        if(length < 2 || (buffer[0] & critical) == 0) { return {}; }
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

    static inline OtpPageLocks readOtpPageLocks() {
        using Raw = Kvasir::Peripheral::OTP_DATA_RAW::Registers<>;
        // lock_s, lock_ns, lock_bl of a page's LOCK1 row (the raw alias, one load); any non-zero means locked.
        auto const isLocked = []<typename Lock1>(Lock1) -> bool {
            auto const v = apply(read(Lock1::lock_s), read(Lock1::lock_ns), read(Lock1::lock_bl));
            return std::to_underlying(get<0>(v)) != 0 || std::to_underlying(get<1>(v)) != 0
                || std::to_underlying(get<2>(v)) != 0;
        };

        return {
          .page1  = isLocked(Raw::PAGE1_LOCK1{}),
          .page2  = isLocked(Raw::PAGE2_LOCK1{}),
          .page29 = isLocked(Raw::PAGE29_LOCK1{}),
          .page30 = isLocked(Raw::PAGE30_LOCK1{}),
          .page31 = isLocked(Raw::PAGE31_LOCK1{}),
        };
    }

    static inline std::optional<Kvasir::StaticString<30>> readWhiteLabelSerialNumber() {
        return readWhiteLabelString<WhiteLabelStrIndex::usbDeviceSerialNumber, 30>();
    }
#endif
}   // namespace detail
}   // namespace Kvasir
