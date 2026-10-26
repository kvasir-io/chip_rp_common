#pragma once
#include "FlashOps.hpp"
#include "chip/RpTraits.hpp"
#include "kvasir/Register/Memory.hpp"
#include "kvasir/Util/attributes.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Flash {

    /// The XIP window: flash byte 0 is read at this address.
    inline constexpr std::uint32_t xipBase = 0x1000'0000U;
    /// The smallest erasable unit (a sector) and the largest programmable unit (a page)
    /// of the parts the boot ROM's flash functions drive.
    inline constexpr std::size_t sectorSize = 4096;
    inline constexpr std::size_t pageSize   = 256;

    /// The same flash through the uncached window: a read neither hits nor fills the XIP cache, so a background
    /// check (Kvasir::ImageCheck) reads the flash itself and leaves the hot code cached. RP2040: XIP_NOCACHE_NOALLOC_BASE
    /// 0x13000000 (datasheet 2.2.2, md l.1249); RP2350: 0x14000000 "XIP, Uncached" (address map md l.14751, 4.4.1
    /// md l.17868), through the QMI's address translation like 0x10000000 - unlike 0x1c000000. Open at reset (RP2350
    /// XIP CTRL.NO_UNCACHED_SEC/NONSEC reset 0, md l.18127-18128).
    inline constexpr std::uint32_t xipUncachedBase = Rp::ChipTraits::xipUncachedBase;

    /// Flash byte 0 regardless of the QMI's address translation. An image the boot ROM launched from a partition
    /// runs with ATRANS0 mapping 0x10000000 onto that partition alone (RP2350 data sheet 5.1.19, 12.14.4) and the
    /// other three windows at size 0, so a read of anything outside the image partition through 0x10000000 or
    /// 0x14000000 - a data partition, say - is a bus fault. RP2350: 0x1c000000 "XIP, uncached, untranslated"
    /// (address map md l.14753, 4.4.1 l.17872), for Secure code only by default: XIP CTRL.NO_UNTRANSLATED_SEC
    /// resets to 0, NO_UNTRANSLATED_NONSEC to 1 (md l.18125-18126) - a Non-secure image would need flash_op
    /// ASPACE_STORAGE instead. Uncached as well, so a read never sees a line the boot ROM's erase/program left
    /// stale. RP2040: no translation exists; XIP_NOCACHE_NOALLOC_BASE (md l.1249) for the same uncached read.
    inline constexpr std::uint32_t xipUntranslatedBase = Rp::ChipTraits::xipUntranslatedBase;

    [[nodiscard]] constexpr std::uint32_t uncached(std::uint32_t xipAddress) {
        return xipAddress - xipBase + xipUncachedBase;
    }

    /// An XIP window address that names a flash byte (as eraseAndWrite() and the boot ROM's flash functions take
    /// it, `address - XipBase`), moved to XipUntranslatedBase: where that byte is read in any image.
    [[nodiscard]] constexpr std::uint32_t untranslated(std::uint32_t xipAddress) {
        return xipAddress - xipBase + xipUntranslatedBase;
    }

    /// A reader for Kvasir::ImageCheck::Software: flash read through the uncached window.
    struct UncachedRead {
        static std::uint32_t word(std::uintptr_t address) {
            return Kvasir::Memory::load<std::uint32_t>(
              uncached(static_cast<std::uint32_t>(address)));
        }

        static std::uint8_t byte(std::uintptr_t address) {
            return Kvasir::Memory::load<std::uint8_t>(
              uncached(static_cast<std::uint32_t>(address)));
        }
    };

    static inline void eraseAndWrite(std::uint32_t              addr,
                                     std::span<std::byte const> data) {
        constexpr std::size_t flashBlockSize{sectorSize};
        assert(addr % flashBlockSize == 0);
        // Erase n 4096-byte sectors, then program page by page.
        std::uint32_t offset = addr - xipBase;
        {
            std::size_t const eraseBlocks
              = data.size() / flashBlockSize + (data.size() % flashBlockSize == 0 ? 0 : 1);
            Kvasir::Core::IrqGuard const guard{};
            Kvasir::detail::flashErase(offset, eraseBlocks);
        }
        constexpr std::size_t writeBlockSize{pageSize};
        while(!data.empty()) {
            std::array<std::byte, writeBlockSize> buffer{};
            std::copy_n(data.begin(), std::min(data.size(), writeBlockSize), buffer.begin());
            {
                Kvasir::Core::IrqGuard const guard{};
                Kvasir::detail::flashWrite(offset,
                                           reinterpret_cast<std::uint8_t const*>(buffer.data()),
                                           buffer.size());
            }
            offset += writeBlockSize;
            data = data.subspan(std::min(data.size(), writeBlockSize));
        }
    }
}}   // namespace Kvasir::Flash
