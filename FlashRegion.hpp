#pragma once
// A region of the QSPI flash the firmware runs from, a Kvasir::Hal::NorRegion (kvasir/Hal/Flash.hpp; the bounds and
// the page split are Hal::FlashRegionBase's) in the shape fs/log/Flash.hpp takes too (nothing here includes fs).
//
// - erase: one 4096-byte sector, boot ROM flash_range_erase (RP2350 data sheet 5.4.8.10).
// - program: whole 256-byte pages only (flash_range_program, 5.4.8.11); the bytes not asked for
//   are 0xFF, which programs nothing. A page is so programmed once per record that touches it.
//   How often that is allowed between erases neither chip states: PY25Q128HA V2.2 10.30 defers
//   to an application note that is not on disk, W25Q128JV 8.2.13 only allows partial pages.
// - read: from XIP, through the untranslated, uncached window (XipUntranslatedBase): a region is
//   addressed in flash bytes, like erase and program, so it must not move with the address
//   translation the boot ROM sets up for an image launched from a partition - through 0x10000000
//   such an image sees its own partition only, and a region in a data partition is a bus fault.
//
// Both mask interrupts and stop XIP for the whole operation (an erase takes tens of ms): nothing
// may run from flash meanwhile, on the other core neither.
#include "FlashOps.hpp"
#include "Rom.hpp"
#include "flash.hpp"
#include "kvasir/Hal/Flash.hpp"
#include "kvasir/Port/LinkerSymbols.hpp"
#include "kvasir/Register/Memory.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Flash {

    /// `Sectors` sectors from `Offset` (bytes from the start of the flash, sector aligned).
    template<std::uint32_t Offset, std::uint32_t Sectors>
    struct Region
      : Hal::FlashRegionBase<Region<Offset, Sectors>,
                             Hal::FlashGeometry{xipBase + Offset,
                                                static_cast<std::uint32_t>(sectorSize),
                                                static_cast<std::uint32_t>(pageSize),
                                                Sectors},
                             Hal::PageSplit::split> {
        static_assert(Offset % sectorSize == 0,
                      "the region must start on a sector");
        static constexpr std::uint32_t pageBytes = pageSize;   // what program() pads to
        static constexpr std::uint32_t start     = Offset;

        static void readAt(std::uint32_t        addr,
                           std::span<std::byte> out) {
            Kvasir::Memory::loadBytes(xipUntranslatedBase + Offset + addr, out.data(), out.size());
        }

        // the integer flash offset only: no flash symbol's address reaches the boot ROM's RAM function
        static bool programPage(std::uint32_t        page,
                                std::span<std::byte const,
                                          pageBytes> bytes) {
            Kvasir::Core::IrqGuard const guard{};
            Kvasir::detail::flashWrite(Offset + page,
                                       reinterpret_cast<std::uint8_t const*>(bytes.data()),
                                       bytes.size());
            return true;
        }

        static bool eraseSector(std::uint32_t sector) {
            Kvasir::Core::IrqGuard const guard{};
            Kvasir::detail::flashErase(Offset + sector * sectorSize, 1);
            return true;
        }

        /// The region lies past the end of the firmware's flash image (_LINKER_image_end_flash_: every section the
        /// linker loads into flash, whatever the chip's script loads after .data). Kvasir::Flash::CellStore refuses to
        /// touch a region that does not. Hides FlashRegionBase's: the RP2350's partitions.
        [[nodiscard]] static bool outsideImage() {
            std::uintptr_t const end = Kvasir::Startup::linkerAddress<_LINKER_image_end_flash_>();
#ifdef KVASIR_CHIP_RP2350
            // `end` is a runtime address, Offset a flash byte: an image launched from a partition is linked at
            // 0x10000000 but lies at the partition's base, so compare in flash bytes (flash_runtime_to_storage_addr,
            // RP2350 data sheet 5.4.8.13; it answers in the XIP space, 0x10000000 for flash byte 0 - measured). Its
            // last byte, since `end` may lie past the mapped window. A region in front of the image (the other A/B
            // partition) is refused too. Not mapped (a RAM image): as before.
            if(auto const last = Kvasir::RomFunctions::flashRuntimeToStorageAddr(
                 static_cast<std::uint32_t>(end - 1U));
               last >= 0)
            {
                return xipBase + Offset > static_cast<std::uint32_t>(last);
            }
#endif
            return xipBase + Offset >= end;
        }
    };

}}   // namespace Kvasir::Flash
