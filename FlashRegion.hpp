#pragma once
// A region of the QSPI flash the firmware runs from, in the shape fs/log/Flash.hpp takes
// (SectorBytes, sectorCount(), read, program, erase; nothing here includes fs).
//
// - erase: one 4096-byte sector, boot ROM flash_range_erase (RP2350 data sheet 5.4.8.10).
// - program: whole 256-byte pages only (flash_range_program, 5.4.8.11); the bytes not asked for
//   are 0xFF, which programs nothing. A page is so programmed once per record that touches it.
//   How often that is allowed between erases neither chip states: PY25Q128HA V2.2 10.30 defers
//   to an application note that is not on disk, W25Q128JV 8.2.13 only allows partial pages.
// - read: straight from XIP; the wrappers flush the XIP cache after every erase and program.
//
// Both mask interrupts and stop XIP for the whole operation (an erase takes tens of ms): nothing
// may run from flash meanwhile, on the other core neither.
#include "bootrom_functions.hpp"
#include "flash.hpp"
#include "kvasir/StartUp/LinkerSymbols.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#if __has_include("chip/rp2040.hpp") && defined(KVASIR_CORE_SCRATCH) && KVASIR_CORE_SCRATCH
// the scratch banks' load image follows .data's in flash (chip_rp2040/linker/chip.ld; declared as in its StartUp.hpp).
// Only with SCRATCH_BANKS: a firmware's own linker script (omniscope) may not define these
extern "C" {
extern std::byte _LINKER_INTERN_scratch_y_load_;
extern std::byte _LINKER_INTERN_scratch_y_data_start_;
extern std::byte _LINKER_INTERN_scratch_y_data_end_;
}
#endif

namespace Kvasir { namespace Flash {

    /// `Sectors` sectors from `Offset` (bytes from the start of the flash, sector aligned).
    template<std::uint32_t Offset, std::uint32_t Sectors>
    struct Region {
        static_assert(Offset % SectorSize == 0,
                      "the region must start on a sector");
        static constexpr std::uint32_t SectorBytes = SectorSize;
        static constexpr std::uint32_t PageBytes   = PageSize;   // what program() pads to
        static constexpr std::uint32_t Start       = Offset;
        static constexpr std::uint32_t Bytes       = Sectors * SectorBytes;

        [[nodiscard]] static constexpr std::uint32_t sectorCount() { return Sectors; }

        static bool read(std::uint32_t        addr,
                         std::span<std::byte> out) {
            if(std::uint64_t{addr} + out.size() > Bytes) { return false; }
            std::memcpy(out.data(),
                        reinterpret_cast<std::byte const*>(XipBase + Offset + addr),
                        out.size());
            return true;
        }

        static bool program(std::uint32_t              addr,
                            std::span<std::byte const> in) {
            if(std::uint64_t{addr} + in.size() > Bytes) { return false; }
            while(!in.empty()) {
                auto const page = addr & ~std::uint32_t{PageSize - 1U};
                auto const at   = addr - page;
                auto const n    = std::min<std::size_t>(PageSize - at, in.size());
                alignas(4) std::array<std::byte, PageSize> buf{};
                buf.fill(std::byte{0xFF});
                std::copy_n(in.begin(), n, buf.begin() + static_cast<std::ptrdiff_t>(at));
                {
                    Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> guard{};
                    Kvasir::detail::flash_write(Offset + page,
                                                reinterpret_cast<std::uint8_t const*>(buf.data()),
                                                buf.size());
                }
                addr += static_cast<std::uint32_t>(n);
                in = in.subspan(n);
            }
            return true;
        }

        /// The region lies past the end of the firmware's flash image (the load end of .data, on an RP2040 image with
        /// SCRATCH_BANKS of the scratch banks' image after it). Kvasir::Flash::CellStore refuses to touch a region that does not.
        [[nodiscard]] static bool outsideImage() {
            std::uintptr_t end
              = reinterpret_cast<std::uintptr_t>(std::addressof(_LINKER_data_start_flash_))
              + reinterpret_cast<std::size_t>(std::addressof(_LINKER_data_size_));
#if __has_include("chip/rp2040.hpp") && defined(KVASIR_CORE_SCRATCH) && KVASIR_CORE_SCRATCH
            end = std::max<std::uintptr_t>(
              end,
              reinterpret_cast<std::uintptr_t>(std::addressof(_LINKER_INTERN_scratch_y_load_))
                + static_cast<std::uintptr_t>(
                  std::addressof(_LINKER_INTERN_scratch_y_data_end_)
                  - std::addressof(_LINKER_INTERN_scratch_y_data_start_)));
#endif
            return XipBase + Offset >= end;
        }

        static bool erase(std::uint32_t sector) {
            if(sector >= Sectors) { return false; }
            Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> guard{};
            Kvasir::detail::flash_erase(Offset + sector * SectorBytes, 1);
            return true;
        }
    };

}}   // namespace Kvasir::Flash
