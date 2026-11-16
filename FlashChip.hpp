#pragma once
// The whole QSPI flash in flash byte offsets (not XIP addresses), for code that writes the flash outside a
// Flash::Region of its own - a firmware writing its own A/B partner, a RAM image working on a chip's flash:
// - read: through the untranslated, uncached XIP window (flash.hpp xipUntranslatedBase), so every byte of the flash,
//   whatever window the boot ROM launched the running image with, and never a line the boot ROM's erase or program
//   left stale in the cache;
// - eraseSector: one 4096-byte sector, program: whole 256-byte pages, both through the boot ROM with interrupts
//   masked and XIP off around the ROM call (FlashOps.hpp: each recorded as the operation in flight, timed, the watchdog
//   boot vector armed while it runs) - nothing may run from flash meanwhile, on the other core neither.
// What it leaves to its caller: which offsets may be written (nothing here refuses one), the read-back, and whether
// the XIP timing fits the clock before a write (timingTrusted(), XipTiming.hpp: what is read back to verify would be
// shifted otherwise). Flash::Region does all three for a region fixed at compile time.
#include "FlashOps.hpp"
#include "XipTiming.hpp"
#include "core/InterruptMask.hpp"
#include "flash.hpp"
#include "kvasir/Register/Memory.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Flash {

    struct Chip {
        /// The erase unit and the program unit (flash.hpp sectorSize, pageSize).
        static constexpr std::uint32_t sectorBytes = sectorSize;
        static constexpr std::uint32_t pageBytes   = pageSize;

        /// out.size() bytes from flash byte @p offset.
        static void read(std::uint32_t           offset,
                         std::span<std::uint8_t> out) {
            auto const bytes = std::as_writable_bytes(out);
            Kvasir::Memory::loadBytes(xipUntranslatedBase + offset, bytes.data(), bytes.size());
        }

        /// Erases the sector at flash byte @p offset (sector aligned). The boot ROM answers nothing: read it back.
        static void eraseSector(std::uint32_t offset) {
            Kvasir::Core::IrqGuard const guard{};
            Kvasir::Detail::flashErase(offset, 1);
        }

        /// Programs @p bytes at flash byte @p offset: page aligned, whole pages (a page's rest as 0xFF programs
        /// nothing), erased before. The boot ROM answers nothing: read it back.
        static void program(std::uint32_t                 offset,
                            std::span<std::uint8_t const> bytes) {
            Kvasir::Core::IrqGuard const guard{};
            Kvasir::Detail::flashWrite(offset, bytes.data(), bytes.size());
        }

        /// Whether XIP reads are right at the clock clk_sys runs at now (Clocks::xipTimingTrusted).
        [[nodiscard]] static bool timingTrusted() { return Kvasir::Clocks::xipTimingTrusted(); }
    };

}}   // namespace Kvasir::Flash
