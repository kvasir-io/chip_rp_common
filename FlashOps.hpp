#pragma once
// Erase and program the flash through the boot ROM, with XIP off around the ROM call.
#include "Rom.hpp"
#include "XipControl.hpp"

#include <cstddef>
#include <cstdint>
#include <kvasir/Util/attributes.hpp>

namespace Kvasir {
namespace detail {
    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void flashEraseImpl(FlashXipDisabler& xipDisabler,
                                                                     void (*erase)(std::uint32_t,
                                                                                   std::size_t,
                                                                                   std::uint32_t,
                                                                                   std::uint8_t),
                                                                     std::uint32_t offset,
                                                                     std::size_t   blocks) {
        KVASIR_RAM_FUNC_MARK();
        XipGuard const guard{xipDisabler};
        erase(offset, blocks * 4096, 1 << 16, 0xD8);
    }

    static inline void flashErase(std::uint32_t offset,
                                  std::size_t   blocks) {
        auto erase = RomFunctions::getRomFunctionPointer<
          'R',
          'E',
          void (*)(std::uint32_t, std::size_t, std::uint32_t, std::uint8_t)>();

        FlashXipDisabler xipDisabler{};
        flashEraseImpl(xipDisabler, erase, offset, blocks);
    }

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
    flashWriteImpl(FlashXipDisabler& xipDisabler,
                   void (*write)(std::uint32_t,
                                 std::uint8_t const*,
                                 std::size_t),
                   std::uint32_t       offset,
                   std::uint8_t const* data,
                   std::size_t         size) {
        KVASIR_RAM_FUNC_MARK();
        XipGuard const guard{xipDisabler};
        write(offset, data, size);
    }

    static inline void flashWrite(std::uint32_t       offset,
                                  std::uint8_t const* data,
                                  std::size_t         size) {
        auto write = RomFunctions::getRomFunctionPointer<
          'R',
          'P',
          void (*)(std::uint32_t, std::uint8_t const*, std::size_t)>();

        FlashXipDisabler xipDisabler{};
        flashWriteImpl(xipDisabler, write, offset, data, size);
    }
}   // namespace detail

namespace Bootrom {
    /// Erase `blocks` 4096-byte sectors at flash `offset` (from the start of flash, not the
    /// XIP window). Runs with XIP disabled; mask interrupts around it.
    inline void flashErase(std::uint32_t offset,
                           std::size_t   blocks) {
        detail::flashErase(offset, blocks);
    }

    /// Program `size` bytes (a multiple of 256, page aligned `offset`) from `data`. Runs
    /// with XIP disabled; mask interrupts around it.
    inline void flashWrite(std::uint32_t       offset,
                           std::uint8_t const* data,
                           std::size_t         size) {
        detail::flashWrite(offset, data, size);
    }
}   // namespace Bootrom
}   // namespace Kvasir
