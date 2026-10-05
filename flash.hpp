#pragma once
#include "bootrom_functions.hpp"

#include <array>
#include <bit>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

namespace Kvasir { namespace Flash {

    /// The XIP window: flash byte 0 is read at this address.
    inline constexpr std::uint32_t XipBase = 0x1000'0000U;
    /// The smallest erasable unit (a sector) and the largest programmable unit (a page)
    /// of the parts the boot ROM's flash functions drive.
    inline constexpr std::size_t SectorSize = 4096;
    inline constexpr std::size_t PageSize   = 256;

    /// The same flash through the uncached window: a read neither hits nor fills the XIP cache, so a background
    /// check (Kvasir::ImageCheck) reads the flash itself and leaves the hot code cached. RP2040: XIP_NOCACHE_NOALLOC_BASE
    /// 0x13000000 (datasheet 2.2.2, md l.1249); RP2350: 0x14000000 "XIP, Uncached" (address map md l.14751, 4.4.1
    /// md l.17868), through the QMI's address translation like 0x10000000 - unlike 0x1c000000. Open at reset (RP2350
    /// XIP CTRL.NO_UNCACHED_SEC/NONSEC reset 0, md l.18127-18128).
#if __has_include("chip/rp2040.hpp")
    inline constexpr std::uint32_t XipUncachedBase = 0x1300'0000U;
#else
    inline constexpr std::uint32_t XipUncachedBase = 0x1400'0000U;
#endif

    [[nodiscard]] constexpr std::uint32_t uncached(std::uint32_t xipAddress) {
        return xipAddress - XipBase + XipUncachedBase;
    }

    /// A reader for Kvasir::ImageCheck::Software: flash read through the uncached window.
    struct UncachedRead {
        static std::uint32_t word(std::uintptr_t address) {
            return *reinterpret_cast<std::uint32_t const volatile*>(
              uncached(static_cast<std::uint32_t>(address)));
        }

        static std::uint8_t byte(std::uintptr_t address) {
            return *reinterpret_cast<std::uint8_t const volatile*>(
              uncached(static_cast<std::uint32_t>(address)));
        }
    };

    static inline void eraseAndWrite(std::uint32_t              addr,
                                     std::span<std::byte const> data) {
        constexpr std::size_t flashBlockSize{SectorSize};
        assert(addr % flashBlockSize == 0);
        // Erase n 4096-byte sectors, then program page by page.
        std::uint32_t offset = addr - XipBase;
        {
            std::size_t eraseBlocks
              = data.size() / flashBlockSize + (data.size() % flashBlockSize == 0 ? 0 : 1);
            Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> guard{};
            Kvasir::detail::flash_erase(offset, eraseBlocks);
        }
        constexpr std::size_t writeBlockSize{PageSize};
        while(!data.empty()) {
            std::array<std::byte, writeBlockSize> buffer{};
            std::copy_n(data.begin(), std::min(data.size(), writeBlockSize), buffer.begin());
            {
                Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> guard{};
                Kvasir::detail::flash_write(offset,
                                            reinterpret_cast<std::uint8_t const*>(buffer.data()),
                                            buffer.size());
            }
            offset += writeBlockSize;
            data = data.subspan(std::min(data.size(), writeBlockSize));
        }
    }

    /// A SimpleEeprom record read without SimpleEeprom (no .eeprom section instantiated): the Legacy of a
    /// Kvasir::Flash::CellStore that imports it once. `XipAddress`: where it lives - the old StorageAddress on the
    /// RP2350, the .eeprom section's address on the RP2040 (the last sector of the chip's linker script, or the
    /// firmware's own). The CRC runs over the record in flash: nothing of T's size is copied to the stack.
    template<typename T, typename Crc, std::uint32_t XipAddress>
    struct LegacySimpleEeprom {
        struct Layout {   // SimpleEeprom's ValueStruct: the same declaration, the same layout
            T                  v{};
            typename Crc::type crc{};
        };

        static bool read(T& out) {
            auto const*        at = reinterpret_cast<std::byte const*>(XipAddress);
            typename Crc::type stored{};
            std::memcpy(&stored, at + offsetof(Layout, crc), sizeof stored);
            if(stored != Crc::calc(std::span<std::byte const>{at, sizeof(T)})) { return false; }
            std::memcpy(std::addressof(out), at, sizeof(T));
            return true;
        }

        /// The sector of `Region` the record is in, if it is: the import goes elsewhere first.
        template<typename Region>
        static constexpr std::optional<std::uint32_t> sectorIn() {
            constexpr std::uint32_t offset = XipAddress - XipBase;
            if(offset >= Region::Start && offset < Region::Start + Region::Bytes) {
                return (offset - Region::Start) / SectorSize;
            }
            return std::nullopt;
        }
    };

    template<typename Clock, typename T, typename Crc, std::uint32_t StorageAddress>
    struct SimpleEeprom {
        static auto calcCrc(T const& v) {
            return Crc::calc(std::as_bytes(std::span{std::addressof(v), 1}));
        }

        struct ValueStruct {
            T                  v{};
            typename Crc::type crc{};
        };

#if __has_include("chip/rp2040.hpp")
        [[gnu::section(".eeprom"), gnu::aligned(4096)]] static inline ValueStruct flashValue{};
#endif

        static_assert(StorageAddress % SectorSize == 0,
                      "StorageAddress needs to be aligned to a flash sector");

        /// Where the value lives, as an XIP window address: StorageAddress on the RP2350,
        /// the .eeprom section's `flashValue` on the RP2040 (the linker places it, hence
        /// not constexpr there).
#if __has_include("chip/rp2040.hpp")
        [[nodiscard]] static std::uint32_t address() {
            return static_cast<std::uint32_t>(
              std::bit_cast<std::uintptr_t>(std::addressof(flashValue)));
        }
#else
        [[nodiscard]] static constexpr std::uint32_t address() { return StorageAddress; }
#endif

        static inline T    ramCopy{};
        static inline bool valueRead{false};

        static T readFlashValue() {
#if __has_include("chip/rp2350.hpp")
            ValueStruct actualValue{};

            constexpr std::uint32_t flags = Kvasir::detail::FlashOpFlags::ASPACE_STORAGE
                                          | Kvasir::detail::FlashOpFlags::SECLEVEL_NONSECURE
                                          | Kvasir::detail::FlashOpFlags::OP_READ;

            auto const ret = Kvasir::detail::flash_op(
              flags,
              StorageAddress,
              sizeof(ValueStruct),
              reinterpret_cast<std::uint8_t*>(std::addressof(actualValue)));

            if(ret != 0) {
                UC_LOG_C("flash_op read failed: {}", ret);
                return T{};
            }

            if(actualValue.crc != calcCrc(actualValue.v)) { return T{}; }
            return actualValue.v;
#else
            if(flashValue.crc != calcCrc(flashValue.v)) { return T{}; }
            return flashValue.v;
#endif
        }

        static T& value() {
            if(!valueRead) {
#if __has_include("chip/rp2040.hpp")
                // The RP2040 reads and writes the linker's .eeprom section; StorageAddress
                // only says where the user expects it. Different means the linker script and
                // the template argument disagree.
                if(address() != StorageAddress) {
                    UC_LOG_E(
                      "SimpleEeprom: StorageAddress {:#010x} is not the .eeprom section "
                      "({:#010x}); the section is used",
                      StorageAddress,
                      address());
                }
#endif
                ramCopy   = readFlashValue();
                valueRead = true;
            }
            return ramCopy;
        }

        static void internalWrite() {
            ValueStruct newV;
            newV.v   = ramCopy;
            newV.crc = calcCrc(ramCopy);

            asm("" : "=m"(newV)::);

            // address(): where readFlashValue() reads - on the RP2040 the .eeprom section. The asm
            // makes it a run-time value: LTO would otherwise carry the section's address as a constant into the RAM
            // function that erases, and check_ram_funcs.py takes a flash address there for a
            // flash read with XIP off.
            std::uint32_t addr = address();
            asm("" : "+r"(addr));
            eraseAndWrite(addr, std::as_bytes(std::span{std::addressof(newV), 1}));
        }

        /// Write value() to flash if it differs from what flash holds. True when a write
        /// happened.
        static bool writeValue() {
            auto const currentFlashValue = readFlashValue();
            if(ramCopy == currentFlashValue) { return false; }
            internalWrite();
            return true;
        }
    };
}}   // namespace Kvasir::Flash
