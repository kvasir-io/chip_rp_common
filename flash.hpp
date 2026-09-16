#pragma once
#include "bootrom_functions.hpp"

#include <array>
#include <bit>
#include <cassert>
#include <cstdint>

namespace Kvasir { namespace Flash {

    /// The XIP window: flash byte 0 is read at this address.
    inline constexpr std::uint32_t XipBase = 0x1000'0000U;
    /// The smallest erasable unit (a sector) and the largest programmable unit (a page)
    /// of the parts the boot ROM's flash functions drive.
    inline constexpr std::size_t SectorSize = 4096;
    inline constexpr std::size_t PageSize   = 256;

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

            eraseAndWrite(StorageAddress, std::as_bytes(std::span{std::addressof(newV), 1}));
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
