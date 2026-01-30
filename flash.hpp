#pragma once
#include "bootrom_functions.hpp"

#include <array>
#include <cassert>
#include <cstdint>

namespace Kvasir { namespace Flash {

    static inline void eraseAndWrite(std::uint32_t              addr,
                                     std::span<std::byte const> data) {
        constexpr std::size_t flashBlockSize{4096};
        assert(addr % flashBlockSize == 0);
        //Löschen von n 4096 byte Speicherbereichen
        std::uint32_t offset = addr - 0x10000000;
        {
            std::size_t eraseBlocks
              = data.size() / flashBlockSize + (data.size() % flashBlockSize == 0 ? 0 : 1);
            Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> guard{};
            Kvasir::detail::flash_erase(offset, eraseBlocks);
        }
        constexpr std::size_t writeBlockSize{256};
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

    template<typename Clock, typename T, typename Crc>
    struct SimpleEeprom {
        static auto calcCrc(T const& v) {
            return Crc::calc(std::as_bytes(std::span{std::addressof(v), 1}));
        }

        struct ValueStruct {
            T                  v{};
            typename Crc::type crc{};
        };

        [[gnu::section(".eeprom"), gnu::aligned(4096)]] static inline ValueStruct flashValue{};

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
              reinterpret_cast<std::uint32_t>(std::addressof(flashValue)),
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

            eraseAndWrite(reinterpret_cast<std::uint32_t>(std::addressof(flashValue)),
                          std::as_bytes(std::span{std::addressof(newV), 1}));
        }

        static void writeValue() {
            auto const currentFlashValue = readFlashValue();
            if(ramCopy != currentFlashValue) { internalWrite(); }
        }
    };
}}   // namespace Kvasir::Flash
