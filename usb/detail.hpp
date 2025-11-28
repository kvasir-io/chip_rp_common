#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace Kvasir { namespace USB { namespace detail {

    static void device_memory_memcpy(void*       dst,
                                     void const* src,
                                     std::size_t n) {
#ifdef __ARM_FEATURE_UNALIGNED
        // Prevent compiler from optimizing into unaligned accesses
        asm volatile("" : "+r"(dst), "+r"(src));

        std::uint8_t*       dst_byte = static_cast<std::uint8_t*>(dst);
        std::uint8_t const* src_byte = static_cast<std::uint8_t const*>(src);

        constexpr std::size_t kWordSize = sizeof(std::uint32_t);

        // Copy bytes until dst is word-aligned
        while(n > 0 && (reinterpret_cast<std::uintptr_t>(dst_byte) % kWordSize) != 0) {
            *dst_byte++ = *src_byte++;
            --n;
        }

        // If both pointers are now word-aligned, use word copies
        if((reinterpret_cast<std::uintptr_t>(src_byte) % kWordSize) == 0) {
            std::uint32_t*       dst_word = reinterpret_cast<std::uint32_t*>(dst_byte);
            std::uint32_t const* src_word = reinterpret_cast<std::uint32_t const*>(src_byte);

            while(n >= kWordSize) {
                *dst_word++ = *src_word++;
                n -= kWordSize;
            }

            dst_byte = reinterpret_cast<std::uint8_t*>(dst_word);
            src_byte = reinterpret_cast<std::uint8_t const*>(src_word);
        }

        // Copy remaining bytes
        while(n > 0) {
            *dst_byte++ = *src_byte++;
            --n;
        }
#else
        std::memcpy(dst, src, n);
#endif
    }

}}}   // namespace Kvasir::USB::detail
