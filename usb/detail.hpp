#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace Kvasir::USB::detail {

static inline void deviceMemoryMemcpy(void*       dst,
                                      void const* src,
                                      std::size_t n) {
    // The memory clobber, here and at the end, keeps the copy between what comes before and
    // after it: the controller reads an IN packet once AVAILABLE is written and writes an OUT one
    // before BUFF_STATUS says so, both volatile register accesses that plain DPRAM stores and
    // loads may otherwise move across. On every core, the M0+'s memcpy path too.
    asm volatile("" ::: "memory");
#ifdef __ARM_FEATURE_UNALIGNED
    // Prevent compiler from optimizing into unaligned accesses.
    asm volatile("" : "+r"(dst), "+r"(src));

    std::uint8_t*       dstByte = static_cast<std::uint8_t*>(dst);
    std::uint8_t const* srcByte = static_cast<std::uint8_t const*>(src);

    constexpr std::size_t kWordSize = sizeof(std::uint32_t);

    // Copy bytes until dst is word-aligned
    while(n > 0 && (reinterpret_cast<std::uintptr_t>(dstByte) % kWordSize) != 0) {
        *dstByte++ = *srcByte++;
        --n;
    }

    // If both pointers are now word-aligned, use word copies
    if((reinterpret_cast<std::uintptr_t>(srcByte) % kWordSize) == 0) {
        // via void* to silence -Wcast-align; alignment checked above
        std::uint32_t*       dstWord = static_cast<std::uint32_t*>(static_cast<void*>(dstByte));
        std::uint32_t const* srcWord
          = static_cast<std::uint32_t const*>(static_cast<void const*>(srcByte));

        while(n >= kWordSize) {
            *dstWord++ = *srcWord++;
            n -= kWordSize;
        }

        dstByte = reinterpret_cast<std::uint8_t*>(dstWord);
        srcByte = reinterpret_cast<std::uint8_t const*>(srcWord);
    }

    // Copy remaining bytes
    while(n > 0) {
        *dstByte++ = *srcByte++;
        --n;
    }
#else
    std::memcpy(dst, src, n);
#endif
    asm volatile("" : : : "memory");
}

}   // namespace Kvasir::USB::detail
