#pragma once
// Who holds the DMA CRC sniffer (DmaCrc.hpp) right now, for a debug check. There is one sniffer per DMA controller
// (SNIFF_CTRL, SNIFF_DATA), and a SnifferEngine keeps its sum in SNIFF_DATA from its constructor to its destructor:
// a second user that arms it in between - an interrupt, the other core - corrupts both CRCs without a word, and none
// of the Startup list rules can see it (the users of the image's engine are library calls, not list entries). With
// debug checks on (KVASIR_DCHECK_LEVEL > 0: the debug and sanitize variants, kvasir/Runtime/Check.hpp) such an arm
// is a programming error and panics (Panic::Cause::checkFailed, the line names the sniffer); with them off nothing of
// this is in the image. Register-free, so it is host-tested (tests/dma_crc_hold_test.cpp).
#include "kvasir/Runtime/Check.hpp"

#include <atomic>

namespace Kvasir::DmaCrc::Detail {

struct Hold {
#if KVASIR_DCHECK_LEVEL > 0
    // A load and a store, no read-modify-write: inline on the Cortex-M0+ too, where an atomic exchange is a call into
    // the lock shim. An interrupt between the two is a window of a few instructions; the check is a debug aid, not
    // a lock.
    static inline std::atomic<bool> held{};
#endif

    static void take() {
#if KVASIR_DCHECK_LEVEL > 0
        KVASIR_DCHECK(
          !held.load(std::memory_order_relaxed),
          "DMA CRC sniffer armed while another user holds it: one context, one core at a time");
        held.store(true, std::memory_order_relaxed);
#endif
    }

    static void give() {
#if KVASIR_DCHECK_LEVEL > 0
        held.store(false, std::memory_order_relaxed);
#endif
    }

    /// Whether a user holds the sniffer (always false with debug checks off).
    [[nodiscard]] static bool taken() {
#if KVASIR_DCHECK_LEVEL > 0
        return held.load(std::memory_order_relaxed);
#else
        return false;
#endif
    }
};

/// The sniffer held for this object's lifetime.
struct Held {
    Held() { Hold::take(); }

    ~Held() { Hold::give(); }

    Held(Held const&)            = delete;
    Held& operator=(Held const&) = delete;
};

}   // namespace Kvasir::DmaCrc::Detail
