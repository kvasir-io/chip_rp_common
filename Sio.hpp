#pragma once
#include "chip/Interrupt.hpp"
#include "core/core.hpp"
#include "kvasir/Register/Register.hpp"

#include <cstdint>
#include <optional>
#include <peripherals/SIO.hpp>
#include <type_traits>

// The single-cycle IO block's inter-core facilities: which core am I, the two-way mailbox
// FIFO, and (RP2350) the doorbells. Every register here is banked per core: the same address
// reads the local core's side, so none of these functions take a core argument.
//
// The hardware spinlocks (SPINLOCK0..31) are deliberately not wrapped. Erratum RP2350-E2:
// a write to any SIO register above +0x180 (doorbells, MTIME, TMDS, PERI_NONSEC) aliases
// the spinlocks and releases whichever one happens to be held. Kvasir::Atomic::Spinlock is
// a software lock on exclusive accesses, which the RP2350 bus supports across cores and
// which no erratum touches; the doorbells are then safe to use. The RP2040 has no
// exclusives: there the atomic shim's cross-core lock is SPINLOCK31 (chip/CrossCoreLock.hpp)
// and Atomic::Spinlock is built on that; SPINLOCK0..30 are the application's.
namespace Kvasir::Sio {

using Regs = Kvasir::Peripheral::SIO::Registers<0>;

// 0 on core 0, 1 on core 1.
[[nodiscard]] inline std::uint32_t cpuId() {
    return get<0>(apply(Register::read(Regs::CPUID::FULLREGISTER)));
}

// Functor form, for template parameters such as uc_log's MulticoreRttComBackend.
struct CpuId {
    std::uint32_t operator()() const { return cpuId(); }
};

namespace detail {
    // The FIFO interrupt of core `Core`: one line with one name on the RP2350 (sio_fifo,
    // banked per core), one line per core on the RP2040 (sio_proc0 / sio_proc1).
    template<unsigned Core,
             typename I = Kvasir::Interrupt>
    constexpr auto fifoIrq() {
        if constexpr(requires { I::sio_fifo; }) {
            return I::sio_fifo;
        } else if constexpr(Core == 0) {
            return I::sio_proc0;
        } else {
            return I::sio_proc1;
        }
    }
}   // namespace detail

// The mailbox: two 8-deep FIFOs of 32-bit words, one per direction. write() puts a word
// into the other core's inbox, read() takes one from this core's. Also the channel the
// bootrom's core-1 launch handshake runs over, which is why Multicore::launchCore1 drains
// it and masks the FIFO interrupt while it works.
struct Fifo {
    // Room to write.
    [[nodiscard]] static bool writable() {
        return get<0>(apply(Register::read(Regs::FIFO_ST::rdy))) != 0;
    }

    // A word is waiting.
    [[nodiscard]] static bool readable() {
        return get<0>(apply(Register::read(Regs::FIFO_ST::vld))) != 0;
    }

    // Write if there is room. The sev() wakes a reader sleeping in wfe().
    static bool tryWrite(std::uint32_t value) {
        if(!writable()) { return false; }
        apply(Register::write(Regs::FIFO_WR::FULLREGISTER, value));
        Core::sev();
        return true;
    }

    // Read if a word is waiting.
    [[nodiscard]] static std::optional<std::uint32_t> tryRead() {
        if(!readable()) { return std::nullopt; }
        return get<0>(apply(Register::read(Regs::FIFO_RD::FULLREGISTER)));
    }

    // Write, spinning until there is room. The reader does not signal a pop, so this cannot
    // sleep; the FIFO drains in a few cycles once the other core reads, so it does not
    // need to.
    static void write(std::uint32_t value) {
        while(!writable()) {}
        apply(Register::write(Regs::FIFO_WR::FULLREGISTER, value));
        Core::sev();
    }

    // Read, sleeping in wfe() until a word arrives. Every writer sends an event, and a
    // stale event only costs one extra poll. Unbounded: for a protocol where the other
    // core cannot die without this one being allowed to hang too (a boot self-test).
    [[nodiscard]] static std::uint32_t read() {
        while(!readable()) { Core::wfe(); }
        return get<0>(apply(Register::read(Regs::FIFO_RD::FULLREGISTER)));
    }

    // Read with a deadline on any clock with a static now(): nullopt if nothing arrived.
    // The read for a mailbox whose other side may have stopped.
    template<typename TimePoint>
    [[nodiscard]] static std::optional<std::uint32_t> read(TimePoint deadline) {
        while(!readable()) {
            if(TimePoint::clock::now() > deadline) { return std::nullopt; }
        }
        return get<0>(apply(Register::read(Regs::FIFO_RD::FULLREGISTER)));
    }

    // Discard everything waiting.
    static void drain() {
        while(readable()) {
            static_cast<void>(get<0>(apply(Register::read(Regs::FIFO_RD::FULLREGISTER))));
        }
    }

    // Sticky error flags: this core read an empty FIFO / wrote a full one.
    [[nodiscard]] static bool hadUnderflow() {
        return get<0>(apply(Register::read(Regs::FIFO_ST::roe))) != 0;
    }

    [[nodiscard]] static bool hadOverflow() {
        return get<0>(apply(Register::read(Regs::FIFO_ST::wof))) != 0;
    }

    // Both flags are write-one-to-clear; the whole-register write avoids a read-modify-write
    // that would also re-clear whichever flag happened to be set.
    static void clearErrors() {
        apply(
          Register::write(Regs::FIFO_ST::FULLREGISTER, Register::value<(1U << 3U) | (1U << 2U)>()));
    }
};

// One of the eight doorbells (RP2350). ring() sets the bell in the *other* core's inbox,
// pending()/clear() look at this core's. With sio_bell enabled in a core's NVIC, a pending
// bell is an interrupt on that core: the cheapest "wake the other core" there is.
//
// R is a template parameter only so the body stays dependent: on a chip whose SIO has no
// doorbells (RP2040) the struct exists but instantiating it is the static_assert below.
template<unsigned N, typename R = Regs>
struct Doorbell {
    static_assert(N < 8,
                  "eight doorbells, 0..7");
    static_assert(
      requires { typename R::DOORBELL_OUT_SET; },
      "this chip has no doorbells");

    static constexpr std::uint32_t bit = 1U << N;

    static void ring() {
        apply(Register::write(R::DOORBELL_OUT_SET::doorbell_out_set, Register::value<bit>()));
    }

    [[nodiscard]] static bool pending() {
        return (get<0>(apply(Register::read(R::DOORBELL_IN_SET::doorbell_in_set))) & bit) != 0;
    }

    static void clear() {
        apply(Register::write(R::DOORBELL_IN_CLR::doorbell_in_clr, Register::value<bit>()));
    }

    // Still pending in the other core's inbox: it has not cleared the ring yet. A second
    // ring while this is true is coalesced with the first, so a sender that needs every
    // ring counted waits for this to drop.
    [[nodiscard]] static bool ringPending() {
        return (get<0>(apply(Register::read(R::DOORBELL_OUT_SET::doorbell_out_set))) & bit) != 0;
    }
};

// Startup-list peripherals that claim the SIO interrupts for the core whose list they are
// in. On the RP2350 the FIFO interrupt is one line, banked per core (sio_fifo); on the
// RP2040 each core has its own line (sio_proc0 / sio_proc1), so the ISR has to know which
// core's list it is in: FifoIsrOn<Core, F> names it, and Startup refuses the other core's
// list (startupCore). FifoIsr<F> is core 0's. Priority is the NVIC priority (0 highest),
// like every other driver's isrPriority; `I` is the chip interrupt table.
template<unsigned Core, void (*F)(), int Priority = 0, typename I = Kvasir::Interrupt>
struct FifoIsrOn {
    static_assert(Core < 2,
                  "two cores");
    static constexpr unsigned startupCore = Core;

    using Irq = std::decay_t<decltype(detail::fifoIrq<Core, I>())>;

    static constexpr auto initStepInterruptConfig
      = list(Nvic::makeSetPriority<Priority>(Irq{}), Nvic::makeClearPending(Irq{}));
    static constexpr auto initStepPeripheryEnable = list(Nvic::makeEnable(Irq{}));

    // Level interrupt from FIFO_ST.vld: it stays asserted until the FIFO is drained, so the
    // handler is expected to read. It is also left pending by the launch handshake, hence
    // the readable() check: an ISR that assumes a word is waiting would underflow.
    static void onIsr() {
        if(Fifo::readable()) { F(); }
    }

    static constexpr Nvic::Isr<std::addressof(onIsr), Irq> isr{};
};

template<void (*F)(), int Priority = 0, typename I = Kvasir::Interrupt>
using FifoIsr = FifoIsrOn<0, F, Priority, I>;

// F receives the mask of bells (bit N for Doorbell<N>) that were pending, restricted to
// Bells...; those are cleared before F runs, so a ring that lands during F sets the bit
// again and the line, which stays asserted while any bell is pending, fires once more.
// Priority is the NVIC priority (0 highest); it comes before the bells because a pack has
// to be last: DoorbellIsr<&onRing, 3, 0, 1> is priority 3, bells 0 and 1.
namespace detail {
    // Makes a lookup depend on a template's own parameters (a struct, not an alias: an alias
    // is substituted at once and the lookup would be checked at definition), so a chip
    // without doorbells (the RP2040) only errors when a DoorbellIsr is instantiated.
    template<typename T, auto...>
    struct Dependent {
        using type = T;
    };
}   // namespace detail

template<void (*F)(std::uint32_t), int Priority, unsigned... Bells>
struct DoorbellIsr {
    using Irq
      = std::decay_t<decltype(detail::Dependent<Kvasir::Interrupt, Bells...>::type::sio_bell)>;

    static constexpr std::uint32_t mask = ((1U << Bells) | ...);

    static constexpr auto initStepInterruptConfig
      = list(Nvic::makeSetPriority<Priority>(Irq{}), Nvic::makeClearPending(Irq{}));
    static constexpr auto initStepPeripheryEnable = list(Nvic::makeEnable(Irq{}));

    static void onIsr() {
        using R = typename detail::Dependent<Regs, Bells...>::type;
        auto const pending
          = get<0>(apply(Register::read(R::DOORBELL_IN_SET::doorbell_in_set))) & mask;
        apply(Register::write(R::DOORBELL_IN_CLR::doorbell_in_clr, pending));
        F(pending);
    }

    static constexpr Nvic::Isr<std::addressof(onIsr), Irq> isr{};
};

}   // namespace Kvasir::Sio
