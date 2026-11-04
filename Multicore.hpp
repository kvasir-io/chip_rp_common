#pragma once
#include "Sio.hpp"
#include "WaitBounds.hpp"
#include "chip/Interrupt.hpp"
#include "core/core.hpp"
#include "kvasir/Atomic/Atomic.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/attributes.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <peripherals/IO_QSPI.hpp>
#include <peripherals/PSM.hpp>
#include <type_traits>
#ifdef KVASIR_CHIP_RP2350
    #include <peripherals/EPPB.hpp>
#else
    #include <peripherals/SYSCFG.hpp>
#endif

// Getting core 1 out of the bootrom and into our code. This is the chip half of
// Kvasir::Startup::SecondaryCore: it knows the power-on state machine and the handshake,
// the SDK half knows vector tables, stacks and init lists.
//
// After any reset the bootrom parks core 1 in its `wait_for_vector` loop, listening on the
// SIO FIFO. Core 0 sends `0, 0, 1, VTOR, SP, PC`; core 1 echoes every word back, and a
// wrong echo means "start over". The bootrom then writes VTOR and SP itself, zeroes the MPU
// and branches to PC. (pico-bootrom `wait_for_vector`, the same protocol on the RP2040 and
// the RP2350; pico-sdk multicore.c `multicore_launch_core1_raw`.) What the RP2350 bootrom
// does not do is enable the FPU: CPACR has only cp7 set when PC is entered, which
// SecondaryCore's trampoline fixes. The RP2040 has nothing to enable.
namespace Kvasir::Multicore {

using PSM = Kvasir::Peripheral::PSM::Registers<>;

// Hard-reset core 1 through the power-on state machine. On release the bootrom takes core 1
// back to its holding pen (on the RP2350 via a check that its RCP salt is still valid,
// which a proc1 reset does not disturb). The read-back after the set is not paranoia: it
// fences the APB write so the clear cannot overtake it.
inline void resetCore1() {
    apply(set(PSM::FRCE_OFF::proc1));
    Kvasir::Register::waitUntil<Kvasir::Chip::ReadBackBound>(
      Kvasir::Register::isSet(PSM::FRCE_OFF::proc1));
    apply(clear(PSM::FRCE_OFF::proc1));
}

// The handshake. `entry` gets its Thumb bit; `sp` is the initial stack pointer; `vtor` the
// vector table core 1 will run with. Bounded by `budget` polls in total so a core 1 that is
// not listening (already running old code after a core-0-only debugger restart, or a
// module with a bootrom that behaves differently) returns false instead of hanging core 0.
//
// Masks this core's FIFO interrupt for the duration so an application handler does not eat
// the echoes, and clears the pending bit afterwards: the echo traffic leaves the level
// interrupt asserted, and re-enabling it as-is would fire once into an empty FIFO.
template<typename I = Kvasir::Interrupt>
[[nodiscard]] inline bool launchCore1(std::uint32_t entry,
                                      std::uint32_t sp,
                                      std::uint32_t vtor,
                                      std::uint32_t budget = 1'000'000) {
    using FifoIrq = std::decay_t<decltype(Sio::Detail::fifoIrq<0, I>())>;
    Nvic::InterruptGuard<FifoIrq> const guard;

    std::array<std::uint32_t, 6> const sequence{0, 0, 1, vtor, sp, entry | 1U};

    std::size_t step = 0;
    while(step < sequence.size()) {
        auto const cmd = sequence[step];
        if(cmd == 0) {
            // Core 1 may be blocked in wfe() waiting for FIFO space; a 0 is what resets its
            // side of the protocol, and it must see the FIFO empty when it does.
            Sio::Fifo::drain();
            Core::sev();
        }
        while(!Sio::Fifo::tryWrite(cmd)) {
            if(budget-- == 0) { break; }
        }
        std::optional<std::uint32_t> response;
        while(!(response = Sio::Fifo::tryRead())) {
            if(budget-- == 0) { break; }
        }
        if(!response) {
            apply(Nvic::makeClearPending(FifoIrq{}));
            return false;
        }
        step = (*response == cmd) ? step + 1 : 0;
    }

    apply(Nvic::makeClearPending(FifoIrq{}));
    return true;
}

// Core 1 stopping core 0 on a fatal path (Port::StopOtherCore, StopOtherCore.hpp). The PSM cannot do it: FRCE_OFF.PROC0
// restarts the power-on sequence from core 0's stage and core 1 with it (RP2040 datasheet 2.13.2), and an RP2350 boot
// ROM reboot that finds an FRCE_OFF bit other than PROC1 set hangs (erratum RP2350-E19). An interrupt routed to core
// 0's NMI can: the NMI ignores PRIMASK and pre-empts every other exception (RP2350 datasheet 3.2.1), and the handler
// here parks core 0 until the reboot resets it.
//
// The line is IO_IRQ_QSPI (RP2040 IRQ 14, datasheet 2.3.2 Table 80; RP2350 IRQ 23, 3.2 Table 95). Core 0's copy of it
// is forced by IO_QSPI PROC0_INTF (RP2040 2.19.6.2 Table 331, RP2350 9.11.2 Table 843), which core 1 can write; nothing
// else drives it, as no Kvasir image enables an interrupt on the flash pins (PROC0_INTE stays 0). The routing is core
// 0's NMI mask: RP2040 SYSCFG PROC0_NMI_MASK (2.21.2 Table 354; chip-wide, FirstInitStep clears it), RP2350 EPPB
// NMI_MASK0 (3.7.5.1 Table 363; core-local, so core 0 writes it itself, reset by a warm reset of that core). Armed by
// core 0 before it launches core 1 (SecondaryCore through SecondaryCoreInit::armOtherCoreStop), whose list also puts
// onNmi() into core 0's NMI vector (SecondaryCoreInit::PrimaryIsr). Another NMI (the RP2350's RCP integrity check,
// 3.2.1) finds the force bit clear and ends as the unhandled-interrupt panic it was before.
struct Core0Park {
    using IoQspi             = Kvasir::Peripheral::IO_QSPI::Registers<>;
    static constexpr int irq = Kvasir::Interrupt::io_qspi.value;
    static_assert(irq >= 0 && irq < 32,
                  "the first NMI mask word");
    static constexpr std::uint32_t nmiMask = 1U << static_cast<unsigned>(irq);
    // Core 0's acknowledgement, polled by core 1: NMI entry takes a few dozen cycles, a stalled XIP fetch some
    // microseconds; this is about 1 ms at 266 MHz, after which core 1 goes on to its reboot regardless.
    static constexpr std::uint32_t ackPolls = 100'000;

    static inline bool volatile armed  = false;
    static inline bool volatile parked = false;

    // Core 0, before core 1 starts. The force bit first: a debugger's reset of core 0 alone after a park leaves it set
    // (IO_QSPI is reset with the chip only), and the mask would take core 0 into the park at once.
    static void arm() {
        apply(clear(IoQspi::PROC0_INTF::gpio_qspi_sclk_level_low));
        parked = false;
#ifdef KVASIR_CHIP_RP2350
        apply(write(Kvasir::Peripheral::EPPB::Registers<>::NMI_MASK0::nmi_mask0,
                    Kvasir::Register::value<nmiMask>()));
#else
        apply(write(Kvasir::Peripheral::SYSCFG::Registers<>::PROC0_NMI_MASK::FULLREGISTER,
                    Kvasir::Register::value<nmiMask>()));
#endif
        armed = true;
    }

    // Core 0's NMI: parked for good if core 1 forced the line, else the default end of an unexpected vector.
    [[noreturn]] static void onNmi() {
        if(get<0>(apply(read(IoQspi::PROC0_INTF::gpio_qspi_sclk_level_low))) == 0) {
            Kvasir::Nvic::DefaultIsrs::onIsr();
        }
        parked = true;
        while(true) { asm volatile("wfe" ::: "memory"); }
    }

    // Core 1: force the line, then wait (bounded) for core 0 to be in onNmi(). True once it is.
    [[nodiscard]] static bool park() {
        if(!armed) { return false; }
        apply(set(IoQspi::PROC0_INTF::gpio_qspi_sclk_level_low));
        for(std::uint32_t i = 0; i != ackPolls; ++i) {
            if(parked) { return true; }
        }
        return parked;
    }

    using Isr = brigand::list<Kvasir::Nvic::Isr<std::addressof(onNmi), Kvasir::Nvic::Index<-14>>>;
};

// Core 0 keeping core 1 out of the flash while XIP is off (Flash::XipDisabler: every erase, program, direct-mode
// command and boot ROM flash call - FlashOps, FlashRegion, BootInfo's loadPartitionTable/explicitBuy, Qmi, the
// RP2040's flash_do_cmd). A core 1 that fetches from flash then reads junk or stalls the QMI; pico-sdk's
// multicore_lockout asks the other core over the SIO FIFO, which an application here may own. This takes the line
// Core0Park takes, mirrored: core 0 forces core 1's copy of IO_IRQ_QSPI (IO_QSPI PROC1_INTF, RP2040 datasheet 2.19.6.2
// Table 334, RP2350 9.11.2 Table 846), routed to core 1's NMI (RP2040 SYSCFG PROC1_NMI_MASK, 2.21.2 Table 355; RP2350
// EPPB NMI_MASK0, 3.7.5.1 Table 363, core-local: core 1 writes its own), and core 1 waits in onNmi(), from RAM, until
// core 0 lets go. An NMI also takes a core 1 that masks its interrupts (RP2350 datasheet 3.2.1: it ignores PRIMASK
// and pre-empts every other exception), which a FIFO interrupt would wait for.
//
// Armed by core 1 itself at its entry (SecondaryCoreInit), disarmed by core 0 before every launch and reset of core 1
// (SecondaryCoreInitBase), so a core 1 in the boot ROM never gets the NMI. Core 0 only: a flash operation on core 1
// leaves core 0 running (core 0's NMI is Core0Park's). A core 1 that does not answer within ackPolls (halted by a
// debugger, in lockup, not yet running Kvasir code) fetches nothing either: core 0 goes on and counts it
// (`unanswered`); the line stays forced, and core 1 takes the NMI when it runs again and returns at once.
struct Core1Lockout {
    using IoQspi                           = Kvasir::Peripheral::IO_QSPI::Registers<>;
    static constexpr std::uint32_t nmiMask = Core0Park::nmiMask;   // the same line, core 1's copy
    // NMI entry is a few dozen cycles, a core 1 stalled on an XIP fetch some microseconds: about 1 ms at 150 MHz
    static constexpr std::uint32_t ackPolls = 100'000;

    static inline bool volatile armed
      = false;   // core 1's NMI routed (core 1 sets it, core 0 clears it)
    static inline bool volatile hold                = false;   // core 0 holds core 1 in onNmi()
    static inline bool volatile parked              = false;   // core 1 is in onNmi()
    static inline std::uint32_t volatile depth      = 0;       // core 0's nested Guards
    static inline std::uint32_t volatile lockouts   = 0;       // core 1 parked within ackPolls
    static inline std::uint32_t volatile unanswered = 0;       // it did not

    // Core 0, before every launch and reset of core 1: nothing forced, nothing routed (the RP2040's mask is chip-wide
    // and survives the PSM reset of core 1; the RP2350's is reset with core 1).
    static void disarm() {
        armed = false;
        apply(clear(IoQspi::PROC1_INTF::gpio_qspi_sclk_level_low));
#ifndef KVASIR_CHIP_RP2350
        apply(write(Kvasir::Peripheral::SYSCFG::Registers<>::PROC1_NMI_MASK::FULLREGISTER,
                    Kvasir::Register::value<0U>()));
#endif
    }

    // Core 1, at its entry: the force bit first (a lockout core 0 gave up on may have left it set), then the route.
    static void arm() {
        apply(clear(IoQspi::PROC1_INTF::gpio_qspi_sclk_level_low));
        parked = false;
#ifdef KVASIR_CHIP_RP2350
        apply(write(Kvasir::Peripheral::EPPB::Registers<>::NMI_MASK0::nmi_mask0,
                    Kvasir::Register::value<nmiMask>()));
#else
        apply(write(Kvasir::Peripheral::SYSCFG::Registers<>::PROC1_NMI_MASK::FULLREGISTER,
                    Kvasir::Register::value<nmiMask>()));
#endif
        armed = true;
    }

    // Core 1's NMI. From RAM: core 0 switches XIP off as soon as `parked` is seen. Another NMI (the RP2350's RCP
    // integrity check) finds the force bit clear and ends as the unhandled-interrupt panic it was before - with XIP
    // on, so the call into flash is safe.
    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static void onNmi() {
        KVASIR_RAM_FUNC_MARK_CALLS_FLASH();
        if(get<0>(apply(read(IoQspi::PROC1_INTF::gpio_qspi_sclk_level_low))) == 0) {
            Kvasir::Nvic::DefaultIsrs::onIsr();
        }
        apply(clear(IoQspi::PROC1_INTF::gpio_qspi_sclk_level_low));
        asm volatile("dsb" ::: "memory");   // the clear and every access before it done
        parked = true;
        while(hold) { asm volatile("wfe" ::: "memory"); }
        parked = false;
    }

    // Core 0: core 1 parked for the Guard's scope (Flash::XipDisabler holds one). Nothing on core 1, before core 1 is
    // armed, or nested inside another Guard (an interrupt's flash write inside a flash write).
    class Guard {
    public:
        Guard() : held_{take()} {}

        ~Guard() {
            if(held_) { give(); }
        }

        /// Whether this Guard holds core 1 (false on core 1, before it is armed, and for a nested Guard's outer
        /// scope's sake: true there too).
        [[nodiscard]] bool held() const { return held_; }

        Guard(Guard const&)            = delete;
        Guard& operator=(Guard const&) = delete;

    private:
        bool held_;
    };

    [[nodiscard]] static bool take() {
        if(Kvasir::Sio::cpuId() != 0U || !armed) { return false; }
        auto const outer = depth;
        depth            = outer + 1U;
        if(outer != 0U) { return true; }
        parked = false;
        hold   = true;
        asm volatile("dmb" ::: "memory");
        apply(set(IoQspi::PROC1_INTF::gpio_qspi_sclk_level_low));
        for(std::uint32_t i = 0; i != ackPolls; ++i) {
            if(parked) {
                lockouts = lockouts + 1U;
                return true;
            }
        }
        unanswered = unanswered + 1U;
        return true;
    }

    static void give() {
        auto const inner = depth - 1U;
        depth            = inner;
        if(inner != 0U) { return; }
        asm volatile("dsb" ::: "memory");   // every flash access of the scope done (XIP back on)
        hold = false;
        asm volatile("dsb\n sev" ::: "memory");
    }

    using Isr = brigand::list<Kvasir::Nvic::Isr<std::addressof(onNmi), Kvasir::Nvic::Index<-14>>>;
};

}   // namespace Kvasir::Multicore
