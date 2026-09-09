#pragma once
#include "Sio.hpp"
#include "chip/Interrupt.hpp"
#include "core/core.hpp"
#include "kvasir/Atomic/Atomic.hpp"
#include "kvasir/Register/Register.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <peripherals/PSM.hpp>
#include <type_traits>

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
    while(get<0>(apply(read(PSM::FRCE_OFF::proc1))) == 0) {}
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
    using FifoIrq = std::decay_t<decltype(Sio::detail::fifoIrq<0, I>())>;
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

}   // namespace Kvasir::Multicore
