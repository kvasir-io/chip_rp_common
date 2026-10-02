// expect: a FIFO can be joined in one direction only
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) { a.nop(); })> {};

struct C {
    static constexpr auto ClockSpeed  = HW::ClockSpeed;
    static constexpr auto PioInstance = 0;
    static constexpr auto SmInstance  = 0;
};

using Sm = Kvasir::Pio::StateMachine<P, C>;

// a run-time shift setting is checked like the config's
void phase() { Sm::setShift<Kvasir::Pio::Shift{.joinTx = true, .joinRx = true}>(); }
