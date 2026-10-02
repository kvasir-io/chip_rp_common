// expect: clock divider out of range
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

// a run-time clock divider is checked like the config's
void slow() { Sm::setClockDiv<0.5>(); }
