// expect: the program has \.origin: ProgramOffset must be that slot
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.origin(0);
    a.nop();
})> {};

struct C {
    static constexpr auto     ClockSpeed    = HW::ClockSpeed;
    static constexpr auto     PioInstance   = 0;
    static constexpr auto     SmInstance    = 0;
    static constexpr unsigned ProgramOffset = 4;
};

static_assert(sizeof(Kvasir::Pio::StateMachine<P,
                                               C>)
              > 0);
