// expect: the program uses a PIO version 1 instruction
// chips: rp2040
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

// mov pindirs is PIO version 1 (RP2350); the RP2040's PIO has version 0
struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.pioVersion(1);
    a.mov(Kvasir::Pio::MovDst::pindirs, Kvasir::Pio::MovSrc::x);
})> {};

struct C {
    static constexpr auto ClockSpeed  = HW::ClockSpeed;
    static constexpr auto PioInstance = 0;
    static constexpr auto SmInstance  = 0;
};

static_assert(sizeof(Kvasir::Pio::StateMachine<P,
                                               C>)
              > 0);
