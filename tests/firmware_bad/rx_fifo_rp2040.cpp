// expect: RX FIFO random access \(\.fifo txput / txget / putget\) is an RP2350 feature
// chips: rp2040
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.pioVersion(1);
    a.fifo(Kvasir::Pio::Fifo::txget);
    a.nop();
})> {};

struct C {
    static constexpr auto ClockSpeed  = HW::ClockSpeed;
    static constexpr auto PioInstance = 0;
    static constexpr auto SmInstance  = 0;
};

static_assert(sizeof(Kvasir::Pio::StateMachine<P,
                                               C>)
              > 0);
