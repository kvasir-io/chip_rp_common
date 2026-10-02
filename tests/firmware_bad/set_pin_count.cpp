// expect: outPins / inPins / setPins must be as many pins as the program's \.out / \.in / \.set
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

// .set 2, one pin mapped
struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.setConfig(2);
    a.set(Kvasir::Pio::Set::pins, 3);
})> {};

struct C {
    static constexpr auto ClockSpeed  = HW::ClockSpeed;
    static constexpr auto PioInstance = 0;
    static constexpr auto SmInstance  = 0;
    static constexpr auto setPins     = brigand::list<HW::Pin::led>{};
};

static_assert(sizeof(Kvasir::Pio::StateMachine<P,
                                               C>)
              > 0);
