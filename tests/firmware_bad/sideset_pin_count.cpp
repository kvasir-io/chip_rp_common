// expect: sidesetPins must be as many pins as the program's \.side_set count
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

// .side_set 2, one pin mapped: SIDESET_COUNT 1 would read side-set bits as delay
struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.sideSet(2);
    a.nop().side(3);
})> {};

struct C {
    static constexpr auto ClockSpeed  = HW::ClockSpeed;
    static constexpr auto PioInstance = 0;
    static constexpr auto SmInstance  = 0;
    static constexpr auto sidesetPins = brigand::list<HW::Pin::led>{};
};

static_assert(sizeof(Kvasir::Pio::StateMachine<P,
                                               C>)
              > 0);
