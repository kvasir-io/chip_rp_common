// expect: sidesetOptional / sidesetPindirs disagree with the program's \.side_set
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

// the program's side-set is optional, the config says it is not
struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.sideSet(1, true);
    a.nop().side(1);
})> {};

struct C {
    static constexpr auto ClockSpeed      = HW::ClockSpeed;
    static constexpr auto PioInstance     = 0;
    static constexpr auto SmInstance      = 0;
    static constexpr auto sidesetPins     = brigand::list<HW::Pin::led>{};
    static constexpr bool sidesetOptional = false;
};

static_assert(sizeof(Kvasir::Pio::StateMachine<P,
                                               C>)
              > 0);
