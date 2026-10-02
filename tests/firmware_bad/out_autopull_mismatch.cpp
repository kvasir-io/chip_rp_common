// expect: disagree with the program's \.out/\.in/\.fifo/\.mov_status
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

// .out 1 right auto 8: the program expects autopull, the config switches it off
struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.outConfig(1, true, true, 8);
    a.out(Kvasir::Pio::Out::pins, 1);
})> {};

struct C {
    static constexpr auto ClockSpeed  = HW::ClockSpeed;
    static constexpr auto PioInstance = 0;
    static constexpr auto SmInstance  = 0;
    static constexpr auto outPins     = brigand::list<HW::Pin::led>{};
    static constexpr bool autopull    = false;
};

static_assert(sizeof(Kvasir::Pio::StateMachine<P,
                                               C>)
              > 0);
