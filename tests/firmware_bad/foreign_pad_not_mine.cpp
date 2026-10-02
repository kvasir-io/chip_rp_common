// expect: foreignPads names a pin that is not an input of this machine
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.wait(1, Kvasir::Pio::Wait::pin, 0);
})> {};

// a foreign pad the machine does not map: nothing would read it
struct C {
    static constexpr auto ClockSpeed  = HW::ClockSpeed;
    static constexpr auto PioInstance = 0;
    static constexpr auto SmInstance  = 0;
    static constexpr auto inPins      = brigand::list<HW::Pin::led>{};
    static constexpr auto foreignPads = brigand::list<HW::Pin::clk_out>{};
};

static_assert(sizeof(Kvasir::Pio::StateMachine<P,
                                               C>)
              > 0);
