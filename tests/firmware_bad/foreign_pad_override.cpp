// expect: name a foreign pad: its pad and GPIO settings are its owner's
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.wait(1, Kvasir::Pio::Wait::pin, 0);
})> {};

// a GPIO override on a foreign pad would be dropped: the machine writes no pin action for it
struct C {
    static constexpr auto ClockSpeed  = HW::ClockSpeed;
    static constexpr auto PioInstance = 0;
    static constexpr auto SmInstance  = 0;
    static constexpr auto inPins      = brigand::list<HW::Pin::clk_out>{};
    static constexpr auto foreignPads = brigand::list<HW::Pin::clk_out>{};
    static constexpr auto oeInverted  = brigand::list<HW::Pin::clk_out>{};
};

static_assert(sizeof(Kvasir::Pio::StateMachine<P,
                                               C>)
              > 0);
