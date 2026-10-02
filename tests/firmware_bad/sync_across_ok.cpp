// expect-ok
// chips: rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

// PIO0 SM0 and PIO2 SM3 in step: PIO2 is PIO0's previous block
struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) { a.nop(); })> {};

template<unsigned I, unsigned S>
struct C {
    static constexpr auto ClockSpeed   = HW::ClockSpeed;
    static constexpr auto PioInstance  = I;
    static constexpr auto SmInstance   = S;
    static constexpr bool startEnabled = false;
};

[[maybe_unused]] static void start() {
    Kvasir::Pio::startInSync<Kvasir::Pio::StateMachine<P, C<0, 0>>,
                             Kvasir::Pio::StateMachine<P, C<2, 3>>>();
}
