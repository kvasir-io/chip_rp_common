// expect: the RP2040 starts machines in step on one PIO block only
// chips: rp2040
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) { a.nop(); })> {};

template<unsigned I>
struct C {
    static constexpr auto ClockSpeed   = HW::ClockSpeed;
    static constexpr auto PioInstance  = I;
    static constexpr auto SmInstance   = 0;
    static constexpr bool startEnabled = false;
};

[[maybe_unused]] static void start() {
    Kvasir::Pio::startInSync<Kvasir::Pio::StateMachine<P, C<0>>,
                             Kvasir::Pio::StateMachine<P, C<1>>>();
}
