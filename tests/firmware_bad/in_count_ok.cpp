// expect-ok
// chips: rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

// .in 1 (PIO version 1): IN_COUNT 1, and the shift settings the program gives
struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.pioVersion(1);
    a.inConfig(1, false, true, 16);
    a.label("start");
    a.in(Kvasir::Pio::In::pins, 1);
})> {};

struct C {
    static constexpr auto ClockSpeed  = HW::ClockSpeed;
    static constexpr auto PioInstance = 0;
    static constexpr auto SmInstance  = 0;
    static constexpr auto inPins      = brigand::list<HW::Pin::led>{};
};

using Sm = Kvasir::Pio::StateMachine<P, C>;
static_assert(Sm::InCount == 1);
static_assert(Sm::ConfiguredShift.autopush && Sm::ConfiguredShift.pushThreshold == 16
              && !Sm::ConfiguredShift.inShiftRight);

// the run-time calls compile for a real machine
[[maybe_unused]] static void phases() {
    Sm::setShift<Kvasir::Pio::Shift{.autopull = true, .pullThreshold = 8, .joinTx = true}>();
    Sm::clearFifos();
    Sm::setClockDiv<2.5>();
    Sm::restartAt(P::offset("start") + 0);
}
