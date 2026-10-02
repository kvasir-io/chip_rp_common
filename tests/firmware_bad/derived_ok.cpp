// expect-ok
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

// A config that names none of it gets the program's directives; a config that repeats them agrees.
struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    using namespace Kvasir::Pio;
    a.sideSet(1, true);
    a.fifo(Fifo::tx);
    a.outConfig(1, false, true, 24);
    a.setConfig(1);
    a.movStatus(MovStatusKind::txLessThan, 2);
    a.out(Out::pins, 1).side(1);
    a.set(Set::pins, 0);
})> {};

struct Named {
    static constexpr auto ClockSpeed  = HW::ClockSpeed;
    static constexpr auto PioInstance = 0;
    static constexpr auto SmInstance  = 0;
    static constexpr auto outPins     = brigand::list<HW::Pin::led>{};
    static constexpr auto setPins     = brigand::list<HW::Pin::led>{};
    static constexpr auto sidesetPins = brigand::list<HW::Pin::led>{};
};

using Sm = Kvasir::Pio::StateMachine<P, Named>;
static_assert(Sm::Config::sidesetOptional && !Sm::Config::sidesetPindirs);
static_assert(Sm::Config::joinTx && !Sm::Config::joinRx);
static_assert(Sm::Config::autopull && Sm::Config::pullThreshold == 24
              && !Sm::Config::outShiftRight);
static_assert(!Sm::Config::autopush && Sm::Config::pushThreshold == 32 && Sm::Config::inShiftRight);
static_assert(Sm::Config::movStatus == Kvasir::Pio::MovStatus::txFifoLessThan
              && Sm::Config::movStatusN == 2);

struct Repeated : Named {
    static constexpr bool     sidesetOptional = true;
    static constexpr bool     joinTx          = true;
    static constexpr bool     autopull        = true;
    static constexpr unsigned pullThreshold   = 24;
    static constexpr bool     outShiftRight   = false;
};

static_assert(sizeof(Kvasir::Pio::StateMachine<P,
                                               Repeated>)
              > 0);
