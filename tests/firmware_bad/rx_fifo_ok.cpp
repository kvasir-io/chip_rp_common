// expect-ok
// chips: rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

// .fifo txput: the machine writes the RX FIFO's entries, the processor reads them
struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.pioVersion(1);
    a.fifo(Kvasir::Pio::Fifo::txput);
    a.movToRx(2);
})> {};

struct C {
    static constexpr auto ClockSpeed  = HW::ClockSpeed;
    static constexpr auto PioInstance = 0;
    static constexpr auto SmInstance  = 0;
};

using Sm = Kvasir::Pio::StateMachine<P, C>;
static_assert(Sm::Config::rxFifoPut && !Sm::Config::rxFifoGet && !Sm::Config::joinRx);
static_assert(Sm::ConfiguredShift.rxPut && !Sm::ConfiguredShift.rxGet);

[[maybe_unused]] static std::uint32_t status() { return Sm::rxEntry<2>(); }
