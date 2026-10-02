// expect-ok
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>

// the pin options, the synchroniser bypass, the run-time calls and a start in step
struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.set(Kvasir::Pio::Set::pindirs, 0);
})> {};

struct C0 {
    static constexpr auto ClockSpeed      = HW::ClockSpeed;
    static constexpr auto PioInstance     = 0;
    static constexpr auto SmInstance      = 0;
    static constexpr auto setPins         = brigand::list<HW::Pin::led>{};
    static constexpr auto pullUpPins      = brigand::list<HW::Pin::led>{};
    static constexpr auto oeInverted      = brigand::list<HW::Pin::led>{};
    static constexpr auto outputInverted  = brigand::list<HW::Pin::led>{};
    static constexpr auto inputSyncBypass = brigand::list<HW::Pin::led>{};
    static constexpr bool startEnabled    = false;
};

struct C1 {
    static constexpr auto ClockSpeed   = HW::ClockSpeed;
    static constexpr auto PioInstance  = 0;
    static constexpr auto SmInstance   = 1;
    static constexpr bool startEnabled = false;
};

using Sm0 = Kvasir::Pio::StateMachine<P, C0>;
using Sm1 = Kvasir::Pio::StateMachine<P, C1>;
static_assert(
  Sm0::SyncBypassMask
  == 1U << Kvasir::Pio::pinIndex(static_cast<unsigned>(Kvasir::Io::pinNumber(HW::Pin::led{})),
                                 Sm0::GpioBase));
static_assert(Sm1::SyncBypassMask == 0);

[[maybe_unused]] static void use() {
    Kvasir::Pio::startInSync<Sm0, Sm1>();
    Sm0::push(1);
    [[maybe_unused]] auto const w = Sm0::pop();
    Sm0::drainTxFifo();
    Sm0::execWait(0xA042);
}
