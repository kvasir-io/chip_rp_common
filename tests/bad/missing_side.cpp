// expect: missing_side\.cpp:[0-9]+: side-set is not optional here \(\.side_set 1\): .side. missing
#include "pio/Asm.hpp"

struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.sideSet(1);
    a.nop().side(1);
    a.nop();
})> {};

static_assert(P::wrap == 1);
