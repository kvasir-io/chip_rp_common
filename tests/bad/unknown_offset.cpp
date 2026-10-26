// expect: pioProgramHasNoLabelOfThatName
#include "pio/Asm.hpp"

struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.label("start");
    a.jmp("start");
})> {};

static_assert(P::offset("stop") == 0);
