// expect: duplicate_label\.cpp:[0-9]+: .loop. is already defined as a label
#include "pio/Asm.hpp"

struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.label("loop");
    a.nop();
    a.label("loop");
    a.jmp("loop");
})> {};

static_assert(P::wrap == 1);
