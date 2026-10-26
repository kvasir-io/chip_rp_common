// expect: unknown_label\.cpp:[0-9]+: jmp to unknown label .bitlop.
#include "pio/Asm.hpp"

struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.label("bitloop");
    a.jmp("bitlop");
})> {};

static_assert(P::wrap == 0);
