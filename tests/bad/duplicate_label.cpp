// expect: line 4: .loop. is already defined, at line 2
#include "pio/AsmParse.hpp"

struct P : Kvasir::Pio::Program<Kvasir::Pio::parse(R"(.program p
loop:
    nop
loop:
    jmp loop
)")> {};

static_assert(P::Wrap == 1);
