// expect: line 3: undefined symbol .bitlop.
#include "pio/AsmParse.hpp"

struct P : Kvasir::Pio::Program<Kvasir::Pio::parse(R"(.program p
bitloop:
    jmp bitlop
)")> {};

static_assert(P::Wrap == 0);
