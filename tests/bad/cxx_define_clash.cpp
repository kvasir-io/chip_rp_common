// expect: line 2: .T1. is given from C\+\+ and defined here
#include "pio/AsmParse.hpp"

// a value from C++ may not be .define-d again in the text: the stale one would win silently
struct P
  : Kvasir::Pio::Program<Kvasir::Pio::parse(R"(.program p
.define T1 3
    nop [T1]
)",
                                            {
                                              {"T1", 5}
})> {};

static_assert(P::Wrap == 0);
