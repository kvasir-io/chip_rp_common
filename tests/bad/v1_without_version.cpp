// expect: line 2: .mov pindirs. needs \.pio_version 1 \(RP2350\)
#include "pio/AsmParse.hpp"

struct P : Kvasir::Pio::Program<Kvasir::Pio::parse(R"(.program p
    mov pindirs, x
)")> {};

static_assert(P::Wrap == 0);
