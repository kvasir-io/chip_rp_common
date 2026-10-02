// expect: line 4: side-set is not optional here \(\.side_set 1\): .side. missing
#include "pio/AsmParse.hpp"

struct P : Kvasir::Pio::Program<Kvasir::Pio::parse(R"(.program p
.side_set 1
    nop side 1
    nop
)")> {};

static_assert(P::Wrap == 1);
