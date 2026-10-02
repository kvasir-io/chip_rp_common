// expect: line 4: delay 16 does not fit: \.side_set 1 leaves 4 delay bits \(max 15\)
#include "pio/AsmParse.hpp"

struct Blink : Kvasir::Pio::Program<Kvasir::Pio::parse(R"(.program blink
.side_set 1
    nop side 1
    nop side 0 [16]
)")> {};

static_assert(Blink::Wrap == 1);
