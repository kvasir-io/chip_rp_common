// expect: delay_overflow\.cpp:[0-9]+: delay 16 does not fit: \.side_set 1 leaves 4 delay bits \(max 15\)
#include "pio/Asm.hpp"

struct Blink : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.sideSet(1);
    a.nop().side(1);
    a.nop().side(0).delay(16);
})> {};

static_assert(Blink::wrap == 1);
