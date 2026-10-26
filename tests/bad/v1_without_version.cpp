// expect: v1_without_version\.cpp:[0-9]+: .mov pindirs. needs \.pio_version 1 \(RP2350\)
#include "pio/Asm.hpp"

struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.mov(Kvasir::Pio::MovDst::pindirs, Kvasir::Pio::MovSrc::x);
})> {};

static_assert(P::wrap == 0);
