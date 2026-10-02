// expect: too_long\.cpp:[0-9]+: more than 32 instructions: instruction memory has 32
#include "pio/Asm.hpp"

// the builder: C++ loops make instructions, and too many is an error that counts them
template<int N>
struct Nops : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    for(int i = 0; i < N; ++i) { a.nop(); }
})> {};

static_assert(Nops<33>::Wrap == 32);
