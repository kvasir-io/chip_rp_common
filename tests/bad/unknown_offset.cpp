// expect: pio_program_has_no_label_of_that_name
#include "pio/AsmParse.hpp"

struct P : Kvasir::Pio::Program<Kvasir::Pio::parse(R"(.program p
start:
    jmp start
)")> {};

static_assert(P::offset("stop") == 0);
