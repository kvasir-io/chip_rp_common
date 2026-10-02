#pragma once
// ClockOut's PIO program, built by the compiler (Asm.hpp). Standalone (Asm.hpp only), so the host
// tests check it against the words pioasm made of the .pio file it replaced
// (chip_rp_common/tests/asm_encoding_test.cpp).
#include "Asm.hpp"

namespace Kvasir { namespace Pio {
    /// A square wave on one pin, for a clock the chip's own clock generators cannot reach.
    ///
    /// Two instructions, so one period is two cycles: the output is ClockSpeed / (2 * clockDiv)
    /// at 50% duty. The shortest loop needs the largest divider and so gives the finest 16.8
    /// step. No wrap is needed: a two-instruction program wraps on its own.
    struct ClockOutProgram : Program<assemble([](Asm& a) {
        a.set(Set::pins, 1);
        a.set(Set::pins, 0);
    })> {};
}}   // namespace Kvasir::Pio
