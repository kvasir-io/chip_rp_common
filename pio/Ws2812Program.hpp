#pragma once
// The WS2812 driver's PIO program (pico-examples' ws2812.pio), built by the compiler (Asm.hpp).
// Standalone (Asm.hpp only), so the host tests check it against pioasm's words
// (chip_rp_common/tests/asm_encoding_test.cpp).
#include "Asm.hpp"

namespace Kvasir { namespace Pio {
    /// One bit is T1 + T2 + T3 cycles: 0 is high for T1, low for T2 + T3; 1 is high for T1 + T2,
    /// low for T3 (ws2812.hpp has the timing and its checks).
    template<unsigned T1_, unsigned T2_, unsigned T3_>
    struct Ws2812Program : Program<assemble([](Asm& a) {
        a.sideSet(1);
        // fed bytes by DMA: one 8-deep TX FIFO, a pull every 8 bits, MSB first (shift left); no
        // out pins, the LED line is the side-set pin (StateMachine takes all this from here)
        a.fifo(Fifo::tx);
        a.outConfig(0, false, true, 8);
        a.wrapTarget();
        a.label("bitloop");
        a.out(Out::x, 1).side(0).delay(T3_ - 1);   // side-set still takes place when out stalls
        a.jmp(Cond::notX, "do_zero")
          .side(1)
          .delay(T1_ - 1);   // branch on the bit shifted out; positive pulse
        a.label("do_one");
        a.jmp("bitloop").side(1).delay(T2_ - 1);   // continue driving high, for a long pulse
        a.label("do_zero");
        a.nop().side(0).delay(T2_ - 1);   // or drive low, for a short pulse
        a.wrap();
    })> {
        // int, as pioasm's `.define public` constants were
        static constexpr int T1           = static_cast<int>(T1_);
        static constexpr int T2           = static_cast<int>(T2_);
        static constexpr int T3           = static_cast<int>(T3_);
        static constexpr int CyclesPerBit = T1 + T2 + T3;
    };
}}   // namespace Kvasir::Pio
