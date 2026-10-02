#pragma once
// PioQspi's PIO program, built by the compiler (Asm.hpp). Standalone (Asm.hpp only), so the host
// tests check it against the words pioasm made of the .pio file it replaced
// (chip_rp_common/tests/asm_encoding_test.cpp).
#include "Asm.hpp"

namespace Kvasir { namespace Display {
    /// QSPI transport for DCS panel controllers (CO5300, ST77916, ST77922). One program with three
    /// entry points (offset("single"), offset("quad"), offset("read")), loaded at offset 0; the
    /// driver forces a jmp into the one it wants through SMx_INSTR while the machine is disabled.
    ///
    /// SPI mode 0, the panel samples on the rising edge (CO5300 datasheet 5.2). Two PIO cycles per
    /// bit: div = clockSpeed / (2*baud). Side-set applies even when an instruction stalls, so an
    /// empty TX FIFO parks SCK low. The stalling `out` raises FDEBUG_TXSTALL after the last rising
    /// edge: that is completion.
    struct QspiPanelProgram : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
        using namespace Kvasir::Pio;
        a.sideSet(1);   // SCK

        // Single-lane write: instruction, address {00h, CMD, 00h}, parameters (datasheet 5.2.1).
        // Only D0 changes; the panel ignores D1..D3 here.
        a.label("single", true);
        a.out(Out::pins, 1).side(0);   // drive the bit while the clock is low
        a.jmp("single").side(1);       // rising edge - the panel samples here

        // Quad-lane write: the pixel payload after a 32h header. OSR bit 31 lands on D3: high
        // nibble first, as the datasheet's RGB565 lane map wants (R4..R1 on SI3..SI0).
        a.label("quad", true);
        a.out(Out::pins, 4).side(0);
        a.jmp("quad").side(1);

        // Single-lane read-back after `single` sent the header, all at the read clock (TSCYC >=
        // 100 ns, datasheet 6.4.1). The bit count - 1 comes as one FIFO word (autopull off).
        a.label("read", true);
        a.pull().side(0);   // bit count - 1
        a.out(Out::x, 32).side(0);
        a.set(Set::pindirs, 0b0000).side(0);   // release all four lanes for the turnaround
        a.label("read_loop");
        a.in(In::pins, 1).side(1);                // sample at the rising edge, one full clock phase
        a.jmp(Cond::xDec, "read_loop").side(0);   // after the panel updated on the falling edge
        a.label("read_park");
        a.jmp("read_park").side(0);   // hold the clock low until the driver disables us
    })> {};
}}   // namespace Kvasir::Display
