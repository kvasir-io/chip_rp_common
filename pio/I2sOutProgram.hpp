#pragma once
// I2sOut's PIO programs, built by the compiler (Asm.hpp). Standalone (Asm.hpp only), so the host
// tests check them against the words pioasm made of the .pio file they replaced
// (chip_rp_common/tests/asm_encoding_test.cpp).
//
// I2S transmit, master and slave. Philips I2S, 16 bits a slot, two slots a frame: one 32-bit FIFO
// word is a frame, left slot in the high half. Shift left with a 32-bit threshold so the MSB
// leaves first. The format's one-BCLK delay is in the encoding: the master moves LRCK one bit
// period early, the slave waits one rising edge after the LRCK edge.
#include "Asm.hpp"

namespace Kvasir { namespace Pio {
    /// Master: side-set bit 0 is BCLK, bit 1 is LRCK, so the clock pins are consecutive with
    /// BCLK the base. Data changes while BCLK is low, the receiver samples on the rising edge.
    ///
    /// Two cycles a bit, 32 bits a frame, so the state machine runs at 64 * the sample rate.
    /// Side-set is applied even when an instruction stalls, so an underrun holds the last sample
    /// instead of breaking the frame.
    struct I2sOutMasterProgram : Program<assemble([](Asm& a) {
        //                   side-set: BCLK is bit 0, LRCK bit 1
        a.sideSet(2);
        a.set(Set::x, 14)
          .side(0b00);   // seed the bit counter once, outside the wrap, clocks idle low
        a.wrapTarget();
        a.label("leftbit");
        a.out(Out::pins, 1).side(0b00);            // BCLK low, LRCK low: drive a left bit
        a.jmp(Cond::xDec, "leftbit").side(0b01);   // BCLK high: the receiver takes it
        a.out(Out::pins, 1).side(0b10);   // the 16th left bit, and LRCK goes high one bit early
        a.set(Set::x, 14).side(0b11);
        a.label("rightbit");
        a.out(Out::pins, 1).side(0b10);   // BCLK low, LRCK high
        a.jmp(Cond::xDec, "rightbit").side(0b11);
        a.out(Out::pins, 1).side(0b00);   // the 16th right bit, and LRCK goes low again
        a.set(Set::x, 14).side(0b01);
        a.wrap();
    })> {};

    /// Slave: the codec drives BCLK and LRCK and this follows them.
    ///
    /// `wait ... pin N` is relative to the IN base, so the base is BCLK and pin 1 is LRCK -- the
    /// same two consecutive pins the master program side-sets. Every edge comes from outside, so
    /// the divider only has to be fast enough to keep up.
    struct I2sOutSlaveProgram : Program<assemble([](Asm& a) {
        constexpr int bclk = 0;
        constexpr int lrck = 1;
        a.wait(1,
               Wait::pin,
               lrck);   // find a frame boundary once; LRCK keeps us in step from there on
        a.wrapTarget();
        a.pull();                     // a frame is one 32-bit word
        a.wait(0, Wait::pin, lrck);   // LRCK falls: the left slot begins
        a.wait(1, Wait::pin, bclk);   // let one BCLK pass -- the I2S one-bit delay
        a.set(Set::x, 15);
        a.label("left_loop");
        a.wait(0, Wait::pin, bclk);   // BCLK falls: safe to change the data
        a.out(Out::pins, 1);
        a.wait(1, Wait::pin, bclk);   // BCLK rises: the codec samples it
        a.jmp(Cond::xDec, "left_loop");
        a.wait(1, Wait::pin, lrck);   // LRCK rises: the right slot
        a.wait(1, Wait::pin, bclk);
        a.set(Set::x, 15);
        a.label("right_loop");
        a.wait(0, Wait::pin, bclk);
        a.out(Out::pins, 1);
        a.wait(1, Wait::pin, bclk);
        a.jmp(Cond::xDec, "right_loop");
        a.wrap();
    })> {};
}}   // namespace Kvasir::Pio
