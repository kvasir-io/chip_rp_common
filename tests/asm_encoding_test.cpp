// Every instruction form of the builder (pio/Asm.hpp), worked out by hand from the RP2350
// datasheet 11.4 (Table 980 and the operand lists of 11.4.2-11.4.12), side-set and delay
// packing (11.4.1), labels, defines, Program<>'s members and the messages.
#include "pio/Asm.hpp"

#include "pio/ClockOutProgram.hpp"
#include "pio/I2sOutProgram.hpp"
#include "pio/PioQspiProgram.hpp"
#include "pio/Ws2812Program.hpp"

#include <cstdio>
#include <source_location>

using namespace Kvasir::Pio;

namespace {
// The first word of a one-instruction program; `version` 1 for the v1 forms. FIFO join
// putget allows both rxfifo forms, and push needs txrx/rx - so a push test sets its own.
template<int  Version = 0,
         Fifo F       = Fifo::txrx>
consteval std::uint16_t one(auto f) {
    auto const a = assemble([&](Asm& as) {
        as.pioVersion(Version);
        as.fifo(F);
        f(as);
    });
    return a.ok() ? a.words[0] : 0xFFFF;
}

// ---- JMP, 11.4.2: 000 | cond | address ----------------------------------------------------
static_assert(one([](Asm& a) { a.jmp(Cond::always, 0U); }) == 0x0000);
static_assert(one([](Asm& a) { a.jmp(Cond::notX, 0U); }) == 0x0020);      // 001
static_assert(one([](Asm& a) { a.jmp(Cond::xDec, 0U); }) == 0x0040);      // 010
static_assert(one([](Asm& a) { a.jmp(Cond::notY, 0U); }) == 0x0060);      // 011
static_assert(one([](Asm& a) { a.jmp(Cond::yDec, 0U); }) == 0x0080);      // 100
static_assert(one([](Asm& a) { a.jmp(Cond::xNeY, 0U); }) == 0x00A0);      // 101
static_assert(one([](Asm& a) { a.jmp(Cond::pin, 0U); }) == 0x00C0);       // 110
static_assert(one([](Asm& a) { a.jmp(Cond::notOsre, 0U); }) == 0x00E0);   // 111
static_assert(assemble([](Asm& a) {
                  a.nop();
                  a.nop();
                  a.jmp(Cond::xDec, 2U);
              }).words[2]
              == 0x0042);   // 010 | 00010

// ---- WAIT, 11.4.3: 001 | pol[7] | source[6:5] | index[4:0] --------------------------------
static_assert(one([](Asm& a) { a.wait(0, Wait::gpio, 5); }) == 0x2005);   // src 00
static_assert(one([](Asm& a) { a.wait(1, Wait::gpio, 31); }) == 0x209F);
static_assert(one([](Asm& a) { a.wait(1, Wait::pin, 3); }) == 0x20A3);   // 0x80 | 01<<5 | 3
static_assert(one([](Asm& a) { a.wait(0, Wait::irq, 7); }) == 0x2047);   // 10<<5 | 7
static_assert(one([](Asm& a) { a.wait(1, Wait::irq, 1, IrqMode::rel); })
              == 0x20D1);   // 0x80 | 0x40 | IdxMode 10<<3 | 1
static_assert(one<1>([](Asm& a) { a.wait(1, Wait::irq, 2, IrqMode::prev); }) == 0x20CA);   // 01<<3
static_assert(one<1>([](Asm& a) { a.wait(1, Wait::irq, 3, IrqMode::next); }) == 0x20DB);   // 11<<3
static_assert(one<1>([](Asm& a) { a.wait(1, Wait::jmppin, 0); }) == 0x20E0);               // src 11
static_assert(one<1>([](Asm& a) { a.wait(0, Wait::jmppin, 3); }) == 0x2063);

// ---- IN, 11.4.4: 010 | source | bit count (32 as 0) ------------------------------------------
static_assert(one([](Asm& a) { a.in(In::pins, 32); }) == 0x4000);
static_assert(one([](Asm& a) { a.in(In::x, 1); }) == 0x4021);
static_assert(one([](Asm& a) { a.in(In::y, 5); }) == 0x4045);
static_assert(one([](Asm& a) { a.in(In::null, 31); }) == 0x407F);
static_assert(one([](Asm& a) { a.in(In::isr, 8); }) == 0x40C8);    // 110
static_assert(one([](Asm& a) { a.in(In::osr, 16); }) == 0x40F0);   // 111

// ---- OUT, 11.4.5: 011 | destination | bit count --------------------------------------------
static_assert(one([](Asm& a) { a.out(Out::pins, 1); }) == 0x6001);
static_assert(one([](Asm& a) { a.out(Out::x, 2); }) == 0x6022);
static_assert(one([](Asm& a) { a.out(Out::y, 3); }) == 0x6043);
static_assert(one([](Asm& a) { a.out(Out::null, 32); }) == 0x6060);
static_assert(one([](Asm& a) { a.out(Out::pindirs, 4); }) == 0x6084);   // 100
static_assert(one([](Asm& a) { a.out(Out::pc, 5); }) == 0x60A5);        // 101
static_assert(one([](Asm& a) { a.out(Out::isr, 6); }) == 0x60C6);       // 110
static_assert(one([](Asm& a) { a.out(Out::exec, 16); }) == 0x60F0);     // 111

// ---- PUSH / PULL, 11.4.6-7: 100 | pull | IfF/IfE | Blk | 00000 ------------------------------
static_assert(one([](Asm& a) { a.push(); }) == 0x8020);   // block by default
static_assert(one([](Asm& a) { a.push(false, false); }) == 0x8000);
static_assert(one([](Asm& a) { a.push(true); }) == 0x8060);
static_assert(one([](Asm& a) { a.push(true, false); }) == 0x8040);
static_assert(one([](Asm& a) { a.pull(); }) == 0x80A0);
static_assert(one([](Asm& a) { a.pull(false, false); }) == 0x8080);
static_assert(one([](Asm& a) { a.pull(true); }) == 0x80E0);
static_assert(one([](Asm& a) { a.pull(true, false); }) == 0x80C0);

// ---- MOV, 11.4.10: 101 | destination | op | source --------------------------------------------
static_assert(one([](Asm& a) { a.mov(MovDst::x, MovSrc::y); }) == 0xA022);   // 001 | 00 | 010
static_assert(one([](Asm& a) { a.mov(MovDst::y, MovSrc::x, MovOp::invert); })
              == 0xA049);   // 010 | 01 | 001
static_assert(one([](Asm& a) { a.mov(MovDst::isr, MovSrc::osr, MovOp::reverse); })
              == 0xA0D7);   // 110 | 10 | 111
static_assert(one([](Asm& a) { a.mov(MovDst::pins, MovSrc::null); }) == 0xA003);
static_assert(one([](Asm& a) { a.mov(MovDst::exec, MovSrc::x); }) == 0xA081);      // 100
static_assert(one([](Asm& a) { a.mov(MovDst::pc, MovSrc::status); }) == 0xA0A5);   // 101 | 00 | 101
static_assert(one([](Asm& a) { a.mov(MovDst::osr, MovSrc::isr); }) == 0xA0E6);
static_assert(one([](Asm& a) { a.nop(); }) == 0xA042);   // mov y, y
static_assert(Enc::nop == 0xA042);
static_assert(one<1>([](Asm& a) { a.mov(MovDst::pindirs, MovSrc::x); }) == 0xA061);   // 011

// ---- MOV to / from RX, 11.4.8-9 (v1): 100 | from | 00 | 1 | IdxI | 0 | index -----------------
static_assert(one<1,
                  Fifo::putget>([](Asm& a) { a.movToRxY(); })
              == 0x8010);
static_assert(one<1,
                  Fifo::txput>([](Asm& a) { a.movToRx(2); })
              == 0x801A);
static_assert(one<1,
                  Fifo::txget>([](Asm& a) { a.movFromRxY(); })
              == 0x8090);
static_assert(one<1,
                  Fifo::putget>([](Asm& a) { a.movFromRx(3); })
              == 0x809B);

// ---- IRQ, 11.4.11: 110 | 0 | Clr | Wait | IdxMode | index ------------------------------------
static_assert(one([](Asm& a) { a.irq(3); }) == 0xC003);
static_assert(one([](Asm& a) { a.irqWait(2); }) == 0xC022);
static_assert(one([](Asm& a) { a.irqClear(7); }) == 0xC047);
static_assert(one([](Asm& a) { a.irq(1, IrqMode::rel); }) == 0xC011);
static_assert(one<1>([](Asm& a) { a.irq(0, IrqMode::prev); }) == 0xC008);
static_assert(one<1>([](Asm& a) { a.irqWait(5, IrqMode::next); }) == 0xC03D);   // 0x20 | 11<<3 | 5
static_assert(one<1>([](Asm& a) { a.irq(0, IrqMode::next); }) == 0xC018);

// ---- SET, 11.4.12: 111 | destination | data --------------------------------------------------
static_assert(one([](Asm& a) { a.set(Set::pins, 1); }) == 0xE001);
static_assert(one([](Asm& a) { a.set(Set::x, 31); }) == 0xE03F);
static_assert(one([](Asm& a) { a.set(Set::y, 0); }) == 0xE040);
static_assert(one([](Asm& a) { a.set(Set::pindirs, 3); }) == 0xE083);   // 100
static_assert(one([](Asm& a) { a.set(Set::pindirs, 0x1f); })
              == 0xE09F);   // = StateMachine's literal
static_assert(one([](Asm& a) { a.word(0xbeef); }) == 0xBEEF);

// ---- which words only PIO v1 has --------------------------------------------------------------
static_assert(isPioV1Only(0x20E0) && isPioV1Only(0x2063));     // wait jmppin
static_assert(isPioV1Only(0x20CA) && isPioV1Only(0x20DB));     // wait irq prev / next
static_assert(!isPioV1Only(0x20D1) && !isPioV1Only(0x2047));   // wait irq rel / plain
static_assert(isPioV1Only(0x8010) && isPioV1Only(0x801A) && isPioV1Only(0x8090)
              && isPioV1Only(0x809B));
static_assert(!isPioV1Only(0x8020) && !isPioV1Only(0x80E0));   // push / pull
static_assert(isPioV1Only(0xA061) && !isPioV1Only(0xA042) && !isPioV1Only(0xA0E6));
static_assert(isPioV1Only(0xC008) && isPioV1Only(0xC03D) && !isPioV1Only(0xC011)
              && !isPioV1Only(0xC047));
static_assert(!isPioV1Only(0x6421) && !isPioV1Only(0x00E0) && !isPioV1Only(0xE09F)
              && !isPioV1Only(0x4000));
// .pio_version 1 alone does not make a program v1 (fan_tacho declares it and uses v0 only)
static_assert(!usesPioV1(assemble([](Asm& a) {
                             a.pioVersion(1);
                             a.jmp(Cond::pin, 0U);
                             a.mov(MovDst::y, MovSrc::null, MovOp::invert);
                             a.push(false, false);
                         }).words));

// ---- side-set and delay, 11.4.1: bits 12:8, side-set MSBs (enable first when opt) -----------
static_assert(one([](Asm& a) {
                  a.sideSet(1);
                  a.nop().side(1).delay(3);
              })
              == 0xB342);   // 1<<4 | 3
static_assert(one([](Asm& a) {
                  a.sideSet(2, true);
                  a.nop().side(2).delay(1);
              })
              == 0xB942);   // 0x10 | 2<<2 | 1
static_assert(one([](Asm& a) {
                  a.sideSet(2, true);
                  a.nop().delay(3);
              })
              == 0xA342);   // no side: enable 0
static_assert(one([](Asm& a) {
                  a.sideSet(3, true, true);
                  a.nop().side(5).delay(1);
              })
              == 0xBB42);   // 0x10 | 5<<1 | 1
static_assert(one([](Asm& a) {
                  a.sideSet(5);
                  a.nop().side(31);
              })
              == 0xBF42);
static_assert(one([](Asm& a) { a.nop().delay(31); }) == 0xBF42);
static_assert(one([](Asm& a) {
                  a.sideSet(1);
                  a.nop().delay(2).side(1);
              })
              == 0xB242);   // either order
constexpr auto pindirsSide = assemble([](Asm& a) {
    a.sideSet(3, true, true);
    a.nop().side(5);
});
static_assert(pindirsSide.sidesetPindirs && pindirsSide.sidesetCount == 3);

// ---- wrap, labels, defines, programs ---------------------------------------------------------
constexpr auto three = assemble([](Asm& a) {
    a.nop();
    a.nop();
    a.nop();
});
static_assert(three.wrapTarget == 0 && three.wrap == 2 && three.length == 3);
constexpr auto wrapped = assemble([](Asm& a) {
    a.nop();
    a.wrapTarget();
    a.nop();
    a.wrap();
    a.nop();
});
static_assert(wrapped.wrapTarget == 1 && wrapped.wrap == 1);
constexpr auto labels = assemble([](Asm& a) {
    a.jmp("later");
    a.label("loop");
    a.nop();
    a.label("later", true);
    a.jmp("loop");
});
static_assert(labels.words[0] == 0x0002 && labels.words[2] == 0x0001);
static_assert(labels.find("later")->isPublic && !labels.find("loop")->isPublic);
constexpr auto defined = assemble([](Asm& a) {
    a.define("T", 9, true);
    a.define("U", -5);
    a.nop();
});
static_assert(defined.find("T")->value == 9 && defined.find("T")->isPublic);
static_assert(defined.find("U")->value == -5 && !defined.find("U")->isPublic);
static_assert(assemble([](Asm& a) {
                  a.clockDiv(2, 128);
                  a.nop();
              }).clockDivInt
                == 2
              && assemble([](Asm& a) {
                     a.clockDiv(2, 128);
                     a.nop();
                 }).clockDivFrac
                   == 128);

// ---- whole programs: the words pioasm (pico-sdk 2.3.1) made of the same text, recorded ---------
constexpr auto ws2812Built = assemble([](Asm& a) {
    a.sideSet(1);
    a.wrapTarget();
    a.label("bitloop");
    a.out(Out::x, 1).side(0).delay(4);
    a.jmp(Cond::notX, "do_zero").side(1).delay(2);
    a.label("do_one");
    a.jmp("bitloop").side(1).delay(3);
    a.label("do_zero");
    a.nop().side(0).delay(3);
    a.wrap();
});
constexpr auto blinkBuilt  = assemble([](Asm& a) {
    a.wrapTarget();
    a.pull(false, false);
    a.out(Out::x, 32);
    a.mov(MovDst::y, MovSrc::x);
    a.set(Set::pins, 1);
    a.label("lp1");
    a.jmp(Cond::yDec, "lp1");
    a.mov(MovDst::y, MovSrc::x);
    a.set(Set::pins, 0);
    a.label("lp2");
    a.jmp(Cond::yDec, "lp2");
    a.wrap();
});
constexpr auto uartRxBuilt = assemble([](Asm& a) {
    constexpr unsigned cyclesPerBit = 8;
    a.define("CyclesPerBit", cyclesPerBit, true);
    a.label("start");
    a.wait(0, Wait::pin, 0);
    a.set(Set::x, 7).delay(10);
    a.label("bitloop");
    a.in(In::pins, 1);
    a.jmp(Cond::xDec, "bitloop").delay(cyclesPerBit - 2);
    a.jmp(Cond::pin, "good_stop");
    a.irq(4, IrqMode::rel);
    a.wait(1, Wait::pin, 0);
    a.jmp("start");
    a.label("good_stop");
    a.push();
});

template<std::size_t N>
consteval bool hasWords(Assembled const&     a,
                        std::array<std::uint16_t,
                                   N> const& w) {
    if(!a.ok() || a.length != N) { return false; }
    for(std::size_t i = 0; i < N; ++i) {
        if(a.words[i] != w[i]) { return false; }
    }
    return true;
}

static_assert(hasWords(ws2812Built,
                       std::array<std::uint16_t,
                                  4>{0x6421,
                                     0x1223,
                                     0x1300,
                                     0xA342}));
static_assert(hasWords(blinkBuilt,
                       std::array<std::uint16_t,
                                  8>{0x8080,
                                     0x6020,
                                     0xA041,
                                     0xE001,
                                     0x0084,
                                     0xA041,
                                     0xE000,
                                     0x0087}));
static_assert(blinkBuilt.wrapTarget == 0 && blinkBuilt.wrap == 7);
static_assert(hasWords(uartRxBuilt,
                       std::array<std::uint16_t,
                                  9>{0x2020,
                                     0xEA27,
                                     0x4001,
                                     0x0642,
                                     0x00C8,
                                     0xC014,
                                     0x20A0,
                                     0x0000,
                                     0x8020}));
static_assert(uartRxBuilt.wrap == 8);

// ---- Program<>: the members a pioasm header has ----------------------------------------------
struct UartRx : Program<uartRxBuilt> {};

static_assert(UartRx::offset("good_stop") == 8 && UartRx::offset("start") == 0);
static_assert(UartRx::define("CyclesPerBit") == 8);
static_assert(UartRx::origin == -1 && UartRx::pioVersion == 0 && UartRx::sidesetCount == 0);
static_assert(Program<ws2812Built>::sidesetCount == 1 && !Program<ws2812Built>::sidesetOptional);

// C++ loops make instructions: the builder's reason to exist
template<unsigned Bits>
struct ShiftIn : Program<assemble([](Asm& a) {
    a.wrapTarget();
    a.wait(1, Wait::gpio, 2);
    for(unsigned i = 0; i < Bits; ++i) { a.in(In::pins, 1).delay(1); }
    a.push();
    a.wrap();
})> {};

static_assert(ShiftIn<3>::instructions.size() == 5 && ShiftIn<3>::instructions[1] == 0x4101);
static_assert(ShiftIn<3>::wrap == 4);

// ---- the builder's messages name the C++ call: <file>:<line>: ---------------------------------
consteval bool builderSays(Assembled const& a,
                           std::uint32_t    line,
                           std::string_view m) {
    std::string_view           msg  = a.message();
    constexpr std::string_view file = "asm_encoding_test.cpp:";
    if(!msg.starts_with(file)) { return false; }
    msg.remove_prefix(file.size());
    std::uint32_t n = 0;
    while(!msg.empty() && msg.front() >= '0' && msg.front() <= '9') {
        n = n * 10 + static_cast<std::uint32_t>(msg.front() - '0');
        msg.remove_prefix(1);
    }
    return n == line && msg.starts_with(": ") && msg.substr(2) == m;
}

// Each test hands the builder a source location of its own (`here`), so the line it expects does
// not depend on how the formatter lays the call out.
// too many instructions: the 33rd call
constexpr auto tooLongAt = std::source_location::current();
constexpr auto tooLong   = assemble([](Asm& a) {
    for(int i = 0; i < 32; ++i) { a.nop(); }
    a.nop(tooLongAt);
});
static_assert(builderSays(tooLong,
                          tooLongAt.line(),
                          "more than 32 instructions: instruction memory has 32"));
// a check in finish(): the call that added the instruction
constexpr auto badDelayAt = std::source_location::current();
constexpr auto badDelay   = assemble([](Asm& a) {
    a.sideSet(1);
    a.nop(badDelayAt).side(0).delay(16);
});
static_assert(builderSays(badDelay,
                          badDelayAt.line(),
                          "delay 16 does not fit: .side_set 1 leaves 4 delay bits (max 15)"));
constexpr auto badLabelAt = std::source_location::current();
constexpr auto badLabel   = assemble([](Asm& a) { a.jmp("nowhere", badLabelAt); });
static_assert(builderSays(badLabel,
                          badLabelAt.line(),
                          "jmp to unknown label 'nowhere'"));
// a program-wide check: the assemble() call
constexpr auto emptyAt = std::source_location::current();
constexpr auto empty   = assemble([](Asm&) {}, emptyAt);
static_assert(builderSays(empty,
                          emptyAt.line(),
                          "the program has no instructions"));
// a directive's check in finish(): the directive's call
constexpr auto lateTargetAt = std::source_location::current();
constexpr auto lateTarget   = assemble([](Asm& a) {
    a.nop();
    a.wrapTarget(lateTargetAt);
});
static_assert(builderSays(lateTarget,
                          lateTargetAt.line(),
                          ".wrap_target after the last instruction"));
// and a call without one names its own line: the message is `<this file>:<some line>: ...`
static_assert(
  assemble([](Asm& a) { a.set(Set::x, 40); }).message().starts_with("asm_encoding_test.cpp:"));

// ---- every other message, once --------------------------------------------------------------
consteval bool fails(Assembled const& a,
                     std::string_view m) {
    auto const msg = a.message();
    return msg.size() >= m.size() && msg.substr(msg.size() - m.size()) == m;
}

static_assert(fails(assemble([](Asm& a) {
                        a.sideSet(2, true);
                        a.nop().delay(8);
                    }),
                    "delay 8 does not fit: .side_set 2 opt leaves 2 delay bits (max 3)"));
static_assert(fails(assemble([](Asm& a) { a.nop().delay(32); }),
                    "delay 32 does not fit (max 31)"));
static_assert(fails(assemble([](Asm& a) { a.sideSet(5, true); }),
                    ".side_set 5 opt needs 6 bits, there are 5"));
static_assert(fails(assemble([](Asm& a) {
                        a.sideSet(1);
                        a.nop().side(2);
                    }),
                    "side 2 does not fit .side_set 1 (max 1)"));
static_assert(fails(assemble([](Asm& a) {
                        a.sideSet(1);
                        a.nop().side(0);
                        a.nop();
                    }),
                    "side-set is not optional here (.side_set 1): 'side' missing"));
static_assert(fails(assemble([](Asm& a) { a.nop().side(0); }),
                    "side 0 without .side_set"));
static_assert(fails(assemble([](Asm& a) {
                        a.nop();
                        a.sideSet(1);
                    }),
                    ".side_set after an instruction"));
static_assert(fails(assemble([](Asm& a) { a.out(Out::x, 33); }),
                    "out bit count 33 (1..32)"));
static_assert(fails(assemble([](Asm& a) { a.in(In::x, 0); }),
                    "in bit count 0 (1..32)"));
static_assert(fails(assemble([](Asm& a) { a.set(Set::x, 32); }),
                    "set value 32 (0..31)"));
static_assert(fails(assemble([](Asm& a) { a.wait(1, Wait::irq, 8); }),
                    "wait irq 8 (0..7)"));
static_assert(fails(assemble([](Asm& a) { a.wait(2, Wait::pin, 0); }),
                    "wait polarity 2 (0 or 1)"));
static_assert(fails(assemble([](Asm& a) {
                        a.pioVersion(1);
                        a.wait(1, Wait::jmppin, 4);
                    }),
                    "wait jmppin + 4: offset 0..3"));
static_assert(fails(assemble([](Asm& a) { a.irq(8); }),
                    "irq 8 (0..7)"));
static_assert(fails(assemble([](Asm& a) {
                        a.pioVersion(1);
                        a.fifo(Fifo::putget);
                        a.movToRx(4);
                    }),
                    "rxfifo index 4 (0..3)"));
static_assert(fails(assemble([](Asm& a) {
                        a.pioVersion(1);
                        a.movToRx(0);
                    }),
                    "mov rxfifo[], isr needs .fifo txput or putget"));
static_assert(fails(assemble([](Asm& a) {
                        a.nop();
                        a.jmp(Cond::always, 5U);
                    }),
                    "jmp target 5 is beyond the end of the program (2 instructions)"));
static_assert(fails(assemble([](Asm& a) {
                        a.label("a");
                        a.nop();
                        a.label("a");
                        a.nop();
                    }),
                    "'a' is already defined as a label"));
static_assert(fails(assemble([](Asm& a) { a.mov(MovDst::pindirs, MovSrc::x); }),
                    "'mov pindirs' needs .pio_version 1 (RP2350)"));
static_assert(fails(assemble([](Asm& a) { a.wait(1, Wait::jmppin, 0); }),
                    "'wait jmppin' needs .pio_version 1 (RP2350)"));
static_assert(fails(assemble([](Asm& a) { a.irq(1, IrqMode::prev); }),
                    "'irq prev' needs .pio_version 1 (RP2350)"));
static_assert(
  fails(assemble([](Asm& a) {
            a.origin(30);
            a.nop();
            a.nop();
            a.nop();
        }),
        "the program is 3 instructions at .origin 30: instruction memory has 32 slots"));
static_assert(fails(assemble([](Asm& a) { a.wrap(); }),
                    ".wrap before the first instruction"));
static_assert(fails(assemble([](Asm& a) {
                        a.pioVersion(1);
                        a.fifo(Fifo::txput);
                        a.inConfig(5, false, true);
                        a.nop();
                    }),
                    "autopush does not go with this .fifo configuration"));
static_assert(fails(assemble([](Asm& a) { a.clockDiv(0, 0); }),
                    ".clock_div 0: 1..65535"));
static_assert(fails(assemble([](Asm& a) {
                        a.pioVersion(1);
                        a.inConfig(33);
                    }),
                    ".in pin count 33 (1..32)"));
static_assert(fails(assemble([](Asm& a) { a.inConfig(5); }),
                    ".in pin count 5: must be 32 for PIO version 0"));
static_assert(fails(assemble([](Asm& a) { a.inConfig(32, false, true, 33); }),
                    ".in threshold 33 (1..32)"));
static_assert(fails(assemble([](Asm& a) { a.outConfig(33); }),
                    ".out pin count 33 (0..32)"));
static_assert(fails(assemble([](Asm& a) { a.outConfig(8, true, true, 0); }),
                    ".out threshold 0 (1..32)"));
static_assert(fails(assemble([](Asm& a) { a.setConfig(6); }),
                    ".set pin count 6 (0..5)"));
static_assert(fails(assemble([](Asm& a) { a.movStatus(MovStatusKind::txLessThan, 32); }),
                    ".mov_status: FIFO level 32 (0..31)"));
static_assert(fails(assemble([](Asm& a) { a.movStatus(MovStatusKind::irqSet, 8); }),
                    ".mov_status irq 8: an IRQ flag is 0..7"));
static_assert(fails(assemble([](Asm& a) { a.movStatus(MovStatusKind::irqSet, 1, IrqMode::rel); }),
                    ".mov_status irq: no 'rel'"));
// NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange) // the reserved value is what the test feeds it
static_assert(fails(assemble([](Asm& a) { a.mov(MovDst::x, MovSrc::y, static_cast<MovOp>(3)); }),
                    "mov: operation 3 is reserved"));
static_assert(fails(assemble([](Asm& a) {
                        a.pioVersion(1);
                        a.fifo(Fifo::txput);
                        a.movFromRx(0);
                    }),
                    "mov osr, rxfifo[] needs .fifo txget or putget"));
static_assert(fails(assemble([](Asm& a) {
                        a.pioVersion(1);
                        a.fifo(Fifo::tx);
                        a.push();
                    }),
                    "push needs .fifo txrx or rx"));
static_assert(fails(assemble([](Asm& a) { a.label("abcdefghijklmnopqrstuvwxyz0123456"); }),
                    "name 'abcdefghijklmnopqrstuvwxyz0123456': 1..31 characters"));
static_assert(fails(assemble([](Asm& a) { a.sideSet(-1); }),
                    "side-set count -1 is negative"));
static_assert(fails(assemble([](Asm& a) { a.wait(1, Wait::gpio, 32); }),
                    "wait gpio 32: 0..31 (for GPIO 32..47 use the GPIO window, base 16)"));
static_assert(fails(assemble([](Asm& a) { a.wait(1, Wait::pin, 0, IrqMode::rel); }),
                    "wait: prev/next/rel belong to 'irq'"));
static_assert(fails(assemble([](Asm& a) {
                        a.nop();
                        a.wrap();
                        a.wrap();
                    }),
                    ".wrap given twice"));
static_assert(fails(assemble([](Asm& a) {
                        a.wrapTarget();
                        a.wrapTarget();
                        a.nop();
                    }),
                    ".wrap_target given twice"));
static_assert(fails(assemble([](Asm& a) { a.word(0x10000); }),
                    ".word 65536: a 16-bit value"));

// ---- the chip package's programs, written with the builder -------------------------------------
// The words are what pioasm (-o kvasir, pico-sdk 2.3.1) made of the .pio files they replaced
// (clockout.pio, i2s.pio, PioQspi.pio, pico-examples' ws2812.pio), recorded before those were
// deleted; the firmware images built from both were byte-identical.
using W = std::uint16_t;
static_assert(ClockOutProgram::instructions
                == std::array<W,
                              2>{0xE001,
                                 0xE000}
              && ClockOutProgram::wrap == 1);
static_assert(I2sOutMasterProgram::instructions
                == std::array<W,
                              9>{0xE02E,
                                 0x6001,
                                 0x0841,
                                 0x7001,
                                 0xF82E,
                                 0x7001,
                                 0x1845,
                                 0x6001,
                                 0xE82E}
              && I2sOutMasterProgram::wrapTarget == 1 && I2sOutMasterProgram::wrap == 8
              && I2sOutMasterProgram::sidesetCount == 2 && !I2sOutMasterProgram::sidesetOptional);
static_assert(I2sOutSlaveProgram::instructions
                == std::array<W,
                              16>{0x20A1,
                                  0x80A0,
                                  0x2021,
                                  0x20A0,
                                  0xE02F,
                                  0x2020,
                                  0x6001,
                                  0x20A0,
                                  0x0045,
                                  0x20A1,
                                  0x20A0,
                                  0xE02F,
                                  0x2020,
                                  0x6001,
                                  0x20A0,
                                  0x004C}
              && I2sOutSlaveProgram::wrapTarget == 1 && I2sOutSlaveProgram::wrap == 15);
using Qspi = Kvasir::Display::QspiPanelProgram;
static_assert(Qspi::instructions
                == std::array<W,
                              10>{0x6001,
                                  0x1000,
                                  0x6004,
                                  0x1002,
                                  0x80A0,
                                  0x6020,
                                  0xE080,
                                  0x5001,
                                  0x0047,
                                  0x0009}
              && Qspi::wrap == 9 && Qspi::sidesetCount == 1 && Qspi::offset("single") == 0
              && Qspi::offset("quad") == 2 && Qspi::offset("read") == 4);
static_assert(Ws2812Program<3,
                            4,
                            5>::instructions
                == std::array<W,
                              4>{0x6421,
                                 0x1223,
                                 0x1300,
                                 0xA342}
              && Ws2812Program<3,
                               4,
                               5>::wrap
                   == 3
              && Ws2812Program<3,
                               4,
                               5>::cyclesPerBit
                   == 12);
}   // namespace

int main() {
    std::printf(
      "pio asm: encodings, packing, labels, programs and messages checked at compile time\n");
    return 0;
}
