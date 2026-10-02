// Every instruction form, worked out by hand from the RP2350 datasheet 11.4 (Table 980 and the
// operand lists of 11.4.2-11.4.12), side-set and delay packing (11.4.1), the builder against the
// parser, and the messages. The cross-check against pioasm (generated files) is the other half.
#include "crosscheck_summary.hpp"
#include "pio/AsmParse.hpp"
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

consteval std::uint16_t text(std::string_view t) {
    auto const a = parse(t);
    return a.ok() ? a.words[0] : 0xFFFF;
}

// ---- JMP, 11.4.2: 000 | cond | address ----------------------------------------------------
static_assert(one([](Asm& a) { a.jmp(Cond::always, 0U); }) == 0x0000);
static_assert(text(".program p\njmp 0\n") == 0x0000);
static_assert(text(".program p\njmp !x 0\n") == 0x0020);      // 001
static_assert(text(".program p\njmp x-- 0\n") == 0x0040);     // 010
static_assert(text(".program p\njmp !y 0\n") == 0x0060);      // 011
static_assert(text(".program p\njmp y-- 0\n") == 0x0080);     // 100
static_assert(text(".program p\njmp x!=y 0\n") == 0x00A0);    // 101
static_assert(text(".program p\njmp pin 0\n") == 0x00C0);     // 110
static_assert(text(".program p\njmp !osre 0\n") == 0x00E0);   // 111
static_assert(text(".program p\nnop\nnop\njmp x--, 2\n") == 0xA042);
static_assert(parse(".program p\nnop\nnop\njmp x--, 2\n").words[2] == 0x0042);   // 010 | 00010

// ---- WAIT, 11.4.3: 001 | pol[7] | source[6:5] | index[4:0] --------------------------------
static_assert(text(".program p\nwait 0 gpio 5\n") == 0x2005);   // src 00
static_assert(text(".program p\nwait 1 gpio 31\n") == 0x209F);
static_assert(text(".program p\nwait 1 pin 3\n") == 0x20A3);   // 0x80 | 01<<5 | 3
static_assert(text(".program p\nwait pin 3\n") == 0x20A3);     // polarity defaults to 1
static_assert(text(".program p\nwait 0 irq 7\n") == 0x2047);   // 10<<5 | 7
static_assert(text(".program p\nwait 1 irq 1 rel\n")
              == 0x20D1);   // 0x80 | 0x40 | IdxMode 10<<3 | 1
static_assert(one([](Asm& a) { a.wait(1, Wait::irq, 1, IrqMode::rel); }) == 0x20D1);
static_assert(text(".program p\n.pio_version 1\nwait 1 irq prev 2\n") == 0x20CA);    // 01<<3 | 2
static_assert(text(".program p\n.pio_version 1\nwait 1 irq next, 3\n") == 0x20DB);   // 11<<3 | 3
static_assert(text(".program p\n.pio_version 1\nwait 1 jmppin\n") == 0x20E0);        // src 11
static_assert(text(".program p\n.pio_version 1\nwait 0 jmppin + 3\n") == 0x2063);

// ---- IN, 11.4.4: 010 | source | bit count (32 as 0) ------------------------------------------
static_assert(text(".program p\nin pins, 32\n") == 0x4000);
static_assert(text(".program p\nin x, 1\n") == 0x4021);
static_assert(text(".program p\nin y, 5\n") == 0x4045);
static_assert(text(".program p\nin null, 31\n") == 0x407F);
static_assert(text(".program p\nin isr, 8\n") == 0x40C8);    // 110
static_assert(text(".program p\nin osr, 16\n") == 0x40F0);   // 111

// ---- OUT, 11.4.5: 011 | destination | bit count --------------------------------------------
static_assert(text(".program p\nout pins, 1\n") == 0x6001);
static_assert(text(".program p\nout x, 2\n") == 0x6022);
static_assert(text(".program p\nout y, 3\n") == 0x6043);
static_assert(text(".program p\nout null, 32\n") == 0x6060);
static_assert(text(".program p\nout pindirs, 4\n") == 0x6084);   // 100
static_assert(text(".program p\nout pc, 5\n") == 0x60A5);        // 101
static_assert(text(".program p\nout isr, 6\n") == 0x60C6);       // 110
static_assert(text(".program p\nout exec, 16\n") == 0x60F0);     // 111

// ---- PUSH / PULL, 11.4.6-7: 100 | pull | IfF/IfE | Blk | 00000 ------------------------------
static_assert(text(".program p\npush\n") == 0x8020);   // block by default
static_assert(text(".program p\npush noblock\n") == 0x8000);
static_assert(text(".program p\npush iffull\n") == 0x8060);
static_assert(text(".program p\npush iffull noblock\n") == 0x8040);
static_assert(text(".program p\npull\n") == 0x80A0);
static_assert(text(".program p\npull noblock\n") == 0x8080);
static_assert(text(".program p\npull ifempty\n") == 0x80E0);
static_assert(text(".program p\npull ifempty block\n") == 0x80E0);
static_assert(text(".program p\npull ifempty noblock\n") == 0x80C0);

// ---- MOV, 11.4.10: 101 | destination | op | source --------------------------------------------
static_assert(text(".program p\nmov x, y\n") == 0xA022);    // 001 | 00 | 010
static_assert(text(".program p\nmov y, ~x\n") == 0xA049);   // 010 | 01 | 001
static_assert(text(".program p\nmov y, !x\n") == 0xA049);
static_assert(text(".program p\nmov isr, ::osr\n") == 0xA0D7);   // 110 | 10 | 111
static_assert(text(".program p\nmov pins, null\n") == 0xA003);
static_assert(text(".program p\nmov exec, x\n") == 0xA081);      // 100
static_assert(text(".program p\nmov pc, status\n") == 0xA0A5);   // 101 | 00 | 101
static_assert(text(".program p\nmov osr, isr\n") == 0xA0E6);
static_assert(text(".program p\nnop\n") == 0xA042);                              // mov y, y
static_assert(text(".program p\n.pio_version 1\nmov pindirs, x\n") == 0xA061);   // 011
static_assert(one<1>([](Asm& a) { a.mov(MovDst::pindirs, MovSrc::x); }) == 0xA061);

// ---- MOV to / from RX, 11.4.8-9 (v1): 100 | from | 00 | 1 | IdxI | 0 | index -----------------
static_assert(text(".program p\n.pio_version 1\n.fifo putget\nmov rxfifo[y], isr\n") == 0x8010);
static_assert(text(".program p\n.pio_version 1\n.fifo txput\nmov rxfifo[2], isr\n") == 0x801A);
static_assert(text(".program p\n.pio_version 1\n.fifo txget\nmov osr, rxfifo[y]\n") == 0x8090);
static_assert(text(".program p\n.pio_version 1\n.fifo putget\nmov osr, rxfifo[3]\n") == 0x809B);
static_assert(one<1,
                  Fifo::putget>([](Asm& a) { a.movToRx(2); })
              == 0x801A);
static_assert(one<1,
                  Fifo::putget>([](Asm& a) { a.movFromRxY(); })
              == 0x8090);

// ---- IRQ, 11.4.11: 110 | 0 | Clr | Wait | IdxMode | index ------------------------------------
static_assert(text(".program p\nirq 3\n") == 0xC003);
static_assert(text(".program p\nirq set 3\n") == 0xC003);
static_assert(text(".program p\nirq nowait 3\n") == 0xC003);
static_assert(text(".program p\nirq wait 2\n") == 0xC022);
static_assert(text(".program p\nirq clear 7\n") == 0xC047);
static_assert(text(".program p\nirq 1 rel\n") == 0xC011);
static_assert(text(".program p\n.pio_version 1\nirq prev 0\n") == 0xC008);
static_assert(text(".program p\n.pio_version 1\nirq next wait 5\n")
              == 0xC03D);   // 0x20 | 11<<3 | 5
static_assert(one<1>([](Asm& a) { a.irq(0, IrqMode::next); }) == 0xC018);

// ---- SET, 11.4.12: 111 | destination | data --------------------------------------------------
static_assert(text(".program p\nset pins, 1\n") == 0xE001);
static_assert(text(".program p\nset x, 31\n") == 0xE03F);
static_assert(text(".program p\nset y, 0\n") == 0xE040);
static_assert(text(".program p\nset pindirs, 3\n") == 0xE083);      // 100
static_assert(text(".program p\nset pindirs, 0x1f\n") == 0xE09F);   // = StateMachine's literal
static_assert(text(".program p\n.word 0xbeef\n") == 0xBEEF);

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
static_assert(
  !usesPioV1(parse(".pio_version 1\n.program p\njmp pin 0\nmov y, ~null\npush noblock\n").words));

// ---- side-set and delay, 11.4.1: bits 12:8, side-set MSBs (enable first when opt) -----------
static_assert(text(".program p\n.side_set 1\nnop side 1 [3]\n") == 0xB342);       // 1<<4 | 3
static_assert(text(".program p\n.side_set 2 opt\nnop side 2 [1]\n") == 0xB942);   // 0x10 | 2<<2 | 1
static_assert(text(".program p\n.side_set 2 opt\nnop [3]\n") == 0xA342);   // no side: enable 0
static_assert(text(".program p\n.side_set 3 opt pindirs\nnop side 5 [1]\n")
              == 0xBB42);   // 0x10 | 5<<1 | 1
static_assert(text(".program p\n.side_set 5\nnop side 31\n") == 0xBF42);
static_assert(text(".program p\nnop [31]\n") == 0xBF42);
static_assert(text(".program p\n.side_set 1\nnop [2] side 1\n") == 0xB242);   // either order
static_assert(parse(".program p\n.side_set 3 opt pindirs\nnop side 5\n").sidesetPindirs);
static_assert(parse(".program p\n.side_set 3 opt pindirs\nnop side 5\n").sidesetCount == 3);

// ---- wrap, labels, defines, programs ---------------------------------------------------------
constexpr auto three = parse(".program p\nnop\nnop\nnop\n");
static_assert(three.wrapTarget == 0 && three.wrap == 2 && three.length == 3);
constexpr auto wrapped = parse(".program p\nnop\n.wrap_target\nnop\n.wrap\nnop\n");
static_assert(wrapped.wrapTarget == 1 && wrapped.wrap == 1);
constexpr auto labels = parse(".program p\n  jmp later\nloop: nop\npublic later:\n  jmp loop\n");
static_assert(labels.words[0] == 0x0002 && labels.words[2] == 0x0001);
static_assert(labels.find("later")->isPublic && !labels.find("loop")->isPublic);
// several programs: the one named is assembled, global defines reach all of them
constexpr std::string_view two
  = ".define public G 4\n.program a\nset x, G\n.program b\nset y, (G + 1)\n";
static_assert(parse(two,
                    {},
                    "a")
                  .words[0]
                == 0xE024
              && parse(two,
                       {},
                       "b")
                     .words[0]
                   == 0xE045);
// values from C++, and the pioasm expression grammar (parser.yy's %left order: & | ^ bind
// tightest, then * /, + -, << >>; unary - as binary -, :: loosest)
constexpr auto ex = parse(
  ".program p\n.define A 1 + 2 * 3 & 1\n.define B -2*3+1\n.define C ::1 + 1\n"
  ".define D 1 << 2 + 1\n.define E (0b101 ^ 0x3) | one\n.define F T - 1\nnop\n",
  {
    {"T", 9}
});
static_assert(ex.find("A")->value == 3 && ex.find("B")->value == -5
              && ex.find("C")->value == 0x40000000);
static_assert(ex.find("D")->value == 8 && ex.find("E")->value == 7 && ex.find("F")->value == 8);
static_assert(ex.find("T")->value == 9 && ex.find("T")->isPublic);
// comments, a c-sdk block, upper case, CRLF
constexpr auto noise
  = parse("; a\r\n.PROGRAM p // b\r\n/* c\r\n d */ JMP PIN 0\r\n% c-sdk {\r\nint x; %%\r\n%}\r\n");
static_assert(noise.ok() && noise.words[0] == 0x00C0 && noise.length == 1);

// ---- a .pio file by #embed: straight into parse(), or through an unsigned char / char array
#if defined(__clang__)
_Pragma("clang diagnostic ignored \"-Wc23-extensions\"")   // #embed in C++
#endif
  // clang-format off
struct Direct : Program<parse({
#embed "pio/many.pio"
}, {}, "SECOND")> {};
constexpr unsigned char manyBytes[] = {
#embed "pio/many.pio"
};
constexpr char manyChars[] = {
#embed "pio/many.pio"
};
// clang-format on
static_assert(Direct::Instructions.size() == 15 && Direct::Origin == 0
              && Direct::offset("entry") == 0);
static_assert(Program<parse(manyBytes,
                            {},
                            "SECOND")>::Instructions
              == Direct::Instructions);
static_assert(Program<parse(manyChars,
                            {},
                            "first")>::Instructions.size()
              == 3);
// a string literal: its terminating NUL is not part of the text
static_assert(parse(".program p\nset x, 1\n").length == 1);

// ---- the builder writes what the text says -----------------------------------------------------
constexpr std::string_view ws2812Text  = R"pio(
.program ws2812
.side_set 1
.wrap_target
bitloop:
    out x, 1       side 0 [4]
    jmp !x do_zero side 1 [2]
do_one:
    jmp  bitloop   side 1 [3]
do_zero:
    nop            side 0 [3]
.wrap
)pio";
constexpr auto             ws2812Built = assemble([](Asm& a) {
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

constexpr std::string_view blinkText  = R"pio(
.program blink
.wrap_target
    pull noblock
    out x, 32
    mov y, x
    set pins, 1
lp1:
    jmp y-- lp1
    mov y, x
    set pins, 0
lp2:
    jmp y-- lp2
.wrap
)pio";
constexpr auto             blinkBuilt = assemble([](Asm& a) {
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

constexpr std::string_view uartRxText  = R"pio(
.program uart_rx
.define public CyclesPerBit 8
start:
    wait 0 pin 0
    set x, 7    [10]
bitloop:
    in pins, 1
    jmp x-- bitloop [CyclesPerBit - 2]
    jmp pin good_stop
    irq 4 rel
    wait 1 pin 0
    jmp start
good_stop:
    push
)pio";
constexpr auto             uartRxBuilt = assemble([](Asm& a) {
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

consteval bool sameProgram(Assembled const& a,
                           Assembled const& b) {
    if(!a.ok() || !b.ok() || a.length != b.length || a.wrap != b.wrap
       || a.wrapTarget != b.wrapTarget)
    {
        return false;
    }
    for(std::size_t i = 0; i < a.length; ++i) {
        if(a.words[i] != b.words[i]) { return false; }
    }
    return a.sidesetCount == b.sidesetCount && a.sidesetOpt == b.sidesetOpt;
}

static_assert(sameProgram(ws2812Built,
                          parse(ws2812Text)));
static_assert(sameProgram(blinkBuilt,
                          parse(blinkText)));
static_assert(sameProgram(uartRxBuilt,
                          parse(uartRxText)));
static_assert(ws2812Built.words[0] == 0x6421 && ws2812Built.words[1] == 0x1223
              && ws2812Built.words[2] == 0x1300 && ws2812Built.words[3] == 0xA342);

// C++ loops make instructions: the builder's reason to exist
template<unsigned Bits>
struct ShiftIn : Program<assemble([](Asm& a) {
    a.wrapTarget();
    a.wait(1, Wait::gpio, 2);
    for(unsigned i = 0; i < Bits; ++i) { a.in(In::pins, 1).delay(1); }
    a.push();
    a.wrap();
})> {};

static_assert(ShiftIn<3>::Instructions.size() == 5 && ShiftIn<3>::Instructions[1] == 0x4101);
static_assert(ShiftIn<3>::Wrap == 4);

// ---- Program<>: the members a pioasm header has ----------------------------------------------
template<unsigned T1, unsigned T2, unsigned T3>
struct TextWs2812Program
  : Program<parse(R"pio(
.program ws2812
.side_set 1
.wrap_target
bitloop:
    out x, 1       side 0 [T3 - 1]
    jmp !x do_zero side 1 [T1 - 1]
do_one:
    jmp  bitloop   side 1 [T2 - 1]
do_zero:
    nop            side 0 [T2 - 1]
.wrap
)pio",
                  {
                    {"T1", T1},
                    {"T2", T2},
                    {"T3", T3}
})> {};

static_assert(TextWs2812Program<3,
                                4,
                                5>::Instructions
              == std::array<std::uint16_t,
                            4>{0x6421,
                               0x1223,
                               0x1300,
                               0xA342});
static_assert(TextWs2812Program<3,
                                4,
                                5>::offset("do_zero")
                == 3
              && TextWs2812Program<3,
                                   4,
                                   5>::offset("bitloop")
                   == 0);
static_assert(TextWs2812Program<3,
                                4,
                                5>::define("T2")
              == 4);
static_assert(TextWs2812Program<3,
                                4,
                                5>::SidesetCount
                == 1
              && !TextWs2812Program<3,
                                    4,
                                    5>::SidesetOptional);
static_assert(TextWs2812Program<3,
                                4,
                                5>::Origin
                == -1
              && TextWs2812Program<3,
                                   4,
                                   5>::PioVersion
                   == 0);
static_assert(TextWs2812Program<2,
                                5,
                                3>::Instructions[0]
              == 0x6221);   // other cycles, other delays

// ---- the messages: the line, and numbers that say what is wrong ------------------------------
consteval bool says(Assembled const& a,
                    std::string_view m) {
    return a.message() == m;
}

static_assert(says(parse(".program p\n.side_set 1\nnop side 0\n  nop side 0 [16]\n"),
                   "line 4: delay 16 does not fit: .side_set 1 leaves 4 delay bits (max 15)"));
static_assert(says(parse(".program p\n.side_set 2 opt\nnop [8]\n"),
                   "line 3: delay 8 does not fit: .side_set 2 opt leaves 2 delay bits (max 3)"));
static_assert(says(parse(".program p\nnop [32]\n"),
                   "line 2: delay 32 does not fit (max 31)"));
static_assert(says(parse(".program p\n.side_set 5 opt\n"),
                   "line 2: .side_set 5 opt needs 6 bits, there are 5"));
static_assert(says(parse(".program p\n.side_set 1\nnop side 2\n"),
                   "line 3: side 2 does not fit .side_set 1 (max 1)"));
static_assert(says(parse(".program p\n.side_set 1\nnop side 0\nnop\n"),
                   "line 4: side-set is not optional here (.side_set 1): 'side' missing"));
static_assert(says(parse(".program p\nnop side 0\n"),
                   "line 2: side 0 without .side_set"));
static_assert(says(parse(".program p\nnop\n.side_set 1\n"),
                   "line 3: .side_set after an instruction"));
static_assert(says(parse(".program p\nout x, 33\n"),
                   "line 2: out bit count 33 (1..32)"));
static_assert(says(parse(".program p\nin x, 0\n"),
                   "line 2: in bit count 0 (1..32)"));
static_assert(says(parse(".program p\nset x, 32\n"),
                   "line 2: set value 32 (0..31)"));
static_assert(says(parse(".program p\nwait 1 irq 8\n"),
                   "line 2: wait irq 8 (0..7)"));
static_assert(says(parse(".program p\nwait 2 pin 0\n"),
                   "line 2: wait polarity 2 (0 or 1)"));
static_assert(says(parse(".program p\n.pio_version 1\nwait 1 jmppin + 4\n"),
                   "line 3: wait jmppin + 4: offset 0..3"));
static_assert(says(parse(".program p\nirq 8\n"),
                   "line 2: irq 8 (0..7)"));
static_assert(says(parse(".program p\n.pio_version 1\n.fifo putget\nmov rxfifo[4], isr\n"),
                   "line 4: rxfifo index 4 (0..3)"));
static_assert(says(parse(".program p\n.pio_version 1\nmov rxfifo[0], isr\n"),
                   "line 3: mov rxfifo[], isr needs .fifo txput or putget"));
static_assert(says(parse(".program p\njmp bitlop\n"),
                   "line 2: undefined symbol 'bitlop'"));
static_assert(says(parse(".program p\nnop\njmp 5\n"),
                   "line 3: jmp target 5 is beyond the end of the program (2 instructions)"));
static_assert(says(parse(".program p\na: nop\na: nop\n"),
                   "line 3: 'a' is already defined, at line 2"));
static_assert(says(parse(".program p\n.define set 1\n"),
                   "line 2: 'set' is a keyword"));
static_assert(says(parse(".program p\n.define T1 3\nnop [T1]\n",
                         {
                           {"T1",
                            1}
}),
                   "line 2: 'T1' is given from C++ and defined here"));
static_assert(says(parse(".program p\n.define a b\n.define b a+1\nnop [a]\n"),
                   "line 3: circular dependency in the definition of 'a'"));
static_assert(says(parse(".program p\nmov pindirs, x\n"),
                   "line 2: 'mov pindirs' needs .pio_version 1 (RP2350)"));
static_assert(says(parse(".program p\nwait 1 jmppin\n"),
                   "line 2: expected irq, gpio or pin, found 'jmppin'")
              || says(parse(".program p\nwait 1 jmppin\n"),
                      "line 2: 'wait jmppin' needs .pio_version 1 (RP2350)"));
static_assert(says(parse(".program p\nirq prev 1\n"),
                   "line 2: 'irq prev' needs .pio_version 1 (RP2350)"));
static_assert(says(parse(".program p\nin status, 1\n"),
                   "line 2: in status: source 101 is reserved (RP2350 datasheet 11.4.4.2)"));
static_assert(says(parse(".program p\nnop [1 / 0]\n"),
                   "line 2: division by zero"));
static_assert(says(parse(".program p\nnop @\n"),
                   "line 2: an invalid character"));
static_assert(says(parse(".program p\nfoo nop\n"),
                   "line 2: expected ':' after a label, found 'nop'"));
static_assert(says(parse(".program p\n.bogus 1\n"),
                   "line 2: unknown directive .bogus"));
static_assert(
  says(parse(".program p\n.origin 30\nnop\nnop\nnop\n"),
       "line 2: the program is 3 instructions at .origin 30: instruction memory has 32 slots"));
static_assert(says(parse("nop\n"),
                   "line 1: an instruction outside of a .program"));
static_assert(says(parse(".program a\nnop\n.program b\nnop\n"),
                   "line 3: the text holds more than one .program: name the one to assemble"));
static_assert(says(parse(".program a\nnop\n",
                         {},
                         "b"),
                   "no .program b in the text"));
static_assert(says(parse(".program p\n"),
                   "the program has no instructions"));
static_assert(says(parse(".program p\n.wrap\n"),
                   "line 2: .wrap before the first instruction"));

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

static_assert(fails(parse(".program p\n.pio_version 1\n.fifo txput\n.in 5 left auto\nnop\n"),
                    "line 4: autopush does not go with this .fifo configuration"));
static_assert(fails(parse(".program p\n.clock_div 0.5\nnop\n"),
                    "line 2: clock divider must be between 1 and 65535"));
static_assert(fails(assemble([](Asm& a) { a.clockDiv(0, 0); }),
                    ".clock_div 0: 1..65535"));
static_assert(parse(".program p\n.clock_div 2.5\nnop\n").clockDivInt == 2
              && parse(".program p\n.clock_div 2.5\nnop\n").clockDivFrac == 128);
static_assert(fails(parse(".program p\n.pio_version 1\n.in 33\nnop\n"),
                    "line 3: .in pin count 33 (1..32)"));
static_assert(fails(parse(".program p\n.in 5\nnop\n"),
                    "line 2: .in pin count 5: must be 32 for PIO version 0"));
static_assert(fails(parse(".program p\n.in 32 left auto 33\nnop\n"),
                    "line 2: .in threshold 33 (1..32)"));
static_assert(fails(parse(".program p\n.out 33\nnop\n"),
                    "line 2: .out pin count 33 (0..32)"));
static_assert(fails(parse(".program p\n.out 8 right auto 0\nnop\n"),
                    "line 2: .out threshold 0 (1..32)"));
static_assert(fails(parse(".program p\n.set 6\nnop\n"),
                    "line 2: .set pin count 6 (0..5)"));
static_assert(fails(parse(".program p\n.mov_status txfifo < 32\nnop\n"),
                    "line 2: .mov_status: FIFO level 32 (0..31)"));
static_assert(fails(parse(".program p\n.mov_status irq set 8\nnop\n"),
                    "line 2: .mov_status irq 8: an IRQ flag is 0..7"));
static_assert(fails(assemble([](Asm& a) { a.movStatus(MovStatusKind::irqSet, 1, IrqMode::rel); }),
                    ".mov_status irq: no 'rel'"));
static_assert(fails(assemble([](Asm& a) { a.mov(MovDst::x, MovSrc::y, static_cast<MovOp>(3)); }),
                    "mov: operation 3 is reserved"));
static_assert(fails(parse(".program p\n.pio_version 1\n.fifo txput\nmov osr, rxfifo[0]\n"),
                    "line 4: mov osr, rxfifo[] needs .fifo txget or putget"));
static_assert(fails(parse(".program p\n.pio_version 1\n.fifo tx\npush\n"),
                    "line 4: push needs .fifo txrx or rx"));
static_assert(fails(parse(".program p\nabcdefghijklmnopqrstuvwxyz0123456: nop\n"),
                    "line 2: name 'abcdefghijklmnopqrstuvwxyz0123456': 1..31 characters"));
static_assert(fails(assemble([](Asm& a) { a.sideSet(-1); }),
                    "side-set count -1 is negative"));
static_assert(fails(parse(".program p\nwait 1 gpio 32\n"),
                    "line 2: wait gpio 32: 0..31 (for GPIO 32..47 use the GPIO window, base 16)"));
static_assert(fails(assemble([](Asm& a) { a.wait(1, Wait::pin, 0, IrqMode::rel); }),
                    "wait: prev/next/rel belong to 'irq'"));
static_assert(fails(parse(".program p\nnop\n.wrap\n.wrap\n"),
                    "line 4: .wrap given twice"));
static_assert(fails(parse(".program p\n.wrap_target\n.wrap_target\nnop\n"),
                    "line 3: .wrap_target given twice"));
static_assert(fails(parse(".program p\nnop\n.wrap_target\n"),
                    "line 3: .wrap_target after the last instruction"));

static_assert(fails(parse(".program p\nnop\n.program q\n",
                          {},
                          "q"),
                    "the program has no instructions"));
static_assert(fails(parse("loop: nop\n"),
                    "line 1: a label outside of a .program"));
static_assert(fails(parse(".program p\nset x, pins\n"),
                    "line 2: expected a value, found 'pins'"));
static_assert(fails(parse(".program p\n.pio_version 1\n.fifo putget\nmov x, rxfifo[0]\n"),
                    "line 4: mov from rxfifo[]: the destination must be osr"));
static_assert(fails(parse(".program p\n.pio_version 1\n.fifo putget\nmov rxfifo[0], x\n"),
                    "line 4: mov rxfifo[]: the source must be isr"));
static_assert(fails(parse(".program p\n.pio_version 1\n.fifo putget\nmov rxfifo[0], rxfifo[1]\n"),
                    "line 4: mov rxfifo[] to rxfifo[]"));
static_assert(fails(parse(".program p\n.pio_version 1\n.fifo putget\nmov rxfifo[0], !isr\n"),
                    "line 4: mov to or from rxfifo[] takes no '!' or '::'"));
static_assert(fails(parse(""),
                    "no .program in the text"));
static_assert(fails(parse(".program p\n.pio_version 1\nirq prev 1 rel\n"),
                    "line 3: 'rel' does not go with 'irq prev' or 'irq next'"));
static_assert(fails(parse(".program p\n.pio_version 1\nwait 1 irq next 1 rel\n"),
                    "line 3: 'rel' does not go with 'irq prev' or 'irq next'"));
static_assert(fails(parse(".program p\nnop [1 << 32]\n"),
                    "line 2: shift by 32 (0..31)"));
static_assert(fails(parse(".program p\n.side_set 1\nnop side -1\n"),
                    "line 3: expected a value, found '-'"));
static_assert(fails(parse(".program p\n.side_set 1\nnop side (-1)\n"),
                    "line 3: side-set value -1 is negative"));
static_assert(fails(parse(".program p\n.word 0 [1]\n"),
                    "line 2: expected the end of the line, found '['"));
// pioasm's grammar: a label goes before an instruction, and `.word` is a directive
static_assert(fails(parse(".program p\nw: .word 0\n"),
                    "line 2: expected an instruction after the label, found '.word'"));
static_assert(fails(parse(".program p\n.side_set 1 opt\n.word 0 side 1\n"),
                    "line 3: expected the end of the line, found 'side'"));

// the syntax errors: what was expected, and what came instead
static_assert(fails(parse(".program p\nnop [(1 + 2]\n"),
                    "line 2: expected ')', found ']'"));
static_assert(fails(parse(".program p\n.mov_status txfifo 3\nnop\n"),
                    "line 2: expected '<', found '3'"));
static_assert(fails(parse(".program p\nnop [1\n"),
                    "line 2: expected ']' before the end of the line"));
static_assert(fails(parse(".program p\n.pio_version x\n"),
                    "line 2: expected 0, 1, rp2040 or rp2350, found 'x'"));
static_assert(fails(parse(".program p\n.clock_div x\n"),
                    "line 2: expected a clock divider, found 'x'"));
static_assert(fails(parse(".program p\nmov null, x\n"),
                    "line 2: expected a mov destination, found 'null'"));
static_assert(fails(parse(".program p\nmov x, pc\n"),
                    "line 2: expected a mov source, found 'pc'"));
static_assert(fails(parse(".program p\n.define 3 4\n"),
                    "line 2: expected a name, found '3'"));
static_assert(fails(parse(".program 7\n"),
                    "line 1: expected a program name, found '7'"));
static_assert(fails(parse(".program p\nin pc, 1\n"),
                    "line 2: expected pins, x, y, null, isr or osr, found 'pc'"));
static_assert(fails(parse(".program p\nout status, 1\n"),
                    "line 2: expected pins, x, y, null, pindirs, pc, isr or exec, found 'status'"));
static_assert(fails(parse(".program p\nset isr, 1\n"),
                    "line 2: expected pins, x, y or pindirs, found 'isr'"));
static_assert(fails(parse(".program p\n.mov_status osr\n"),
                    "line 2: expected 'txfifo < N', 'rxfifo < N' or 'irq set N', found 'osr'"));
static_assert(fails(parse(".program p\n.fifo both\n"),
                    "line 2: expected txrx, tx, rx, txput, txget or putget, found 'both'"));
static_assert(fails(parse(".program p\njmp !pin 0\n"),
                    "line 2: expected x, y or osre after '!', found 'pin'"));
static_assert(fails(parse(".program p\njmp x!=x 0\n"),
                    "line 2: expected y after 'x!=', found 'x'"));
static_assert(fails(parse(".program p\njmp x 0\n"),
                    "line 2: expected '--' or '!=' after x, found '0'"));
static_assert(fails(parse(".program p\njmp y!=x 0\n"),
                    "line 2: expected '--' after y, found '!='"));
static_assert(fails(parse(".program p\n.pio_version 1\nwait 1 status\n"),
                    "line 3: expected irq, gpio, pin or jmppin, found 'status'"));
static_assert(fails(parse(".program p\n.mov_status irq next 3\n"),
                    "line 2: expected 'set', found '3'"));
static_assert(fails(parse(".program p\n.pio_version 1\n.fifo putget\nmov osr, rxfifo 1\n"),
                    "line 4: expected '[', found '1'"));

// ---- the chip package's programs, written with the builder since 2026-09-30 --------------------
// The words are what pioasm (-o kvasir, pico-sdk 2.3.1) made of the .pio files they replaced
// (clockout.pio, i2s.pio, PioQspi.pio, pico-examples' ws2812.pio), recorded before those were
// deleted; the firmware images built from both were byte-identical.
using W = std::uint16_t;
static_assert(ClockOutProgram::Instructions
                == std::array<W,
                              2>{0xE001,
                                 0xE000}
              && ClockOutProgram::Wrap == 1);
static_assert(I2sOutMasterProgram::Instructions
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
              && I2sOutMasterProgram::WrapTarget == 1 && I2sOutMasterProgram::Wrap == 8
              && I2sOutMasterProgram::SidesetCount == 2 && !I2sOutMasterProgram::SidesetOptional);
static_assert(I2sOutSlaveProgram::Instructions
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
              && I2sOutSlaveProgram::WrapTarget == 1 && I2sOutSlaveProgram::Wrap == 15);
using Qspi = Kvasir::Display::QspiPanelProgram;
static_assert(Qspi::Instructions
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
              && Qspi::Wrap == 9 && Qspi::SidesetCount == 1 && Qspi::offset("single") == 0
              && Qspi::offset("quad") == 2 && Qspi::offset("read") == 4);
static_assert(Ws2812Program<3,
                            4,
                            5>::Instructions
                == std::array<W,
                              4>{0x6421,
                                 0x1223,
                                 0x1300,
                                 0xA342}
              && Ws2812Program<3,
                               4,
                               5>::Wrap
                   == 3
              && Ws2812Program<3,
                               4,
                               5>::CyclesPerBit
                   == 12);
static_assert(sameProgram(Ws2812Program<3,
                                        4,
                                        5>::assembled(),
                          parse(ws2812Text)));
}   // namespace

int main() {
    std::printf(
      "pio asm: encodings, packing, builder == parser and messages checked at compile time\n");
    std::printf("pio asm: %d programs from %d .pio files equal to pioasm's output:\n",
                crosscheck::Programs,
                crosscheck::Files);
    for(auto const& c : crosscheck::Compared) {
        std::printf("  %.*s\n", static_cast<int>(c.size()), c.data());
    }
    std::printf("pio asm: left out of the comparison:\n");
    for(auto const& c : crosscheck::LeftOut) {
        std::printf("  %.*s\n", static_cast<int>(c.size()), c.data());
    }
    return 0;
}
