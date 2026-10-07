#pragma once
// A PIO assembler that runs in the compiler: the program is a C++ value, built by `Asm` (this
// file) or parsed from pioasm text (`parse()`, AsmParse.hpp), and `Program<...>` turns it into
// the type every PIO driver takes - the same members a pioasm `-o kvasir` header has
// (Instructions, WrapTarget, Wrap, ...), so StateMachine, Provides and programId need nothing new.
//
//   template<unsigned Bits>
//   struct ShiftIn : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
//       using namespace Kvasir::Pio;
//       a.wrapTarget();
//       a.wait(1, Wait::gpio, 2);
//       for(unsigned i = 0; i < Bits; ++i) { a.in(In::pins, 1).delay(1); }
//       a.push();
//       a.wrap();
//   })> {};
//
// A mistake is a compile error that names the line (text) or the instruction (builder):
//   static assertion failed: line 6: delay 16 does not fit: .side_set 1 leaves 4 delay bits (max 15)
//
// Name the program with a struct that derives from Program<...>, as above: the struct's name is
// what appears in symbols and error messages, instead of the whole assembled value.
//
// Standalone on purpose (std headers only): a host compiler checks it without the chip package.
// Encodings: RP2350 datasheet 11.4 (Table 980 and 11.4.2-11.4.12); the RP2040's PIO (RP2040
// datasheet 3.4) has the same encodings minus the forms marked v1 below. pioasm (../pioasm,
// pico-sdk 2.3.1) is the reference for the syntax and the checks, and the host tests compare
// every .pio file they can find with it (chip_rp_common/tests).

#include <algorithm>
#include <array>
#include <cstdint>
#include <source_location>
#include <string_view>
#include <type_traits>

namespace Kvasir { namespace Pio {

    // The instruction words without delay/side-set (bits 12:8), which Asm::finish() adds.
    namespace Enc {
        // 11.4.2 JMP: 000 | cond[7:5] | address[4:0]
        constexpr std::uint16_t jmp(unsigned cond,
                                    unsigned addr) {
            return static_cast<std::uint16_t>((cond << 5U) | addr);
        }

        // 11.4.3 WAIT: 001 | pol[7] | source[6:5] | index[4:0]; for IRQ, index[4:3] is the
        // IdxMode of 11.4.11 (on the RP2040, 3.4.3, bit 4 alone is "rel": the same bits)
        constexpr std::uint16_t wait(unsigned pol,
                                     unsigned src,
                                     unsigned idx) {
            return static_cast<std::uint16_t>(0x2000U | (pol << 7U) | (src << 5U) | idx);
        }

        // 11.4.4 IN: 010 | source[7:5] | bit count[4:0], 32 encoded as 0
        constexpr std::uint16_t in(unsigned src,
                                   unsigned n) {
            return static_cast<std::uint16_t>(0x4000U | (src << 5U) | (n & 0x1FU));
        }

        // 11.4.5 OUT: 011 | destination[7:5] | bit count[4:0], 32 encoded as 0
        constexpr std::uint16_t out(unsigned dst,
                                    unsigned n) {
            return static_cast<std::uint16_t>(0x6000U | (dst << 5U) | (n & 0x1FU));
        }

        // 11.4.6 PUSH: 100 | 0 | IfF[6] | Blk[5] | 00000
        constexpr std::uint16_t push(bool ifFull,
                                     bool block) {
            return static_cast<std::uint16_t>(0x8000U | (ifFull ? 0x40U : 0U)
                                              | (block ? 0x20U : 0U));
        }

        // 11.4.7 PULL: 100 | 1 | IfE[6] | Blk[5] | 00000
        constexpr std::uint16_t pull(bool ifEmpty,
                                     bool block) {
            return static_cast<std::uint16_t>(0x8080U | (ifEmpty ? 0x40U : 0U)
                                              | (block ? 0x20U : 0U));
        }

        // 11.4.8 MOV to RX (v1): 100 | 0 | 00 | 1 | IdxI[3] | 0 | index[1:0]
        constexpr std::uint16_t movToRx(bool     immediate,
                                        unsigned idx) {
            return static_cast<std::uint16_t>(0x8010U | (immediate ? 0x08U : 0U) | idx);
        }

        // 11.4.9 MOV from RX (v1): 100 | 1 | 00 | 1 | IdxI[3] | 0 | index[1:0]
        constexpr std::uint16_t movFromRx(bool     immediate,
                                          unsigned idx) {
            return static_cast<std::uint16_t>(0x8090U | (immediate ? 0x08U : 0U) | idx);
        }

        // 11.4.10 MOV: 101 | destination[7:5] | op[4:3] | source[2:0]
        constexpr std::uint16_t mov(unsigned dst,
                                    unsigned op,
                                    unsigned src) {
            return static_cast<std::uint16_t>(0xA000U | (dst << 5U) | (op << 3U) | src);
        }

        // 11.4.11 IRQ: 110 | 0 | Clr[6] | Wait[5] | IdxMode[4:3] | index[2:0]
        constexpr std::uint16_t irq(bool     clear,
                                    bool     wait,
                                    unsigned mode,
                                    unsigned idx) {
            return static_cast<std::uint16_t>(0xC000U | (clear ? 0x40U : 0U) | (wait ? 0x20U : 0U)
                                              | (mode << 3U) | idx);
        }

        // 11.4.12 SET: 111 | destination[7:5] | data[4:0]
        constexpr std::uint16_t set(unsigned dst,
                                    unsigned data) {
            return static_cast<std::uint16_t>(0xE000U | (dst << 5U) | data);
        }

        // nop is `mov y, y`, as pioasm writes it
        inline constexpr std::uint16_t nop = 0xA042;
    }   // namespace Enc

    // Whether a word is an instruction only PIO version 1 (RP2350) has: each of these is
    // reserved on the RP2040 (RP2040 datasheet 3.4.3.2 WAIT source 11, 3.4.6-7 PUSH/PULL bits 4:0
    // zero, 3.4.8.2 MOV destination 011, 3.4.10.2 IRQ index = rel bit 4 + three LSBs) and is, on
    // the RP2350 (11.4), WAIT JMPPIN, WAIT/IRQ PREV or NEXT, MOV to/from RX, MOV PINDIRS.
    // Decided by the words, not by `.pio_version`, which a header also carries when the
    // program only uses v0 (pioasm's -v 1 default does that to every program).
    constexpr bool isPioV1Only(std::uint16_t word) {
        unsigned const op = word >> 13U;
        if(op == 1U) {   // WAIT
            unsigned const src = (word >> 5U) & 3U;
            return src == 3U || (src == 2U && (word & 0x08U) != 0U);
        }
        if(op == 4U) { return (word & 0x1FU) != 0U; }        // PUSH/PULL: MOV to/from RX
        if(op == 5U) { return ((word >> 5U) & 7U) == 3U; }   // MOV PINDIRS
        if(op == 6U) { return (word & 0x08U) != 0U; }        // IRQ PREV/NEXT
        return false;
    }

    template<typename Words>
    constexpr bool usesPioV1(Words const& words) {
        for(auto const w : words) {
            if(isPioV1Only(static_cast<std::uint16_t>(w))) { return true; }
        }
        return false;
    }

    // Operands, spelled as pioasm spells them; the values are the encodings' field values
    // (RP2350 datasheet: JMP condition 11.4.2.2, IN source 11.4.4.2, OUT destination 11.4.5.2,
    // MOV 11.4.10.2, WAIT source 11.4.3.2, IRQ IdxMode 11.4.11.2, SET destination 11.4.12.2).
    // The forms only PIO version 1 (RP2350) has are marked v1.
    enum class Cond : std::uint8_t { always, notX, xDec, notY, yDec, xNeY, pin, notOsre };
    enum class In : std::uint8_t { pins = 0, x = 1, y = 2, null = 3, isr = 6, osr = 7 };
    enum class Out : std::uint8_t { pins, x, y, null, pindirs, pc, isr, exec };
    enum class MovDst : std::uint8_t { pins, x, y, pindirs /*v1*/, exec, pc, isr, osr };
    enum class MovSrc : std::uint8_t {
        pins   = 0,
        x      = 1,
        y      = 2,
        null   = 3,
        status = 5,
        isr    = 6,
        osr    = 7
    };
    enum class MovOp : std::uint8_t { none, invert, reverse };
    enum class Wait : std::uint8_t { gpio, pin, irq, jmppin /*v1*/ };
    enum class IrqMode : std::uint8_t { plain = 0, prev = 1 /*v1*/, rel = 2, next = 3 /*v1*/ };
    enum class Set : std::uint8_t { pins = 0, x = 1, y = 2, pindirs = 4 };
    // pioasm's `.fifo` values (pio_enums.h), as its headers carry them in FifoMode
    enum class Fifo : std::uint8_t { txrx, tx, rx, txget /*v1*/, txput /*v1*/, putget /*v1*/ };
    // pioasm's `.mov_status` types, as its headers carry them in MovStatusType (-1: none)
    enum class MovStatusKind : std::int8_t {
        none       = -1,
        txLessThan = 0,
        rxLessThan = 1,
        irqSet     = 2
    };

    // A name in a fixed array, so that Assembled stays structural (a template argument).
    struct SymbolName {
        char         text[32]{};
        std::uint8_t size{};

        constexpr std::string_view view() const { return {text, size}; }
    };

    struct Symbol {
        SymbolName name{};
        int        value{};
        bool       isLabel{};
        bool       isPublic{};
    };

    // What the assembler produces: a structural value, so it can be a template argument.
    struct Assembled {
        static constexpr std::size_t MaxSymbols = 32;

        std::array<std::uint16_t, 32> words{};
        std::uint8_t                  length{};
        std::uint8_t                  wrapTarget{};
        std::uint8_t                  wrap{};
        std::int8_t                   origin{-1};   // -1: relocatable
        std::uint8_t sidesetCount{};                // side-set PINS, the enable bit not included
        bool         sidesetOpt{};
        bool         sidesetPindirs{};
        std::uint8_t pioVersion{};
        // pioasm's other directives, with pioasm's defaults (-1: not given)
        std::uint8_t                   fifo{};
        std::int8_t                    movStatusType{-1};
        std::uint8_t                   movStatusN{};
        std::int8_t                    setCount{-1};
        std::int8_t                    inPinCount{-1};
        bool                           inRight{true};
        bool                           inAuto{};
        std::uint8_t                   inThreshold{32};
        std::int8_t                    outPinCount{-1};
        bool                           outRight{true};
        bool                           outAuto{};
        std::uint8_t                   outThreshold{32};
        std::uint16_t                  clockDivInt{1};
        std::uint8_t                   clockDivFrac{};
        std::uint8_t                   usedGpioRanges{};
        std::array<Symbol, MaxSymbols> symbols{};
        std::uint8_t                   symbolCount{};
        char                           error[192]{};   // empty = ok; the first error only

        constexpr bool ok() const { return error[0] == 0; }

        constexpr std::string_view message() const {
            std::size_t n = 0;
            while(n < sizeof(error) && error[n] != 0) { ++n; }
            return {error, n};
        }

        constexpr Symbol const* find(std::string_view name) const {
            for(std::size_t i = 0; i < symbolCount; ++i) {
                if(symbols[i].name.view() == name) { return &symbols[i]; }
            }
            return nullptr;
        }

        // The symbol's index, symbolCount when there is none. For a lookup in a program that is a template argument:
        // gcc's -fsanitize=null cannot constant-evaluate "a pointer into that object != nullptr" (find()'s result).
        constexpr std::size_t indexOf(std::string_view name) const {
            for(std::size_t i = 0; i < symbolCount; ++i) {
                if(symbols[i].name.view() == name) { return i; }
            }
            return symbolCount;
        }
    };

    namespace detail {
        // A tiny formatter for Asm's messages: string pieces and integers, nothing else (no <format> in the
        // constant evaluator).
        struct Message {
            char        text[sizeof(Assembled{}.error)]{};
            std::size_t size{};

            constexpr void append(std::string_view s) {
                for(char c : s) {
                    if(size + 1 < sizeof(text)) { text[size++] = c; }
                }
            }

            constexpr void append(char const* s) { append(std::string_view{s}); }

            template<typename I>
                requires std::is_integral_v<I>
            constexpr void append(I v) {
                long long x = static_cast<long long>(v);
                if(x < 0) {
                    append("-");
                    x = -x;
                }
                char digits[24]{};
                int  n = 0;
                do {
                    digits[n++] = static_cast<char>('0' + x % 10);
                    x /= 10;
                } while(x != 0);
                while(n > 0) {
                    char const c[2] = {digits[--n], 0};
                    append(std::string_view{c, 1});
                }
            }
        };

        template<typename... Parts>
        constexpr Message concat(Parts const&... parts) {
            Message m{};
            (m.append(parts), ...);
            return m;
        }
    }   // namespace detail

    class Asm {
    public:
        // Where a builder call was made, for messages: `<file>:<line>: ...`.
        struct Site {
            char const*   file{};
            std::uint32_t line{};
        };

        // One instruction as the builder keeps it until finish(): side-set and delay are only
        // placed once every instruction is known (the side-set layout comes from .side_set).
        struct Instr {
            std::uint16_t word{};
            std::int16_t  side_{-1};   // -1: no side-set value
            std::int16_t  delay_{};
            std::int8_t   labelRef{-1};   // a JMP to a label by name, resolved in finish()
            std::int16_t  target{};       // a JMP to an address
            bool          isJmp{};
            bool          isWord{};   // .word: taken as it is, no side-set or delay
            std::uint16_t line{};     // the text line, 0 for the builder
            Site          site{};     // the builder's call, for messages

            constexpr Instr& side(unsigned v) {
                side_ = static_cast<std::int16_t>(v > 0x7FFFU ? 0x7FFF : v);
                return *this;
            }

            constexpr Instr& delay(unsigned d) {
                delay_ = static_cast<std::int16_t>(d > 0x7FFFU ? 0x7FFF : d);
                return *this;
            }
        };

        // The text line the next call belongs to; the parser sets it. The builder leaves it 0,
        // and its messages name the C++ call instead (every call takes a std::source_location).
        std::uint16_t currentLine = 0;
        // set by the parser: a message without a line is about the whole text
        bool fromText = false;

        // The first error wins: a mistake does not drag a cascade of others behind it.
        template<typename... Parts>
        constexpr void fail(Parts const&... parts) {
            if(failed()) { return; }
            detail::Message m{};
            if(currentLine != 0) {
                m.append("line ");
                m.append(static_cast<long long>(currentLine));
                m.append(": ");
            } else if(!fromText && site_.file != nullptr) {
                // the builder: the call that added the instruction, or the directive's
                std::string_view file{site_.file};
                if(auto const slash = file.find_last_of('/'); slash != std::string_view::npos) {
                    file.remove_prefix(slash + 1);
                }
                m.append(file);
                m.append(":");
                m.append(static_cast<long long>(site_.line));
                m.append(": ");
            } else if(!fromText) {
                m.append("instruction ");
                m.append(static_cast<long long>(
                  instrIndex_ >= 0 ? static_cast<std::size_t>(instrIndex_) : count_));
                m.append(": ");
            }
            (m.append(parts), ...);
            std::copy_n(m.text, sizeof(result_.error), result_.error);
            result_.error[sizeof(result_.error) - 1] = 0;
        }

        // ---- directives ------------------------------------------------------------------
        constexpr void sideSet(int                        count,
                               bool                       opt     = false,
                               bool                       pindirs = false,
                               std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(!beforeInstructions(".side_set")) { return; }
            if(count < 0) { return fail("side-set count ", count, " is negative"); }
            if(count + (opt ? 1 : 0) > 5) {
                // 11.4.1: side-set and delay share the five bits 12:8, opt takes one of them
                return fail(".side_set ",
                            count,
                            opt ? " opt needs " : " needs ",
                            count + (opt ? 1 : 0),
                            " bits, there are 5");
            }
            haveSideset_           = true;
            result_.sidesetCount   = static_cast<std::uint8_t>(count);
            result_.sidesetOpt     = opt;
            result_.sidesetPindirs = pindirs;
            sidesetLine_           = currentLine;
        }

        constexpr void wrapTarget(std::source_location const loc
                                  = std::source_location::current()) {
            at(loc);
            if(haveWrapTarget_) { return fail(".wrap_target given twice"); }
            haveWrapTarget_    = true;
            result_.wrapTarget = static_cast<std::uint8_t>(count_);
            wrapTargetLine_    = currentLine;
            wrapTargetSite_    = site_;
        }

        constexpr void wrap(std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(haveWrap_) { return fail(".wrap given twice"); }
            if(count_ == 0) { return fail(".wrap before the first instruction"); }
            haveWrap_    = true;
            result_.wrap = static_cast<std::uint8_t>(count_ - 1);
        }

        constexpr void origin(int                        o,
                              std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(!beforeInstructions(".origin")) { return; }
            if(o < 0 || o > 31) {
                return fail(".origin ", o, ": instruction memory has 32 slots (0..31)");
            }
            result_.origin = static_cast<std::int8_t>(o);
            originLine_    = currentLine;
            originSite_    = site_;
        }

        constexpr void pioVersion(int                        v,
                                  std::source_location const loc
                                  = std::source_location::current()) {
            at(loc);
            if(!beforeInstructions(".pio_version")) { return; }
            if(v < 0 || v > 1) { return fail(".pio_version ", v, ": 0 (RP2040) or 1 (RP2350)"); }
            result_.pioVersion = static_cast<std::uint8_t>(v);
        }

        constexpr void fifo(Fifo                       f,
                            std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(!beforeInstructions(".fifo")) { return; }
            if(f >= Fifo::txget && !needsV1(".fifo txget/txput/putget")) { return; }
            result_.fifo = static_cast<std::uint8_t>(f);
        }

        // .mov_status: txfifo < n, rxfifo < n (n 0..31), irq [prev|next] set n (n 0..7)
        constexpr void movStatus(MovStatusKind              kind,
                                 int                        n,
                                 IrqMode                    irqMode = IrqMode::plain,
                                 std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(!beforeInstructions(".mov_status")) { return; }
            if(kind == MovStatusKind::irqSet) {
                if(n < 0 || n > 7) { return fail(".mov_status irq ", n, ": an IRQ flag is 0..7"); }
                if(irqMode == IrqMode::rel) { return fail(".mov_status irq: no 'rel'"); }
                // pioasm: prev = 1, next = 2, times 8 on top of the flag
                int const param = irqMode == IrqMode::prev ? 1 : irqMode == IrqMode::next ? 2 : 0;
                result_.movStatusN = static_cast<std::uint8_t>(param * 8 + n);
            } else {
                if(n < 0 || n > 31) { return fail(".mov_status: FIFO level ", n, " (0..31)"); }
                result_.movStatusN = static_cast<std::uint8_t>(n);
            }
            result_.movStatusType = static_cast<std::int8_t>(kind);
        }

        // .in count [left|right] [auto|manual] [threshold]
        constexpr void inConfig(int                        count,
                                bool                       right     = true,
                                bool                       autoPush  = false,
                                int                        threshold = 32,
                                std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(!beforeInstructions(".in")) { return; }
            if(count < 1 || count > 32) { return fail(".in pin count ", count, " (1..32)"); }
            if(result_.pioVersion == 0 && count != 32) {
                return fail(".in pin count ", count, ": must be 32 for PIO version 0");
            }
            if(threshold < 1 || threshold > 32) {
                return fail(".in threshold ", threshold, " (1..32)");
            }
            result_.inPinCount  = static_cast<std::int8_t>(count);
            result_.inRight     = right;
            result_.inAuto      = autoPush;
            result_.inThreshold = static_cast<std::uint8_t>(threshold);
            inLine_             = currentLine;
            inSite_             = site_;
        }

        // .out count [left|right] [auto|manual] [threshold]
        constexpr void outConfig(int                        count,
                                 bool                       right     = true,
                                 bool                       autoPull  = false,
                                 int                        threshold = 32,
                                 std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(!beforeInstructions(".out")) { return; }
            if(count < 0 || count > 32) { return fail(".out pin count ", count, " (0..32)"); }
            if(threshold < 1 || threshold > 32) {
                return fail(".out threshold ", threshold, " (1..32)");
            }
            result_.outPinCount  = static_cast<std::int8_t>(count);
            result_.outRight     = right;
            result_.outAuto      = autoPull;
            result_.outThreshold = static_cast<std::uint8_t>(threshold);
        }

        // .set count
        constexpr void setConfig(int                        count,
                                 std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(!beforeInstructions(".set")) { return; }
            if(count < 0 || count > 5) { return fail(".set pin count ", count, " (0..5)"); }
            result_.setCount = static_cast<std::int8_t>(count);
        }

        // .clock_div, as integer and 1/256ths
        constexpr void clockDiv(unsigned                   integer,
                                unsigned                   frac256,
                                std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(!beforeInstructions(".clock_div")) { return; }
            if(integer < 1 || integer > 65535) {
                return fail(".clock_div ", integer, ": 1..65535");
            }
            result_.clockDivInt  = static_cast<std::uint16_t>(integer);
            result_.clockDivFrac = static_cast<std::uint8_t>(frac256);
        }

        constexpr void label(std::string_view           name,
                             bool                       isPublic = false,
                             std::source_location const loc = std::source_location::current()) {
            at(loc);
            addSymbol(name, static_cast<int>(count_), true, isPublic);
        }

        constexpr void define(std::string_view           name,
                              int                        value,
                              bool                       isPublic = false,
                              std::source_location const loc = std::source_location::current()) {
            at(loc);
            addSymbol(name, value, false, isPublic);
        }

        // ---- instructions (pioasm names and defaults) --------------------------------------
        constexpr Instr& jmp(std::string_view           target,
                             std::source_location const loc = std::source_location::current()) {
            return jmp(Cond::always, target, loc);
        }

        constexpr Instr& jmp(Cond                       c,
                             std::string_view           target,
                             std::source_location const loc = std::source_location::current()) {
            at(loc);
            auto& i = add(Enc::jmp(static_cast<unsigned>(c), 0));
            i.isJmp = true;
            if(refCount_ == refs_.size()) {
                fail("more than ", static_cast<int>(refs_.size()), " jumps to labels");
                return i;
            }
            if(!setName(refs_[refCount_], target)) { return i; }
            i.labelRef = static_cast<std::int8_t>(refCount_++);
            return i;
        }

        constexpr Instr& jmp(Cond                       c,
                             unsigned                   addr,
                             std::source_location const loc = std::source_location::current()) {
            at(loc);
            auto& i  = add(Enc::jmp(static_cast<unsigned>(c), 0));
            i.isJmp  = true;
            i.target = static_cast<std::int16_t>(addr > 0x7FFFU ? 0x7FFF : addr);
            return i;
        }

        constexpr Instr& wait(int                        pol,
                              Wait                       src,
                              int                        idx,
                              IrqMode                    m   = IrqMode::plain,
                              std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(pol < 0 || pol > 1) { fail("wait polarity ", pol, " (0 or 1)"); }
            switch(src) {
            case Wait::gpio:
                // an absolute GPIO; the RP2350 reaches 32..47 only through the GPIO window
                // (GPIOBASE 16), and then the index is counted from 16 - write it that way
                if(idx < 0 || idx > 31) {
                    fail("wait gpio ",
                         idx,
                         ": 0..31 (for GPIO 32..47 use the GPIO window, base 16)");
                } else {
                    result_.usedGpioRanges
                      |= static_cast<std::uint8_t>(1U << (static_cast<unsigned>(idx) >> 4U));
                }
                break;
            case Wait::pin:
                if(idx < 0 || idx > 31) { fail("wait pin ", idx, " (0..31)"); }
                break;
            case Wait::irq:
                if(idx < 0 || idx > 7) { fail("wait irq ", idx, " (0..7)"); }
                if(m == IrqMode::prev || m == IrqMode::next) {
                    needsV1(m == IrqMode::prev ? "wait irq prev" : "wait irq next");
                }
                break;
            case Wait::jmppin:
                needsV1("wait jmppin");
                if(idx < 0 || idx > 3) { fail("wait jmppin + ", idx, ": offset 0..3"); }
                break;
            }
            if(src != Wait::irq && m != IrqMode::plain) {
                fail("wait: prev/next/rel belong to 'irq'");
            }
            auto const index = static_cast<unsigned>(idx) & 0x1FU;
            return add(Enc::wait(
              static_cast<unsigned>(pol) & 1U,
              static_cast<unsigned>(src),
              src == Wait::irq ? ((static_cast<unsigned>(m) << 3U) | (index & 7U)) : index));
        }

        constexpr Instr& in(In                         src,
                            int                        bits,
                            std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(bits < 1 || bits > 32) { fail("in bit count ", bits, " (1..32)"); }
            return add(Enc::in(static_cast<unsigned>(src), static_cast<unsigned>(bits)));
        }

        constexpr Instr& out(Out                        dst,
                             int                        bits,
                             std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(bits < 1 || bits > 32) { fail("out bit count ", bits, " (1..32)"); }
            return add(Enc::out(static_cast<unsigned>(dst), static_cast<unsigned>(bits)));
        }

        constexpr Instr& push(bool                       ifFull = false,
                              bool                       block  = true,
                              std::source_location const loc    = std::source_location::current()) {
            at(loc);
            if(result_.fifo != static_cast<std::uint8_t>(Fifo::txrx)
               && result_.fifo != static_cast<std::uint8_t>(Fifo::rx))
            {
                fail("push needs .fifo txrx or rx");
            }
            return add(Enc::push(ifFull, block));
        }

        constexpr Instr& pull(bool                       ifEmpty = false,
                              bool                       block   = true,
                              std::source_location const loc = std::source_location::current()) {
            at(loc);
            return add(Enc::pull(ifEmpty, block));
        }

        constexpr Instr& mov(MovDst                     d,
                             MovSrc                     s,
                             MovOp                      op  = MovOp::none,
                             std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(d == MovDst::pindirs) { needsV1("mov pindirs"); }
            if(op > MovOp::reverse) { fail("mov: operation 3 is reserved"); }
            return add(Enc::mov(static_cast<unsigned>(d),
                                static_cast<unsigned>(op),
                                static_cast<unsigned>(s)));
        }

        // mov rxfifo[idx], isr
        constexpr Instr& movToRx(int                        idx,
                                 std::source_location const loc = std::source_location::current()) {
            at(loc);
            return movRx(true, true, idx);
        }

        // mov rxfifo[y], isr
        constexpr Instr& movToRxY(std::source_location const loc
                                  = std::source_location::current()) {
            at(loc);
            return movRx(true, false, 0);
        }

        // mov osr, rxfifo[idx]
        constexpr Instr& movFromRx(int                        idx,
                                   std::source_location const loc
                                   = std::source_location::current()) {
            at(loc);
            return movRx(false, true, idx);
        }

        // mov osr, rxfifo[y]
        constexpr Instr& movFromRxY(std::source_location const loc
                                    = std::source_location::current()) {
            at(loc);
            return movRx(false, false, 0);
        }

        constexpr Instr& irq(int                        idx,
                             IrqMode                    m   = IrqMode::plain,
                             std::source_location const loc = std::source_location::current()) {
            at(loc);
            return irqAny(false, false, idx, m);
        }

        constexpr Instr& irqWait(int                        idx,
                                 IrqMode                    m   = IrqMode::plain,
                                 std::source_location const loc = std::source_location::current()) {
            at(loc);
            return irqAny(false, true, idx, m);
        }

        constexpr Instr& irqClear(int                        idx,
                                  IrqMode                    m = IrqMode::plain,
                                  std::source_location const loc
                                  = std::source_location::current()) {
            at(loc);
            return irqAny(true, false, idx, m);
        }

        constexpr Instr& set(Set                        d,
                             int                        v,
                             std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(v < 0 || v > 31) { fail("set value ", v, " (0..31)"); }
            return add(Enc::set(static_cast<unsigned>(d), static_cast<unsigned>(v) & 0x1FU));
        }

        constexpr Instr& nop(std::source_location const loc = std::source_location::current()) {
            at(loc);
            return add(Enc::nop);
        }

        constexpr Instr& word(int                        raw,
                              std::source_location const loc = std::source_location::current()) {
            at(loc);
            if(raw < 0 || raw > 0xFFFF) { fail(".word ", raw, ": a 16-bit value"); }
            auto& i  = add(static_cast<std::uint16_t>(raw));
            i.isWord = true;
            return i;
        }

        // ---- the end ----------------------------------------------------------------------
        constexpr Assembled finish(std::source_location const loc
                                   = std::source_location::current()) {
            at(loc);
            if(!failed() && count_ == 0) { fail("the program has no instructions"); }
            if(!haveWrap_) {
                result_.wrap = static_cast<std::uint8_t>(count_ == 0 ? 0 : count_ - 1);
            }
            if(haveWrapTarget_ && result_.wrapTarget >= count_ && !failed()) {
                currentLine = wrapTargetLine_;
                site_       = wrapTargetSite_;
                fail(".wrap_target after the last instruction");
            }
            if(result_.origin >= 0 && static_cast<std::size_t>(result_.origin) + count_ > 32
               && !failed())
            {
                currentLine = originLine_;
                site_       = originSite_;
                fail("the program is ",
                     static_cast<int>(count_),
                     " instructions at .origin ",
                     static_cast<int>(result_.origin),
                     ": instruction memory has 32 slots");
            }
            if(result_.inAuto && result_.fifo > static_cast<std::uint8_t>(Fifo::rx) && !failed()) {
                currentLine = inLine_;
                site_       = inSite_;
                fail("autopush does not go with this .fifo configuration");
            }

            // 11.4.1: bits 12:8 hold side-set (MSBs, SIDESET_COUNT of them, the enable bit
            // first when optional) and the delay (the rest)
            unsigned const ssBits
              = haveSideset_ ? result_.sidesetCount + (result_.sidesetOpt ? 1U : 0U) : 0U;
            int const delayMax = static_cast<int>((1U << (5U - ssBits)) - 1U);
            int const sideMax  = static_cast<int>((1U << result_.sidesetCount) - 1U);
            for(std::size_t n = 0; n < count_ && !failed(); ++n) {
                auto& i       = code_[n];
                currentLine   = i.line;
                site_         = i.site;
                instrIndex_   = static_cast<int>(n);
                unsigned word = i.word;
                if(i.isJmp) {
                    int target = i.target;
                    if(i.labelRef >= 0) {
                        auto const* s
                          = result_.find(refs_[static_cast<std::size_t>(i.labelRef)].view());
                        if(s == nullptr || !s->isLabel) {
                            return failed_(
                              detail::concat("jmp to unknown label '",
                                             refs_[static_cast<std::size_t>(i.labelRef)].view(),
                                             "'"));
                        }
                        target = s->value;
                    }
                    if(target < 0 || target >= static_cast<int>(count_)) {
                        return failed_(detail::concat("jmp target ",
                                                      target,
                                                      " is beyond the end of the program (",
                                                      static_cast<int>(count_),
                                                      " instructions)"));
                    }
                    word = (word & ~0x1FU) | static_cast<unsigned>(target);
                }
                if(!i.isWord) {
                    if(i.delay_ > delayMax) {
                        if(haveSideset_) {
                            return failed_(detail::concat("delay ",
                                                          i.delay_,
                                                          " does not fit: .side_set ",
                                                          static_cast<int>(result_.sidesetCount),
                                                          result_.sidesetOpt ? " opt" : "",
                                                          " leaves ",
                                                          static_cast<int>(5 - ssBits),
                                                          " delay bits (max ",
                                                          delayMax,
                                                          ")"));
                        }
                        return failed_(
                          detail::concat("delay ", i.delay_, " does not fit (max 31)"));
                    }
                    unsigned field = static_cast<unsigned>(i.delay_);
                    if(i.side_ >= 0) {
                        if(!haveSideset_) {
                            return failed_(detail::concat("side ", i.side_, " without .side_set"));
                        }
                        if(i.side_ > sideMax) {
                            return failed_(detail::concat("side ",
                                                          i.side_,
                                                          " does not fit .side_set ",
                                                          static_cast<int>(result_.sidesetCount),
                                                          " (max ",
                                                          sideMax,
                                                          ")"));
                        }
                        field |= static_cast<unsigned>(i.side_) << (5U - ssBits);
                        if(result_.sidesetOpt) { field |= 0x10U; }
                    } else if(haveSideset_ && !result_.sidesetOpt) {
                        return failed_(detail::concat("side-set is not optional here (.side_set ",
                                                      static_cast<int>(result_.sidesetCount),
                                                      "): 'side' missing"));
                    }
                    word |= field << 8U;
                }
                result_.words[n] = static_cast<std::uint16_t>(word);
            }
            result_.length = static_cast<std::uint8_t>(count_);
            if(failed()) { result_.length = 0; }
            return result_;
        }

        constexpr bool failed() const { return result_.error[0] != 0; }

        constexpr std::uint8_t pioVersionNow() const { return result_.pioVersion; }

        constexpr std::size_t instructionCount() const { return count_; }

    private:
        // Remembers the builder's call site for the messages of this call; the parser's calls
        // leave it alone (their place is the text line).
        constexpr void at(std::source_location const& loc) {
            if(!fromText) { site_ = Site{loc.file_name(), loc.line()}; }
        }

        constexpr Assembled failed_(detail::Message const& m) {
            fail(std::string_view{m.text, m.size});
            result_.length = 0;
            return result_;
        }

        constexpr bool beforeInstructions(std::string_view what) {
            if(count_ != 0) {
                fail(what, " after an instruction");
                return false;
            }
            return true;
        }

        constexpr bool needsV1(std::string_view what) {
            if(result_.pioVersion >= 1) { return true; }
            fail("'", what, "' needs .pio_version 1 (RP2350)");
            return false;
        }

        constexpr bool setName(SymbolName&      n,
                               std::string_view name) {
            if(name.empty() || name.size() >= sizeof(n.text)) {
                fail("name '", name, "': 1..", static_cast<int>(sizeof(n.text) - 1), " characters");
                return false;
            }
            std::copy_n(name.begin(), name.size(), n.text);
            n.size = static_cast<std::uint8_t>(name.size());
            return true;
        }

        constexpr void addSymbol(std::string_view name,
                                 int              value,
                                 bool             isLabel,
                                 bool             isPublic) {
            if(auto const* s = result_.find(name)) {
                return fail("'",
                            name,
                            "' is already defined as a ",
                            s->isLabel ? "label" : "value");
            }
            if(result_.symbolCount == Assembled::MaxSymbols) {
                return fail("more than ",
                            static_cast<int>(Assembled::MaxSymbols),
                            " labels and defines");
            }
            auto& s = result_.symbols[result_.symbolCount];
            if(!setName(s.name, name)) { return; }
            s.value    = value;
            s.isLabel  = isLabel;
            s.isPublic = isPublic;
            ++result_.symbolCount;
        }

        constexpr Instr& add(std::uint16_t word) {
            if(count_ == code_.size()) {
                fail("more than 32 instructions: instruction memory has 32");
                return spare_;
            }
            auto& i = code_[count_++];
            i       = Instr{};
            i.word  = word;
            i.line  = currentLine;
            i.site  = site_;
            return i;
        }

        constexpr Instr& movRx(bool toRx,
                               bool immediate,
                               int  idx) {
            needsV1(toRx ? "mov rxfifo[], isr" : "mov osr, rxfifo[]");
            // 11.4.8.3 / 11.4.9.3: the index is 0..3 (pioasm takes 0..7, the datasheet does not)
            if(immediate && (idx < 0 || idx > 3)) { fail("rxfifo index ", idx, " (0..3)"); }
            auto const f = static_cast<Fifo>(result_.fifo);
            if(toRx && f != Fifo::txput && f != Fifo::putget) {
                fail("mov rxfifo[], isr needs .fifo txput or putget");
            }
            if(!toRx && f != Fifo::txget && f != Fifo::putget) {
                fail("mov osr, rxfifo[] needs .fifo txget or putget");
            }
            auto const index = static_cast<unsigned>(idx) & 3U;
            return add(toRx ? Enc::movToRx(immediate, index) : Enc::movFromRx(immediate, index));
        }

        constexpr Instr& irqAny(bool    clear,
                                bool    wait,
                                int     idx,
                                IrqMode m) {
            if(idx < 0 || idx > 7) { fail("irq ", idx, " (0..7)"); }
            if(m == IrqMode::prev || m == IrqMode::next) {
                needsV1(m == IrqMode::prev ? "irq prev" : "irq next");
            }
            return add(
              Enc::irq(clear, wait, static_cast<unsigned>(m), static_cast<unsigned>(idx) & 7U));
        }

        std::array<Instr, 32>      code_{};
        std::array<SymbolName, 32> refs_{};
        std::size_t                refCount_{};
        std::size_t                count_{};
        Instr                      spare_{};
        Assembled                  result_{};
        bool                       haveSideset_{};
        bool                       haveWrapTarget_{};
        bool                       haveWrap_{};
        std::uint16_t              sidesetLine_{};
        std::uint16_t              wrapTargetLine_{};
        std::uint16_t              inLine_{};
        std::uint16_t              originLine_{};
        int                        instrIndex_{-1};
        Site                       site_{};
        Site                       wrapTargetSite_{};
        Site                       originSite_{};
        Site                       inSite_{};
    };

    // The builder's front door: `f` gets an Asm to call, the result is the checked program.
    template<typename F>
    consteval Assembled assemble(F                          f,
                                 std::source_location const loc = std::source_location::current()) {
        Asm a{};
        f(a);
        return a.finish(loc);   // a message about the whole program names the assemble() call
    }

    // Called only when a program asks for a name it does not have: not constexpr, so the
    // compiler stops right there and shows the name.
    inline std::uint8_t pio_program_has_no_label_of_that_name(std::string_view) { return 0; }

    inline int pio_program_has_no_define_of_that_name(std::string_view) { return 0; }

    // The Program concept as a type: what a pioasm `-o kvasir` header provides, from a value.
    template<Assembled A>
    struct Program {
        // As a named constant, so that clang's diagnostic reads "requirement 'Ok': line 6: ..."
        // instead of printing the whole assembled value first
        static constexpr bool Ok = A.ok();
        static_assert(Ok,
                      A.message());

        static constexpr std::array<std::uint16_t, A.length> Instructions = [] {
            std::array<std::uint16_t, A.length> r{};
            std::copy_n(A.words.begin(), A.length, r.begin());
            return r;
        }();
        static constexpr int  WrapTarget      = A.wrapTarget;
        static constexpr int  Wrap            = A.wrap;
        static constexpr int  PioVersion      = A.pioVersion;
        static constexpr int  Origin          = A.origin;
        static constexpr int  SidesetCount    = A.sidesetCount;
        static constexpr bool SidesetOptional = A.sidesetOpt;
        static constexpr bool SidesetPindirs  = A.sidesetPindirs;
        // pioasm's other fields, with its names
        static constexpr int  ClockDivInt    = A.clockDivInt;
        static constexpr int  ClockDivFrac   = A.clockDivFrac;
        static constexpr int  FifoMode       = A.fifo;
        static constexpr int  UsedGpioRanges = A.usedGpioRanges;
        static constexpr int  MovStatusType  = A.movStatusType;
        static constexpr int  MovStatusN     = A.movStatusN;
        static constexpr int  SetCount       = A.setCount;
        static constexpr int  InPinCount     = A.inPinCount;
        static constexpr bool InRight        = A.inRight;
        static constexpr bool InAutoP        = A.inAuto;
        static constexpr int  InThreshold    = A.inThreshold;
        static constexpr int  OutPinCount    = A.outPinCount;
        static constexpr bool OutRight       = A.outRight;
        static constexpr bool OutAutoP       = A.outAuto;
        static constexpr int  OutThreshold   = A.outThreshold;

        static constexpr Assembled const& assembled() { return A; }

        // A label's offset in the program, as pioasm's `offset_<label>`; any label, public or not.
        static consteval std::uint8_t offset(std::string_view label) {
            auto const i = A.indexOf(label);
            if(i == A.symbolCount || !A.symbols[i].isLabel) {
                return pio_program_has_no_label_of_that_name(label);
            }
            return static_cast<std::uint8_t>(A.symbols[i].value);
        }

        // A `.define`'s value (or one given from C++ to parse()).
        static consteval int define(std::string_view name) {
            auto const i = A.indexOf(name);
            if(i == A.symbolCount || A.symbols[i].isLabel) {
                return pio_program_has_no_define_of_that_name(name);
            }
            return A.symbols[i].value;
        }
    };
}}   // namespace Kvasir::Pio
