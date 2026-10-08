#pragma once
// pioasm's .pio syntax, read by the compiler: parse() drives the Asm builder (Asm.hpp) from the
// text and returns the same Assembled value, so `Program<parse(...)>` is a program type like
// any other.
//
//   template<unsigned T1, unsigned T2, unsigned T3>
//   struct Ws2812Program : Kvasir::Pio::Program<Kvasir::Pio::parse(R"pio(
//   .program ws2812
//   .side_set 1
//   bitloop:
//       out x, 1       side 0 [T3 - 1]
//       jmp !x do_zero side 1 [T1 - 1]
//       jmp  bitloop   side 1 [T2 - 1]
//   do_zero:
//       nop            side 0 [T2 - 1]
//   )pio", {{"T1", T1}, {"T2", T2}, {"T3", T3}})> {};
//
// A .pio file goes in as it is, by #embed (overloads further down):
//
//   struct Blink : Kvasir::Pio::Program<Kvasir::Pio::parse({
//   #embed "blink.pio"
//   })> {};
//
// The names given from C++ are values like `.define public` ones (Program::define() returns
// them); a name given from C++ and also `.define`d in the text is an error, so a stale
// `.define` cannot win silently. A text with several programs names the one to assemble in the
// third argument; `% c-sdk { ... %}` blocks, `.lang_opt` and comments (`;`, `//`, `/* */`) are
// skipped. Keywords are case-insensitive, names are not (as in pioasm).
//
// The grammar is pioasm's (../pioasm/src/parser.yy, lexer.ll), checked against it by the host
// tests over every .pio file at hand. Where pioasm is looser than the RP2350 datasheet the
// datasheet wins: `in status` (source 101 is reserved, 11.4.4.2) and an rxfifo index above 3
// (11.4.8.3) are errors here.
//
// Two passes over the text, each linear: the first collects labels and `.define`s (a jump or a
// define may name a label further down), the second assembles. The constant evaluator reads a
// string one element at a time, so nothing here searches the text inside a loop.

#include "Asm.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <string_view>
#include <type_traits>

namespace Kvasir { namespace Pio {

    // A value given to parse() from C++.
    struct Define {
        std::string_view name;
        int              value;
    };

    namespace detail { namespace pioparse {
        enum class Tok : std::uint8_t {
            End,
            Newline,
            Int,
            Float,   // .clock_div only; num holds the value in 1/256ths
            Ident,
            Directive,
            Comma,
            Colon,
            LParen,
            RParen,
            LBracket,
            RBracket,
            Plus,
            Minus,
            Star,
            Slash,
            Or,
            And,
            Xor,
            Shl,
            Shr,
            Dec,
            NotEq,
            Not,
            Reverse,
            Less,
            Assign,
            Bad
        };

        struct Token {
            Tok         kind{};
            std::size_t begin{};
            std::size_t end{};
            long long   num{};
            unsigned    line{};
        };

        constexpr char lower(char c) {
            return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c;
        }

        constexpr bool isDigit(char c) { return c >= '0' && c <= '9'; }

        constexpr bool isIdStart(char c) {
            return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
        }

        constexpr bool isIdChar(char c) { return isIdStart(c) || isDigit(c); }

        constexpr bool isBlank(char c) { return c == ' ' || c == '\t' || c == '\r'; }

        // Case-insensitive: `word` is lower case.
        constexpr bool sameWord(std::string_view text,
                                std::string_view word) {
            if(text.size() != word.size()) { return false; }
            for(std::size_t i = 0; i < text.size(); ++i) {
                if(lower(text[i]) != word[i]) { return false; }
            }
            return true;
        }

        // lexer.ll's keywords: a name spelled like one of them is that keyword, never a name.
        inline constexpr std::string_view Keywords[]
          = {"jmp",    "wait",    "in",     "out",      "push",   "pull",    "mov",     "irq",
             "set",    "nop",     "public", "optional", "opt",    "side",    "sideset", "side_set",
             "pin",    "gpio",    "osre",   "pins",     "null",   "pindirs", "block",   "noblock",
             "iffull", "ifempty", "rel",    "clear",    "nowait", "jmppin",  "next",    "prev",
             "txrx",   "tx",      "rx",     "txput",    "txget",  "putget",  "one",     "zero",
             "rp2040", "rp2350",  "rxfifo", "txfifo",   "left",   "right",   "auto",    "manual",
             "x",      "y",       "pc",     "exec",     "isr",    "osr",     "status"};

        inline constexpr std::string_view Instructions[]
          = {"jmp", "wait", "in", "out", "push", "pull", "mov", "irq", "set", "nop"};

        constexpr bool isKeyword(std::string_view name) {
            for(auto k : Keywords) {
                if(sameWord(name, k)) { return true; }
            }
            return false;
        }

        class Lexer {
        public:
            constexpr Lexer(std::string_view text,
                            std::size_t      pos  = 0,
                            unsigned         line = 1)
              : text_{text}
              , pos_{pos}
              , line_{line} {}

            constexpr std::string_view text(Token const& t) const {
                return text_.substr(t.begin, t.end - t.begin);
            }

            constexpr std::string_view problem() const { return problem_; }

            constexpr Token peek() const {
                auto copy = *this;
                return copy.next();
            }

            constexpr Token next() {
                while(true) {
                    while(pos_ < text_.size() && isBlank(text_[pos_])) { ++pos_; }
                    if(pos_ >= text_.size()) { return make(Tok::End, pos_); }
                    char const c = text_[pos_];
                    if(c == ';' || (c == '/' && at(1) == '/')) {
                        while(pos_ < text_.size() && text_[pos_] != '\n') { ++pos_; }
                        continue;
                    }
                    if(c == '/' && at(1) == '*') {
                        // no line break is a token inside a block comment (lexer.ll's c_comment)
                        pos_ += 2;
                        while(pos_ < text_.size() && !(text_[pos_] == '*' && at(1) == '/')) {
                            if(text_[pos_] == '\n') { ++line_; }
                            ++pos_;
                        }
                        if(pos_ >= text_.size()) { return bad("a /* comment without its */"); }
                        pos_ += 2;
                        continue;
                    }
                    if(c == '\n') {
                        auto t = make(Tok::Newline, pos_);
                        ++pos_;
                        t.end = pos_;
                        ++line_;
                        return t;
                    }
                    if(c == '%') {
                        if(!skipCodeBlock()) { return bad("a % { block without its %}"); }
                        continue;
                    }
                    return token(c);
                }
            }

        private:
            constexpr char at(std::size_t ahead) const {
                return pos_ + ahead < text_.size() ? text_[pos_ + ahead] : '\0';
            }

            constexpr Token make(Tok         k,
                                 std::size_t begin) const {
                return Token{k, begin, pos_, 0, line_};
            }

            constexpr Token bad(std::string_view why) {
                problem_ = why;
                return make(Tok::Bad, pos_);
            }

            // `% <lang> {` up to a line that holds only `%}`: code for other output formats.
            constexpr bool skipCodeBlock() {
                while(pos_ < text_.size() && text_[pos_] != '\n') { ++pos_; }
                while(pos_ < text_.size()) {
                    ++pos_;   // the '\n'
                    ++line_;
                    while(pos_ < text_.size() && isBlank(text_[pos_])) { ++pos_; }
                    bool const end = at(0) == '%' && at(1) == '}';
                    if(end) {
                        pos_ += 2;
                        while(pos_ < text_.size() && isBlank(text_[pos_])) { ++pos_; }
                        if(pos_ >= text_.size() || text_[pos_] == '\n') { return true; }
                    }
                    while(pos_ < text_.size() && text_[pos_] != '\n') { ++pos_; }
                }
                return false;
            }

            constexpr Token number() {
                auto const begin  = pos_;
                long long  v      = 0;
                auto const tooBig = [&] { return v > 0x7FFFFFFF; };
                if(text_[pos_] == '0' && lower(at(1)) == 'x'
                   && std::string_view{"0123456789abcdef"}.find(lower(at(2)))
                        != std::string_view::npos)
                {
                    pos_ += 2;
                    while(pos_ < text_.size()) {
                        char const d = lower(text_[pos_]);
                        int const  n = isDigit(d)             ? d - '0'
                                     : (d >= 'a' && d <= 'f') ? d - 'a' + 10
                                                              : -1;
                        if(n < 0) { break; }
                        v = v * 16 + n;
                        if(tooBig()) { return bad("a number above 2^31 - 1"); }
                        ++pos_;
                    }
                    auto t = make(Tok::Int, begin);
                    t.num  = v;
                    return t;
                }
                if(text_[pos_] == '0' && lower(at(1)) == 'b' && (at(2) == '0' || at(2) == '1'))
                {
                    pos_ += 2;
                    while(pos_ < text_.size() && (text_[pos_] == '0' || text_[pos_] == '1')) {
                        v = v * 2 + (text_[pos_] - '0');
                        if(tooBig()) { return bad("a number above 2^31 - 1"); }
                        ++pos_;
                    }
                    auto t = make(Tok::Int, begin);
                    t.num  = v;
                    return t;
                }
                while(pos_ < text_.size() && isDigit(text_[pos_])) {
                    v = v * 10 + (text_[pos_] - '0');
                    if(tooBig()) { return bad("a number above 2^31 - 1"); }
                    ++pos_;
                }
                if(at(0) == '.' && isDigit(at(1))) {
                    ++pos_;
                    long long frac = 0;
                    long long div  = 1;
                    while(pos_ < text_.size() && isDigit(text_[pos_])) {
                        if(div < 1'000'000'000) {
                            frac = frac * 10 + (text_[pos_] - '0');
                            div *= 10;
                        }
                        ++pos_;
                    }
                    auto t = make(Tok::Float, begin);
                    t.num  = v * 256 + frac * 256 / div;
                    return t;
                }
                auto t = make(Tok::Int, begin);
                t.num  = v;
                return t;
            }

            constexpr Token token(char c) {
                auto const begin = pos_;
                auto const one   = [&](Tok k) {
                    ++pos_;
                    return make(k, begin);
                };
                auto const two = [&](Tok k) {
                    pos_ += 2;
                    return make(k, begin);
                };
                if(isDigit(c)) { return number(); }
                if(isIdStart(c)) {
                    while(pos_ < text_.size() && isIdChar(text_[pos_])) { ++pos_; }
                    return make(Tok::Ident, begin);
                }
                if(c == '.') {
                    if(isIdStart(at(1))) {
                        ++pos_;
                        while(pos_ < text_.size() && isIdChar(text_[pos_])) { ++pos_; }
                        return make(Tok::Directive, begin);
                    }
                    if(isDigit(at(1))) { return number(); }
                    return bad("a '.' that starts no directive");
                }
                switch(c) {
                case ',': return one(Tok::Comma);
                case ':': return at(1) == ':' ? two(Tok::Reverse) : one(Tok::Colon);
                case '(': return one(Tok::LParen);
                case ')': return one(Tok::RParen);
                case '[': return one(Tok::LBracket);
                case ']': return one(Tok::RBracket);
                case '+': return one(Tok::Plus);
                case '-': return at(1) == '-' ? two(Tok::Dec) : one(Tok::Minus);
                case '*': return one(Tok::Star);
                case '/': return one(Tok::Slash);
                case '|': return one(Tok::Or);
                case '&': return one(Tok::And);
                case '^': return one(Tok::Xor);
                case '~': return one(Tok::Not);
                case '=': return one(Tok::Assign);
                case '!': return at(1) == '=' ? two(Tok::NotEq) : one(Tok::Not);
                case '<': return at(1) == '<' ? two(Tok::Shl) : one(Tok::Less);
                case '>':
                    if(at(1) == '>') { return two(Tok::Shr); }
                    break;
                default: break;
                }
                // lexer.ll takes U+2212 twice ("−−", what a word processor makes of "--") as --
                if(text_.substr(pos_, 6) == "\xE2\x88\x92\xE2\x88\x92") {
                    pos_ += 6;
                    return make(Tok::Dec, begin);
                }
                return bad("an invalid character");
            }

            std::string_view text_;
            std::size_t      pos_;
            unsigned         line_;
            std::string_view problem_{};
        };

        class Parser {
        public:
            constexpr Parser(std::string_view              text,
                             std::initializer_list<Define> defines,
                             std::string_view              program)
              : text_{text}
              , program_{program} {
                for(auto const& d : defines) {
                    if(defineCount_ == defines_.size()) {
                        a_.fail("more than ",
                                static_cast<int>(defines_.size()),
                                " values from C++");
                        break;
                    }
                    defines_[defineCount_++] = d;
                }
            }

            constexpr Assembled run() {
                a_.fromText = true;
                for(std::size_t i = 0; i < defineCount_ && !a_.failed(); ++i) {
                    if(isKeyword(defines_[i].name)) {
                        a_.fail("'", defines_[i].name, "' (given from C++) is a keyword");
                    }
                    a_.define(defines_[i].name, defines_[i].value, true);
                }
                if(!a_.failed()) { collect(); }
                if(!a_.failed()) { assemble(); }
                a_.currentLine = 0;
                return a_.finish();
            }

        private:
            // ---- errors -------------------------------------------------------------------
            template<typename... Parts>
            constexpr bool failAt(unsigned line,
                                  Parts const&... parts) {
                if(!a_.failed()) {
                    a_.currentLine = static_cast<std::uint16_t>(line);
                    a_.fail(parts...);
                }
                return false;
            }

            constexpr bool unexpected(Lexer const&     lx,
                                      Token const&     t,
                                      std::string_view wanted) {
                if(t.kind == Tok::Bad) { return failAt(t.line, lx.problem()); }
                if(t.kind == Tok::Newline || t.kind == Tok::End) {
                    return failAt(t.line, "expected ", wanted, " before the end of the line");
                }
                return failAt(t.line, "expected ", wanted, ", found '", lx.text(t), "'");
            }

            constexpr bool expectEnd(Lexer& lx) {
                auto const t = lx.next();
                if(t.kind == Tok::Newline || t.kind == Tok::End) { return true; }
                return unexpected(lx, t, "the end of the line");
            }

            constexpr void skipLine(Lexer& lx) {
                while(true) {
                    auto const t = lx.next();
                    if(t.kind == Tok::Newline || t.kind == Tok::End) { return; }
                    if(t.kind == Tok::Bad) {
                        failAt(t.line, lx.problem());
                        return;
                    }
                }
            }

            // ---- symbols ------------------------------------------------------------------
            struct Sym {
                std::string_view name{};
                std::size_t      exprPos{};   // .define: where its expression starts
                unsigned         line{};
                int              value{};   // label: its offset; define: once resolved
                bool             isLabel{};
                bool             isPublic{};
                bool             resolved{};
                bool             resolving{};
            };

            constexpr bool addSym(Sym const& s) {
                if(isKeyword(s.name)) { return failAt(s.line, "'", s.name, "' is a keyword"); }
                for(std::size_t i = 0; i < defineCount_; ++i) {
                    if(defines_[i].name == s.name) {
                        return failAt(s.line,
                                      "'",
                                      s.name,
                                      "' is given from C++ and ",
                                      s.isLabel ? "is a label here" : "defined here");
                    }
                }
                for(std::size_t i = 0; i < symCount_; ++i) {
                    if(syms_[i].name == s.name) {
                        return failAt(s.line,
                                      "'",
                                      s.name,
                                      "' is already defined, at line ",
                                      static_cast<int>(syms_[i].line));
                    }
                }
                if(symCount_ == syms_.size()) {
                    return failAt(s.line,
                                  "more than ",
                                  static_cast<int>(syms_.size()),
                                  " labels and defines");
                }
                syms_[symCount_++] = s;
                return true;
            }

            constexpr bool resolve(std::string_view name,
                                   unsigned         line,
                                   int&             out) {
                if(sameWord(name, "one") || sameWord(name, "zero")) {
                    out = sameWord(name, "one") ? 1 : 0;
                    return true;
                }
                if(isKeyword(name)) { return failAt(line, "expected a value, found '", name, "'"); }
                for(std::size_t i = 0; i < defineCount_; ++i) {
                    if(defines_[i].name == name) {
                        out = defines_[i].value;
                        return true;
                    }
                }
                for(std::size_t i = 0; i < symCount_; ++i) {
                    auto& s = syms_[i];
                    if(s.name != name) { continue; }
                    if(s.isLabel || s.resolved) {
                        out = s.value;
                        return true;
                    }
                    if(s.resolving) {
                        return failAt(line,
                                      "circular dependency in the definition of '",
                                      name,
                                      "'");
                    }
                    s.resolving = true;
                    Lexer lx{text_, s.exprPos, s.line};
                    int   v = 0;
                    if(!expression(lx, 0, v) || !expectEnd(lx)) { return false; }
                    s.resolving = false;
                    s.resolved  = true;
                    s.value     = v;
                    out         = v;
                    return true;
                }
                return failAt(line, "undefined symbol '", name, "'");
            }

            // ---- expressions (parser.yy: value, expression, and its %left precedence) -----
            static constexpr int precedence(Tok k) {
                switch(k) {
                case Tok::Shl:
                case Tok::Shr:   return 1;
                case Tok::Plus:
                case Tok::Minus: return 2;
                case Tok::Star:
                case Tok::Slash: return 3;
                case Tok::And:
                case Tok::Or:
                case Tok::Xor:   return 4;
                default:         return -1;
                }
            }

            // value: an integer, a name, or a parenthesised expression
            constexpr bool value(Lexer& lx,
                                 int&   out) {
                auto const t = lx.next();
                if(t.kind == Tok::Int) {
                    out = static_cast<int>(t.num);
                    return true;
                }
                if(t.kind == Tok::Ident) { return resolve(lx.text(t), t.line, out); }
                if(t.kind == Tok::LParen) {
                    if(!expression(lx, 0, out)) { return false; }
                    auto const r = lx.next();
                    if(r.kind != Tok::RParen) { return unexpected(lx, r, "')'"); }
                    return true;
                }
                return unexpected(lx, t, "a value");
            }

            constexpr bool expression(Lexer& lx,
                                      int    minPrecedence,
                                      int&   out) {
                int  lhs = 0;
                auto t   = lx.peek();
                if(t.kind == Tok::Minus) {
                    // `- expression` binds like binary minus: tighter operators go into it
                    lx.next();
                    if(!expression(lx, 3, lhs)) { return false; }
                    lhs = static_cast<int>(0U - static_cast<unsigned>(lhs));
                } else if(t.kind == Tok::Reverse) {
                    // `:: expression` binds loosest of all
                    lx.next();
                    if(!expression(lx, 1, lhs)) { return false; }
                    unsigned v = static_cast<unsigned>(lhs);
                    unsigned r = 0;
                    for(int i = 0; i < 32; ++i) {
                        r = (r << 1U) | (v & 1U);
                        v >>= 1U;
                    }
                    lhs = static_cast<int>(r);
                } else if(!value(lx, lhs)) {
                    return false;
                }
                while(true) {
                    auto const op = lx.peek();
                    int const  p  = precedence(op.kind);
                    if(p < 0 || p < minPrecedence) { break; }
                    lx.next();
                    int rhs = 0;
                    if(!expression(lx, p + 1, rhs)) { return false; }
                    auto const l = static_cast<unsigned>(lhs);
                    auto const r = static_cast<unsigned>(rhs);
                    switch(op.kind) {
                    case Tok::Plus:  lhs = static_cast<int>(l + r); break;
                    case Tok::Minus: lhs = static_cast<int>(l - r); break;
                    case Tok::Star:  lhs = static_cast<int>(l * r); break;
                    case Tok::Slash:
                        if(rhs == 0) { return failAt(op.line, "division by zero"); }
                        lhs = (lhs == -2147483647 - 1 && rhs == -1) ? lhs : lhs / rhs;
                        break;
                    case Tok::And: lhs = static_cast<int>(l & r); break;
                    case Tok::Or:  lhs = static_cast<int>(l | r); break;
                    case Tok::Xor: lhs = static_cast<int>(l ^ r); break;
                    case Tok::Shl:
                    case Tok::Shr:
                        if(rhs < 0 || rhs > 31) {
                            return failAt(op.line, "shift by ", rhs, " (0..31)");
                        }
                        lhs = op.kind == Tok::Shl ? static_cast<int>(l << r) : (lhs >> rhs);
                        break;
                    default: break;
                    }
                }
                out = lhs;
                return true;
            }

            // ---- pass 1: labels and defines ---------------------------------------------
            enum class Where : std::uint8_t { global, selected, other, after };

            // An instruction keyword. `.word` makes an instruction too, but it is a directive:
            // pioasm's grammar takes `label: jmp ...`, not `label: .word ...` (parser.yy `line`).
            constexpr bool isOpcode(Lexer const& lx,
                                    Token const& t) const {
                if(t.kind != Tok::Ident) { return false; }
                for(auto k : Instructions) {
                    if(sameWord(lx.text(t), k)) { return true; }
                }
                return false;
            }

            constexpr bool isInstruction(Lexer const& lx,
                                         Token const& t) const {
                if(t.kind == Tok::Directive) { return sameWord(lx.text(t), ".word"); }
                if(t.kind != Tok::Ident) { return false; }
                for(auto k : Instructions) {
                    if(sameWord(lx.text(t), k)) { return true; }
                }
                return false;
            }

            // `.program name`: which program the lines below belong to
            constexpr bool program(Lexer&    lx,
                                   Where&    where,
                                   unsigned& programs) {
                auto const n = lx.next();
                if(n.kind != Tok::Ident || isKeyword(lx.text(n))) {
                    return unexpected(lx, n, "a program name");
                }
                ++programs;
                bool const selected = program_.empty() ? programs == 1 : lx.text(n) == program_;
                if(program_.empty() && programs == 2) {
                    return failAt(
                      n.line,
                      "the text holds more than one .program: name the one to assemble");
                }
                if(where == Where::selected || where == Where::after) {
                    where = Where::after;
                } else {
                    where = selected ? Where::selected : Where::other;
                }
                selectedFound_ = selectedFound_ || selected;
                return true;
            }

            // symbol_def: name | public name | *name
            constexpr bool symbolDef(Lexer& lx,
                                     Token  t,
                                     Sym&   s) {
                if(t.kind == Tok::Star || (t.kind == Tok::Ident && sameWord(lx.text(t), "public")))
                {
                    s.isPublic = true;
                    t          = lx.next();
                }
                if(t.kind != Tok::Ident) { return unexpected(lx, t, "a name"); }
                s.name = lx.text(t);
                s.line = t.line;
                return true;
            }

            constexpr void collect() {
                Lexer    lx{text_};
                Where    where    = Where::global;
                unsigned programs = 0;
                int      count    = 0;   // instructions of the selected program so far
                while(!a_.failed()) {
                    auto const t = lx.next();
                    if(t.kind == Tok::End) { break; }
                    if(t.kind == Tok::Newline) { continue; }
                    if(t.kind == Tok::Bad) {
                        failAt(t.line, lx.problem());
                        return;
                    }
                    if(t.kind == Tok::Directive && sameWord(lx.text(t), ".program")) {
                        if(!program(lx, where, programs)) { return; }
                    } else if(where == Where::global || where == Where::selected) {
                        if(t.kind == Tok::Directive && sameWord(lx.text(t), ".define")) {
                            Sym s{};
                            if(!symbolDef(lx, lx.next(), s)) { return; }
                            s.exprPos = lx.peek().begin;
                            if(!addSym(s)) { return; }
                        } else if(isInstruction(lx, t)) {
                            if(where == Where::global) {
                                failAt(t.line, "an instruction outside of a .program");
                                return;
                            }
                            ++count;
                        } else if(t.kind == Tok::Ident || t.kind == Tok::Star) {
                            if(where == Where::global) {
                                failAt(t.line, "a label outside of a .program");
                                return;
                            }
                            // a label, and maybe an instruction after it
                            Sym s{};
                            if(!symbolDef(lx, t, s)) { return; }
                            auto const colon = lx.next();
                            if(colon.kind != Tok::Colon) {
                                unexpected(lx, colon, "':' after a label");
                                return;
                            }
                            s.isLabel = true;
                            s.value   = count;
                            if(where == Where::selected && !addSym(s)) { return; }
                            if(isOpcode(lx, lx.peek())) { ++count; }
                        }
                    }
                    if(t.kind != Tok::Newline) { skipLine(lx); }
                }
                if(!a_.failed() && programs == 0) { failAt(0, "no .program in the text"); }
                if(!a_.failed() && !selectedFound_) {
                    failAt(0, "no .program ", program_, " in the text");
                }
            }

            // ---- pass 2: assemble ----------------------------------------------------------
            constexpr bool keyword(Lexer const&     lx,
                                   Token const&     t,
                                   std::string_view word) const {
                return t.kind == Tok::Ident && sameWord(lx.text(t), word);
            }

            constexpr bool optionalComma(Lexer& lx) {
                if(lx.peek().kind == Tok::Comma) { lx.next(); }
                return true;
            }

            constexpr void assemble() {
                Lexer    lx{text_};
                Where    where         = Where::global;
                unsigned programs      = 0;
                int      globalVersion = 0;
                while(!a_.failed()) {
                    auto const t = lx.next();
                    if(t.kind == Tok::End) { break; }
                    if(t.kind == Tok::Newline) { continue; }
                    if(t.kind == Tok::Bad) {
                        failAt(t.line, lx.problem());
                        return;
                    }
                    a_.currentLine = static_cast<std::uint16_t>(t.line);
                    if(t.kind == Tok::Directive && sameWord(lx.text(t), ".program")) {
                        if(!program(lx, where, programs)) { return; }
                        if(where == Where::after) { return; }   // the selected program has ended
                        if(where == Where::selected && globalVersion != 0) {
                            a_.pioVersion(globalVersion);
                        }
                        if(!expectEnd(lx)) { return; }
                        continue;
                    }
                    if(where == Where::other) {
                        skipLine(lx);
                        continue;
                    }
                    bool const global = where == Where::global;
                    if(t.kind == Tok::Directive) {
                        if(!directive(lx, t, global, globalVersion)) { return; }
                        continue;
                    }
                    if(global) {
                        failAt(t.line,
                               isInstruction(lx, t) ? "an instruction" : "a label",
                               " outside of a .program");
                        return;
                    }
                    if(!isInstruction(lx, t)) {
                        Sym s{};
                        if(!symbolDef(lx, t, s)) { return; }
                        lx.next();   // the ':' (collect() checked it)
                        a_.label(s.name, s.isPublic);
                        auto const n = lx.peek();
                        if(n.kind == Tok::Newline || n.kind == Tok::End) { continue; }
                        if(!isOpcode(lx, n)) {
                            unexpected(lx, n, "an instruction after the label");
                            return;
                        }
                        lx.next();
                        a_.currentLine = static_cast<std::uint16_t>(n.line);
                        if(!instruction(lx, n)) { return; }
                        continue;
                    }
                    if(!instruction(lx, t)) { return; }
                }
            }

            constexpr bool directive(Lexer&       lx,
                                     Token const& t,
                                     bool         global,
                                     int&         globalVersion) {
                auto const name = lx.text(t);
                auto const d    = [&](std::string_view w) { return sameWord(name, w); };
                if(d(".define")) {
                    // collect() recorded it; resolving it here puts it into the program's symbols
                    Sym s{};
                    if(!symbolDef(lx, lx.next(), s)) { return false; }
                    int v = 0;
                    if(!resolve(s.name, s.line, v)) { return false; }
                    a_.define(s.name, v, s.isPublic);
                    skipLine(lx);
                    return !a_.failed();
                }
                if(d(".lang_opt")) {
                    skipLine(lx);
                    return !a_.failed();
                }
                if(d(".pio_version")) {
                    auto const v       = lx.next();
                    int        version = 0;
                    if(v.kind == Tok::Int) {
                        version = static_cast<int>(v.num);
                    } else if(keyword(lx, v, "one") || keyword(lx, v, "zero")) {
                        version = keyword(lx, v, "one") ? 1 : 0;   // lexer.ll: integers
                    } else if(keyword(lx, v, "rp2040")) {
                        version = 0;
                    } else if(keyword(lx, v, "rp2350")) {
                        version = 1;
                    } else {
                        return unexpected(lx, v, "0, 1, rp2040 or rp2350");
                    }
                    if(global) {
                        if(version < 0 || version > 1) {
                            return failAt(v.line,
                                          ".pio_version ",
                                          version,
                                          ": 0 (RP2040) or 1 (RP2350)");
                        }
                        globalVersion = version;
                    } else {
                        a_.pioVersion(version);
                    }
                    return expectEnd(lx);
                }
                if(global) { return failAt(t.line, name, " outside of a .program"); }
                int v = 0;
                if(d(".wrap_target")) {
                    a_.wrapTarget();
                } else if(d(".wrap")) {
                    a_.wrap();
                } else if(d(".origin")) {
                    if(!value(lx, v)) { return false; }
                    a_.origin(v);
                } else if(d(".word")) {
                    if(!value(lx, v)) { return false; }
                    a_.word(v);
                } else if(d(".side_set")) {
                    if(!value(lx, v)) { return false; }
                    bool opt = false, pindirs = false;
                    auto n = lx.peek();
                    if(keyword(lx, n, "opt") || keyword(lx, n, "optional")) {
                        opt = true;
                        lx.next();
                        n = lx.peek();
                    }
                    if(keyword(lx, n, "pindirs")) {
                        pindirs = true;
                        lx.next();
                    }
                    a_.sideSet(v, opt, pindirs);
                } else if(d(".in") || d(".out")) {
                    if(!value(lx, v)) { return false; }
                    bool right = true, autoFlag = false;
                    int  threshold = 32;
                    auto n         = lx.peek();
                    if(keyword(lx, n, "left") || keyword(lx, n, "right")) {
                        right = keyword(lx, n, "right");
                        lx.next();
                        n = lx.peek();
                    }
                    if(keyword(lx, n, "auto") || keyword(lx, n, "manual")) {
                        autoFlag = keyword(lx, n, "auto");
                        lx.next();
                        n = lx.peek();
                    }
                    if(n.kind != Tok::Newline && n.kind != Tok::End && !value(lx, threshold)) {
                        return false;
                    }
                    if(d(".in")) {
                        a_.inConfig(v, right, autoFlag, threshold);
                    } else {
                        a_.outConfig(v, right, autoFlag, threshold);
                    }
                } else if(d(".set")) {
                    if(!value(lx, v)) { return false; }
                    a_.setConfig(v);
                } else if(d(".clock_div")) {
                    auto const n = lx.next();
                    if(n.kind != Tok::Int && n.kind != Tok::Float) {
                        return unexpected(lx, n, "a clock divider");
                    }
                    long long const x = n.kind == Tok::Int ? n.num * 256 : n.num;
                    if(x < 256 || x >= 65536LL * 256) {
                        return failAt(n.line, "clock divider must be between 1 and 65535");
                    }
                    a_.clockDiv(static_cast<unsigned>(x / 256), static_cast<unsigned>(x % 256));
                } else if(d(".fifo")) {
                    auto const                 n = lx.next();
                    constexpr std::string_view names[]
                      = {"txrx", "tx", "rx", "txget", "txput", "putget"};
                    bool found = false;
                    for(std::size_t i = 0; i < 6; ++i) {
                        if(keyword(lx, n, names[i])) {
                            a_.fifo(static_cast<Fifo>(i));
                            found = true;
                        }
                    }
                    if(!found) { return unexpected(lx, n, "txrx, tx, rx, txput, txget or putget"); }
                } else if(d(".mov_status")) {
                    auto const n = lx.next();
                    if(keyword(lx, n, "txfifo") || keyword(lx, n, "rxfifo")) {
                        auto const less = lx.next();
                        if(less.kind != Tok::Less) { return unexpected(lx, less, "'<'"); }
                        if(!value(lx, v)) { return false; }
                        a_.movStatus(keyword(lx, n, "txfifo") ? MovStatusKind::txLessThan
                                                              : MovStatusKind::rxLessThan,
                                     v);
                    } else if(keyword(lx, n, "irq")) {
                        auto    s    = lx.next();
                        IrqMode mode = IrqMode::plain;
                        if(keyword(lx, s, "next") || keyword(lx, s, "prev")) {
                            mode = keyword(lx, s, "next") ? IrqMode::next : IrqMode::prev;
                            s    = lx.next();
                        }
                        if(!keyword(lx, s, "set")) { return unexpected(lx, s, "'set'"); }
                        if(!value(lx, v)) { return false; }
                        a_.movStatus(MovStatusKind::irqSet, v, mode);
                    } else {
                        return unexpected(lx, n, "'txfifo < N', 'rxfifo < N' or 'irq set N'");
                    }
                } else {
                    return failAt(t.line, "unknown directive ", name);
                }
                return !a_.failed() && expectEnd(lx);
            }

            // ---- instructions -------------------------------------------------------------
            constexpr bool instruction(Lexer&       lx,
                                       Token const& t) {
                auto const  name = lx.text(t);
                auto const  is   = [&](std::string_view w) { return sameWord(name, w); };
                Asm::Instr* i    = nullptr;
                int         v    = 0;
                if(is("nop")) {
                    i = &a_.nop();
                } else if(is("jmp")) {
                    if(!jmp(lx, i)) { return false; }
                } else if(is("wait")) {
                    if(!wait(lx, i)) { return false; }
                } else if(is("in") || is("out")) {
                    if(!inOut(lx, is("in"), i)) { return false; }
                } else if(is("push") || is("pull")) {
                    bool const push = is("push");
                    bool       cond = false, block = true;
                    auto       n = lx.peek();
                    if(keyword(lx, n, push ? "iffull" : "ifempty")) {
                        cond = true;
                        lx.next();
                        n = lx.peek();
                    }
                    if(keyword(lx, n, "block") || keyword(lx, n, "noblock")) {
                        block = keyword(lx, n, "block");
                        lx.next();
                    }
                    i = push ? &a_.push(cond, block) : &a_.pull(cond, block);
                } else if(is("mov")) {
                    if(!mov(lx, i)) { return false; }
                } else if(is("irq")) {
                    if(!irq(lx, i)) { return false; }
                } else if(is("set")) {
                    auto const d = lx.next();
                    Set        dst{};
                    if(keyword(lx, d, "pins")) {
                        dst = Set::pins;
                    } else if(keyword(lx, d, "x")) {
                        dst = Set::x;
                    } else if(keyword(lx, d, "y")) {
                        dst = Set::y;
                    } else if(keyword(lx, d, "pindirs")) {
                        dst = Set::pindirs;
                    } else {
                        return unexpected(lx, d, "pins, x, y or pindirs");
                    }
                    optionalComma(lx);
                    if(!value(lx, v)) { return false; }
                    i = &a_.set(dst, v);
                }
                if(a_.failed() || i == nullptr) { return false; }
                return sideAndDelay(lx, *i);
            }

            // `side value` and `[expression]`, in either order, each at most once
            constexpr bool sideAndDelay(Lexer&      lx,
                                        Asm::Instr& i) {
                bool haveSide = false, haveDelay = false;
                while(true) {
                    auto const n = lx.peek();
                    if(!haveSide
                       && (keyword(lx, n, "side") || keyword(lx, n, "sideset")
                           || keyword(lx, n, "side_set")))
                    {
                        lx.next();
                        int v = 0;
                        if(!value(lx, v)) { return false; }
                        if(v < 0) { return failAt(n.line, "side-set value ", v, " is negative"); }
                        i.side(static_cast<unsigned>(v));
                        haveSide = true;
                    } else if(!haveDelay && n.kind == Tok::LBracket) {
                        lx.next();
                        int v = 0;
                        if(!expression(lx, 0, v)) { return false; }
                        auto const r = lx.next();
                        if(r.kind != Tok::RBracket) { return unexpected(lx, r, "']'"); }
                        if(v < 0) { return failAt(n.line, "delay ", v, " is negative"); }
                        i.delay(static_cast<unsigned>(v));
                        haveDelay = true;
                    } else {
                        return expectEnd(lx);
                    }
                }
            }

            constexpr bool jmp(Lexer&       lx,
                               Asm::Instr*& i) {
                Cond c = Cond::always;
                auto n = lx.peek();
                if(n.kind == Tok::Not) {
                    lx.next();
                    auto const r = lx.next();
                    if(keyword(lx, r, "x")) {
                        c = Cond::notX;
                    } else if(keyword(lx, r, "y")) {
                        c = Cond::notY;
                    } else if(keyword(lx, r, "osre")) {
                        c = Cond::notOsre;
                    } else {
                        return unexpected(lx, r, "x, y or osre after '!'");
                    }
                } else if(keyword(lx, n, "x") || keyword(lx, n, "y")) {
                    bool const x = keyword(lx, n, "x");
                    lx.next();
                    auto const op = lx.next();
                    if(op.kind == Tok::Dec) {
                        c = x ? Cond::xDec : Cond::yDec;
                    } else if(x && op.kind == Tok::NotEq) {
                        auto const y = lx.next();
                        if(!keyword(lx, y, "y")) { return unexpected(lx, y, "y after 'x!='"); }
                        c = Cond::xNeY;
                    } else {
                        return unexpected(lx, op, x ? "'--' or '!=' after x" : "'--' after y");
                    }
                } else if(keyword(lx, n, "pin")) {
                    lx.next();
                    c = Cond::pin;
                }
                optionalComma(lx);
                auto const where  = lx.peek();
                int        target = 0;
                if(!expression(lx, 0, target)) { return false; }
                if(target < 0) { return failAt(where.line, "jmp target ", target, " is negative"); }
                i = &a_.jmp(c, static_cast<unsigned>(target));
                return true;
            }

            constexpr bool wait(Lexer&       lx,
                                Asm::Instr*& i) {
                int        pol      = 1;
                auto       n        = lx.peek();
                auto const isSource = [&](Token const& s) {
                    return keyword(lx, s, "irq") || keyword(lx, s, "gpio") || keyword(lx, s, "pin")
                        || keyword(lx, s, "jmppin");
                };
                if(!isSource(n)) {
                    if(!value(lx, pol)) { return false; }
                }
                auto const s   = lx.next();
                int        idx = 0;
                if(keyword(lx, s, "irq")) {
                    IrqMode    mode = IrqMode::plain;
                    auto const m    = lx.peek();
                    if(keyword(lx, m, "prev") || keyword(lx, m, "next")) {
                        mode = keyword(lx, m, "prev") ? IrqMode::prev : IrqMode::next;
                        lx.next();
                    }
                    optionalComma(lx);
                    if(!value(lx, idx)) { return false; }
                    auto const r = lx.peek();
                    if(keyword(lx, r, "rel")) {
                        if(mode != IrqMode::plain) {
                            return failAt(r.line,
                                          "'rel' does not go with 'irq prev' or 'irq next'");
                        }
                        mode = IrqMode::rel;
                        lx.next();
                    }
                    i = &a_.wait(pol, Wait::irq, idx, mode);
                } else if(keyword(lx, s, "gpio") || keyword(lx, s, "pin")) {
                    optionalComma(lx);
                    if(!value(lx, idx)) { return false; }
                    i = &a_.wait(pol, keyword(lx, s, "gpio") ? Wait::gpio : Wait::pin, idx);
                } else if(keyword(lx, s, "jmppin")) {
                    if(lx.peek().kind == Tok::Plus) {
                        lx.next();
                        if(!value(lx, idx)) { return false; }
                    }
                    i = &a_.wait(pol, Wait::jmppin, idx);
                } else {
                    return unexpected(lx,
                                      s,
                                      a_.pioVersionNow() >= 1 ? "irq, gpio, pin or jmppin"
                                                              : "irq, gpio or pin");
                }
                return true;
            }

            constexpr bool inOut(Lexer&       lx,
                                 bool         isIn,
                                 Asm::Instr*& i) {
                auto const s     = lx.next();
                auto const k     = [&](std::string_view w) { return keyword(lx, s, w); };
                int        where = -1;
                if(isIn) {
                    if(k("status")) {
                        return failAt(
                          s.line,
                          "in status: source 101 is reserved (RP2350 datasheet 11.4.4.2)");
                    }
                    where = k("pins") ? 0
                          : k("x")    ? 1
                          : k("y")    ? 2
                          : k("null") ? 3
                          : k("isr")  ? 6
                          : k("osr")  ? 7
                                      : -1;
                    if(where < 0) { return unexpected(lx, s, "pins, x, y, null, isr or osr"); }
                } else {
                    where = k("pins")    ? 0
                          : k("x")       ? 1
                          : k("y")       ? 2
                          : k("null")    ? 3
                          : k("pindirs") ? 4
                          : k("pc")      ? 5
                          : k("isr")     ? 6
                          : k("exec")    ? 7
                                         : -1;
                    if(where < 0) {
                        return unexpected(lx, s, "pins, x, y, null, pindirs, pc, isr or exec");
                    }
                }
                optionalComma(lx);
                int bits = 0;
                if(!value(lx, bits)) { return false; }
                i = isIn ? &a_.in(static_cast<In>(where), bits)
                         : &a_.out(static_cast<Out>(where), bits);
                return true;
            }

            // rxfifo[y] or rxfifo[value]; `index` is -1 for y
            constexpr bool rxfifoIndex(Lexer& lx,
                                       int&   index) {
                auto const l = lx.next();
                if(l.kind != Tok::LBracket) { return unexpected(lx, l, "'['"); }
                if(keyword(lx, lx.peek(), "y")) {
                    lx.next();
                    index = -1;
                } else if(!value(lx, index)) {
                    return false;
                }
                auto const r = lx.next();
                if(r.kind != Tok::RBracket) { return unexpected(lx, r, "']'"); }
                return true;
            }

            constexpr bool mov(Lexer&       lx,
                               Asm::Instr*& i) {
                auto const d = lx.next();
                auto const k
                  = [&](Token const& t, std::string_view w) { return keyword(lx, t, w); };
                int dst  = -1;
                int rxTo = -2;   // rxfifo destination: -1 = [y], 0..3 = index
                if(k(d, "rxfifo")) {
                    if(!rxfifoIndex(lx, rxTo)) { return false; }
                } else {
                    dst = k(d, "pins")    ? 0
                        : k(d, "x")       ? 1
                        : k(d, "y")       ? 2
                        : k(d, "pindirs") ? 3
                        : k(d, "exec")    ? 4
                        : k(d, "pc")      ? 5
                        : k(d, "isr")     ? 6
                        : k(d, "osr")     ? 7
                                          : -1;
                    if(dst < 0) { return unexpected(lx, d, "a mov destination"); }
                }
                optionalComma(lx);
                MovOp op = MovOp::none;
                auto  o  = lx.peek();
                if(o.kind == Tok::Not || o.kind == Tok::Reverse) {
                    op = o.kind == Tok::Not ? MovOp::invert : MovOp::reverse;
                    lx.next();
                }
                auto const s      = lx.next();
                int        src    = -1;
                int        rxFrom = -2;
                if(k(s, "rxfifo")) {
                    if(!rxfifoIndex(lx, rxFrom)) { return false; }
                } else {
                    src = k(s, "pins")   ? 0
                        : k(s, "x")      ? 1
                        : k(s, "y")      ? 2
                        : k(s, "null")   ? 3
                        : k(s, "status") ? 5
                        : k(s, "isr")    ? 6
                        : k(s, "osr")    ? 7
                                         : -1;
                    if(src < 0) { return unexpected(lx, s, "a mov source"); }
                }
                if(rxTo != -2 || rxFrom != -2) {
                    if(op != MovOp::none) {
                        return failAt(o.line, "mov to or from rxfifo[] takes no '!' or '::'");
                    }
                    if(rxTo != -2 && rxFrom != -2) {
                        return failAt(d.line, "mov rxfifo[] to rxfifo[]");
                    }
                    if(rxTo != -2) {
                        if(src != 6) {
                            return failAt(s.line, "mov rxfifo[]: the source must be isr");
                        }
                        i = rxTo < 0 ? &a_.movToRxY() : &a_.movToRx(rxTo);
                    } else {
                        if(dst != 7) {
                            return failAt(d.line, "mov from rxfifo[]: the destination must be osr");
                        }
                        i = rxFrom < 0 ? &a_.movFromRxY() : &a_.movFromRx(rxFrom);
                    }
                    return true;
                }
                i = &a_.mov(static_cast<MovDst>(dst), static_cast<MovSrc>(src), op);
                return true;
            }

            // irq [prev|next] [set|nowait|wait|clear] n [rel]
            constexpr bool irq(Lexer&       lx,
                               Asm::Instr*& i) {
                IrqMode mode = IrqMode::plain;
                auto    n    = lx.peek();
                if(keyword(lx, n, "prev") || keyword(lx, n, "next")) {
                    mode = keyword(lx, n, "prev") ? IrqMode::prev : IrqMode::next;
                    lx.next();
                    n = lx.peek();
                }
                bool clear = false, wait = false;
                if(keyword(lx, n, "clear")) {
                    clear = true;
                    lx.next();
                } else if(keyword(lx, n, "wait")) {
                    wait = true;
                    lx.next();
                } else if(keyword(lx, n, "nowait") || keyword(lx, n, "set")) {
                    lx.next();
                }
                int idx = 0;
                if(!value(lx, idx)) { return false; }
                auto const r = lx.peek();
                if(keyword(lx, r, "rel")) {
                    if(mode != IrqMode::plain) {
                        return failAt(r.line, "'rel' does not go with 'irq prev' or 'irq next'");
                    }
                    mode = IrqMode::rel;
                    lx.next();
                }
                i = clear ? &a_.irqClear(idx, mode)
                  : wait  ? &a_.irqWait(idx, mode)
                          : &a_.irq(idx, mode);
                return true;
            }

            std::string_view       text_;
            std::string_view       program_;
            std::array<Define, 32> defines_{};
            std::size_t            defineCount_{};
            std::array<Sym, 64>    syms_{};
            std::size_t            symCount_{};
            bool                   selectedFound_{};
            Asm                    a_{};
        };
    }}   // namespace detail::pioparse

    // Assemble the program in `text` (pioasm syntax). `defines` are values from C++, `program`
    // names the .program to assemble when the text holds more than one.
    consteval Assembled parse(std::string_view              text,
                              std::initializer_list<Define> defines = {},
                              std::string_view              program = {}) {
        return detail::pioparse::Parser{text, defines, program}.run();
    }

    // A .pio file taken in with #embed, as it is:
    //
    //   // clang-format off  (it breaks #embed lines)
    //   struct Blink : Kvasir::Pio::Program<Kvasir::Pio::parse({
    //   #embed "blink.pio"
    //   })> {};
    //   // clang-format on
    //
    // #embed gives the file's bytes as a list of ints (0..255), taken here as unsigned char: in a
    // `char` gcc refuses every byte above 127 (UTF-8 in a comment) as narrowing. The file is
    // searched next to the including file and in `--embed-dir=<dir>`, not in -I directories.
    // clang warns that #embed is a C23 extension in C++ (-Wc23-extensions).
    consteval Assembled parse(std::initializer_list<unsigned char> file,
                              std::initializer_list<Define>        defines = {},
                              std::string_view                     program = {}) {
        // the parser reads chars; a string, since the list's length is not a constant here
        std::string text;
        text.reserve(file.size());
        for(auto const c : file) { text.push_back(static_cast<char>(c)); }
        std::size_t const n = text.size();
        return parse(std::string_view{text.data(), n}, defines, program);
    }

    // The same from an array #embed filled (`constexpr unsigned char file[] = { #embed "x.pio" };`),
    // or from a char array or string literal: without a terminating NUL, which a string_view
    // made from the array would read past.
    template<typename Char,
             std::size_t N>
        requires(std::is_same_v<Char,
                                unsigned char>
                 || std::is_same_v<Char,
                                   char>)
    consteval Assembled parse(Char const (&file)[N],
                              std::initializer_list<Define> defines = {},
                              std::string_view              program = {}) {
        std::array<char, N> text{};
        std::size_t         n = 0;
        for(std::size_t i = 0; i < N; ++i) { text[n++] = static_cast<char>(file[i]); }
        if(n != 0 && text[n - 1] == '\0') { --n; }   // a string literal's
        return parse(std::string_view{text.data(), n}, defines, program);
    }
}}   // namespace Kvasir::Pio
