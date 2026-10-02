#!/usr/bin/env python3
"""Writes random, valid .pio files, for pioasm and parse() to assemble and the cross-check to compare.

Usage: gen_random_pio.py <out dir> <seed> <files>

Hand-written programs use few of the combinations; these use all of them: every instruction form
(v1 ones under .pio_version 1), every side-set shape (count, opt, pindirs) with side values and
delays up to their limits, labels, jumps to labels and to expressions, `.define`s (global, public,
forward references to labels), expressions with every operator unparenthesised, `.word`, `.origin`,
`.fifo`, `.in`, `.out`, `.set`, `.mov_status`, `.wrap_target` / `.wrap`, keywords in random case,
optional commas, and all three comment styles. Deterministic for a seed.

The generator keeps every value in its range by evaluating the expressions with pioasm's grammar
(parser.yy: `%left` REVERSE < SHL SHR < PLUS MINUS < MULTIPLY DIVIDE < AND OR XOR; unary minus
binds like binary minus, `::` loosest). Were that evaluation wrong, pioasm would refuse the file or
the two assemblers would disagree - either way the build says so.

Only what both accept is generated: parse() is stricter than pioasm where the datasheet is (no
`in status`, rxfifo index 0..3, `wait gpio` 0..31) and refuses an empty program.
"""

import os
import random
import sys

BIN_OPS = {"<<": 1, ">>": 1, "+": 2, "-": 2,
           "*": 3, "/": 3, "&": 4, "|": 4, "^": 4}


def wrap32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v & 0x80000000 else v


class Expr:
    """A random expression as pioasm tokens, and its value by pioasm's grammar (None if invalid)."""

    def __init__(self, rng, names):
        self.rng = rng
        self.names = names   # name -> value

    def tokens(self, depth):
        r = self.rng.random()
        if depth <= 0 or r < 0.3:
            return self.primary(depth)
        if r < 0.4:
            return ["-"] + self.tokens(depth - 1)
        if r < 0.43:
            return ["::"] + self.tokens(depth - 1)
        op = self.rng.choice(list(BIN_OPS))
        rhs = self.tokens(depth - 1)
        if op in ("<<", ">>"):
            rhs = [str(self.rng.randint(0, 6))]
        return self.tokens(depth - 1) + [op] + rhs

    def primary(self, depth):
        r = self.rng.random()
        if r < 0.15 and self.names:
            return [self.rng.choice(sorted(self.names))]
        if r < 0.3 and depth > 0:
            return ["("] + self.tokens(depth - 1) + [")"]
        v = self.rng.randint(0, 20)
        form = self.rng.random()
        if form < 0.2:
            return [hex(v)]
        if form < 0.3:
            return [bin(v)]
        return [str(v)]

    # pioasm's evaluation: precedence climbing over the token list
    def evaluate(self, toks):
        self.toks, self.pos = toks, 0
        try:
            v = self.expr(0)
        except (ZeroDivisionError, ValueError, KeyError):
            return None
        return v if self.pos == len(self.toks) else None

    def peek(self):
        return self.toks[self.pos] if self.pos < len(self.toks) else None

    def expr(self, min_prec):
        t = self.peek()
        if t == "-":
            self.pos += 1
            lhs = wrap32(-self.expr(3))
        elif t == "::":
            self.pos += 1
            v = self.expr(1) & 0xFFFFFFFF
            lhs = wrap32(int(f"{v:032b}"[::-1], 2))
        else:
            lhs = self.value()
        while True:
            op = self.peek()
            p = BIN_OPS.get(op, -1) if op is not None else -1
            if p < 0 or p < min_prec:
                return lhs
            self.pos += 1
            rhs = self.expr(p + 1)
            if op == "+":
                lhs = wrap32(lhs + rhs)
            elif op == "-":
                lhs = wrap32(lhs - rhs)
            elif op == "*":
                lhs = wrap32(lhs * rhs)
            elif op == "/":
                if rhs == 0:
                    raise ZeroDivisionError
                q = abs(lhs) // abs(rhs)
                lhs = wrap32(q if (lhs < 0) == (rhs < 0) else -q)
            elif op == "&":
                lhs = wrap32(lhs & rhs)
            elif op == "|":
                lhs = wrap32(lhs | rhs)
            elif op == "^":
                lhs = wrap32(lhs ^ rhs)
            elif op in ("<<", ">>"):
                if not 0 <= rhs <= 31:
                    raise ValueError
                lhs = wrap32(lhs << rhs) if op == "<<" else lhs >> rhs

    def value(self):
        t = self.toks[self.pos]
        self.pos += 1
        if t == "(":
            v = self.expr(0)
            if self.peek() != ")":
                raise ValueError
            self.pos += 1
            return v
        if t in self.names:
            return self.names[t]
        return int(t, 0)

    def make(self, lo, hi, depth=3, tries=40):
        """Tokens of an expression whose value is in lo..hi, with the value."""
        for _ in range(tries):
            toks = self.tokens(depth)
            v = self.evaluate(toks)
            if v is not None and lo <= v <= hi:
                return toks, v
        v = self.rng.randint(lo, hi)
        return [str(v)], v


def is_v1_only(w):
    """Asm.hpp isPioV1Only: encodings the RP2040 reserves and the RP2350 uses."""
    op = w >> 13
    if op == 1:
        src = (w >> 5) & 3
        return src == 3 or (src == 2 and (w & 8) != 0)
    if op == 4:
        return (w & 0x1F) != 0
    if op == 5:
        return ((w >> 5) & 7) == 3
    if op == 6:
        return (w & 8) != 0
    return False


def text(toks):
    return " ".join(toks)


def value_text(toks):
    """pioasm's `value`: an integer, a name, or a parenthesised expression."""
    if len(toks) == 1:
        return toks[0]
    return "(" + text(toks) + ")"


class Program:
    def __init__(self, rng, name, global_names, global_version):
        self.rng = rng
        self.name = name
        self.global_version = global_version
        self.version = global_version if global_version is not None else rng.choice([
                                                                                    0, 0, 1])
        self.lines = []
        self.names = dict(global_names)
        self.used = set(global_names)

    def kw(self, word):
        r = self.rng.random()
        if r < 0.7:
            return word
        if r < 0.85:
            return word.upper()
        return "".join(c.upper() if self.rng.random() < 0.5 else c for c in word)

    def comma(self):
        return ", " if self.rng.random() < 0.7 else " "

    def comment(self):
        r = self.rng.random()
        if r < 0.1:
            return " ; a comment"
        if r < 0.15:
            return " // another"
        if r < 0.18:
            return " /* block */"
        return ""

    def fresh(self, prefix):
        while True:
            n = f"{prefix}{self.rng.randint(0, 999)}"
            if n not in self.used:
                self.used.add(n)
                return n

    def generate(self):
        rng = self.rng
        out = [f".program {self.name}"]
        # a global .pio_version 1 already applies; otherwise a v1 program says so (pioasm runs at -v 0)
        if self.version == 1 and self.global_version is None:
            out.append(self.kw(".pio_version") + " " +
                       rng.choice(["1", "rp2350", "one"] if self.version else ["0"]))
        # side-set
        ss_count, ss_opt, ss_pindirs = None, False, False
        if rng.random() < 0.6:
            ss_opt = rng.random() < 0.5
            ss_count = rng.randint(0, 4 if ss_opt else 5)
            ss_pindirs = rng.random() < 0.3
            out.append(
                f".side_set {ss_count}" + (" opt" if ss_opt else "") + (" pindirs" if ss_pindirs else ""))
        bits = (ss_count + (1 if ss_opt else 0)) if ss_count is not None else 0
        delay_max = (1 << (5 - bits)) - 1
        side_max = (1 << ss_count) - 1 if ss_count is not None else None
        side_needed = ss_count is not None and not ss_opt
        # fifo and the other directives
        fifos = ["txrx", "tx", "rx"] + \
            (["txput", "txget", "putget"] if self.version else [])
        fifo = rng.choice(fifos) if rng.random() < 0.4 else "txrx"
        if fifo != "txrx" or rng.random() < 0.1:
            out.append(f".fifo {fifo}")
        if rng.random() < 0.3:
            n = 32 if self.version == 0 else rng.randint(1, 32)
            auto = fifo in ("txrx", "tx", "rx") and rng.random() < 0.5
            parts = [f".in {n}"]
            if rng.random() < 0.6:
                parts.append(rng.choice(["left", "right"]))
            if rng.random() < 0.6:
                parts.append("auto" if auto else "manual")
            if rng.random() < 0.6:
                parts.append(str(rng.randint(1, 32)))
            out.append(" ".join(parts))
        if rng.random() < 0.3:
            parts = [f".out {rng.randint(0, 32)}"]
            if rng.random() < 0.6:
                parts.append(rng.choice(["left", "right"]))
            if rng.random() < 0.6:
                parts.append(rng.choice(["auto", "manual"]))
            if rng.random() < 0.6:
                parts.append(str(rng.randint(1, 32)))
            out.append(" ".join(parts))
        if rng.random() < 0.2:
            out.append(f".set {rng.randint(0, 5)}")
        if rng.random() < 0.2:
            r = rng.random()
            if r < 0.4:
                out.append(f".mov_status txfifo < {rng.randint(0, 31)}")
            elif r < 0.7:
                out.append(f".mov_status rxfifo < {rng.randint(0, 31)}")
            else:
                out.append(
                    ".mov_status irq " + rng.choice(["", "next ", "prev "]) + f"set {rng.randint(0, 7)}")
        count = rng.randint(1, 20)
        if rng.random() < 0.15:
            out.append(f".origin {rng.randint(0, 32 - count)}")
        # program defines (values known now; a label may be named in them, see below)
        for _ in range(rng.randint(0, 3)):
            name = self.fresh("D")
            toks, v = Expr(rng, self.names).make(-50, 50)
            out.append(
                f".define {'public ' if rng.random() < 0.5 else ''}{name} {text(toks)}")
            self.names[name] = v
        # labels: positions first, so jumps can name labels further down
        labels = {}
        for _ in range(rng.randint(0, 4)):
            labels[self.fresh("L")] = rng.randint(0, count - 1)
        wrap_target = rng.randint(0, count - 1) if rng.random() < 0.4 else None
        wrap_at = rng.randint(wrap_target if wrap_target is not None else 0,
                              count - 1) if rng.random() < 0.4 else None
        for i in range(count):
            if i == wrap_target:
                out.append(self.kw(".wrap_target"))
            here = [n for n, at in labels.items() if at == i]
            line = ""
            for n in here[:-1]:
                out.append(f"{'public ' if rng.random() < 0.3 else ''}{n}:")
            instr, word = self.instruction(count, fifo, labels, side_needed)
            if here and word:   # pioasm: `label: .word` is no line; the label goes on its own
                out.append(
                    f"{'public ' if rng.random() < 0.3 else ''}{here[-1]}:")
            elif here:
                line = f"{'public ' if rng.random() < 0.3 else ''}{here[-1]}: "
            if not word:
                line += instr + \
                    self.side_delay(side_max, delay_max, side_needed)
            else:
                line += instr
            out.append("    " + line + self.comment())
            if i == wrap_at:
                out.append(self.kw(".wrap"))
        # a define after the code that names a label (pioasm resolves at the end)
        if labels and rng.random() < 0.3:
            name = self.fresh("P")
            out.append(
                f".define public {name} {rng.choice(sorted(labels))} + 1")
        return out

    def side_delay(self, side_max, delay_max, side_needed):
        rng = self.rng
        parts = []
        if side_max is not None and (side_needed or rng.random() < 0.5):
            toks, _ = Expr(rng, self.names).make(0, side_max, depth=1)
            parts.append(self.kw("side") + " " + value_text(toks))
        if rng.random() < 0.5:
            toks, _ = Expr(rng, self.names).make(0, delay_max, depth=2)
            parts.append("[" + text(toks) + "]")
        rng.shuffle(parts)
        return (" " + " ".join(parts)) if parts else ""

    def val(self, lo, hi):
        toks, _ = Expr(self.rng, self.names).make(lo, hi, depth=2)
        return value_text(toks)

    def instruction(self, count, fifo, labels, side_needed):
        """(text, is_word)"""
        rng = self.rng
        v1 = self.version == 1
        choices = ["jmp", "wait", "in", "out",
                   "pull", "mov", "irq", "set", "nop"]
        if fifo in ("txrx", "rx"):
            choices.append("push")
        if not side_needed:
            choices.append(".word")
        if v1 and fifo in ("txput", "putget"):
            choices.append("mov_to_rx")
        if v1 and fifo in ("txget", "putget"):
            choices.append("mov_from_rx")
        op = rng.choice(choices)
        k = self.kw
        if op == "jmp":
            cond = rng.choice(
                ["", "!x", "x--", "!y", "y--", "x!=y", "pin", "!osre"])
            cond = cond.replace("osre", k("osre")).replace(
                "pin", k("pin")).replace("x", k("x")).replace("y", k("y"))
            if labels and rng.random() < 0.6:
                tgt = rng.choice(sorted(labels))
                if rng.random() < 0.2 and labels[tgt] + 1 < count:
                    tgt = f"{tgt} + 1"
            else:
                toks, _ = Expr(rng, self.names).make(0, count - 1, depth=2)
                tgt = text(toks)
            return f"{k('jmp')} {cond}{self.comma() if cond else ''}{tgt}", False
        if op == "wait":
            pol = rng.choice(["", "0 ", "1 "])
            srcs = ["gpio", "pin", "irq"] + \
                (["jmppin", "irq_prev", "irq_next"] if v1 else [])
            s = rng.choice(srcs)
            if s == "gpio":
                return f"{k('wait')} {pol}{k('gpio')}{self.comma()}{self.val(0, 31)}", False
            if s == "pin":
                return f"{k('wait')} {pol}{k('pin')}{self.comma()}{self.val(0, 31)}", False
            if s == "irq":
                return f"{k('wait')} {pol}{k('irq')}{self.comma()}{self.val(0, 7)}{' ' + k('rel') if rng.random() < 0.3 else ''}", False
            if s == "jmppin":
                return f"{k('wait')} {pol}{k('jmppin')}{(' + ' + self.val(0, 3)) if rng.random() < 0.5 else ''}", False
            which = "prev" if s == "irq_prev" else "next"
            return f"{k('wait')} {pol}{k('irq')} {k(which)}{self.comma()}{self.val(0, 7)}", False
        if op == "in":
            return f"{k('in')} {k(rng.choice(['pins', 'x', 'y', 'null', 'isr', 'osr']))}{self.comma()}{self.val(1, 32)}", False
        if op == "out":
            return f"{k('out')} {k(rng.choice(['pins', 'x', 'y', 'null', 'pindirs', 'pc', 'isr', 'exec']))}{self.comma()}{self.val(1, 32)}", False
        if op in ("push", "pull"):
            parts = [k(op)]
            if rng.random() < 0.5:
                parts.append(k("iffull" if op == "push" else "ifempty"))
            if rng.random() < 0.6:
                parts.append(k(rng.choice(["block", "noblock"])))
            return " ".join(parts), False
        if op == "mov":
            dsts = ["pins", "x", "y", "exec", "pc", "isr",
                    "osr"] + (["pindirs"] if v1 else [])
            srcs = ["pins", "x", "y", "null", "status", "isr", "osr"]
            opn = rng.choice(["", "!", "~", "::"])
            return f"{k('mov')} {k(rng.choice(dsts))}{self.comma()}{opn}{k(rng.choice(srcs))}", False
        if op == "mov_to_rx":
            idx = k("y") if rng.random() < 0.4 else self.val(0, 3)
            return f"{k('mov')} {k('rxfifo')}[{idx}]{self.comma()}{k('isr')}", False
        if op == "mov_from_rx":
            idx = k("y") if rng.random() < 0.4 else self.val(0, 3)
            return f"{k('mov')} {k('osr')}{self.comma()}{k('rxfifo')}[{idx}]", False
        if op == "irq":
            mode = rng.choice(["", "", "prev ", "next "]) if v1 else ""
            mod = rng.choice(["", "set ", "nowait ", "wait ", "clear "])
            rel = " " + k("rel") if not mode and rng.random() < 0.3 else ""
            return f"{k('irq')} {k(mode.strip()) + ' ' if mode else ''}{k(mod.strip()) + ' ' if mod else ''}{self.val(0, 7)}{rel}", False
        if op == "set":
            return f"{k('set')} {k(rng.choice(['pins', 'x', 'y', 'pindirs']))}{self.comma()}{self.val(0, 31)}", False
        if op == "nop":
            return k("nop"), False
        # a raw word can be any encoding; in a version 0 program it must not be a v1 one (the
        # cross-check and StateMachine's RP2040 check take such a word for a v1 instruction)
        while True:
            w = rng.randint(0, 0xFFFF)
            if v1 or not is_v1_only(w):
                break
        return f"{k('.word')} {hex(w)}", True


def main():
    out_dir, seed, files = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    os.makedirs(out_dir, exist_ok=True)
    for f in os.listdir(out_dir):
        if f.endswith(".pio"):
            os.remove(os.path.join(out_dir, f))
    for n in range(files):
        rng = random.Random(seed * 1000 + n)
        lines = [f"; generated by gen_random_pio.py, seed {seed}, file {n}"]
        global_names = {}
        global_version = None
        if rng.random() < 0.2:
            global_version = 1
            lines.append(".pio_version 1")
        for _ in range(rng.randint(0, 2)):
            name = f"G{len(global_names)}"
            toks, v = Expr(rng, global_names).make(-30, 30)
            lines.append(
                f".define {'public ' if rng.random() < 0.5 else ''}{name} {text(toks)}")
            global_names[name] = v
        for p in range(rng.randint(1, 3)):
            lines += [""] + Program(rng, f"r{n}_{p}",
                                    global_names, global_version).generate()
        with open(os.path.join(out_dir, f"random_{n:03}.pio"), "w") as fh:
            fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
