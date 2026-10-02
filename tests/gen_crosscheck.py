#!/usr/bin/env python3
"""Writes one C++ file per .pio file that compares parse() with pioasm's header of the same text.

Usage: gen_crosscheck.py <out dir> <label>=<root> ...   (every .pio below each root, build* dirs left out)

For each file it writes <out dir>/xc_<id>.cpp: the text #embed-ded, parse()d once per .program, and
static_asserts that the instructions, wrap, version, side-set, origin, the other directives'
fields and every public define and label equal what pioasm (-o kvasir -v 0) wrote. On stdout, one
line per file for CMake: "<id>\t<path>\t<stem>". <out dir>/crosscheck_summary.hpp lists what was
compared, for the test program to print.

The text is read here only to find the program names and public symbols; the assembling is done
by the compiler (parse()) and by pioasm.
"""

import os
import re
import sys


def strip(text):
    """The text without comments and % { %} blocks, line numbers kept."""
    out = []
    in_block_comment = False
    in_code_block = False
    for line in text.split("\n"):
        if in_code_block:
            if re.fullmatch(r"[ \t\r]*%}[ \t\r]*", line):
                in_code_block = False
            out.append("")
            continue
        s = ""
        i = 0
        while i < len(line):
            if in_block_comment:
                j = line.find("*/", i)
                if j < 0:
                    i = len(line)
                    break
                in_block_comment = False
                i = j + 2
                continue
            c = line[i]
            if c == ";" or line.startswith("//", i):
                break
            if line.startswith("/*", i):
                in_block_comment = True
                i += 2
                continue
            if c == "%":
                in_code_block = True
                break
            s += c
            i += 1
        out.append(s)
    return out


def programs_of(text):
    """[(program, [public defines], [public labels])], and the global public defines."""
    progs = []
    globals_ = []
    for line in strip(text):
        m = re.match(r"\s*\.program\s+(\w+)", line, re.I)
        if m:
            progs.append((m.group(1), [], []))
            continue
        m = re.match(r"\s*\.define\s+(?:public\s+|\*\s*)(\w+)", line, re.I)
        if m:
            (progs[-1][1] if progs else globals_).append(m.group(1))
            continue
        m = re.match(r"\s*(?:public\s+|\*\s*)(\w+)\s*:", line, re.I)
        if m and progs:
            progs[-1][2].append(m.group(1))
    return progs, globals_


# Programs pioasm assembles but no Kvasir header can hold, with the reason. The rest of the file
# is still compared, through a copy of the text without them; the test program lists them.
GPIO_ABOVE_31 = ("wait gpio above 31: pioasm keeps bit 5 of the GPIO number above the 16-bit word "
                 "for pico-sdk's loader; kvasir_output refuses it, parse() too (use the GPIO window)")
NO_INSTRUCTIONS = ("no instructions: kvasir_output refuses it (it wrote `std::array Instructions{}`, "
                   "which does not compile), parse() too")
LEFT_OUT = {
    ("tools/pioasm/test/amethyst.pio", "bar"): NO_INSTRUCTIONS,
    ("tools/pioasm/test/amethyst.pio", "wibble2"): GPIO_ABOVE_31,
    ("tools/pioasm/test/amethyst.pio", "wibble3"): NO_INSTRUCTIONS,
    ("tools/pioasm/test/amethyst.pio", "wee2"): GPIO_ABOVE_31,
}


def without_programs(text, names):
    """The text with the named programs' lines (from .program to the next one) left out."""
    out = []
    skipping = False
    for line, code in zip(text.split("\n"), strip(text)):
        m = re.match(r"\s*\.program\s+(\w+)", code, re.I)
        if m:
            skipping = m.group(1) in names
        out.append("" if skipping else line)   # blank, so line numbers stay
    return "\n".join(out)


def ident(s):
    return re.sub(r"[^A-Za-z0-9_]", "_", s)


def main():
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    for f in os.listdir(out_dir):   # the files of .pio files that are gone
        if f.startswith("xc_") and f.endswith(".cpp"):
            os.remove(os.path.join(out_dir, f))
    files = []
    for arg in sys.argv[2:]:
        label, root = arg.split("=", 1)
        if not os.path.isdir(root):
            print(
                f"-- pio cross-check: {root} does not exist, left out", file=sys.stderr)
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = sorted(
                d for d in dirnames if not d.startswith("build") and d != ".git")
            for f in sorted(filenames):
                if f.endswith(".pio"):
                    path = os.path.join(dirpath, f)
                    files.append(
                        (label + "_" + ident(os.path.relpath(path, root)[:-4]), os.path.abspath(path)))

    summary = []
    left_out = []
    for id_, path in files:
        with open(path, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        stem = os.path.splitext(os.path.basename(path))[0]
        dropped = {prog: why for (
            suffix, prog), why in LEFT_OUT.items() if path.endswith("/" + suffix)}
        if dropped:
            left_out += [f"{path}: {prog}: {why}" for prog,
                         why in dropped.items()]
            text = without_programs(text, dropped)
            os.makedirs(os.path.join(out_dir, id_), exist_ok=True)
            path = os.path.join(out_dir, id_, stem + ".pio")
            with open(path, "w") as fh:
                fh.write(text)
        progs, globals_ = programs_of(text)
        target = "xc_" + id_
        clock_div = re.search(r"^\s*\.clock_div",
                              "\n".join(strip(text)), re.I | re.M) is not None
        cpp = [
            f"// Generated by gen_crosscheck.py from {path}.",
            f"// parse() of the text against pioasm's header of it ({target}/{stem}.hpp).",
            f'#include "{target}/{stem}.hpp"',
            '#include "crosscheck_support.hpp"',
            "",
            f"namespace {target} {{",
            "#if defined(__clang__)",
            '_Pragma("clang diagnostic ignored \\"-Wc23-extensions\\"")   // #embed in C++',
            "#endif",
        ]
        for name, defines, labels in progs:
            where = f"{os.path.basename(path)}: {name}"
            cpp += [
                "",
                "// clang-format off",
                f"struct Ours_{name} : Kvasir::Pio::Program<Kvasir::Pio::parse({{",
                f'#embed "{path}"',
                f'}}, {{}}, "{name}")> {{}};',
                "// clang-format on",
                f"using Theirs_{name} = Kvasir::Pio::{name}Programm;",
                f'static_assert(crosscheck::sameWords<Ours_{name}, Theirs_{name}>(), "{where}: instructions");',
                f'static_assert(crosscheck::sameWrap<Ours_{name}, Theirs_{name}>(), "{where}: wrap");',
                f'static_assert(crosscheck::sameVersion<Ours_{name}, Theirs_{name}>(), "{where}: PIO version");',
                f'static_assert(crosscheck::v1Consistent<Ours_{name}, Theirs_{name}>(), "{where}: a v1 word in a version 0 program");',
                f'static_assert(crosscheck::sameSideset<Ours_{name}, Theirs_{name}>(), "{where}: side-set");',
                f'static_assert(crosscheck::sameOrigin<Ours_{name}, Theirs_{name}>(), "{where}: origin");',
                f'static_assert(crosscheck::sameDirectives<Ours_{name}, Theirs_{name}, {"false" if clock_div else "true"}>(), "{where}: directives");',
            ]
            for d in defines:
                cpp.append(
                    f'static_assert(Ours_{name}::define("{d}") == Theirs_{name}::{d}, "{where}: .define {d}");')
            for l in labels:
                cpp.append(
                    f'static_assert(Ours_{name}::offset("{l}") == Theirs_{name}::offset_{l}, "{where}: label {l}");')
            for d in globals_:
                cpp.append(
                    f'static_assert(Ours_{name}::define("{d}") == Kvasir::Pio::{d}, "{where}: global .define {d}");')
        cpp += ["}", ""]
        with open(os.path.join(out_dir, target + ".cpp"), "w") as fh:
            fh.write("\n".join(cpp))
        summary.append((path, [p[0] for p in progs]))
        print(f"{id_}\t{path}\t{stem}")

    n = sum(len(p) for _, p in summary)
    lines = [
        "// Generated by gen_crosscheck.py: what the cross-check compares.",
        "#pragma once",
        "#include <string_view>",
        "namespace crosscheck {",
        f"inline constexpr int Files = {len(summary)};",
        f"inline constexpr int Programs = {n};",
        "inline constexpr std::string_view Compared[] = {",
    ]
    for path, progs in summary:
        lines.append(
            f'    "{path}: {", ".join(progs) if progs else "(no program)"}",')
    lines += ["};", "inline constexpr std::string_view LeftOut[] = {"]
    for l in left_out or ["(none)"]:
        lines.append("    " + '"' + l.replace('"', "'") + '",')
    lines += ["};", "}", ""]
    with open(os.path.join(out_dir, "crosscheck_summary.hpp"), "w") as fh:
        fh.write("\n".join(lines))


if __name__ == "__main__":
    main()
