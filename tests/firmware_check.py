#!/usr/bin/env python3
"""Compiles one test file the way a firmware tree compiles its code, and checks the outcome.

Usage: firmware_check.py <compile_commands.json> <file> (--expect REGEX | --ok)

StateMachine's checks (PioStateMachine.hpp) need the chip's generated registers and a board, so
they cannot be built on the host. This takes the compile command of test_examples'
22_pio_blink/main.cpp (release) from a configured firmware tree, compiles <file> with it
(-fsyntax-only, nothing written into the tree), and passes when the compiler fails with a
message matching REGEX (--expect) or succeeds without a warning (--ok).
"""

import json
import re
import shlex
import subprocess
import sys


def main():
    db, source = sys.argv[1], sys.argv[2]
    expect = sys.argv[4] if sys.argv[3] == "--expect" else None
    with open(db) as fh:
        entries = json.load(fh)
    entry = next((e for e in entries
                  if e["file"].endswith("22_pio_blink/main.cpp") and "_release" in e.get("output", e.get("command", ""))
                  and "_release_log" not in e.get("output", e.get("command", ""))), None)
    if entry is None:
        print(f"no 22_pio_blink release entry in {db}")
        return 1
    args = entry["arguments"] if "arguments" in entry else shlex.split(
        entry["command"])
    if "ccache" in args[0]:
        args = args[1:]
    out = []
    skip = False
    for a in args:
        if skip:
            skip = False
            continue
        if a in ("-o", "-MF", "-MT", "-MQ", "-c"):
            skip = True
            continue
        if a in ("-MD", "-MMD") or a == entry["file"] or a.endswith("main.cpp"):
            continue
        out.append(a)
    # a test file has no main(): StartUp.hpp's `startup` is then unused, which the firmware never sees
    cmd = out + ["-fsyntax-only", "-Wno-unneeded-internal-declaration", source]
    r = subprocess.run(
        cmd, cwd=entry["directory"], capture_output=True, text=True)
    text = r.stdout + r.stderr
    if expect is None:
        if r.returncode != 0 or "warning:" in text:
            print(text[-4000:])
            print("FAIL: expected a clean compile")
            return 1
        print("ok: compiles without a warning")
        return 0
    if r.returncode == 0:
        print("FAIL: it compiled, expected an error matching", expect)
        return 1
    if not re.search(expect, text):
        print(text[-4000:])
        print("FAIL: the error does not match", expect)
        return 1
    # type: ignore[union-attr]
    print("ok: fails with", re.search(expect, text).group(0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
