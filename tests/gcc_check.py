#!/usr/bin/env python3
"""Compiles the host test's sources with a gcc (-fsyntax-only, -Werror): the chip headers are also
built by gcc firmware trees, and gcc's constant evaluator and warnings differ from clang's.

Usage: gcc_check.py <compiler> <include dir>... -- <source>...
"""

import concurrent.futures
import os
import subprocess
import sys


def main():
    compiler = sys.argv[1]
    split = sys.argv.index("--")
    includes = [f"-I{d}" for d in sys.argv[2:split]]
    sources = sys.argv[split + 1:]
    flags = ["-std=c++26", "-Wall", "-Wextra",
             "-Wpedantic", "-Werror", "-fsyntax-only"]

    def one(src):
        r = subprocess.run([compiler, *flags, *includes, src],
                           capture_output=True, text=True)
        return src, r.returncode, r.stdout + r.stderr

    failed = 0
    with concurrent.futures.ThreadPoolExecutor(os.cpu_count() or 4) as pool:
        for src, rc, out in pool.map(one, sources):
            if rc != 0:
                failed += 1
                print(f"FAIL {src}\n{out[-3000:]}")
    print(f"{compiler}: {len(sources) - failed} of {len(sources)} files compile")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
