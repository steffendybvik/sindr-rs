#!/usr/bin/env python3
"""Fail when a core file's line coverage drops below its floor.

    cargo llvm-cov --workspace --all-features --summary-only > coverage.txt
    python3 scripts/coverage_floor.py coverage.txt

Reads the `cargo llvm-cov` summary table. Each floor sits one or two points
under the coverage the file had when it was set, so noise doesn't fail a
build but a block of untested code does. Raise a floor when a file's
coverage rises; lower one only with a reason in the PR.
"""

import sys

# Line coverage (%) each file must keep.
FLOORS = {
    "sindr/src/stamp.rs": 94.0,
    "sindr/src/transient.rs": 95.0,
    "sindr/src/newton_raphson.rs": 95.0,
    "sindr/src/mna.rs": 94.0,
    "TOTAL": 92.0,
}


def line_coverage(path: str) -> dict[str, float]:
    out = {}
    with open(path, encoding="utf-8") as f:
        for row in f:
            cols = row.split()
            # Filename, then regions (3), functions (3), lines (3), branches (3).
            if len(cols) >= 10 and cols[9].endswith("%"):
                name = cols[0].replace("\\", "/")
                for key in FLOORS:
                    if name == key or name.endswith("/" + key):
                        out[key] = float(cols[9].rstrip("%"))
    return out


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    found = line_coverage(sys.argv[1])
    failed = False
    print("| file | lines | floor |")
    print("|---|---:|---:|")
    for name, floor in FLOORS.items():
        if name not in found:
            print(f"| {name} | missing | {floor:.1f}% |")
            failed = True
            continue
        mark = "" if found[name] >= floor else " ❌"
        failed |= found[name] < floor
        print(f"| {name} | {found[name]:.2f}% | {floor:.1f}%{mark} |")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
