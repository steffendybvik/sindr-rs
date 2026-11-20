#!/usr/bin/env python3
"""Compare two criterion baselines and fail on a slowdown.

    cargo bench -p sindr --bench transient --bench newton -- --quick --save-baseline base
    # ...change the code...
    cargo bench -p sindr --bench transient --bench newton -- --quick --save-baseline head
    python3 scripts/bench_compare.py base head [--threshold 25]

Reads target/criterion/<group>/<case>/<baseline>/estimates.json and compares
median times. Exits 1 if any case present in both is more than THRESHOLD %
slower. Cases in only one baseline are listed and skipped.
"""

import argparse
import json
import sys
from pathlib import Path


def medians(root: Path, baseline: str) -> dict[str, float]:
    out = {}
    for est in root.glob(f"**/{baseline}/estimates.json"):
        case = est.parent.parent.relative_to(root).as_posix()
        out[case] = json.loads(est.read_text())["median"]["point_estimate"]
    return out


def fmt(ns: float) -> str:
    for unit, scale in (("s", 1e9), ("ms", 1e6), ("µs", 1e3)):
        if ns >= scale:
            return f"{ns / scale:.3g} {unit}"
    return f"{ns:.3g} ns"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("base")
    ap.add_argument("head")
    ap.add_argument("--threshold", type=float, default=25.0, help="percent")
    ap.add_argument("--dir", default="target/criterion")
    args = ap.parse_args()

    root = Path(args.dir)
    base, head = medians(root, args.base), medians(root, args.head)
    shared = sorted(base.keys() & head.keys())
    if not shared:
        print("no benchmark appears in both baselines; nothing to compare")
        return 0

    regressed = []
    print(f"| case | {args.base} | {args.head} | change |")
    print("|---|---:|---:|---:|")
    for case in shared:
        change = (head[case] / base[case] - 1) * 100
        flag = " ⚠️" if change > args.threshold else ""
        print(f"| {case} | {fmt(base[case])} | {fmt(head[case])} | {change:+.1f}%{flag} |")
        if change > args.threshold:
            regressed.append(case)
    for case in sorted(base.keys() ^ head.keys()):
        print(f"\nonly in one baseline, skipped: {case}")

    if regressed:
        print(f"\n{len(regressed)} case(s) more than {args.threshold:g}% slower: {', '.join(regressed)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
