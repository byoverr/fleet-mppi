"""Smoothing options against the plain noise bank: success, collisions, speed of successes and
turn-rate change per cycle (dw rms, rad/s), per regime, option and batch size. Fisher exact test
of success against the plain bank (same regime and K) and against stock MPPI K = 2000 if a
reference file with it is given (--ref hard_main.jsonl).

Usage: uv run --with scipy python cl_smooth_stats.py smooth.jsonl [--ref hard_main.jsonl]
"""

import json
import math
import statistics as st
import sys
from collections import defaultdict

from scipy.stats import fisher_exact

args = sys.argv[1:]
ref_file = None
if "--ref" in args:
    i = args.index("--ref")
    ref_file = args[i + 1]
    del args[i:i + 2]
TIGHT = {5, 6, 7, 8}


def regime(r):
    t = r["scene"] in TIGHT
    if r["vx_max"] > 0.75:
        return "tightfast" if t else "fast"
    return "tight" if t else "easy"


def option(r):
    p = ";".join(x for x in r.get("p", "").split(";") if x and not x.startswith("vx_std="))
    f = r.get("bank_file", "")
    bank = f"bank {f.rsplit('/', 1)[-1].split('_')[0]}" if f else "bank"
    v = "fresh" if r.get("regen") else (bank if r.get("bank") else "MPPI")
    return f"{v} {p}".strip()


def wilson(k, n, z=1.96):
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return 100 * (c - h), 100 * (c + h)


rows = [json.loads(x) for f in args for x in open(f) if x.strip()]
g = defaultdict(list)
for r in rows:
    g[(regime(r), option(r), r["batch"])].append(r)
ref = defaultdict(list)
if ref_file:
    for x in open(ref_file):
        r = json.loads(x)
        if not r.get("bank") and not r.get("regen") and r["batch"] == 2000:
            ref[regime(r)].append(r)
opts = sorted({k[1] for k in g}, key=lambda o: (o != "bank", o))
for reg in ("tight", "fast", "tightfast"):
    if not any(k[0] == reg for k in g):
        continue
    print(f"\n### {reg}")
    if ref[reg]:
        rr = ref[reg]
        s = sum(r["status"] == "success" for r in rr)
        print(f"reference: MPPI K=2000 {100 * s / len(rr):.0f} % (n={len(rr)}), "
              f"dw {st.mean(r['dw_rms'] for r in rr):.4f}")
    print("\n| option | K | success % [95 %] | col | speed | dw | p vs bank |")
    print("|---|---|---|---|---|---|---|")
    for k in sorted({k[2] for k in g if k[0] == reg}, reverse=True):
        base = [r["status"] == "success" for r in g.get((reg, "bank", k), [])]
        for o in opts:
            rs = g.get((reg, o, k), [])
            if not rs:
                continue
            s = [r["status"] == "success" for r in rs]
            lo, hi = wilson(sum(s), len(s))
            p = (fisher_exact([[sum(s), len(s) - sum(s)], [sum(base), len(base) - sum(base)]])[1]
                 if base and o != "bank" else float("nan"))
            ok = [r for r in rs if r["status"] == "success"]
            sp = st.mean(r["len"] / r["time"] for r in ok) if ok else float("nan")
            col = sum(r["status"] == "collision" for r in rs)
            print(f"| {o} | {k} | {100 * sum(s) / len(s):.0f} [{lo:.0f}–{hi:.0f}] (n={len(s)}) | "
                  f"{col} | {sp:.2f} | {st.mean(r['dw_rms'] for r in rs):.4f} | {p:.2g} |")
