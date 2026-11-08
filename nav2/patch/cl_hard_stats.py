"""Success per regime, variant and batch for the hard closed-loop runs (cl_hard.sh), with Wilson
95 % intervals and Fisher tests against plain MPPI at K = 2000 in the same regime.

Regimes: tight = scenes 5-8 at vx_max 0.5; fast = scenes 0,2,3,4 at vx_max 1.0 (vx_std 0.4);
tightfast = scenes 5-8 at vx_max 1.0.

Usage: uv run --with scipy python cl_hard_stats.py a.jsonl [b.jsonl ...]
"""

import json
import math
import statistics as st
import sys
from collections import defaultdict

from scipy.stats import fisher_exact

TIGHT = {5, 6, 7, 8}


def wilson(k, n, z=1.96):
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return 100 * (c - h), 100 * (c + h)


def regime(r):
    t = r["scene"] in TIGHT
    if r["vx_max"] > 0.75:
        return "tightfast" if t else "fast"
    return "tight" if t else "easy"


def variant(r):
    if r.get("regen"):
        return "fresh noise"
    return f"bank {r['bank']}" if r.get("bank") else "MPPI"


rows = [json.loads(x) for f in sys.argv[1:] for x in open(f) if x.strip()]
g = defaultdict(list)
for r in rows:
    g[(regime(r), variant(r), r["batch"])].append(r)
batches = sorted({r["batch"] for r in rows}, reverse=True)
for reg in ("tight", "fast", "tightfast"):
    if not any(k[0] == reg for k in g):
        continue
    ref = [r["status"] == "success" for r in g[(reg, "MPPI", 2000)]]
    print(f"\n### {reg}\n")
    print("| variant | " + " | ".join(f"K={k}" for k in batches) + " |")
    print("|---|" + "---|" * len(batches))
    for v in ("MPPI", "bank 4096", "fresh noise"):
        cells = []
        for k in batches:
            rs = g.get((reg, v, k), [])
            if not rs:
                cells.append("–")
                continue
            s = [r["status"] == "success" for r in rs]
            lo, hi = wilson(sum(s), len(s))
            p = fisher_exact([[sum(s), len(s) - sum(s)], [sum(ref), len(ref) - sum(ref)]])[1]
            col = sum(r["status"] == "collision" for r in rs)
            mark = " *" if p < 0.05 and sum(s) / len(s) < sum(ref) / len(ref) else ""
            cells.append(f"{100 * sum(s) / len(s):.0f} [{lo:.0f}–{hi:.0f}]{mark} c{col}")
        print(f"| {v} | " + " | ".join(cells) + " |")
    print("\nper scene (success/n), speed of successes m/s, dw rms:\n")
    for v in ("MPPI", "bank 4096", "fresh noise"):
        for k in batches:
            rs = g.get((reg, v, k), [])
            if not rs:
                continue
            sc = sorted({r["scene"] for r in rs})
            per = " ".join(
                f"s{s}:{sum(r['status'] == 'success' for r in rs if r['scene'] == s)}/"
                f"{sum(r['scene'] == s for r in rs)}" for s in sc)
            ok = [r for r in rs if r["status"] == "success"]
            sp = st.mean(r["len"] / r["time"] for r in ok) if ok else float("nan")
            dw = st.mean(r["dw_rms"] for r in rs)
            print(f"{v:12s} K={k:<5d} {per}  speed {sp:.2f}  dw {dw:.4f}")
print("\n* lower than MPPI K=2000 in the same regime (Fisher exact, p < 0.05); cN = collisions")
