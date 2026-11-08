"""Success rate with Wilson 95 % intervals and Fisher tests against plain MPPI at K = 2000,
scenes 0, 2, 3, 4 (scene 1, a box on the plan, is failed by every variant).

Usage: uv run --with scipy python cl_stats.py a.jsonl [b.jsonl ...]
"""

import json
import math
import sys
from collections import defaultdict

from scipy.stats import fisher_exact


def wilson(k, n, z=1.96):
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return 100 * (c - h), 100 * (c + h)


def name(lp, bias, regen, bank):
    v = ([f"LP {lp:g} Hz"] if lp else []) + (["stop + plan"] if bias else [])
    v += ["fresh noise"] if regen else []
    v += [f"bank {bank}"] if bank else []
    return " + ".join(v) or "MPPI"


rows = [json.loads(x) for f in sys.argv[1:] for x in open(f) if x.strip()]
rows = [r for r in rows if r["scene"] != 1]
g = defaultdict(list)
for r in rows:
    bank = 0 if r.get("regen") else r.get("bank", 0)
    g[(r["lp"], r["bias"], r.get("regen", 0), bank, r["batch"])].append(r["status"] == "success")
ref = g[(0, 0, 0, 0, 2000)]
variants = sorted({k[:4] for k in g})
print("| variant | " + " | ".join(f"K={k}" for k in (2000, 1000, 500, 250, 125, 64, 32)) + " |")
print("|---|" + "---|" * 7)
for lp, b, rg, bk in variants:
    cells = []
    for k in (2000, 1000, 500, 250, 125, 64, 32):
        s = g.get((lp, b, rg, bk, k), [])
        if not s:
            cells.append("–")
            continue
        lo, hi = wilson(sum(s), len(s))
        p = fisher_exact([[sum(s), len(s) - sum(s)], [sum(ref), len(ref) - sum(ref)]])[1]
        mark = "" if p >= 0.05 else " *"
        cells.append(f"{100 * sum(s) / len(s):.0f} % [{lo:.0f}–{hi:.0f}]{mark}")
    print(f"| {name(lp, b, rg, bk)} | " + " | ".join(cells) + " |")
print("\n* differs from MPPI K=2000 (Fisher exact, p < 0.05); n = 120 per cell unless noted")
pairs = [("LP 4 Hz", (4, 0, 0, 0), "MPPI", (0, 0, 0, 0)),
         ("bank", (0, 0, 0, 4096), "MPPI", (0, 0, 0, 0)),
         ("bank", (0, 0, 0, 4096), "LP 4 Hz", (4, 0, 0, 0)),
         ("LP 4 Hz + bank", (4, 0, 0, 4096), "bank", (0, 0, 0, 4096))]
for na, ka, nb, kb in pairs:
    for k in (250, 125, 64, 32):
        a, c = g.get(kb + (k,), []), g.get(ka + (k,), [])
        if not a or not c:
            continue
        p = fisher_exact([[sum(c), len(c) - sum(c)], [sum(a), len(a) - sum(a)]])[1]
        print(f"K={k}: {na} {sum(c)}/{len(c)} vs {nb} {sum(a)}/{len(a)}, Fisher p = {p:.2g}")
