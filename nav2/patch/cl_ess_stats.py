"""Effective sample size of the MPPI softmax weights (ESS = 1 / sum w^2, per control cycle) against
the turn-rate change per cycle (dw rms), per regime, noise kind and K.

If the jitter of the command is the Monte Carlo error of the weighted mean, dw should scale like
1 / sqrt(ESS): the last column, dw * sqrt(ESS), is then roughly constant across K.

Usage: python3 cl_ess_stats.py ess.jsonl [...]
"""

import json
import statistics as st
import sys
from collections import defaultdict

TIGHT = {5, 6, 7, 8}


def regime(r):
    t = r["scene"] in TIGHT
    if r["vx_max"] > 0.75:
        return "tightfast" if t else "fast"
    return "tight" if t else "easy"


def kind(r):
    if r.get("regen"):
        return "fresh"
    if r.get("bank"):
        f = r.get("bank_file", "")
        name = f.rsplit("/", 1)[-1].split("_")[0] if f else "bank"
        p = ";".join(x for x in r.get("p", "").split(";") if x and not x.startswith("vx_std="))
        return f"{name} {p}".strip()
    return "fixed"


g = defaultdict(list)
for f in sys.argv[1:]:
    for x in open(f):
        r = json.loads(x)
        if "ess_med" in r:
            g[(regime(r), kind(r), r["batch"])].append(r)
print("| regime | noise | K | success | ESS median | ESS / K | ESS p10 | dw | dw * sqrt(ESS) |")
print("|---|---|---|---|---|---|---|---|---|")
for key in sorted(g, key=lambda k: (k[0], k[1], -k[2])):
    rs = g[key]
    ess = st.mean(r["ess_med"] for r in rs)
    dw = st.mean(r["dw_rms"] for r in rs)
    s = sum(r["status"] == "success" for r in rs)
    print(f"| {key[0]} | {key[1]} | {key[2]} | {s}/{len(rs)} | {ess:.1f} | {ess / key[2]:.2f} | "
          f"{st.mean(r['ess_p10'] for r in rs):.1f} | {dw:.4f} | {dw * ess ** 0.5:.3f} |")
