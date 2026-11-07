"""Adaptive server rate (res_adapt_*.jsonl) against the fixed rates of the rate grid
(res_rate_*.jsonl, naive warm-start shift). Same links, scenarios and seeds.

Usage: python3 analyze_adapt.py v06 v022
"""

import json
import statistics as st
import sys
from collections import defaultdict


def load(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def label(r):
    if r.get("adaptive"):
        a = r["adapt"]
        return f"adapt ≤{10 / a['max_every']:g} Hz, near {a['near']:g} m"
    return f"fixed {10 / r['server_every']:g} Hz"


def ep(r):
    return (r["link"], r["scenario"], r["seed"])


def summary(rs):
    ok = [r for r in rs if r["success"]]
    con = sum(r["collisions"] > 0 for r in rs)
    tm = st.median(r["time"] for r in ok) if ok else float("nan")
    calls = sum(r["srv_calls"] for r in rs)
    gap = min(r["min_gap"] for r in rs)
    return len(ok), len(rs), con, tm, calls, gap


def main():
    for tag in sys.argv[1:]:
        rows = [r for r in load(f"res_rate_{tag}.jsonl") if not r["shift_aligned"]]
        rows += load(f"res_adapt_{tag}.jsonl")
        v = rows[0]["v"]
        links = sorted({r["link"] for r in rows}, key=lambda s: (len(s), s))
        labels = sorted({label(r) for r in rows}, key=lambda s: (not s.startswith("fixed"), s))
        print(f"\n## v_max = {v} m/s\n")
        for s in ("S2", "S3"):
            g = defaultdict(list)
            for r in rows:
                if r["strategy"] == s:
                    g[label(r)].append(r)
            base = {ep(r): r for r in g["fixed 10 Hz"]}
            print(f"\n**{s}, all links together**\n")
            print("| server rate | success | contact episodes | median time, s | MPPI calls, % of 10 Hz | min gap, m |")
            print("|---|---|---|---|---|---|")
            for lab in labels:
                if not g[lab]:
                    continue
                ok, n, con, tm, calls, gap = summary(g[lab])
                # calls relative to the same episodes (link, scenario, seed) at 10 Hz
                bc = sum(base[ep(r)]["srv_calls"] for r in g[lab])
                print(f"| {lab} | {ok}/{n} | {con} | {tm:.1f} | {100 * calls / bc:.0f} % | {gap:.3f} |")
            print(f"\n**{s}, per link: success / contacts / median time / calls %**\n")
            print("| server rate | " + " | ".join(links) + " |")
            print("|---" * (len(links) + 1) + "|")
            for lab in labels:
                cells = []
                for ln in links:
                    rs = [r for r in g[lab] if r["link"] == ln]
                    if not rs:
                        cells.append("–")
                        continue
                    ok, n, con, tm, calls, _ = summary(rs)
                    bc = sum(base[ep(r)]["srv_calls"] for r in rs)
                    cells.append(f"{ok}/{n} / {con} / {tm:.1f} / {100 * calls / bc:.0f} %")
                print(f"| {lab} | " + " | ".join(cells) + " |")
        ad = [r for r in rows if r.get("adaptive")]
        if ad:
            tot = defaultdict(int)
            for r in ad:
                for k, c in r["why"].items():
                    tot[k] += c
            n = sum(tot.values())
            print("\nWhy the server planned (adaptive runs, share of ticks with fresh state): "
                  + ", ".join(f"{k} {100 * c / n:.1f} %" for k, c in sorted(tot.items(), key=lambda x: -x[1])))
            print(f"Cost of the check: {st.mean(r['check_ms'] for r in ad):.3f} ms per tick "
                  f"vs MPPI {st.median(r['srv_ms'] for r in ad):.2f} ms per call")


if __name__ == "__main__":
    main()
