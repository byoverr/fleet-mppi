"""Markdown tables from res_*.jsonl. Usage (from expW): python3 analyze.py > tables.md"""

from __future__ import annotations

import glob
import json
import math
import statistics as stt
from collections import defaultdict

LINK_ORDER = [
    "d=0ms", "d=50ms", "d=150ms", "d=300ms", "d=600ms",
    "d=50ms p=5%", "d=50ms p=20%",
    "d=50ms GE0.5s/10%", "d=50ms GE2s/10%", "d=50ms drop@box2s",
    "d=150ms p=5% GE2s/10%",
]


def load():
    rows = []
    for fn in sorted(glob.glob("res_*.jsonl")):
        with open(fn) as f:
            rows += [json.loads(line) for line in f if line.strip()]
    return rows


def med(xs):
    xs = [x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return stt.median(xs) if xs else float("nan")


def mean(xs):
    xs = [x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return sum(xs) / len(xs) if xs else float("nan")


def fmt(x, nd=1):
    return "–" if isinstance(x, float) and math.isnan(x) else f"{x:.{nd}f}"


def table(rows, strategies, cell, title, note=""):
    groups = defaultdict(list)
    for r in rows:
        groups[(r["link"], r["strategy"])].append(r)
    links = [link for link in LINK_ORDER if any((link, s) in groups for s in strategies)]
    out = [f"**{title}**" + (f" — {note}" if note else ""), ""]
    out.append("| link | " + " | ".join(strategies) + " |")
    out.append("|---|" + "---|" * len(strategies))
    for link in links:
        cells = [cell(groups.get((link, s), [])) if groups.get((link, s)) else "" for s in strategies]
        out.append(f"| {link} | " + " | ".join(cells) + " |")
    return "\n".join(out) + "\n"


def c_success(g):
    k = sum(r["success"] for r in g)
    return f"{100 * k / len(g):.0f}% ({k}/{len(g)})"


def c_coll(g):
    runs = sum(r["collisions"] > 0 for r in g)
    ev = sum(r["collisions"] for r in g)
    return f"{runs}/{len(g)} ({ev})"


def c_time(g):
    ok = [r["time"] for r in g if r["success"]]
    return fmt(med(ok))


def c_gap(g):
    gaps = [r["min_gap"] for r in g]
    return f"{fmt(med(gaps), 3)} / {fmt(min(gaps), 3)}"


def c_stop(g):
    return fmt(mean([r["stopped_time"] for r in g]))


def c_bw(g):
    return f"{mean([r['up_Bps'] for r in g]) / 1000:.2f} / {mean([r['down_Bps'] for r in g]) / 1000:.2f}"


def main():
    rows = load()
    base = [r for r in rows if r["grid"] == "baseline"]
    print("## Baseline (plain run_episode)\n")
    print("| v | planner | scenario | success | time (median) | min gap (min) |")
    print("|---|---|---|---|---|---|")
    bg = defaultdict(list)
    for r in base:
        bg[(r["v"], r["planner"], r["scenario"])].append(r)
    for k in sorted(bg):
        g = bg[k]
        print(f"| {k[0]} | {k[1]} | {k[2]} | {sum(r['success'] for r in g)}/{len(g)} | "
              f"{fmt(med([r['time'] for r in g]))} | {fmt(min(r['min_gap'] for r in g), 3)} |")
    print()
    # sanity: d=0 vs baseline, same seed
    bmap = {(r["v"], r["planner"], r["scenario"], r["seed"]): r for r in base}
    mism = 0
    tot = 0
    for r in rows:
        if r["grid"] in ("main", "dwa") and r["link"] == "d=0ms":
            b = bmap.get((r["v"], r["planner"], r["scenario"], r["seed"]))
            if b is None:
                continue
            tot += 1
            if b["success"] != r["success"] or abs(b["time"] - (r["time"] if r["success"] else b["time"])) > 0.05 \
                    or abs(b["min_gap"] - r["min_gap"]) > 0.002:
                mism += 1
    print(f"Sanity d=0 vs baseline: {tot - mism}/{tot} runs identical (success, time ±0.05 s, min gap ±2 mm).\n")

    S5 = ["S0", "S1", "S2", "S3", "S4"]
    for grid, planner, strategies in (("main", "mppi", S5), ("dwa", "dwa", ["S0", "S1", "S2", "S3"])):
        for v in (0.22, 0.6):
            sub = [r for r in rows if r["grid"] == grid and r["v"] == v]
            if not sub:
                continue
            n_sc = sorted({r["scenario"] for r in sub})
            print(f"## {planner.upper()}, v_max = {v} m/s (scenarios: {', '.join(n_sc)})\n")
            print(table(sub, strategies, c_success, "Success (goal reached, final error ≤ 0.3 m, no contacts)"))
            print(table(sub, strategies, c_coll, "Runs with contacts / runs (total contact events)"))
            print(table(sub, strategies, c_time, "Time to goal, s (median over successful runs)"))
            print(table(sub, strategies, c_gap, "Min gap to obstacles, m (median / worst)"))
            print(table(sub, strategies, c_stop, "Time standing still before the end, s (mean)"))
            if grid == "main":
                print(table(sub, strategies, c_bw, "Link load, kB/s uplink / downlink (mean)"))
                fb = [r for r in sub if r["strategy"] == "S4"]
                s3 = [r for r in sub if r["strategy"] == "S3"]
                print(f"S3 safety check: {mean([r['safety_ms'] for r in s3 if r['safety_ms']]):.3f} ms mean per call; "
                      f"S4 onboard DWA: {mean([r['fb_ms'] for r in fb if r['fb_ms']]):.2f} ms per call, "
                      f"server MPPI: {mean([r['srv_ms'] for r in sub]):.2f} ms per call.\n")
            # per scenario success for the main grid
            if grid == "main":
                print(f"**Success by scenario (v = {v})**\n")
                print("| link | scenario | " + " | ".join(strategies) + " |")
                print("|---|---|" + "---|" * len(strategies))
                g = defaultdict(list)
                for r in sub:
                    g[(r["link"], r["scenario"], r["strategy"])].append(r)
                for link in LINK_ORDER:
                    for sc in ("bypass", "blocked", "open"):
                        if (link, sc, "S0") not in g:
                            continue
                        cells = []
                        for s in strategies:
                            gg = g.get((link, sc, s), [])
                            cells.append(f"{sum(r['success'] for r in gg)}/{len(gg)}" if gg else "")
                        print(f"| {link} | {sc} | " + " | ".join(cells) + " |")
                print()

    fbm = [r for r in rows if r["grid"] == "fbmppi"]
    if fbm:
        print("## S4 with onboard MPPI (K=16) instead of DWA\n")
        print("| v | link | S4/DWA success | S4/DWA time | S4/MPPI16 success | S4/MPPI16 time | MPPI16 gap worst | MPPI16 ms/call |")
        print("|---|---|---|---|---|---|---|---|")
        for v in (0.22, 0.6):
            for link in LINK_ORDER:
                a = [r for r in rows if r["grid"] == "main" and r["v"] == v and r["link"] == link and r["strategy"] == "S4"]
                b = [r for r in fbm if r["v"] == v and r["link"] == link]
                if not b:
                    continue
                print(f"| {v} | {link} | {c_success(a)} | {c_time(a)} | {c_success(b)} | {c_time(b)} | "
                      f"{fmt(min(r['min_gap'] for r in b), 3)} | {mean([r['fb_ms'] for r in b if r['fb_ms']]):.2f} |")
        print()


if __name__ == "__main__":
    main()
