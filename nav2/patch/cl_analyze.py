"""Tables for the closed-loop sweep (closed_loop_benchmark JSON lines).

Usage: python3 cl_analyze.py a.jsonl [b.jsonl ...] [scenes]   (scenes e.g. "0234", default all)
Per variant and batch: success, collisions, timeouts, mean time of successes, min clearance,
command smoothness and CPU per cycle (median of the per-episode medians).
"""

import json
import statistics as st
import sys
from collections import defaultdict


def variant(r):
    v = []
    if r["lp"]:
        v.append(f"LP {r['lp']:g} Hz")
    if r["bias"] & 1:
        v.append("stop")
    if r["bias"] & 2:
        v.append("plan")
    if r.get("regen"):
        v.append("fresh noise")
    if r.get("bank") and not r.get("regen"):
        v.append(f"bank {r['bank']}")
    return " + ".join(v) or "MPPI"


def main():
    files = [a for a in sys.argv[1:] if a.endswith(".jsonl")]
    rest = [a for a in sys.argv[1:] if not a.endswith(".jsonl")]
    rows = [json.loads(x) for f in files for x in open(f) if x.strip()]
    scenes = rest[0] if rest else "".join(sorted({str(r["scene"]) for r in rows}))
    rows = [r for r in rows if str(r["scene"]) in scenes]
    g = defaultdict(list)
    for r in rows:
        g[(variant(r), r["batch"])].append(r)
    print(f"{len(rows)} episodes, scenes {scenes}\n")
    print("| variant | K | success | collisions | timeouts | time, s | min clear, cm | "
          "dw rms | CPU/cycle, us |")
    print("|---|---|---|---|---|---|---|---|---|")
    for (v, k) in sorted(g, key=lambda t: (t[0] != "MPPI", t[0], -t[1])):
        rs = g[(v, k)]
        ok = [r for r in rs if r["status"] == "success"]
        col = sum(r["status"] == "collision" for r in rs)
        to = sum(r["status"] in ("timeout", "novalid") for r in rs)
        tm = st.mean(r["time"] for r in ok) if ok else float("nan")
        mc = st.median(r["min_clear"] for r in rs) * 100
        dw = st.mean(r["dw_rms"] for r in rs)
        cpu = st.median(r["cpu_med"] for r in rs)
        print(f"| {v} | {k} | {len(ok)}/{len(rs)} | {col} | {to} | {tm:.1f} | {mc:.1f} | "
              f"{dw:.4f} | {cpu:.0f} |")
    print("\nPer scene: success / collisions\n")
    sc = sorted({r["scene"] for r in rows})
    print("| variant | K | " + " | ".join(f"scene {s}" for s in sc) + " |")
    print("|---|---|" + "---|" * len(sc))
    for (v, k) in sorted(g, key=lambda t: (t[0] != "MPPI", t[0], -t[1])):
        cells = []
        for s in sc:
            rs = [r for r in g[(v, k)] if r["scene"] == s]
            ok = sum(r["status"] == "success" for r in rs)
            col = sum(r["status"] == "collision" for r in rs)
            cells.append(f"{ok}/{len(rs)} / {col}")
        print(f"| {v} | {k} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main()
