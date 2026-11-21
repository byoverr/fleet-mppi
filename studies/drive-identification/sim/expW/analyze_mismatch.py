"""Model-mismatch grid (res_mismatch_v06.jsonl): planner model x server rate.

Usage: python3 analyze_mismatch.py res_mismatch_v06.jsonl
"""

import json
import statistics as st
import sys
from collections import defaultdict

MODELS = ["nominal", "arx", "oe"]
RATES = ["10 Hz", "5 Hz", "2.5 Hz", "adapt"]


def rate(r):
    return "adapt" if r.get("adaptive") else f"{10 / r['server_every']:g} Hz"


def ep(r):
    return (r["link"], r["scenario"], r["seed"])


def main():
    rows = [json.loads(line) for line in open(sys.argv[1]) if line.strip()]
    g = defaultdict(list)
    for r in rows:
        g[(r["model_name"], rate(r))].append(r)
    base = {ep(r): r for r in g[("nominal", "10 Hz")]}
    links = sorted({r["link"] for r in rows}, key=lambda s: (len(s), s))
    print(f"{len(rows)} episodes, plant {rows[0]['plant']}, v = {rows[0]['v']} m/s, strategy S3\n")
    print("| model | rate | success | contact episodes | median time, s | MPPI calls, % of nominal 10 Hz | "
          "plan error mean / p95, cm | adaptive: dev share |")
    print("|---|---|---|---|---|---|---|---|")
    for m in MODELS:
        for rt in RATES:
            rs = g[(m, rt)]
            if not rs:
                continue
            ok = [r for r in rs if r["success"]]
            con = sum(r["collisions"] > 0 for r in rs)
            tm = st.median(r["time"] for r in ok) if ok else float("nan")
            bc = sum(base[ep(r)]["srv_calls"] for r in rs if ep(r) in base)
            calls = sum(r["srv_calls"] for r in rs if ep(r) in base)
            pe = [r["plan_err"] for r in rs if r["plan_err"] is not None]
            p95 = [r["plan_err95"] for r in rs if r["plan_err95"] is not None]
            dev = ""
            if rt == "adapt":
                tot = defaultdict(int)
                for r in rs:
                    for k, c in r["why"].items():
                        tot[k] += c
                plan = sum(c for k, c in tot.items() if k != "skip")
                dev = f"{100 * tot['dev'] / max(plan, 1):.0f} % of plans"
            print(f"| {m} | {rt} | {len(ok)}/{len(rs)} | {con} | {tm:.1f} | {100 * calls / bc:.0f} % | "
                  f"{100 * st.mean(pe):.1f} / {100 * st.mean(p95):.1f} | {dev} |")
    print("\nPer link: success / contact episodes / median time\n")
    print("| model | rate | " + " | ".join(links) + " |")
    print("|---" * (len(links) + 2) + "|")
    for m in MODELS:
        for rt in RATES:
            cells = []
            for ln in links:
                rs = [r for r in g[(m, rt)] if r["link"] == ln]
                ok = [r for r in rs if r["success"]]
                con = sum(r["collisions"] > 0 for r in rs)
                tm = st.median(r["time"] for r in ok) if ok else float("nan")
                cells.append(f"{len(ok)}/{len(rs)} / {con} / {tm:.1f}")
            print(f"| {m} | {rt} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main()
