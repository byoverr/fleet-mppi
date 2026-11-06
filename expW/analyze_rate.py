"""Tables for the server planning rate grid (res_rate_*.jsonl): success / contact episodes /
median time to goal / server calls per episode, per strategy and server rate."""

import json
import statistics as st
import sys
from collections import defaultdict

RATES = [(1, False, "10 Hz"), (2, False, "5 Hz naive"), (2, True, "5 Hz aligned"),
         (4, False, "2.5 Hz naive"), (4, True, "2.5 Hz aligned")]


def load(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def main():
    for path in sys.argv[1:]:
        rows = load(path)
        v = rows[0]["v"]
        links = sorted({r["link"] for r in rows}, key=lambda s: (len(s), s))
        g = defaultdict(list)
        for r in rows:
            g[(r["strategy"], r["server_every"], r["shift_aligned"], r["link"])].append(r)
        print(f"\n## v_max = {v} m/s\n")
        print("Cell: success / episodes with contact / median time to goal, s (successful runs) / "
              "server MPPI calls per episode (median)\n")
        for s in ("S1", "S2", "S3"):
            print(f"\n**{s}**\n")
            print("| link | " + " | ".join(lab for _, _, lab in RATES) + " |")
            print("|---" * (len(RATES) + 1) + "|")
            for ln in links:
                cells = []
                for n, al, _ in RATES:
                    rs = g[(s, n, al, ln)]
                    if not rs:
                        cells.append("–")
                        continue
                    ok = [r for r in rs if r["success"]]
                    con = sum(r["collisions"] > 0 for r in rs)
                    tm = st.median(r["time"] for r in ok) if ok else float("nan")
                    calls = st.median(r["srv_calls"] for r in rs)
                    cells.append(f"{len(ok)}/{len(rs)} / {con} / {tm:.1f} / {calls:.0f}")
                print(f"| {ln} | " + " | ".join(cells) + " |")
        # totals per rate
        print("\n**All links together** (success / contact episodes / min gap, worst)\n")
        print("| strategy | " + " | ".join(lab for _, _, lab in RATES) + " |")
        print("|---" * (len(RATES) + 1) + "|")
        for s in ("S1", "S2", "S3"):
            cells = []
            for n, al, _ in RATES:
                rs = [r for ln in links for r in g[(s, n, al, ln)]]
                ok = sum(r["success"] for r in rs)
                con = sum(r["collisions"] > 0 for r in rs)
                gap = min(r["min_gap"] for r in rs)
                cells.append(f"{ok}/{len(rs)} / {con} / {gap:.3f}")
            print(f"| {s} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main()
