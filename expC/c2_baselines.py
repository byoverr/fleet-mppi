"""C2: fixes and configuration baselines for the unseen far side of low obstacles.

Re-uses the C1 placements (all, or only those that ever failed). Conditions:
  none        hit-only marking (Nav2 obstacle layer, track_unknown_space=false)
  unknown     never-observed cells get a high soft cost (Nav2 track_unknown_space=true idea)
  safety      blanket: the controller keeps 12 cm instead of 2 cm (MPPI family only)
  shadow15    0.15 m shadow behind hits (inverse sensor model with obstacle thickness)
  unseen30    cells within 0.3 m of a sensed obstacle and never observed count as occupied
  oracle      whole true box marked as soon as any part is seen
Speed is set by MPP_MAX_V in the environment (whole process).
Usage: MPP_MAX_V=0.22 uv run --project <lab>/mpp python c2_baselines.py <c1.json> <out.json>
       <planners,comma> [workers] [only_failing:0/1]
"""

from __future__ import annotations

import json
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from c1_general import BOX, register
from mpp.runner import EpisodeConfig, run_episode

CONDS = ["none", "unknown", "safety", "shadow15", "unseen30", "oracle"]
SEEDS = [1, 2, 3]


def cfg_for(cond, planner):
    kw, lkw = {}, {}
    if cond == "unknown":
        kw["unknown_cost"] = 0.95
    elif cond == "shadow15":
        kw["obstacle_shadow"] = 0.15
    elif cond == "unseen30":
        kw["unseen_dilate"] = 0.3
    elif cond == "safety":
        if planner not in ("mppi", "log_mppi"):
            return None
        lkw["safety"] = 0.12
    return kw, lkw


def run(job):
    pl, planner, cond, seed = job
    c = cfg_for(cond, planner)
    if c is None:
        return None
    kw, lkw = c
    name = register(pl)
    x0, y0 = pl["x"] - BOX[0] / 2, pl["y"] - BOX[1] / 2
    x1, y1 = pl["x"] + BOX[0] / 2, pl["y"] + BOX[1] / 2

    def oracle(sim, nav):
        if cond != "oracle":
            return True
        cm = nav.cm
        ix, iy = cm.world_to_cell(np.array([[x0, y0], [x1 - 1e-6, y1 - 1e-6]]))
        win = cm.obstacles[iy[0] : iy[1] + 1, ix[0] : ix[1] + 1]
        if win.any() and not win.all():
            cm.fill_rect(x0, y0, x1, y1, layer="obstacles")
            cm.update()
        return True

    r = run_episode(
        EpisodeConfig(
            scenario=name, local_planner=planner, sensor="depth", seed=seed, local_kwargs=lkw, **kw
        ),
        on_step=oracle,
    )
    return dict(**pl, planner=planner, cond=cond, seed=seed,
                ok=bool(r.success and r.collisions == 0), coll=int(r.collisions), status=r.status,
                t=float(r.sim_time), gap=float(r.min_gap), rec=int(r.recoveries))


def main():
    c1, out, planners = sys.argv[1], sys.argv[2], sys.argv[3].split(",")
    workers = int(sys.argv[4]) if len(sys.argv) > 4 else 6
    only_failing = len(sys.argv) > 5 and sys.argv[5] == "1"
    rows = json.load(open(c1))
    keys = ("x", "y", "start", "goal", "turn")
    pls = {tuple(r[k] for k in keys): {k: r[k] for k in keys} for r in rows}
    if only_failing:
        bad = {tuple(r[k] for k in keys) for r in rows if not r["ok"]}
        pls = {k: v for k, v in pls.items() if k in bad}
    jobs = [(p, lp, c, s) for p in pls.values() for lp in planners for c in CONDS for s in SEEDS]
    print(len(pls), "placements,", len(jobs), "jobs", flush=True)
    res = []
    with ProcessPoolExecutor(workers) as ex:
        for i, r in enumerate(ex.map(run, jobs, chunksize=2)):
            if r is not None:
                res.append(r)
            if i % 200 == 0:
                print(i, "/", len(jobs), flush=True)
    json.dump(res, open(out, "w"))


if __name__ == "__main__":
    main()
