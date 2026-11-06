"""C1: is the low-box failure caused by partial marking, and does it generalise?

Box placements are generated around every turn of several routes (and on straight stretches as
a control). Conditions per run:
  none    camera only, Nav2-like marking (hits only)
  shadow  + 0.15 m shadow behind hits
  oracle  as soon as any cell of the box is marked, the whole true box footprint is marked
          (perfect object completion) -> isolates "partial marking" as the cause
Usage: uv run --project <lab>/mpp python c1_general.py <out.json> [workers]
"""

from __future__ import annotations

import dataclasses
import json
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from mpp.mapping.costmap import Costmap
from mpp.planning.global_planners import make_global_planner
from mpp.runner import EpisodeConfig, run_episode
from mpp.world.layout import warehouse
from mpp.world.scenarios import SCENARIOS, get_scenario

ROUTES = [("S1", "G1"), ("S3", "G3"), ("S1", "G4"), ("S3", "G2"), ("S2", "G1"), ("S1", "G2")]
BOX = (0.3, 0.3, 0.12)
PLANNERS = ["mppi", "log_mppi", "dwa", "rpp"]
CONDS = ["none", "shadow", "oracle"]
SEEDS = [1, 2]


def placements(max_per_route: int = 14, seed: int = 0):
    rng = np.random.default_rng(seed)
    lay = warehouse()
    cm = Costmap.from_layout(lay, robot_radius=0.105)
    out = []
    for s, g in ROUTES:
        p = make_global_planner("astar").plan(cm, lay.pose(s).xy, lay.pose(g).xy).path
        d = np.diff(p, axis=0)
        seg = np.hypot(d[:, 0], d[:, 1])
        s_acc = np.concatenate([[0], np.cumsum(seg)])
        head = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
        cand = []
        for i in range(len(p) - 1):
            # heading change over the next 0.8 m of path
            j = int(np.searchsorted(s_acc, s_acc[i] + 0.8))
            if j >= len(head):
                break
            turn = abs(head[j] - head[i]) > np.radians(55)
            for ahead in (0.5, 0.8):
                k = int(np.searchsorted(s_acc, s_acc[i] + ahead))
                if k >= len(p) - 1:
                    continue
                t = d[min(k, len(d) - 1)] / max(seg[min(k, len(d) - 1)], 1e-9)
                nrm = np.array([-t[1], t[0]])
                for off in (-0.18, 0.0, 0.18):
                    c = p[k] + off * nrm
                    cand.append((turn, float(c[0]), float(c[1]), s, g))
        turns = [c for c in cand if c[0]]
        straight = [c for c in cand if not c[0]]
        for group, n in ((turns, max_per_route), (straight, max_per_route // 2)):
            idx = rng.permutation(len(group))
            taken = 0
            for ii in idx:
                kind, x, y, s_, g_ = group[ii]
                if not valid(cm, lay, x, y, s_, g_):
                    continue
                if any(np.hypot(x - o["x"], y - o["y"]) < 0.25 for o in out):
                    continue
                out.append(dict(x=round(x, 3), y=round(y, 3), start=s_, goal=g_, turn=kind))
                taken += 1
                if taken >= n:
                    break
    return out


def valid(cm: Costmap, lay, x, y, s, g) -> bool:
    hx, hy = BOX[0] / 2, BOX[1] / 2
    # the box must not touch shelves/walls and must stay away from start and goal
    corners = np.array([[x + a * hx, y + b * hy] for a in (-1, 0, 1) for b in (-1, 0, 1)])
    if (cm.clearance_at(corners) < 0.03).any():
        return False
    for name in (s, g):
        if np.hypot(*(lay.pose(name).xy - np.array([x, y]))) < 0.8:
            return False
    test = cm.copy()
    test.fill_rect(x - hx, y - hy, x + hx, y + hy, layer="obstacles")
    test.update()
    return make_global_planner("astar").plan(test, lay.pose(s).xy, lay.pose(g).xy).path is not None


def register(pl):
    base = get_scenario("low_box")
    name = f"c1_{pl['x']}_{pl['y']}_{pl['start']}{pl['goal']}"
    box = dataclasses.replace(base.boxes[0], x=pl["x"], y=pl["y"], size=BOX)
    SCENARIOS[name] = dataclasses.replace(
        base, name=name, start=pl["start"], goal=pl["goal"], boxes=(box,)
    )
    return name


def run(job):
    pl, planner, cond, seed = job
    name = register(pl)
    x0, y0 = pl["x"] - BOX[0] / 2, pl["y"] - BOX[1] / 2
    x1, y1 = pl["x"] + BOX[0] / 2, pl["y"] + BOX[1] / 2
    seen_t = [None]

    def oracle(sim, nav):
        cm = nav.cm
        ix, iy = cm.world_to_cell(np.array([[x0, y0], [x1 - 1e-6, y1 - 1e-6]]))
        win = cm.obstacles[iy[0] : iy[1] + 1, ix[0] : ix[1] + 1]
        if win.any():
            if seen_t[0] is None:
                seen_t[0] = sim.time
            if cond == "oracle" and not win.all():
                cm.fill_rect(x0, y0, x1, y1, layer="obstacles")
                cm.update()
        return True

    r = run_episode(
        EpisodeConfig(
            scenario=name,
            local_planner=planner,
            sensor="depth",
            seed=seed,
            obstacle_shadow=0.15 if cond == "shadow" else 0.0,
        ),
        on_step=oracle,
    )
    return dict(
        **pl,
        planner=planner,
        cond=cond,
        seed=seed,
        ok=bool(r.success and r.collisions == 0),
        coll=int(r.collisions),
        status=r.status,
        t=float(r.sim_time),
        gap=float(r.min_gap),
        seen=seen_t[0],
    )


def main():
    out, workers = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 6
    pls = placements()
    print(len(pls), "placements;", sum(p["turn"] for p in pls), "at turns", flush=True)
    jobs = [(p, lp, c, s) for p in pls for lp in PLANNERS for c in CONDS for s in SEEDS]
    rows = []
    with ProcessPoolExecutor(workers) as ex:
        for i, r in enumerate(ex.map(run, jobs, chunksize=2)):
            rows.append(r)
            if i % 200 == 0:
                print(i, "/", len(jobs), flush=True)
    json.dump(rows, open(out, "w"))


if __name__ == "__main__":
    main()
