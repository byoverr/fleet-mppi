"""B2: do the K / proposal conclusions survive in cluttered scenes?

Random clutter: tall boxes (seen by the lidar) placed in the free space from the start, at
least one feasible path kept. MPPI variants are compared on success, time and clearance.
Usage: uv run --project <lab>/mpp python b2_clutter.py <out.json> [workers] [n_scenes]
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
from mpp.world.scenarios import SCENARIOS, FallingBox, get_scenario

ROUTES = [("S1", "G1"), ("S3", "G3"), ("S2", "G4"), ("S1", "G2"), ("S3", "G2")]
VARIANTS = {
    "gauss K16": dict(samples=16),
    "gauss K64": dict(samples=64),
    "gauss K400": dict(samples=400),
    "gauss K400 wo6": dict(samples=400, w_obstacle=6.0),
    "log K16": dict(samples=16, sampler="lognormal"),
    "knots K16": dict(samples=16, sampler="knots"),
    "pursuit K16": dict(samples=16, ancillary="pursuit"),
    "pursuit K16 wo6": dict(samples=16, ancillary="pursuit", w_obstacle=6.0),
    "pursuit K64 wo6": dict(samples=64, ancillary="pursuit", w_obstacle=6.0),
}
SEEDS = [1, 2]


def make_scenes(n: int, seed: int = 7):
    rng = np.random.default_rng(seed)
    lay = warehouse()
    base = Costmap.from_layout(lay, robot_radius=0.105)
    scenes = []
    while len(scenes) < n:
        s, g = ROUTES[len(scenes) % len(ROUTES)]
        cm = base.copy()
        boxes = []
        p0 = make_global_planner("astar").plan(cm, lay.pose(s).xy, lay.pose(g).xy).path
        tries = 0
        while len(boxes) < 10 and tries < 400:
            tries += 1
            # sample near the nominal path so the clutter is actually in the way
            k = rng.integers(len(p0))
            c = p0[k] + rng.normal(0, 0.35, 2)
            w, h = rng.uniform(0.2, 0.45, 2)
            if any(np.hypot(*(lay.pose(q).xy - c)) < 0.7 for q in (s, g)):
                continue
            corners = np.array([[c[0] + a * w / 2, c[1] + b * h / 2] for a in (-1, 1) for b in (-1, 1)])
            if (cm.clearance_at(np.vstack([corners, c[None]])) < 0.02).any():
                continue
            test = cm.copy()
            test.fill_rect(c[0] - w / 2, c[1] - h / 2, c[0] + w / 2, c[1] + h / 2, layer="obstacles")
            test.update()
            # keep a margin: the path must exist for a slightly fatter robot
            test2 = test.copy()
            test2.inscribed += 0.03
            test2.update()
            if make_global_planner("astar").plan(test2, lay.pose(s).xy, lay.pose(g).xy).path is None:
                continue
            cm = test
            boxes.append((float(c[0]), float(c[1]), float(w), float(h)))
        scenes.append(dict(start=s, goal=g, boxes=boxes))
    return scenes


def register(i, sc):
    base = get_scenario("bypass")
    name = f"clutter_{i}"
    boxes = tuple(
        FallingBox(f"cb{i}_{j}", x, y, size=(w, h, 0.4), trigger_distance=100.0, drop_height=0.0, mass=8.0)
        for j, (x, y, w, h) in enumerate(sc["boxes"])
    )
    SCENARIOS[name] = dataclasses.replace(base, name=name, start=sc["start"], goal=sc["goal"], boxes=boxes)
    return name


def run(job):
    i, sc, label, seed = job
    name = register(i, sc)
    r = run_episode(
        EpisodeConfig(scenario=name, local_planner="mppi", seed=seed, local_kwargs=VARIANTS[label])
    )
    return dict(scene=i, label=label, seed=seed, ok=bool(r.success and r.collisions == 0),
                coll=int(r.collisions), status=r.status, t=float(r.sim_time), gap=float(r.min_gap),
                ms=float(r.local_ms), rec=int(r.recoveries), replans=int(r.replans))


def main():
    out = sys.argv[1]
    workers = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 20
    scenes = make_scenes(n)
    print("boxes per scene:", [len(s["boxes"]) for s in scenes], flush=True)
    jobs = [(i, sc, lab, s) for i, sc in enumerate(scenes) for lab in VARIANTS for s in SEEDS]
    rows = []
    with ProcessPoolExecutor(workers) as ex:
        for k, r in enumerate(ex.map(run, jobs, chunksize=1)):
            rows.append(r)
            if k % 50 == 0:
                print(k, "/", len(jobs), flush=True)
    json.dump(dict(scenes=scenes, rows=rows), open(out, "w"))


if __name__ == "__main__":
    main()
