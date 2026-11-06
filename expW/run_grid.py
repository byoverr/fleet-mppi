"""Grid of networked episodes. Speed comes from MPP_MAX_V in the environment (whole process).

Usage (from expW):
  MPP_MAX_V=0.22 uv run --project ../mpp python run_grid.py main out_v022.jsonl [workers=3]
  grids: main (MPPI, all strategies), dwa (DWA, S0-S2), fbmppi (S4 with onboard MPPI K=16),
         baseline (plain run_episode)
Results are appended as JSON lines; finished jobs are skipped on restart.
"""

from __future__ import annotations

import json
import multiprocessing as mp
import os
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed

STRATS = ["S0", "S1", "S2", "S3", "S4"]
SCEN = ["bypass", "blocked", "open"]


def links():
    L = []
    for d in (0.0, 0.05, 0.15, 0.3, 0.6):
        L.append(dict(delay=d))
    for p in (0.05, 0.2):
        L.append(dict(delay=0.05, loss=p))
    for g in (0.5, 2.0):
        L.append(dict(delay=0.05, ge_len=g, ge_frac=0.1))
    L.append(dict(delay=0.05, ev_len=2.0))  # 2 s dropout exactly when the box falls
    L.append(dict(delay=0.15, loss=0.05, ge_len=2.0, ge_frac=0.1))  # "bad Wi-Fi" combo
    return L


def seeds_for(link):
    random_link = link.get("loss", 0) or link.get("ge_len", 0)
    return [1, 2, 3, 4, 5, 6] if random_link else [1, 2, 3]


def jobs(grid):
    out = []
    if grid == "baseline":
        for sc in SCEN:
            for pl in ("mppi", "dwa"):
                for seed in (1, 2, 3):
                    out.append(dict(grid=grid, scenario=sc, planner=pl, seed=seed))
        return out
    for link in links():
        if grid == "main":
            strats, planner, fbp = STRATS, "mppi", "dwa"
        elif grid == "dwa":
            strats, planner, fbp = ["S0", "S1", "S2", "S3"], "dwa", "dwa"
        elif grid == "fbmppi":
            if not (link.get("ge_len") or link.get("ev_len")):
                continue
            strats, planner, fbp = ["S4"], "mppi", "mppi"
        elif grid == "rate":
            # server planning rate with the trajectory buffer
            if not (set(link) <= {"delay"} and link["delay"] in (0.0, 0.05, 0.15)) and not (
                    link.get("loss") and link.get("ge_len")):
                continue
            for sc in SCEN:
                for s in ("S1", "S2", "S3"):
                    for n, al in ((1, False), (2, False), (2, True), (4, False), (4, True)):
                        for seed in seeds_for(link):
                            out.append(dict(grid=grid, scenario=sc, planner="mppi", strategy=s,
                                            seed=seed, link=link, fb="dwa", every=n, aligned=al))
            continue
        else:
            raise SystemExit(f"unknown grid {grid}")
        for sc in SCEN:
            for s in strats:
                for seed in seeds_for(link):
                    out.append(dict(grid=grid, scenario=sc, planner=planner, strategy=s,
                                    seed=seed, link=link, fb=fbp))
    return out


def key(j):
    return json.dumps(j, sort_keys=True)


def run(job):
    from mpp.runner import EpisodeConfig, run_episode
    from netsim import LinkConfig, NetConfig, run_net_episode

    cfg = EpisodeConfig(scenario=job["scenario"], local_planner=job["planner"], seed=job["seed"])
    if job["grid"] == "baseline":
        r = run_episode(cfg)
        res = dict(scenario=r.scenario, planner=r.local_planner, seed=r.seed, strategy="base",
                   status=r.status, success=bool(r.success and r.collisions == 0),
                   collisions=r.collisions, min_gap=r.min_gap, time=r.sim_time)
    else:
        net = NetConfig(strategy=job["strategy"], link=LinkConfig(**job["link"]),
                        fallback_planner=job["fb"], server_every=job.get("every", 1),
                        shift_aligned=job.get("aligned", False))
        res = run_net_episode(cfg, net)
    res["grid"] = job["grid"]
    res["v"] = float(os.environ.get("MPP_MAX_V", "0.22"))
    res["job"] = key(job)
    return res


def main():
    grid, out = sys.argv[1], sys.argv[2]
    workers = int(sys.argv[3]) if len(sys.argv) > 3 else 3
    done = set()
    if os.path.exists(out):
        with open(out) as f:
            done = {json.loads(line)["job"] for line in f if line.strip()}
    todo = [j for j in jobs(grid) if key(j) not in done]
    print(f"{grid}: {len(todo)} jobs to run ({len(done)} done)", flush=True)
    with ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as ex, \
            open(out, "a") as f:
        futs = [ex.submit(run, j) for j in todo]
        for i, fu in enumerate(as_completed(futs), 1):
            f.write(json.dumps(fu.result()) + "\n")
            f.flush()
            if i % 50 == 0:
                print(f"{i} / {len(todo)}", flush=True)


if __name__ == "__main__":
    main()
