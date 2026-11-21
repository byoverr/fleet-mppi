"""Factorial test of ARX placement, fixed MPPI sample count and matched seeds."""
import argparse
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from mpp.runner import EpisodeConfig
from netsim import LinkConfig, NetConfig, run_net_episode

ROOT = Path(__file__).resolve().parents[1]
FIT = json.loads((ROOT / "results/res_identify_pose.json").read_text())
ARX = FIT["closed_loop_models"]["arx"]

def run(job):
    seed, planner, onboard = job
    cfg = EpisodeConfig(scenario="blocked", local_planner="mppi", seed=seed, local_kwargs={"samples": 100})
    net = NetConfig(strategy="S3", link=LinkConfig(delay=0), plant=FIT["true"],
                    planner_model=ARX if planner else None, override_onboard_model=True,
                    onboard_model=ARX if onboard else None, log_drive=seed == 303)
    record = run_net_episode(cfg, net)
    row = {k:record[k] for k in ("success","collisions","status","min_gap","sim_time","srv_calls","safety_stops")}
    row.update(seed=seed, planner=bool(planner), onboard=bool(onboard))
    return row, record.get("pose_log")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--workers",type=int,default=2)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    jobs = itertools.product(range(301,317),(False,True),(False,True))
    with ProcessPoolExecutor(args.workers) as pool, (args.output/"ablation.jsonl").open("w") as out:
        for row,trace in pool.map(run,jobs):
            out.write(json.dumps(row)+"\n")
            if trace:
                path=args.output/f"route-303-{int(row['planner'])}{int(row['onboard'])}.json"
                path.write_text(json.dumps(trace))
