"""Paired closed-loop experiment. Run from the project containing data/ and code/."""
import itertools
import json
import random
from pathlib import Path

from mpp.runner import EpisodeConfig
from netsim import LinkConfig, NetConfig, run_net_episode

data = Path(__file__).resolve().parents[1] / "results"
calibration = json.loads((data / "res_identify_pose.json").read_text())
models = {"nominal": None, **calibration["closed_loop_models"]}
jobs = list(itertools.product(("bypass", "blocked"), range(301, 305), models, (100, 400)))
random.Random(20261120).shuffle(jobs)
with (data / "closed_loop.jsonl").open("w") as out:
    for scene, seed, name, size in jobs:
        cfg = EpisodeConfig(scenario=scene, local_planner="mppi", seed=seed, local_kwargs={"samples": size})
        net = NetConfig(strategy="S3", link=LinkConfig(delay=0), plant=calibration["true"], planner_model=models[name])
        record = run_net_episode(cfg, net)
        fields = ("success", "collisions", "min_gap", "sim_time", "srv_ms", "srv_calls", "status")
        result = {key: record[key] for key in fields}
        result.update(scene=scene, seed=seed, model=name, K=size)
        out.write(json.dumps(result, allow_nan=False) + "\n")
        out.flush()
