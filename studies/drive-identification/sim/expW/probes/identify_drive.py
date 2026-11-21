"""Identify the drive model from one logged run (scenario 'open', seed 101, 10 Hz, nominal
planner) and compare open-loop prediction error of the nominal and identified models on
other runs. Usage: MPP_MAX_V=0.6 uv run --project ../mpp python probes/identify_drive.py"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from mpp.runner import EpisodeConfig
from netsim import NetConfig, LinkConfig, run_net_episode, SUB
from drive import identify, model_rollout

TRUE = (0.8, 0.5, 1.0, 0.7)

def log_run(sc, seed):
    r = run_net_episode(EpisodeConfig(scenario=sc, local_planner="mppi", seed=seed),
                        NetConfig(strategy="S3", link=LinkConfig(delay=0.0), plant=TRUE, log_drive=True))
    drive_log = np.asarray(r["drive_log"])
    pose_log = np.asarray(r["pose_log"])
    np.testing.assert_allclose(drive_log[:, 0], pose_log[:-1, 0], atol=1e-9, rtol=0.0)
    return drive_log, pose_log[:, 1:]

def main():
    log, _ = log_run("open", 101)
    print("true      ", TRUE)
    models = {}
    for meth in ("arx", "oe"):
        mm, info = identify(log, SUB, meth)
        models[meth] = mm
        print(meth, "from closed-loop log:", tuple(round(x, 3) for x in mm.as_tuple()), {k: round(v, 4) if isinstance(v, float) else v for k, v in info.items()})
    # Start each prediction from the measured state and compare against MuJoCo's body pose.
    out = dict(
        closed_loop_models={k: v.as_tuple() for k, v in models.items()},
        true=TRUE,
        reference="MuJoCo body pose",
        training={"scenario": "open", "seed": 101},
        err={},
    )
    candidates = {"nominal": None, **models}
    for sc in ("bypass", "blocked"):
        episodes = {seed: log_run(sc, seed) for seed in (202, 203, 204)}
        for H in (0.25, 0.5, 1.0):
            n = int(round(H / SUB))
            by_seed = {}
            for seed, (drive_log, pose) in episodes.items():
                _, vc, wc, vm, wm = drive_log.T
                errors = {name: [] for name in candidates}
                # Non-overlapping windows within an episode.
                for k in range(0, len(drive_log) - n + 1, n):
                    if abs(vm[k]) < 0.05 and abs(vc[k]) < 0.05:
                        continue
                    for name, model in candidates.items():
                        prediction = model_rollout(
                            tuple(pose[k]), vc[None, k:k + n], wc[None, k:k + n],
                            SUB, (vm[k], wm[k]), model,
                        )[0, -1]
                        delta = prediction - pose[k + n]
                        heading = abs(np.rad2deg(np.arctan2(np.sin(delta[2]), np.cos(delta[2]))))
                        errors[name].append((np.linalg.norm(delta[:2]), heading))
                by_seed[seed] = errors
            pooled = {
                name: np.concatenate([errors[name] for errors in by_seed.values()])
                for name in candidates
            }
            row = {}
            for name, samples in pooled.items():
                quantiles = np.percentile(samples, [50, 95], axis=0)
                row[name] = {
                    "position_m_median_p95": quantiles[:, 0].round(4).tolist(),
                    "heading_deg_median_p95": quantiles[:, 1].round(3).tolist(),
                }
            windows_by_seed = {seed: len(errors["nominal"]) for seed, errors in by_seed.items()}
            out["err"][f"{sc} H={H}"] = {"windows_by_seed": windows_by_seed, "models": row}
            print(sc, f"H={H}s", row, "windows by seed:", windows_by_seed)
    with open("res_identify_pose.json", "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)

if __name__ == "__main__":
    main()
