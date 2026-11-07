"""Identify the drive model from one logged run (scenario 'open', seed 101, 10 Hz, nominal
planner) and compare open-loop prediction error of the nominal and identified models on
other runs. Usage: MPP_MAX_V=0.6 uv run --project ../mpp python probes/identify_drive.py"""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from mpp.runner import EpisodeConfig
from netsim import NetConfig, LinkConfig, run_net_episode, SUB
from drive import identify, DriveModel, model_rollout

TRUE = (0.8, 0.5, 1.0, 0.7)

def log_run(sc, seed):
    r = run_net_episode(EpisodeConfig(scenario=sc, local_planner="mppi", seed=seed),
                        NetConfig(strategy="S3", link=LinkConfig(delay=0.0), plant=TRUE, log_drive=True))
    return r, np.array(r["drive_log"])

def main():
    r, log = log_run("open", 101)
    print("true      ", TRUE)
    models = {}
    for meth in ("arx", "oe"):
        mm, info = identify(log, SUB, meth)
        models[meth] = mm
        print(meth, "from closed-loop log:", tuple(round(x, 3) for x in mm.as_tuple()), {k: round(v, 4) if isinstance(v, float) else v for k, v in info.items()})
    m = models["oe"]
    # open-loop prediction error: from each logged state, play the next H s of commands
    out = dict(closed_loop_models={k: v.as_tuple() for k, v in models.items()}, true=TRUE, err={})
    for sc in ("bypass", "blocked"):
        _, lg = log_run(sc, 202)
        _, _ = None, None
        # positions are not in the drive log: rebuild them by integrating measured velocities
        t, vc, wc, vm, wm = lg.T
        pose = np.zeros((len(lg) + 1, 3))
        for k in range(len(lg)):
            x, y, th = pose[k]
            pose[k + 1] = (x + vm[k] * np.cos(th) * SUB, y + vm[k] * np.sin(th) * SUB, th + wm[k] * SUB)
        for H in (0.25, 0.5, 1.0):
            n = int(round(H / SUB))
            e = {"nominal": [], "arx": [], "oe": []}
            for k in range(0, len(lg) - n, 5):
                if abs(vm[k]) < 0.05 and abs(vc[k]) < 0.05:
                    continue
                for name, mod in (("nominal", None), ("arx", models["arx"]), ("oe", models["oe"])):
                    tr = model_rollout(tuple(pose[k]), vc[None, k:k + n], wc[None, k:k + n], SUB, (vm[k], wm[k]), mod)[0, -1]
                    e[name].append(np.hypot(*(tr[:2] - pose[k + n, :2])))
            row = {name: (round(float(np.median(v)), 4), round(float(np.percentile(v, 95)), 4)) for name, v in e.items()}
            out["err"][f"{sc} H={H}"] = row
            print(sc, f"H={H}s", "median / p95 position error, m:", row)
    with open("res_identify.json", "w") as f:
        json.dump(out, f, indent=1)

if __name__ == "__main__":
    main()
