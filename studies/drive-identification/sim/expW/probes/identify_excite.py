"""Open-loop identification with a random step (PRBS-like) excitation on the true drive,
compared with identification from closed-loop navigation logs (identify_drive.py).
Usage: MPP_MAX_V=0.6 uv run --project ../mpp python probes/identify_excite.py"""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from mpp.runner import EpisodeConfig, make_episode
from mpp.world.layout import Layout, Pose2D
from netsim import SUB, _substep
from drive import DriveModel, LaggedDrive, identify

TRUE = (0.8, 0.5, 1.0, 0.7)

def excite(seed, dur=30.0, hold=(0.6, 2.0)):
    rng = np.random.default_rng(seed)
    # an empty 30 x 30 m hall, so the excitation never touches a wall
    c = Pose2D("c", 15.0, 15.0, 0.0)
    hall = Layout(30.0, 30.0, [], starts={n: c for n in ("S1", "S2", "S3")},
                  goals={n: c for n in ("G1", "G2", "G3", "G4")})
    *_, sim, _, _ = make_episode(EpisodeConfig(scenario="open", local_planner="mppi", seed=seed), hall)
    drive = LaggedDrive(DriveModel(*TRUE), SUB)
    log, t, nxt, cmd = [], 0.0, 0.0, (0.0, 0.0)
    while t < dur:
        if t >= nxt:
            cmd = (float(rng.choice([0.0, 0.15, 0.3, 0.45])), float(rng.choice([-1.5, -0.7, 0.0, 0.7, 1.5])))
            nxt = t + rng.uniform(*hold)
        v, w = sim.velocity()
        log.append((t, cmd[0], cmd[1], v, w))
        sim.set_cmd(*drive(*cmd))
        _substep(sim, update_boxes=False)
        t += SUB
    assert sim.contacts.events == 0, "the excitation touched something"
    return np.array(log)

def main():
    res = {}
    for seed in (1, 2, 3):
        lg = excite(seed)
        for meth in ("arx", "oe"):
            m, info = identify(lg, SUB, meth)
            res[f"{seed} {meth}"] = dict(model=[round(x, 3) for x in m.as_tuple()], **{k: round(v, 4) for k, v in info.items() if k != "n"})
            print("seed", seed, meth, res[f"{seed} {meth}"])
    print("true", TRUE)
    with open("res_identify_excite.json", "w") as f:
        json.dump(dict(true=TRUE, runs=res), f, indent=1)

if __name__ == "__main__":
    main()
