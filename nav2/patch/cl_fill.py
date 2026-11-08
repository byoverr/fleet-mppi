"""Print the episodes of the hard sweep (cl_hard.sh) that are missing from a results file, one
shell command per line, so they can be (re)run in parallel:

  python3 cl_fill.py OUT "batches" seed0 seeds | xargs -P 6 -I{} bash -c {} >> OUT

Each line runs one episode with a 15-minute timeout. Use it after a crash or a hang, or to run a
regime that has not started yet.
"""

import json
import sys

out, batches, seed0, seeds = sys.argv[1], sys.argv[2].split(), int(sys.argv[3]), int(sys.argv[4])
B = "/ws/build/nav2_mppi_controller/benchmark/closed_loop_benchmark"
REGIMES = {  # name: (env, scenes) -- the same settings as cl_hard.sh
    "tight": ("VX_MAX=0.5", "5678"),
    "fast": ("VX_MAX=1.0 P=vx_std=0.4", "0234"),
    "tightfast": ("VX_MAX=1.0 P=vx_std=0.4", "5678"),
}
VARIANTS = {"mppi": "", "bank": "BANK=4096", "regen": "REGEN=1"}


def key(r):
    v = "regen" if r.get("regen") else ("bank" if r.get("bank") else "mppi")
    return (r["vx_max"] > 0.75, v, r["batch"], r["scene"], r["seed"])


try:
    done = {key(json.loads(x)) for x in open(out) if x.strip()}
except FileNotFoundError:
    done = set()
for env, scenes in REGIMES.values():
    fast = "VX_MAX=1.0" in env
    for v, venv in VARIANTS.items():
        for k in batches:
            for sc in scenes:
                for s in range(seed0, seed0 + seeds):
                    if (fast, v, int(k), int(sc), s) not in done:
                        print(f"source /ws/install/setup.bash; OMP_NUM_THREADS=1 {env} {venv} "
                              f"timeout -s KILL 900 {B} {k} 0 0 {s} 1 {sc} 0 2>/dev/null")
