"""Print one shell command per closed-loop episode for a grid of settings, skipping episodes that
are already in the results file (so a crashed or interrupted run can simply be restarted):

  python3 cl_grid.py OUT "batches" seed0 seeds "regimes" CONF [CONF ...] \
    | xargs -P 6 -I{} bash -c {} >> OUT

regimes: any of tight, fast, tightfast (see cl_hard.sh).
CONF: "ENV|PARAMS", e.g. "BANK=4096|gamma=0.05;sgf_order=1". ENV goes to the environment,
PARAMS are optimizer parameters added to the regime's own (the P variable of the benchmark).
"""

import json
import sys

out, batches, seed0, seeds, regimes = sys.argv[1:6]
confs = sys.argv[6:]
B = "/ws/build/nav2_mppi_controller/benchmark/closed_loop_benchmark"
REGIMES = {  # name: (vx_max, own params, scenes) -- as in cl_hard.sh
    "tight": ("0.5", "", "5678"),
    "fast": ("1.0", "vx_std=0.4", "0234"),
    "tightfast": ("1.0", "vx_std=0.4", "5678"),
}


def key(r):
    return (r["vx_max"], r.get("bank", 0), r.get("regen", 0), r.get("temp", 0.3), r.get("p", ""),
            r.get("bank_file", ""),
            r["batch"], r["scene"], r["seed"])


done = set()
try:
    done = {key(json.loads(x)) for x in open(out) if x.strip()}
except FileNotFoundError:
    pass
for reg in regimes.split():
    vx, own, scenes = REGIMES[reg]
    for conf in confs:
        env, _, extra = conf.partition("|")
        p = ";".join(x for x in (own, extra) if x)
        e = dict(x.split("=", 1) for x in env.split())
        for k in batches.split():
            for sc in scenes:
                for s in range(int(seed0), int(seed0) + int(seeds)):
                    kk = (float(vx), int(e.get("BANK", 0)), int(e.get("REGEN", 0)),
                          float(e.get("TEMP", 0.3)), p, e.get("BANK_FILE", ""), int(k), int(sc), s)
                    if kk in done:
                        continue
                    pe = f'P="{p}"' if p else ""
                    print(f"source /ws/install/setup.bash; OMP_NUM_THREADS=1 VX_MAX={vx} {env} {pe} "
                          f"timeout -s KILL 900 {B} {k} 0 0 {s} 1 {sc} 0 2>/dev/null")
