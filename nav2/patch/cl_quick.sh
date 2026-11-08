#!/bin/bash
# Quick summary of one closed_loop_benchmark run: cl_quick.sh batch lp bias seed0 seeds scenes [stock]
# (the environment passes through). Prints success, collisions, mean speed of successes,
# dw rms, median min clearance and CPU per cycle.
B=/ws/build/nav2_mppi_controller/benchmark/closed_loop_benchmark
source /ws/install/setup.bash
export OMP_NUM_THREADS=1
$B "$@" 2>/dev/null | python3 -c '
import json, sys, statistics as s
r = [json.loads(l) for l in sys.stdin]
ok = [x for x in r if x["status"] == "success"]
col = sum(x["status"] == "collision" for x in r)
sp = s.mean(x["len"] / x["time"] for x in ok) if ok else 0
dw = s.mean(x["dw_rms"] for x in r)
clr = s.median(x["min_clear"] for x in r)
cpu = s.median(x["cpu_med"] for x in r)
print(f"succ {len(ok)}/{len(r)} col {col} speed {sp:.2f} dw {dw:.4f} clr {clr:.3f} cpu {cpu:.0f}")'
