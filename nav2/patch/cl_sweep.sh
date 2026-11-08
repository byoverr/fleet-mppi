#!/bin/bash
# Closed-loop quality sweep: runs closed_loop_benchmark for every (batch, lowpass, bias, scene)
# in parallel and appends JSON lines to the output file. One process per episode (seed) with a
# 15-minute timeout, so a rare crash or hang costs one episode, not a whole cell; the missing
# episodes can be rerun with cl_fill.py.
# Usage: cl_sweep.sh OUT "batches" "lowpass" "bias" seed0 seeds "scenes" [jobs]
OUT=$1; BATCHES=$2; LPS=$3; BIASES=$4; SEED0=$5; SEEDS=$6; SCENES=$7; JOBS=${8:-8}
B=/ws/build/nav2_mppi_controller/benchmark/closed_loop_benchmark
source /ws/install/setup.bash
export OMP_NUM_THREADS=1
for k in $BATCHES; do for lp in $LPS; do for bi in $BIASES; do
  for (( s=0; s<${#SCENES}; s++ )); do
    for (( e=SEED0; e<SEED0+SEEDS; e++ )); do echo "$k $lp $bi $e 1 ${SCENES:$s:1}"; done
  done
done; done; done | xargs -P "$JOBS" -L 1 bash -c "timeout -s KILL 900 $B \$0 \$1 \$2 \$3 \$4 \$5 2>/dev/null" >> "$OUT"
