#!/bin/bash
# Closed-loop quality sweep: runs closed_loop_benchmark for every (batch, lowpass, bias, scene)
# in parallel and appends JSON lines to the output file.
# Usage: cl_sweep.sh OUT "batches" "lowpass" "bias" seed0 seeds "scenes" [jobs]
OUT=$1; BATCHES=$2; LPS=$3; BIASES=$4; SEED0=$5; SEEDS=$6; SCENES=$7; JOBS=${8:-8}
B=/ws/build/nav2_mppi_controller/benchmark/closed_loop_benchmark
source /ws/install/setup.bash
export OMP_NUM_THREADS=1
for k in $BATCHES; do for lp in $LPS; do for bi in $BIASES; do
  for (( s=0; s<${#SCENES}; s++ )); do
    echo "$k $lp $bi $SEED0 $SEEDS ${SCENES:$s:1}"
  done
done; done; done | xargs -P "$JOBS" -L 1 bash -c "$B \$0 \$1 \$2 \$3 \$4 \$5 2>/dev/null" >> "$OUT"
