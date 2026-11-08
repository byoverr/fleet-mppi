#!/bin/bash
# CPU per control cycle in closed loop, one process at a time (no background load).
# Stock Nav2 (stock=1) and with the exact speedups, for each batch size; scenes 0,2,3,4,
# seeds 1-3. Output: JSON lines (closed_loop_benchmark format) in $1.
OUT=$1
B=/ws/build/nav2_mppi_controller/benchmark/closed_loop_benchmark
source /ws/install/setup.bash
export OMP_NUM_THREADS=1
for k in 2000 1000 500 250 125 64 32; do
  for stock in 1 0; do
    $B $k 0 0 1 3 0234 $stock 2>/dev/null >> "$OUT"
  done
  $B $k 4 3 1 3 0234 0 2>/dev/null >> "$OUT"
done
