#!/bin/bash
# Quiet CPU per control cycle (one process at a time, rolling 5 m window), for every batch size:
#   stock Nav2 (fixed noise), ours (exact speedups, fixed noise), ours + LP 4 Hz,
#   ours + noise bank 4096, ours + LP 4 Hz + bank, ours + fresh noise (REGEN).
# Scenes 0,2,3,4, seeds 1-5. Output: JSON lines (closed_loop_benchmark format) in $1.
OUT=$1
B=/ws/build/nav2_mppi_controller/benchmark/closed_loop_benchmark
source /ws/install/setup.bash
export OMP_NUM_THREADS=1
for k in 2000 1000 500 250 125 64 32; do
  $B $k 0 0 1 5 0234 1 2>/dev/null >> "$OUT"
  $B $k 0 0 1 5 0234 0 2>/dev/null >> "$OUT"
  $B $k 4 0 1 5 0234 0 2>/dev/null >> "$OUT"
  BANK=4096 $B $k 0 0 1 5 0234 0 2>/dev/null >> "$OUT"
  BANK=4096 $B $k 4 0 1 5 0234 0 2>/dev/null >> "$OUT"
  REGEN=1 $B $k 0 0 1 5 0234 0 2>/dev/null >> "$OUT"
done
