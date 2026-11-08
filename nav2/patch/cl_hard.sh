#!/bin/bash
# Hard regimes: tight scenes 5-8 at 0.5 m/s, the easy scenes 0,2,3,4 fast (vx_max 1.0,
# vx_std 0.4), and the tight scenes fast. For each: plain MPPI (fixed noise), noise bank 4096,
# fresh noise. Usage: cl_hard.sh OUT "batches" seed0 seeds [jobs]
OUT=$1; BATCHES=$2; SEED0=$3; SEEDS=$4; JOBS=${5:-6}
S=/ws/patch/cl_sweep.sh
for regime in tight fast tightfast; do
  case $regime in
    tight) export VX_MAX=0.5; unset P; SC=5678 ;;
    fast) export VX_MAX=1.0 P="vx_std=0.4"; SC=0234 ;;
    tightfast) export VX_MAX=1.0 P="vx_std=0.4"; SC=5678 ;;
  esac
  (unset BANK REGEN; $S "$OUT" "$BATCHES" 0 0 "$SEED0" "$SEEDS" $SC "$JOBS")
  (unset REGEN; BANK=4096 $S "$OUT" "$BATCHES" 0 0 "$SEED0" "$SEEDS" $SC "$JOBS")
  (unset BANK; REGEN=1 $S "$OUT" "$BATCHES" 0 0 "$SEED0" "$SEEDS" $SC "$JOBS")
done
