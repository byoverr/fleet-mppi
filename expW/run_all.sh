#!/bin/sh
# Full experiment: 3 worker processes, both robot speeds. Run from expW.
set -e
P=../mpp
for v in 0.22 0.6; do
  tag=$(echo $v | tr -d .)
  for g in baseline main dwa fbmppi; do
    MPP_MAX_V=$v uv run --project $P python run_grid.py $g res_${g}_v${tag}.jsonl 3
  done
done
