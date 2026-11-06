#!/bin/bash
# Build each variant of nav2_mppi_controller and run the CostCritic benchmark (inside the container).
set -e
source /opt/ros/rolling/setup.bash
cd /ws
source install/setup.bash
mkdir -p /ws/results
for v in main pr bitmask; do
  echo "=== $v"
  python3 /ws/patch/prepare_variant.py $v /ws/src/navigation2 /ws/patch
  MAKEFLAGS=-j6 colcon build --packages-select nav2_mppi_controller \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DENABLE_OPENMP=ON \
    > /ws/results/build_$v.log 2>&1 || { tail -40 /ws/results/build_$v.log; exit 1; }
  B=/ws/build/nav2_mppi_controller/benchmark
  case $v in
    main) filter='threads:1/scene:[01]/bitmask:0' ;;
    pr) filter='bitmask:0' ;;
    bitmask)
      $B/cost_critic_equivalence 2>/dev/null | tee /ws/results/equivalence.txt
      filter='bitmask:1' ;;
  esac
  $B/cost_critic_benchmark --benchmark_filter="$filter" --benchmark_repetitions=3 \
    --benchmark_report_aggregates_only=true --benchmark_out=/ws/results/bench_$v.json \
    --benchmark_out_format=json 2>/dev/null | grep -E "median|^BM|Run on|CPU" | tee /ws/results/bench_$v.txt
done
