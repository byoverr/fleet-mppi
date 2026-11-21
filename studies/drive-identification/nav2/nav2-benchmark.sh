#!/bin/bash
# Inside the prepared nav2lab container. Both builds must already exist.
set -e
for run in 1 2 3; do
  for wall in 0 1; do
    builds=(ws_main ws)
    if [[ $run == 2 ]]; then builds=(ws ws_main); fi
    for build in "${builds[@]}"; do
      (
        source /opt/ros/rolling/setup.bash
        source "/$build/install/setup.bash"
        export FP=amr NO_CCR=1
        mask=0; fuse=0
        if [[ $build == ws ]]; then mask=1; fuse=5; fi
        echo "$run $wall $build"
        "/$build/build/nav2_mppi_controller/benchmark/fleet_server_benchmark" 8 1 "$mask" 1 3 "$wall" 2000 "$fuse"
      )
    done
  done
done
