# Minimal ROS 2 Rolling workspace to build and benchmark nav2_mppi_controller.
FROM ros:rolling-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      git ccache libbenchmark-dev linux-tools-common python3-colcon-common-extensions \
      ros-rolling-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src
ARG NAV2_REVISION=bdb75f435ecb79d92266060801ba6d95e23447db
RUN git init navigation2 \
    && git -C navigation2 remote add origin https://github.com/ros-navigation/navigation2.git \
    && git -C navigation2 fetch --depth 1 origin "$NAV2_REVISION" \
    && git -C navigation2 checkout --detach FETCH_HEAD

# install the system dependencies of nav2_mppi_controller and the Nav2 packages it needs only
WORKDIR /ws
RUN . /opt/ros/rolling/setup.sh \
    && paths=$(colcon list --packages-up-to nav2_mppi_controller --paths-only) \
    && echo "$paths" \
    && apt-get update && rosdep update \
    && rosdep install -y --ignore-src --from-paths $paths \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
