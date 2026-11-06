// Copyright (c) 2026 Sergey Shchetinkin
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// What one local costmap update costs on a fleet server: rolling ObstacleLayer fed with a fresh
// 2D lidar scan (raytrace clearing + marking) followed by the new or the legacy inflation layer.
// The robot drives along a corridor with posts; every update moves it and replaces the scan.

#include <benchmark/benchmark.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{

struct Config
{
  const char * name;
  double size, res, inflation, range;
  int beams;
  const char * footprint;
};

const Config kConfigs[] = {
  {"turtlebot 3x3m@0.05 infl0.55 360beams", 3.0, 0.05, 0.55, 3.5, 360,
    "[[0.1,0.09],[-0.1,0.09],[-0.1,-0.09],[0.1,-0.09]]"},
  {"amr 6x6m@0.05 infl1.0 720beams", 6.0, 0.05, 1.0, 10.0, 720,
    "[[0.35,0.25],[-0.35,0.25],[-0.35,-0.25],[0.35,-0.25]]"},
  {"big 30x30m@0.1 infl5.0 1080beams", 30.0, 0.1, 5.0, 15.0, 1080,
    "[[2.230,0.65],[-1.4312,0.65],[-1.4312,-0.65],[2.230,-0.65]]"},
};
const char * kInflation[] = {"nav2_costmap_2d::InflationLayer",
  "nav2_costmap_2d::LegacyInflationLayer"};

constexpr double kHalfWidth = 1.5;  // corridor walls at y = +-1.5 m

// range of a beam from (x, y) at angle a against the corridor walls and posts
double castBeam(double x, double y, double a, double max_range)
{
  const double s = std::sin(a), c = std::cos(a);
  double best = max_range;
  if (std::abs(s) > 1e-9) {
    const double d = ((s > 0 ? kHalfWidth : -kHalfWidth) - y) / s;
    if (d > 0) {best = std::min(best, d);}
  }
  // posts of radius 0.1 every 2 m on both sides
  const int k0 = static_cast<int>(std::floor((x - max_range) / 2.0));
  for (int k = k0; k <= k0 + static_cast<int>(max_range); ++k) {
    for (double py : {kHalfWidth - 0.4, -kHalfWidth + 0.4}) {
      const double px = 2.0 * k + 1.0, dx = px - x, dy = py - y;
      const double t = dx * c + dy * s;
      const double d2 = dx * dx + dy * dy - t * t;
      if (t > 0 && d2 < 0.01) {best = std::min(best, t - std::sqrt(0.01 - d2));}
    }
  }
  return best;
}

nav2_costmap_2d::Observation makeScan(double x, double y, const Config & cfg)
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(cfg.beams);
  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
  for (int b = 0; b < cfg.beams; ++b, ++ix, ++iy, ++iz) {
    const double a = 2.0 * M_PI * b / cfg.beams;
    const double r = castBeam(x, y, a, cfg.range);
    *ix = static_cast<float>(x + r * std::cos(a));
    *iy = static_cast<float>(y + r * std::sin(a));
    *iz = 0.2f;
  }
  geometry_msgs::msg::Point origin;
  origin.x = x; origin.y = y; origin.z = 0.2;
  // points at max range are free-space returns: they clear but must not mark
  return nav2_costmap_2d::Observation(origin, cloud, cfg.range - 0.01, 0.0, cfg.range, 0.0);
}

}  // namespace

class RosGuard
{
public:
  RosGuard() {rclcpp::init(0, nullptr);}
  ~RosGuard() {rclcpp::shutdown();}
};
RosGuard g_rclcpp;

static void BM_CostmapUpdate(benchmark::State & state)
{
  const Config & cfg = kConfigs[state.range(0)];
  const int infl = static_cast<int>(state.range(1));
  state.SetLabel(std::string(cfg.name) + (infl ? " legacy-inflation" : " inflation"));
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"width", static_cast<int>(cfg.size)}, {"height", static_cast<int>(cfg.size)},
    {"resolution", cfg.res}, {"rolling_window", true}, {"footprint", cfg.footprint},
    {"global_frame", "odom"}, {"robot_base_frame", "base_link"},
    {"plugins", std::vector<std::string>{"obstacle_layer", "inflation_layer"}},
    {"obstacle_layer.plugin", "nav2_costmap_2d::ObstacleLayer"},
    {"obstacle_layer.observation_sources", ""},
    {"inflation_layer.plugin", kInflation[infl]},
    {"inflation_layer.inflation_radius", cfg.inflation},
    {"inflation_layer.cost_scaling_factor", 3.0},
  });
  auto ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  ros->on_configure(rclcpp_lifecycle::State{});
  auto * layered = ros->getLayeredCostmap();
  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> obstacles;
  for (auto & p : *layered->getPlugins()) {
    if (auto o = std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(p)) {obstacles = o;}
  }
  if (!obstacles) {state.SkipWithError("no obstacle layer"); return;}

  // 1.0 m/s at the default local costmap rate of 5 Hz
  double x = 0.0;
  const double y = 0.1;
  for (int k = 0; k < 20; ++k) {
    obstacles->clearStaticObservations(true, true);
    obstacles->addStaticObservation(makeScan(x, y, cfg), true, true);
    layered->updateMap(x, y, 0.0);
    x += 0.2;
  }
  for (auto _ : state) {
    state.PauseTiming();
    obstacles->clearStaticObservations(true, true);
    obstacles->addStaticObservation(makeScan(x, y, cfg), true, true);
    x += 0.2;
    state.ResumeTiming();
    layered->updateMap(x, y, 0.0);
  }
  int lethal = 0;
  auto * cm = layered->getCostmap();
  for (unsigned int i = 0; i < cm->getSizeInCellsX() * cm->getSizeInCellsY(); ++i) {
    lethal += cm->getCharMap()[i] == nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  state.counters["lethal_cells"] = lethal;
}

BENCHMARK(BM_CostmapUpdate)->ArgNames({"config", "legacy"})
->ArgsProduct({{0, 1, 2}, {0, 1}})
->Unit(benchmark::kMillisecond)->MeasureProcessCPUTime()->UseRealTime()->MinTime(1.0);

BENCHMARK_MAIN();
