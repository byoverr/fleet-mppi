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

// Whole MPPI cycle (Optimizer::evalControl, default Nav2 critics, 2000 x 56) in the scenes of
// the CostCritic benchmark: 30x30 m costmap at 0.1 m, 3.9 x 1.5 m footprint, corridor or wall.

#include <benchmark/benchmark.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/inflation_layer_interface.hpp>
#include "nav2_ros_common/tf2_factories.hpp"

#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

#include "utils.hpp"

namespace
{

constexpr double kRes = 0.1, kRobotX = 12.0, kRobotY = 15.0;

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> makeCostmapRos()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"width", 30}, {"height", 30}, {"resolution", kRes},
    {"footprint", "[ [2.230, 0.65], [-1.4312, 0.65], [-1.4312, -0.65], [2.230, -0.65] ]"},
    {"footprint_padding", 0.1},
    {"plugins", std::vector<std::string>{"inflation_layer"}},
    {"inflation_layer.plugin", "nav2_costmap_2d::InflationLayer"},
    {"inflation_layer.inflation_radius", 5.0},
    {"inflation_layer.cost_scaling_factor", 0.7},
  });
  auto ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  ros->on_configure(rclcpp_lifecycle::State{});
  auto * cm = ros->getCostmap();
  ros->getLayeredCostmap()->resizeMap(
    cm->getSizeInCellsX(), cm->getSizeInCellsY(), kRes, 0.0, 0.0);
  return ros;
}

void fillScene(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros, int scene)
{
  auto * cm = ros->getCostmap();
  auto infl = nav2_costmap_2d::InflationLayerInterface::getInflationLayer(ros, "inflation_layer");
  std::vector<std::pair<int, int>> obs;
  auto rect = [&](double x0, double y0, double x1, double y1) {
      for (double x = x0; x <= x1; x += kRes) {
        for (double y = y0; y <= y1; y += kRes) {
          unsigned int mx, my;
          if (cm->worldToMap(x, y, mx, my)) {obs.emplace_back(mx, my);}
        }
      }
    };
  if (scene == 0) {
    rect(8.0, kRobotY + 3.0, 28.0, kRobotY + 3.0);
    rect(8.0, kRobotY - 3.0, 28.0, kRobotY - 3.0);
    rect(19.0, kRobotY + 0.9, 19.6, kRobotY + 3.0);
  } else {
    rect(8.0, kRobotY + 1.7, 28.0, kRobotY + 1.7);
  }
  const int rc = static_cast<int>(infl->getInflationRadius() / kRes);
  const int sx = cm->getSizeInCellsX(), sy = cm->getSizeInCellsY();
  for (auto [ox, oy] : obs) {
    for (int x = std::max(0, ox - rc); x < std::min(sx, ox + rc + 1); ++x) {
      for (int y = std::max(0, oy - rc); y < std::min(sy, oy + rc + 1); ++y) {
        const double d = std::hypot(x - ox, y - oy);
        if (d > rc) {continue;}
        const unsigned char c = infl->computeCost(d);
        if (c > cm->getCost(x, y)) {cm->setCost(x, y, c);}
      }
    }
  }
}

// Exposes the protected stages of one optimizer iteration for timing
class ProbeOptimizer : public mppi::Optimizer
{
public:
  void timeStages(double & gen_us, double & critics_us, double & update_us)
  {
    using clk = std::chrono::steady_clock;
    auto t0 = clk::now();
    generateNoisedTrajectories();
    auto t1 = clk::now();
    critic_manager_.evalTrajectoriesScores(critics_data_);
    auto t2 = clk::now();
    updateControlSequence();
    auto t3 = clk::now();
    gen_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
    critics_us += std::chrono::duration<double, std::micro>(t2 - t1).count();
    update_us += std::chrono::duration<double, std::micro>(t3 - t2).count();
  }
};

}  // namespace

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

static void BM_FullCycle(benchmark::State & state)
{
  const int threads = static_cast<int>(state.range(0));
  const int scene = static_cast<int>(state.range(1));
  const bool bitmask = state.range(2) != 0;
  const int critics_on = static_cast<int>(state.range(3));  // 1 = all default critics, 0 = only CostCritic
  const bool fuse = state.range(4) != 0;  // fused trajectory generation

  auto costmap_ros = makeCostmapRos();
  fillScene(costmap_ros, scene);

  std::vector<std::string> critics = critics_on ?
    std::vector<std::string>{"ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
    "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"} :
    std::vector<std::string>{"ConstraintCritic", "GoalCritic"};
  TestOptimizerSettings s{2000, 56, 1, 3.0, "diff_drive", true};
  std::vector<rclcpp::Parameter> params;
  setUpOptimizerParams(s, critics, params);
  params.emplace_back("dummy.model_dt", 0.05);
  params.emplace_back("dummy.vx_max", 1.67);
  params.emplace_back("dummy.CostCritic.consider_footprint", true);
  params.emplace_back("dummy.CostCritic.inflation_layer_name", "inflation_layer");
  params.emplace_back("dummy.CostCritic.near_goal_distance", 1.0);
  params.emplace_back("dummy.CostCritic.num_threads", threads);
  params.emplace_back("dummy.CostCritic.use_footprint_bitmask", bitmask);
  params.emplace_back("dummy.fuse_generation", fuse);
  rclcpp::NodeOptions options;
  options.parameter_overrides(params);
  auto node = getDummyNode(options);
  std::string name = "test";
  auto ph = std::make_unique<mppi::ParametersHandler>(node, name);
  auto tf_buffer = nav2::create_transform_buffer(node);
  auto optimizer = std::make_shared<ProbeOptimizer>();
  nav2::LifecycleNode::WeakPtr weak_node{node};
  optimizer->initialize(weak_node, node->get_name(), costmap_ros, tf_buffer, ph.get());

  TestPose start{kRobotX, kRobotY};
  TestPathSettings ps{start, 120u, 0.1, 0.0};
  auto pose = getDummyPointStamped(node, start);
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 1.0;
  auto path = getIncrementalDummyPath(node, ps);
  nav2_core::GoalChecker * goal_checker{nullptr};

  // warm start as in normal operation (no reset per cycle: reset regenerates all noise)
  for (int k = 0; k < 5; ++k) {
    optimizer->evalControl(pose, velocity, path, path.poses.back().pose, goal_checker);
  }
  double gen = 0, crit = 0, upd = 0;
  int64_t n = 0;
  for (auto _ : state) {
    auto [cmd, traj] = optimizer->evalControl(
      pose, velocity, path, path.poses.back().pose, goal_checker);
    benchmark::DoNotOptimize(cmd);
    state.PauseTiming();
    optimizer->timeStages(gen, crit, upd);
    ++n;
    state.ResumeTiming();
  }
  state.counters["gen_ms"] = gen / n / 1000.0;
  state.counters["critics_ms"] = crit / n / 1000.0;
  state.counters["update_ms"] = upd / n / 1000.0;
}

BENCHMARK(BM_FullCycle)->ArgNames({"threads", "scene", "bitmask", "critics", "fuse"})
->ArgsProduct({{1, 8}, {0, 1}, {0, 1}, {1}, {0, 1}})
->Unit(benchmark::kMillisecond)->UseRealTime()->MeasureProcessCPUTime()->MinTime(2.0);

BENCHMARK_MAIN();
