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

// Fleet server: N independent MPPI controllers (own costmap each, default Nav2 critics,
// 2000 x 56) shared by K worker threads. Robots alternate between the corridor and wall scenes.
// Reports control cycles per second, per-cycle latency percentiles and CPU time per cycle.
// Usage: fleet_server_benchmark <robots> <workers> <bitmask 0/1> <costcritic threads> <seconds> [wall_share]

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <utility>
#include <sys/resource.h>

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


struct Robot
{
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  nav2::LifecycleNode::SharedPtr node;
  std::unique_ptr<mppi::ParametersHandler> ph;
  decltype(nav2::create_transform_buffer(std::declval<nav2::LifecycleNode::SharedPtr>())) tf;
  std::shared_ptr<ProbeOptimizer> opt;
  geometry_msgs::msg::PoseStamped pose;
  nav_msgs::msg::Path path;
};

int main(int argc, char ** argv)
{
  if (argc < 6) {std::fprintf(stderr, "usage: robots workers bitmask cc_threads seconds [wall_share]\n"); return 2;}
  const int n = std::atoi(argv[1]), k = std::atoi(argv[2]), bitmask = std::atoi(argv[3]);
  const int cc_threads = std::atoi(argv[4]);
  const double seconds = std::atof(argv[5]);
  const double wall_share = argc > 6 ? std::atof(argv[6]) : 0.5;
  rclcpp::init(0, nullptr);
  std::vector<Robot> robots(n);
  for (int r = 0; r < n; ++r) {
    auto & R = robots[r];
    R.costmap = makeCostmapRos();
    const int scene = (r < static_cast<int>(wall_share * n + 0.5)) ? 1 : 0;
    fillScene(R.costmap, scene);
    std::vector<std::string> critics{"ConstraintCritic", "CostCritic", "GoalCritic",
      "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
      "PreferForwardCritic"};
    TestOptimizerSettings s{2000, 56, 1, 3.0, "diff_drive", true};
    std::vector<rclcpp::Parameter> params;
    setUpOptimizerParams(s, critics, params);
    params.emplace_back("dummy.model_dt", 0.05);
    params.emplace_back("dummy.vx_max", 1.67);
    params.emplace_back("dummy.CostCritic.consider_footprint", true);
    params.emplace_back("dummy.CostCritic.inflation_layer_name", "inflation_layer");
    params.emplace_back("dummy.CostCritic.near_goal_distance", 1.0);
    params.emplace_back("dummy.CostCritic.num_threads", cc_threads);
    params.emplace_back("dummy.CostCritic.use_footprint_bitmask", bitmask != 0);
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    R.node = getDummyNode(options);
    std::string name = "test";
    R.ph = std::make_unique<mppi::ParametersHandler>(R.node, name);
    R.tf = nav2::create_transform_buffer(R.node);
    R.opt = std::make_shared<ProbeOptimizer>();
    nav2::LifecycleNode::WeakPtr weak{R.node};
    R.opt->initialize(weak, R.node->get_name(), R.costmap, R.tf, R.ph.get());
    TestPose start{kRobotX, kRobotY};
    TestPathSettings ps{start, 120u, 0.1, 0.0};
    R.pose = getDummyPointStamped(R.node, start);
    R.path = getIncrementalDummyPath(R.node, ps);
  }
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 1.0;
  for (auto & R : robots) {  // warm start
    for (int i = 0; i < 3; ++i) {R.opt->evalControl(R.pose, vel, R.path, R.path.poses.back().pose, nullptr);}
  }
  std::atomic<bool> stop{false};
  std::vector<std::vector<double>> lat(k);
  std::vector<std::thread> pool;
  rusage ru0; getrusage(RUSAGE_SELF, &ru0);
  const auto t0 = std::chrono::steady_clock::now();
  for (int w = 0; w < k; ++w) {
    pool.emplace_back([&, w]() {
      // worker w owns robots w, w + k, w + 2k, ... and runs their cycles round-robin
      while (!stop.load(std::memory_order_relaxed)) {
        for (int r = w; r < n; r += k) {
          auto & R = robots[r];
          const auto a = std::chrono::steady_clock::now();
          R.opt->evalControl(R.pose, vel, R.path, R.path.poses.back().pose, nullptr);
          lat[w].push_back(std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - a).count());
        }
      }
    });
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  stop = true;
  for (auto & t : pool) {t.join();}
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  rusage ru1; getrusage(RUSAGE_SELF, &ru1);
  auto sec = [](const timeval & tv) {return tv.tv_sec + tv.tv_usec * 1e-6;};
  const double cpu = sec(ru1.ru_utime) - sec(ru0.ru_utime) + sec(ru1.ru_stime) - sec(ru0.ru_stime);
  std::vector<double> all;
  for (auto & v : lat) {all.insert(all.end(), v.begin(), v.end());}
  std::sort(all.begin(), all.end());
  auto pct = [&](double p) {return all.empty() ? 0.0 : all[static_cast<size_t>(p * (all.size() - 1))];};
  const double cps = all.size() / wall;
  std::printf(
    "robots=%d workers=%d bitmask=%d cc_threads=%d wall_share=%.2f cycles/s=%.1f "
    "robots@20Hz=%.1f p50_ms=%.2f p99_ms=%.2f max_ms=%.2f cpu_ms_per_cycle=%.2f cpu_cores_busy=%.2f\n",
    n, k, bitmask, cc_threads, wall_share, cps, cps / 20.0, pct(0.5), pct(0.99), pct(1.0),
    1000.0 * cpu / all.size(), cpu / wall);
  robots.clear();
  rclcpp::shutdown();
  return 0;
}
