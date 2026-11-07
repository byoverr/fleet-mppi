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

// Cost of each default Nav2 critic in different robot states (straight, misaligned, curve, near
// goal), inside a warm MPPI cycle (2000 x 56), CostCritic with masks, fused generation.
// The critic set is ConstraintCritic + one critic; ConstraintCritic alone is the baseline.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

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

constexpr double kRobotX = 12.0, kRobotY = 15.0;
// Robot and map, FP=big (default) or FP=amr in the environment
double kRes = 0.1, kInflation = 5.0, kWall = 1.7, kCorridor = 3.0;
const char * kFootprint = "[ [2.230, 0.65], [-1.4312, 0.65], [-1.4312, -0.65], [2.230, -0.65] ]";
void selectRobot()
{
  const char * fp = std::getenv("FP");
  if (fp && std::string(fp) == "amr") {  // 0.7 x 0.5 m AMR, 5 cm map, 2.4 m aisle
    kFootprint = "[[0.35,0.25],[-0.35,0.25],[-0.35,-0.25],[0.35,-0.25]]";
    kRes = 0.05; kInflation = 1.5; kWall = 0.6; kCorridor = 1.2;
  }
}

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> makeCostmapRos()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"width", 30}, {"height", 30}, {"resolution", kRes},
    {"footprint", kFootprint},
    {"footprint_padding", 0.1},
    {"plugins", std::vector<std::string>{"inflation_layer"}},
    {"inflation_layer.plugin", "nav2_costmap_2d::InflationLayer"},
    {"inflation_layer.inflation_radius", kInflation},
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
    rect(8.0, kRobotY + kCorridor, 28.0, kRobotY + kCorridor);
    rect(8.0, kRobotY - kCorridor, 28.0, kRobotY - kCorridor);
    rect(19.0, kRobotY + 0.3 * kCorridor, 19.6, kRobotY + kCorridor);
  } else {
    rect(8.0, kRobotY + kWall, 28.0, kRobotY + kWall);
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


const char * kCritics[] = {"none", "CostCritic", "GoalCritic", "GoalAngleCritic",
  "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic", "all"};
const char * kStates[] = {"straight", "misaligned -0.8 rad", "curve R=3 m", "near goal 0.7 m"};

nav_msgs::msg::Path makePath(nav2::LifecycleNode::SharedPtr node, int st)
{
  auto path = getDummyPath(node);
  const int n = st == 3 ? 8 : 120;
  for (int i = 0; i < n; ++i) {
    double x = kRobotX + 0.1 * i, y = kRobotY, yaw = 0.0;
    if (st == 2) {  // left arc of radius 3 m starting at the robot, heading +x
      const double a = 0.1 * i / 3.0;
      x = kRobotX + 3.0 * std::sin(a);
      y = kRobotY + 3.0 * (1.0 - std::cos(a));
      yaw = a;
    }
    auto p = getDummyPointStamped(node, TestPose{x, y});
    p.pose.orientation.z = std::sin(yaw / 2);
    p.pose.orientation.w = std::cos(yaw / 2);
    path.poses.push_back(p);
  }
  return path;
}

double g_gen = 0, g_upd = 0;
int g_n = 0;

double run(int scene, int st, int which, int cycles)
{
  auto costmap_ros = makeCostmapRos();
  fillScene(costmap_ros, scene);
  std::vector<std::string> critics{"ConstraintCritic"};
  if (which >= 9) {  // all but one
    critics = {"ConstraintCritic"};
    for (int c = 1; c < 8; ++c) {if (c != which - 8) {critics.push_back(kCritics[c]);}}
  } else if (which == 8) {
    critics = {"ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
      "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"};
  } else if (which > 0) {
    critics.push_back(kCritics[which]);
  }
  TestOptimizerSettings s{2000, 56, 1, 3.0, "diff_drive", true};
  std::vector<rclcpp::Parameter> params;
  setUpOptimizerParams(s, critics, params);
  params.emplace_back("dummy.model_dt", 0.05);
  params.emplace_back("dummy.vx_max", 1.67);
  params.emplace_back("dummy.CostCritic.consider_footprint", true);
  params.emplace_back("dummy.CostCritic.inflation_layer_name", "inflation_layer");
  params.emplace_back("dummy.CostCritic.near_goal_distance", 1.0);
  params.emplace_back("dummy.CostCritic.use_footprint_bitmask", true);
  params.emplace_back("dummy.fuse_generation", true);
  params.emplace_back("dummy.PathAlignCritic.blocked_evaluation", std::getenv("PA_BLOCKED") != nullptr);
  params.emplace_back("dummy.fast_reductions", std::getenv("FAST") != nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides(params);
  auto node = getDummyNode(options);
  std::string name = "test";
  auto ph = std::make_unique<mppi::ParametersHandler>(node, name);
  auto tf_buffer = nav2::create_transform_buffer(node);
  auto optimizer = std::make_shared<ProbeOptimizer>();
  nav2::LifecycleNode::WeakPtr weak_node{node};
  optimizer->initialize(weak_node, node->get_name(), costmap_ros, tf_buffer, ph.get());
  auto pose = getDummyPointStamped(node, TestPose{kRobotX, kRobotY});
  if (st == 1) {
    pose.pose.orientation.z = std::sin(-0.4);  // turned 0.8 rad away from the wall side
    pose.pose.orientation.w = std::cos(-0.4);
  }
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = st == 3 ? 0.3 : 1.0;
  auto path = makePath(node, st);
  std::vector<double> v(static_cast<size_t>(cycles), 0.0);
  try {
    for (int k = 0; k < 5; ++k) {
      optimizer->evalControl(pose, velocity, path, path.poses.back().pose, nullptr);
    }
  } catch (const std::exception &) {
    return -1.0;
  }
  for (int k = 0; k < cycles; ++k) {
    try {
      optimizer->evalControl(pose, velocity, path, path.poses.back().pose, nullptr);
    } catch (const std::exception &) {
      return -1.0;
    }
    double g = 0, c = 0, u = 0;
    optimizer->timeStages(g, c, u);
    v[static_cast<size_t>(k)] = c;
    g_gen += g;
    g_upd += u;
    ++g_n;
  }
  std::sort(v.begin(), v.end());
  return v.empty() ? 0.0 : v.at(v.size() / 2);
}

int main()
{
  selectRobot();
  rclcpp::init(0, nullptr);
  std::printf("critics time per cycle, us (median of 200), 2000 x 56, big robot, masks + fused gen\n");
  std::printf("| scene | state |");
  for (auto c : kCritics) {std::printf(" %s |", c);}
  std::printf("\n");
  for (int scene : {0, 1}) {
    for (int st = 0; st < 4; ++st) {
      std::printf("| %s | %s |", scene ? "wall" : "corridor", kStates[st]);
      double base = 0;
      for (int w = 0; w < 9; ++w) {
        const double t = run(scene, st, w, 200);
        if (w == 0) {base = t; std::printf(" %.0f |", t);} else {std::printf(" %.0f |", w == 8 ? t : t - base);}
      }
      std::printf("\n");
      std::fflush(stdout);
    }
  }
  std::printf("\nall minus one critic: what the critic costs inside the full set, us\n");
  std::printf("| scene | state |");
  for (int c = 1; c < 8; ++c) {std::printf(" -%s |", kCritics[c]);}
  std::printf("\n");
  for (int scene : {0, 1}) {
    for (int st = 0; st < 4; ++st) {
      g_gen = g_upd = 0;
      g_n = 0;
      const double all = run(scene, st, 8, 200);
      const double gen = g_gen / g_n, upd = g_upd / g_n;
      std::printf("| %s | %s |", scene ? "wall" : "corridor", kStates[st]);
      for (int c = 1; c < 8; ++c) {std::printf(" %.0f |", all - run(scene, st, 8 + c, 200));}
      std::printf(" all critics %.0f | generation %.0f | update %.0f |", all, gen, upd);
      std::printf("\n");
      std::fflush(stdout);
    }
  }
  rclcpp::shutdown();
  return 0;
}
