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

// Equivalence of a lab option that must not change results: two optimizers with all default
// critics and the same noise seed, the second with <param> = true (argv[1], e.g.
// PathAlignCritic.blocked_evaluation). Robot states: straight path, misaligned, curve, near
// goal; corridor and wall; batch 2000 / 1000 / 257; use_path_orientations off and on.
// After every cycle the costs, trajectories and the command must be bit-identical.

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>

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



nav_msgs::msg::Path makePath(nav2::LifecycleNode::SharedPtr node, int st)
{
  auto path = getDummyPath(node);
  const int n = st == 3 ? 8 : 120;
  for (int i = 0; i < n; ++i) {
    double x = kRobotX + 0.1 * i, y = kRobotY, yaw = 0.0;
    if (st == 2) {
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

class Probe : public mppi::Optimizer
{
public:
  const mppi::models::Trajectories & traj() const {return generated_trajectories_;}
};

size_t diffArr(const Eigen::ArrayXXf & a, const Eigen::ArrayXXf & b)
{
  size_t d = 0;
  for (Eigen::Index i = 0; i < a.size(); ++i) {
    d += std::memcmp(a.data() + i, b.data() + i, 4) != 0 &&
      !(std::isnan(a.data()[i]) && std::isnan(b.data()[i]));
  }
  return d;
}

int main(int argc, char ** argv)
{
  if (argc < 2) {std::fprintf(stderr, "usage: critic_equivalence <Critic.param> [value for the second optimizer]\n"); return 2;}
  const std::string toggle = std::string("dummy.") + argv[1];
  rclcpp::init(0, nullptr);
  size_t total = 0, points = 0, active_cycles = 0;
  for (int scene : {0, 1}) {
    for (int st = 0; st < 4; ++st) {
      for (int batch : {2000, 1000, 257}) {
        for (bool orient : {false, true}) {
          auto costmap_ros = makeCostmapRos();
          fillScene(costmap_ros, scene);
          std::shared_ptr<Probe> opt[2];
          std::vector<nav2::LifecycleNode::SharedPtr> nodes;
          std::vector<std::unique_ptr<mppi::ParametersHandler>> phs;
          for (int f = 0; f < 2; ++f) {
            std::vector<std::string> critics{"ConstraintCritic", "CostCritic", "GoalCritic",
              "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
              "PreferForwardCritic"};
            TestOptimizerSettings s{batch, 56, 1, 3.0, "diff_drive", true};
            std::vector<rclcpp::Parameter> params;
            setUpOptimizerParams(s, critics, params);
            params.emplace_back("dummy.model_dt", 0.05);
            params.emplace_back("dummy.vx_max", 1.67);
            params.emplace_back("dummy.CostCritic.consider_footprint", true);
            params.emplace_back("dummy.CostCritic.inflation_layer_name", "inflation_layer");
            params.emplace_back("dummy.CostCritic.near_goal_distance", 1.0);
            params.emplace_back("dummy.CostCritic.use_footprint_bitmask", true);
            params.emplace_back("dummy.PathAlignCritic.use_path_orientations", orient);
            if (argc > 2) {  // numeric parameter for the second optimizer (negative control)
              if (f == 1) {params.emplace_back(toggle, std::atof(argv[2]));}
            } else {
              params.emplace_back(toggle, f == 1);
            }
            rclcpp::NodeOptions options;
            options.parameter_overrides(params);
            auto node = getDummyNode(options);
            nodes.push_back(node);
            std::string name = "test";
            phs.push_back(std::make_unique<mppi::ParametersHandler>(node, name));
            auto tf = nav2::create_transform_buffer(node);
            opt[f] = std::make_shared<Probe>();
            nav2::LifecycleNode::WeakPtr weak{node};
            opt[f]->initialize(weak, node->get_name(), costmap_ros, tf, phs.back().get());
          }
          auto path = makePath(nodes[0], st);
          size_t d = 0;
          int ok = 0;
          for (int c = 0; c < 80; ++c) {
            TestPose p{kRobotX + 0.02 * c, kRobotY + 0.02 * std::sin(0.4 * c)};
            auto pose = getDummyPointStamped(nodes[0], p);
            const double yaw = (st == 1 ? -0.8 : 0.0) + 0.1 * std::sin(0.3 * c);
            pose.pose.orientation.z = std::sin(yaw / 2);
            pose.pose.orientation.w = std::cos(yaw / 2);
            geometry_msgs::msg::Twist vel;
            vel.linear.x = st == 3 ? 0.3 : 1.2 + 0.01 * c;
            vel.angular.z = 0.2 * std::sin(0.5 * c);
            geometry_msgs::msg::TwistStamped cmd[2];
            bool fail[2] = {false, false};
            for (int f = 0; f < 2; ++f) {
              try {
                cmd[f] = std::get<0>(opt[f]->evalControl(pose, vel, path, path.poses.back().pose, nullptr));
              } catch (const std::exception &) {fail[f] = true;}
            }
            d += fail[0] != fail[1];
            if (fail[0] || fail[1]) {continue;}
            ++ok;
            d += diffArr(opt[0]->traj().x, opt[1]->traj().x) + diffArr(opt[0]->traj().y, opt[1]->traj().y);
            Eigen::ArrayXXf c0 = opt[0]->getCosts(), c1 = opt[1]->getCosts();
            d += diffArr(c0, c1);
            d += (cmd[0].twist.linear.x != cmd[1].twist.linear.x) +
              (cmd[0].twist.angular.z != cmd[1].twist.angular.z);
            points += opt[0]->traj().x.size();
          }
          active_cycles += ok;
          std::printf("scene=%d state=%d batch=%d orient=%d cycles_ok=%d mismatches=%zu\n",
            scene, st, batch, orient, ok, d);
          total += d;
        }
      }
    }
  }
  std::printf("TOTAL %s: mismatches=%zu over %zu cycles, %zu trajectory points\n", argv[1], total,
    active_cycles, points);
  rclcpp::shutdown();
  return total == 0 ? 0 : 1;
}
