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

// Equivalence of fuse_generation: two optimizers with the same parameters and noise seed, one
// with the fused trajectory generation. After every cycle the generated trajectories, state
// velocities, raw controls, costs and the output command must be bit-identical.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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

class Probe : public mppi::Optimizer
{
public:
  const mppi::models::Trajectories & traj() const {return generated_trajectories_;}
  const mppi::models::State & st() const {return state_;}
  const Eigen::ArrayXf & costs() const {return costs_;}
};

size_t diff(const float * a, const float * b, size_t n)
{
  size_t d = 0;
  for (size_t i = 0; i < n; ++i) {
    if (std::memcmp(a + i, b + i, sizeof(float)) != 0 && !(std::isnan(a[i]) && std::isnan(b[i]))) {++d;}
  }
  return d;
}

template<typename A>
size_t diffArr(const A & a, const A & b)
{
  if (a.size() != b.size()) {return static_cast<size_t>(-1);}
  return diff(a.data(), b.data(), a.size());
}

}  // namespace

int main()
{
  rclcpp::init(0, nullptr);
  size_t total = 0, checked = 0;
  for (int scene : {0, 1}) {
    for (int batch : {2000, 1000, 256}) {
      for (bool clamp : {false, true}) {
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
          // negative control: GEN_EQ_NEG=1 gives the fused optimizer a slightly different dt
          params.emplace_back("dummy.model_dt", (f == 1 && std::getenv("GEN_EQ_NEG")) ? 0.0501 : 0.05);
          params.emplace_back("dummy.vx_max", 1.67);
          params.emplace_back("dummy.clamp_raw_controls", clamp);
          params.emplace_back("dummy.fuse_generation", f == 1);
          params.emplace_back("dummy.CostCritic.consider_footprint", true);
          params.emplace_back("dummy.CostCritic.inflation_layer_name", "inflation_layer");
          params.emplace_back("dummy.CostCritic.near_goal_distance", 1.0);
          params.emplace_back("dummy.CostCritic.use_footprint_bitmask", true);
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
        TestPose start{kRobotX, kRobotY};
        TestPathSettings ps{start, 120u, 0.1, 0.0};
        auto path = getIncrementalDummyPath(nodes[0], ps);
        size_t d = 0;
        for (int c = 0; c < 30; ++c) {
          TestPose p{kRobotX + 0.07 * c, kRobotY + 0.03 * std::sin(0.4 * c)};
          auto pose = getDummyPointStamped(nodes[0], p);
          pose.pose.orientation.z = std::sin(0.05 * std::sin(0.3 * c));
          pose.pose.orientation.w = std::cos(0.05 * std::sin(0.3 * c));
          geometry_msgs::msg::Twist vel;
          vel.linear.x = 0.2 + 0.05 * c;
          vel.angular.z = 0.3 * std::sin(0.5 * c);
          geometry_msgs::msg::TwistStamped cmd[2];
          for (int f = 0; f < 2; ++f) {
            auto r = opt[f]->evalControl(pose, vel, path, path.poses.back().pose, nullptr);
            cmd[f] = std::get<0>(r);
          }
          const auto & A = opt[0]->traj(); const auto & B = opt[1]->traj();
          const auto & SA = opt[0]->st(); const auto & SB = opt[1]->st();
          d += diffArr(A.x, B.x) + diffArr(A.y, B.y) + diffArr(A.yaws, B.yaws);
          d += diffArr(SA.vx, SB.vx) + diffArr(SA.wz, SB.wz) + diffArr(SA.cvx, SB.cvx) +
            diffArr(SA.cwz, SB.cwz);
          d += diffArr(opt[0]->costs(), opt[1]->costs());
          d += (cmd[0].twist.linear.x != cmd[1].twist.linear.x) +
            (cmd[0].twist.angular.z != cmd[1].twist.angular.z);
          checked += A.x.size();
        }
        std::printf("scene=%d batch=%d clamp_raw_controls=%d cycles=30 mismatches=%zu\n",
          scene, batch, clamp, d);
        total += d;
      }
    }
  }
  std::printf("TOTAL mismatches=%zu over %zu trajectory points x 8 arrays\n", total, checked);
  rclcpp::shutdown();
  return total == 0 ? 0 : 1;
}
