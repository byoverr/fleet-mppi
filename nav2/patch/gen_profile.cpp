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

// Where the trajectory generation of one MPPI cycle spends its time (2000 x 56, corridor),
// for the stock path and for fuse_generation.

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

class Probe : public mppi::Optimizer
{
public:
  double sink_{0};
  void profile(int reps, bool fuse)
  {
    using clk = std::chrono::steady_clock;
    double t[6] = {0, 0, 0, 0, 0, 0};
    for (int r = 0; r < reps; ++r) {
      auto a = clk::now();
      applyControlSequenceInterIterationConstraints();
      auto b = clk::now();
      noise_generator_.setNoisedControls(state_, control_sequence_);
      auto c = clk::now();
      noise_generator_.generateNextNoises();
      auto d = clk::now();
      if (fuse) {
        generateFused();
      } else {
        updateStateVelocities(state_);
      }
      auto e = clk::now();
      if (!fuse) {integrateStateVelocities(generated_trajectories_, state_);}
      auto f = clk::now();
      // the trigonometry alone, on the same array
      Eigen::ArrayXXf cs = generated_trajectories_.yaws.cos();
      Eigen::ArrayXXf sn = generated_trajectories_.yaws.sin();
      auto g = clk::now();
      sink_ += cs.sum() * 0.0f + sn.size();
      double us[6] = {
        std::chrono::duration<double, std::micro>(b - a).count(),
        std::chrono::duration<double, std::micro>(c - b).count(),
        std::chrono::duration<double, std::micro>(d - c).count(),
        std::chrono::duration<double, std::micro>(e - d).count(),
        std::chrono::duration<double, std::micro>(f - e).count(),
        std::chrono::duration<double, std::micro>(g - f).count()};
      for (int k = 0; k < 6; ++k) {t[k] += us[k];}
    }
    std::printf("fuse=%d  inter-iteration constraints %.1f us | setNoisedControls %.1f | "
      "generateNextNoises %.1f | %s %.1f | integrate %.1f | (cos+sin of all yaws alone %.1f)\n",
      fuse, t[0] / reps, t[1] / reps, t[2] / reps, fuse ? "fused predict+integrate" : "predict",
      t[3] / reps, t[4] / reps, t[5] / reps);
  }
};

}  // namespace

int main()
{
  rclcpp::init(0, nullptr);
  for (int fuse : {0, 1}) {
    auto costmap_ros = makeCostmapRos();
    fillScene(costmap_ros, 0);
    std::vector<std::string> critics{"ConstraintCritic", "CostCritic", "GoalCritic",
      "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
      "PreferForwardCritic"};
    TestOptimizerSettings s{2000, 56, 1, 3.0, "diff_drive", true};
    std::vector<rclcpp::Parameter> params;
    setUpOptimizerParams(s, critics, params);
    params.emplace_back("dummy.model_dt", 0.05);
    params.emplace_back("dummy.vx_max", 1.67);
    params.emplace_back("dummy.fuse_generation", fuse == 1);
    params.emplace_back("dummy.CostCritic.consider_footprint", true);
    params.emplace_back("dummy.CostCritic.inflation_layer_name", "inflation_layer");
    params.emplace_back("dummy.CostCritic.use_footprint_bitmask", true);
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    auto node = getDummyNode(options);
    std::string name = "test";
    auto ph = std::make_unique<mppi::ParametersHandler>(node, name);
    auto tf = nav2::create_transform_buffer(node);
    auto opt = std::make_shared<Probe>();
    nav2::LifecycleNode::WeakPtr weak{node};
    opt->initialize(weak, node->get_name(), costmap_ros, tf, ph.get());
    TestPose start{kRobotX, kRobotY};
    TestPathSettings ps{start, 120u, 0.1, 0.0};
    auto pose = getDummyPointStamped(node, start);
    auto path = getIncrementalDummyPath(node, ps);
    geometry_msgs::msg::Twist vel;
    vel.linear.x = 1.0;
    for (int k = 0; k < 5; ++k) {opt->evalControl(pose, vel, path, path.poses.back().pose, nullptr);}
    opt->profile(500, fuse == 1);
  }
  rclcpp::shutdown();
  return 0;
}
