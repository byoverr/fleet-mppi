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
    // pieces of updateControlSequence, same expressions on the same arrays
    double q[5] = {0, 0, 0, 0, 0};
    for (int r = 0; r < reps; ++r) {
      auto & st = settings_;
      auto a = clk::now();
      auto vx_T = control_sequence_.vx.transpose();
      auto bvx = state_.cvx.rowwise() - vx_T;
      const float gvx = st.gamma / (st.sampling_std.vx * st.sampling_std.vx);
      Eigen::ArrayXf cc = (gvx * (bvx.rowwise() * vx_T).rowwise().sum()).eval();
      auto wz_T = control_sequence_.wz.transpose();
      auto bwz = state_.cwz.rowwise() - wz_T;
      const float gwz = st.gamma / (st.sampling_std.wz * st.sampling_std.wz);
      cc += (gwz * (bwz.rowwise() * wz_T).rowwise().sum()).eval();
      auto b = clk::now();
      auto cn = cc - cc.minCoeff();
      auto sm = ((-(1.0f / st.temperature) * cn).cwiseMax(-80.0f)).exp().eval();
      sm /= sm.sum();
      auto c = clk::now();
      Eigen::VectorXf v1 = state_.cvx.transpose().matrix() * sm.matrix();
      Eigen::VectorXf v2 = state_.cwz.transpose().matrix() * sm.matrix();
      auto d = clk::now();
      auto e = clk::now();
      updateControlSequence();
      auto f = clk::now();
      sink_ += v1.sum() + v2.sum() + cc.sum();
      q[0] += std::chrono::duration<double, std::micro>(b - a).count();
      q[1] += std::chrono::duration<double, std::micro>(c - b).count();
      q[2] += std::chrono::duration<double, std::micro>(d - c).count();
      q[4] += std::chrono::duration<double, std::micro>(f - e).count();
    }
    std::printf("update: control cost %.1f us | softmax %.1f | weighted sum (2 GEMV) %.1f | "
      "whole updateControlSequence %.1f\n", q[0] / reps, q[1] / reps, q[2] / reps, q[4] / reps);
    std::printf("fuse=%d  inter-iteration constraints %.1f us | setNoisedControls %.1f | "
      "generateNextNoises %.1f | %s %.1f | integrate %.1f | (cos+sin of all yaws alone %.1f)\n",
      fuse, t[0] / reps, t[1] / reps, t[2] / reps, fuse ? "fused predict+integrate" : "predict",
      t[3] / reps, t[4] / reps, t[5] / reps);
  }
};

}  // namespace

int main()
{
  selectRobot();
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
