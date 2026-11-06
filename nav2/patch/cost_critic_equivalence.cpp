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

// CostCritic with and without the footprint bitmask must produce bit-identical costs and the
// same fail flag on random scenes, footprints, resolutions and trajectory batches.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/inflation_layer_interface.hpp>

#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/critics/cost_critic.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace
{

struct Case
{
  const char * footprint;
  double resolution;
  double inflation;
  bool track_unknown;
};

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> makeCostmap(const Case & c, int id)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"width", 20}, {"height", 20}, {"resolution", c.resolution},
    {"footprint", c.footprint}, {"footprint_padding", 0.05},
    {"track_unknown_space", c.track_unknown},
    {"plugins", std::vector<std::string>{"inflation_layer"}},
    {"inflation_layer.plugin", "nav2_costmap_2d::InflationLayer"},
    {"inflation_layer.inflation_radius", c.inflation},
    {"inflation_layer.cost_scaling_factor", 1.5},
  });
  (void)id;
  auto ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  ros->on_configure(rclcpp_lifecycle::State{});
  auto * cm = ros->getCostmap();
  ros->getLayeredCostmap()->resizeMap(
    cm->getSizeInCellsX(), cm->getSizeInCellsY(), c.resolution, 0.0, 0.0);
  return ros;
}

// random clutter: walls, boxes and single-cell posts (posts can hide inside a footprint)
void fillRandom(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros, std::mt19937 & rng)
{
  auto * cm = ros->getCostmap();
  cm->resetMap(0, 0, cm->getSizeInCellsX(), cm->getSizeInCellsY());
  const bool tracking = ros->getLayeredCostmap()->isTrackingUnknown();
  if (tracking) {
    // tracked unknown space: free map with large unknown patches
    for (unsigned int k = 0; k < cm->getSizeInCellsX() * cm->getSizeInCellsY(); ++k) {
      cm->getCharMap()[k] = 0;
    }
    std::uniform_real_distribution<double> up(1.0, 17.0), ul(0.3, 3.0);
    for (int k = 0; k < 4; ++k) {
      const double x = up(rng), y = up(rng), w = ul(rng), h = ul(rng);
      for (double px = x; px <= x + w; px += cm->getResolution()) {
        for (double py = y; py <= y + h; py += cm->getResolution()) {
          unsigned int mx, my;
          if (cm->worldToMap(px, py, mx, my)) {cm->setCost(mx, my, 255);}
        }
      }
    }
  }
  auto inflation = nav2_costmap_2d::InflationLayerInterface::getInflationLayer(
    ros, "inflation_layer");
  const double res = cm->getResolution();
  std::uniform_real_distribution<double> ux(1.0, 19.0), us(0.05, 2.5);
  std::vector<std::pair<int, int>> obs;
  auto rect = [&](double x0, double y0, double x1, double y1) {
      for (double x = x0; x <= x1; x += res) {
        for (double y = y0; y <= y1; y += res) {
          unsigned int mx, my;
          if (cm->worldToMap(x, y, mx, my)) {obs.emplace_back(mx, my);}
        }
      }
    };
  std::uniform_int_distribution<int> n_obj(3, 25);
  for (int k = n_obj(rng); k > 0; --k) {
    const double x = ux(rng), y = ux(rng);
    rect(x, y, x + us(rng) * (k % 3 == 0 ? 0.05 : 1.0), y + us(rng) * (k % 3 == 1 ? 0.05 : 1.0));
  }
  // a few unknown cells too
  for (int k = 0; k < 5; ++k) {
    unsigned int mx, my;
    if (cm->worldToMap(ux(rng), ux(rng), mx, my)) {cm->setCost(mx, my, 255);}
  }
  const int rc = static_cast<int>(inflation->getInflationRadius() / res);
  const int sx = cm->getSizeInCellsX(), sy = cm->getSizeInCellsY();
  for (auto [ox, oy] : obs) {
    for (int x = std::max(0, ox - rc); x < std::min(sx, ox + rc + 1); ++x) {
      for (int y = std::max(0, oy - rc); y < std::min(sy, oy + rc + 1); ++y) {
        const double d = std::hypot(x - ox, y - oy);
        if (d > rc) {continue;}
        const unsigned char cost = inflation->computeCost(d);
        if (cm->getCost(x, y) != 255 && cost > cm->getCost(x, y)) {cm->setCost(x, y, cost);}
      }
    }
  }
}

void fillTrajectories(mppi::models::Trajectories & t, int batch, int steps, std::mt19937 & rng)
{
  t.reset(batch, steps);
  std::uniform_real_distribution<float> u01(0.0f, 1.0f);
  std::normal_distribution<float> vx(1.0f, 0.4f), wz(0.0f, 0.8f);
  const float x0 = 2.0f + 16.0f * u01(rng), y0 = 2.0f + 16.0f * u01(rng);
  const float yaw0 = 12.0f * (u01(rng) - 0.5f);  // also |yaw| > pi
  for (int i = 0; i < batch; ++i) {
    float x = x0, y = y0, yaw = yaw0;
    for (int j = 0; j < steps; ++j) {
      yaw += wz(rng) * 0.05f;
      const float v = vx(rng);
      x += v * std::cos(yaw) * 0.05f;
      y += v * std::sin(yaw) * 0.05f;
      t.x(i, j) = x; t.y(i, j) = y; t.yaws(i, j) = yaw;
    }
  }
}

std::unique_ptr<mppi::critics::CostCritic> makeCritic(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros, bool bitmask, int id,
  std::shared_ptr<nav2::LifecycleNode> & node, std::unique_ptr<mppi::ParametersHandler> & ph)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"CostCritic.consider_footprint", true},
    {"CostCritic.near_goal_distance", 0.0},
    {"CostCritic.trajectory_point_step", 1},
    {"CostCritic.inflation_layer_name", "inflation_layer"},
    {"CostCritic.num_threads", 1},
    {"CostCritic.use_footprint_bitmask", bitmask},
  });
  node = std::make_shared<nav2::LifecycleNode>("eq_" + std::to_string(id), options);
  std::string name = "mppi";
  ph = std::make_unique<mppi::ParametersHandler>(node, name);
  auto critic = std::make_unique<mppi::critics::CostCritic>();
  critic->on_configure(node, "mppi", "CostCritic", ros, ph.get());
  return critic;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const Case cases[] = {
    {"[ [2.230, 0.65], [-1.4312, 0.65], [-1.4312, -0.65], [2.230, -0.65] ]", 0.1, 5.0, false},
    {"[ [0.35, 0.25], [-0.35, 0.25], [-0.35, -0.25], [0.35, -0.25] ]", 0.05, 1.0, false},
    {"[ [0.3, 0.0], [0.1, 0.25], [-0.25, 0.2], [-0.3, -0.1], [0.05, -0.28] ]", 0.05, 1.0, false},
    // wider than 64 cells: mask rows are split into several words
    {"[ [2.230, 0.65], [-1.4312, 0.65], [-1.4312, -0.65], [2.230, -0.65] ]", 0.05, 3.0, false},
    // unknown space tracked: NO_INFORMATION cells do not block
    {"[ [2.230, 0.65], [-1.4312, 0.65], [-1.4312, -0.65], [2.230, -0.65] ]", 0.1, 5.0, true},
    {"[ [0.35, 0.25], [-0.35, 0.25], [-0.35, -0.25], [0.35, -0.25] ]", 0.05, 1.0, true},
  };
  const int kBatch = 400, kSteps = 60, kTrials = 150;
  long mismatches = 0, flag_mismatch = 0, checked = 0, colliding = 0, fail_flags = 0;
  int id = 0;
  for (const auto & c : cases) {
    auto ros = makeCostmap(c, id++);
    std::shared_ptr<nav2::LifecycleNode> na, nb;
    std::unique_ptr<mppi::ParametersHandler> pa, pb;
    auto plain = makeCritic(ros, false, id++, na, pa);
    auto masked = makeCritic(ros, true, id++, nb, pb);
    std::mt19937 rng(7 + id);
    for (int trial = 0; trial < kTrials; ++trial) {
      fillRandom(ros, rng);
      mppi::models::State state;
      state.local_path_length = 10.0f;
      mppi::models::Trajectories traj;
      fillTrajectories(traj, kBatch, kSteps, rng);
      mppi::models::Path path;
      geometry_msgs::msg::Pose goal;
      Eigen::ArrayXf ca = Eigen::ArrayXf::Zero(kBatch), cb = Eigen::ArrayXf::Zero(kBatch);
      float dt = 0.05f;
      mppi::CriticData da = {state, traj, path, goal, ca, dt,
        false, nullptr, nullptr, std::nullopt, std::nullopt, {}};
      mppi::CriticData db = {state, traj, path, goal, cb, dt,
        false, nullptr, nullptr, std::nullopt, std::nullopt, {}};
      plain->score(da);
      masked->score(db);
      checked += kBatch;
      colliding += (ca > 100.0f).count();
      fail_flags += da.fail_flag;
      if (std::memcmp(ca.data(), cb.data(), sizeof(float) * kBatch) != 0) {
        for (int i = 0; i < kBatch; ++i) {mismatches += ca(i) != cb(i);}
      }
      flag_mismatch += da.fail_flag != db.fail_flag;
    }
  }
  std::printf(
    "trajectories checked %ld, colliding %ld, all-colliding batches %ld, cost mismatches %ld, fail_flag mismatches %ld\n",
    checked, colliding, fail_flags, mismatches, flag_mismatch);
  rclcpp::shutdown();
  return (mismatches || flag_mismatch) ? 1 : 0;
}
