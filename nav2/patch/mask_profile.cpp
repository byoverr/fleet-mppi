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

// Where the time of the masked CostCritic loop goes, and whether cheaper pre-checks help.
// Real MPPI rollouts (default critics, 2000 x 56, warm optimizer) are taken from the optimizer,
// then the CostCritic collision loop is replayed with several filters in front of the exact
// footprint check. Every variant must give the same per-trajectory cost as the exact loop.

#include <chrono>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_costmap_2d/inflation_layer_interface.hpp>
#include "nav2_ros_common/tf2_factories.hpp"

#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/footprint_bitmask.hpp"
#include "nav2_mppi_controller/tools/footprint_bitmask_v2.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

#include "utils.hpp"

namespace
{

struct Robot
{
  const char * name;
  const char * footprint;
  double res, inflation, wall;
};

const Robot kRobots[] = {
  {"amr_0.7x0.5@0.05", "[[0.35,0.25],[-0.35,0.25],[-0.35,-0.25],[0.35,-0.25]]", 0.05, 1.5, 0.6},
  {"forklift_2.4x1.2@0.05", "[[1.6,0.6],[-0.8,0.6],[-0.8,-0.6],[1.6,-0.6]]", 0.05, 3.0, 0.95},
  {"big_3.66x1.3@0.1", "[[2.230,0.65],[-1.4312,0.65],[-1.4312,-0.65],[2.230,-0.65]]", 0.1, 5.0, 1.7},
  {"big_3.66x1.3@0.05", "[[2.230,0.65],[-1.4312,0.65],[-1.4312,-0.65],[2.230,-0.65]]", 0.05, 5.0,
    1.7},
};
const char * kScenes[] = {"corridor", "wall", "clutter"};
constexpr double kRobotX = 12.0, kRobotY = 15.0;

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> makeCostmapRos(const Robot & r)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"width", 30}, {"height", 30}, {"resolution", r.res},
    {"footprint", r.footprint}, {"footprint_padding", 0.1},
    {"plugins", std::vector<std::string>{"inflation_layer"}},
    {"inflation_layer.plugin", "nav2_costmap_2d::InflationLayer"},
    {"inflation_layer.inflation_radius", r.inflation},
    {"inflation_layer.cost_scaling_factor", 0.7},
  });
  auto ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  ros->on_configure(rclcpp_lifecycle::State{});
  auto * cm = ros->getCostmap();
  ros->getLayeredCostmap()->resizeMap(
    cm->getSizeInCellsX(), cm->getSizeInCellsY(), r.res, 0.0, 0.0);
  return ros;
}

void fillScene(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros, const Robot & r, int scene)
{
  auto * cm = ros->getCostmap();
  const double res = r.res;
  auto infl = nav2_costmap_2d::InflationLayerInterface::getInflationLayer(ros, "inflation_layer");
  std::vector<std::pair<int, int>> obs;
  auto rect = [&](double x0, double y0, double x1, double y1) {
      for (double x = x0; x <= x1; x += res) {
        for (double y = y0; y <= y1; y += res) {
          unsigned int mx, my;
          if (cm->worldToMap(x, y, mx, my)) {obs.emplace_back(mx, my);}
        }
      }
    };
  const double half = r.wall + 1.3;
  if (scene == 0) {
    rect(8.0, kRobotY + half, 28.0, kRobotY + half);
    rect(8.0, kRobotY - half, 28.0, kRobotY - half);
    rect(19.0, kRobotY + 0.9 * half / 3.0, 19.6, kRobotY + half);
  } else if (scene == 1) {
    rect(8.0, kRobotY + r.wall, 28.0, kRobotY + r.wall);
  } else {
    // shelves and posts around the path, the robot itself starts in a free spot
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> ux(kRobotX + 1.0, kRobotX + 12.0), uy(-4.0, 4.0);
    std::uniform_real_distribution<double> us(0.05, 1.2);
    for (int k = 0; k < 18; ++k) {
      const double x = ux(rng), y = kRobotY + uy(rng);
      if (std::abs(y - kRobotY) < r.wall) {continue;}
      rect(x, y, x + us(rng), y + (k % 3 == 0 ? 0.05 : us(rng)));
    }
  }
  const int rc = static_cast<int>(infl->getInflationRadius() / res);
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

class ProbeOptimizer : public mppi::Optimizer
{
public:
  const mppi::models::Trajectories & rollouts() const {return generated_trajectories_;}
};

struct Counts
{
  long poses = 0, center_free = 0, center_hit = 0, gated = 0, footprint = 0;
  long box_free = 0, mask_free = 0, must_hit = 0, exact = 0, exact_hit = 0, collided = 0;
};

enum Mode {kExact, kMask, kMaskMust, kBoxMask, kBoxMaskMust, kBand4, kBand8, kBand16, kTable};
constexpr int kModes = 9;

// The CostCritic loop (PR #6228 order, without the cost accumulation constants)
struct Replay
{
  nav2_costmap_2d::Costmap2D * cm;
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker{nullptr};
  nav2_costmap_2d::Footprint footprint;
  mppi::FootprintBitmask v1;
  mppi::FootprintBitmaskV2 v2, vb[3];
  float possible = 0.0f;

  void run(const mppi::models::Trajectories & t, Mode mode, std::vector<float> & out, Counts & c)
  {
    const float ox = cm->getOriginX(), oy = cm->getOriginY(), res = cm->getResolution();
    const unsigned int sx = cm->getSizeInCellsX(), sy = cm->getSizeInCellsY();
    const unsigned char * grid = cm->getCharMap();
    const int rows = t.x.rows(), cols = t.x.cols();
    out.assign(rows, 0.0f);
    for (int i = 0; i < rows; ++i) {
      float traj_cost = 0.0f;
      for (int j = cols - 1; j >= 0; --j) {
        const float X = t.x(i, j), Y = t.y(i, j), yaw = t.yaws(i, j);
        ++c.poses;
        float cost;
        if (X < ox || Y < oy || static_cast<unsigned>((X - ox) / res) >= sx ||
          static_cast<unsigned>((Y - oy) / res) >= sy)
        {
          cost = 255.0f;
        } else {
          cost = grid[static_cast<unsigned>((Y - oy) / res) * sx +
              static_cast<unsigned>((X - ox) / res)];
          if (cost < 1.0f) {++c.center_free; continue;}
        }
        bool hit;
        if (cost == 254.0f || cost == 253.0f) {
          ++c.center_hit; hit = true;
        } else if (!(cost >= possible || possible < 1.0f)) {
          ++c.gated; hit = cost == 255.0f;
        } else {
          ++c.footprint;
          int verdict = 2;
          if (mode == kMask) {
            verdict = v1.mayCollide(X, Y, yaw) ? 2 : 0;
          } else if (mode == kTable) {
            verdict = v2.queryTable(X, Y, yaw);
          } else if (mode >= kBand4) {
            verdict = vb[mode - kBand4].queryBand(X, Y, yaw);
          } else if (mode != kExact) {
            verdict = v2.query(X, Y, yaw, mode == kBoxMask || mode == kBoxMaskMust,
                mode == kMaskMust || mode == kBoxMaskMust);
          }
          if (verdict == 3) {++c.box_free; verdict = 0;}
          if (verdict == 0) {
            ++c.mask_free; hit = false;
          } else if (verdict == 1) {
            ++c.must_hit; hit = true;
          } else {
            ++c.exact;
            const unsigned char s = static_cast<unsigned char>(
              checker.footprintCostAtPose(X, Y, yaw, footprint));
            hit = s == 254 || s == 255;  // unknown space is not tracked in these scenes
            c.exact_hit += hit;
          }
        }
        if (hit) {traj_cost = -1.0f; ++c.collided; break;}
        traj_cost += cost;
      }
      out[i] = traj_cost;
    }
  }
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const int reps = argc > 1 ? std::atoi(argv[1]) : 20;
  std::printf("robot scene | footprint-checked share | per pose in footprint branch: mask free, "
    "must hit, exact | exact hits | time per cycle (ms) for: exact, mask, mask+must, box+mask, "
    "box+mask+must | plane, sat build (us)\n");
  int node_id = 0;
  for (const auto & robot : kRobots) {
    for (int scene = 0; scene < 3; ++scene) {
      auto ros = makeCostmapRos(robot);
      fillScene(ros, robot, scene);
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
      params.emplace_back("dummy.CostCritic.num_threads", 1);
      rclcpp::NodeOptions options;
      options.parameter_overrides(params);
      auto node = getDummyNode(options);
      (void)node_id++;
      std::string name = "test";
      auto ph = std::make_unique<mppi::ParametersHandler>(node, name);
      auto tf_buffer = nav2::create_transform_buffer(node);
      auto optimizer = std::make_shared<ProbeOptimizer>();
      nav2::LifecycleNode::WeakPtr weak_node{node};
      optimizer->initialize(weak_node, node->get_name(), ros, tf_buffer, ph.get());
      TestPose start{kRobotX, kRobotY};
      TestPathSettings ps{start, 120u, 0.1, 0.0};
      auto pose = getDummyPointStamped(node, start);
      geometry_msgs::msg::Twist velocity;
      velocity.linear.x = 1.0;
      auto path = getIncrementalDummyPath(node, ps);
      nav2_core::GoalChecker * goal_checker{nullptr};
      for (int k = 0; k < 5; ++k) {
        optimizer->evalControl(pose, velocity, path, path.poses.back().pose, goal_checker);
      }

      Replay rp;
      rp.cm = ros->getCostmap();
      rp.checker.setCostmap(rp.cm);
      rp.footprint = ros->getRobotFootprint();
      auto infl = nav2_costmap_2d::InflationLayerInterface::getInflationLayer(ros,
          "inflation_layer");
      rp.possible = static_cast<float>(infl->computeCost(
          ros->getLayeredCostmap()->getCircumscribedRadius() / robot.res));
      rp.v1.configure(rp.footprint, robot.res, 128, 2);
      rp.v2.configure(rp.footprint, robot.res, 128, 2);
      for (int q = 0; q < 3; ++q) {
        rp.vb[q].setBand(4 << q);
        rp.vb[q].configure(rp.footprint, robot.res, 128, 2);
        rp.vb[q].buildPlane(*rp.cm, false);
      }
      auto tb = std::chrono::steady_clock::now();
      for (int k = 0; k < 100; ++k) {rp.vb[1].buildBands();}
      const double band_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - tb).count() / 100;
      rp.vb[0].buildBands(); rp.vb[2].buildBands();
      rp.v2.buildPlane(*rp.cm, false);
      tb = std::chrono::steady_clock::now();
      for (int k = 0; k < 100; ++k) {rp.v2.buildTable();}
      const double table_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - tb).count() / 100;
      using clk = std::chrono::steady_clock;
      auto t0 = clk::now();
      for (int k = 0; k < 100; ++k) {rp.v2.buildPlane(*rp.cm, false);}
      auto t1 = clk::now();
      for (int k = 0; k < 100; ++k) {rp.v2.buildSat();}
      auto t2 = clk::now();
      rp.v1.buildPlane(*rp.cm, false);
      const double plane_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / 100;
      const double sat_us = std::chrono::duration<double, std::micro>(t2 - t1).count() / 100;

      double ms[kModes] = {};
      Counts c[kModes];
      long mismatches = 0;
      for (int rep = 0; rep < reps; ++rep) {
        optimizer->evalControl(pose, velocity, path, path.poses.back().pose, goal_checker);
        const auto & tr = optimizer->rollouts();
        std::vector<float> ref, out;
        for (int m = 0; m < kModes; ++m) {
          auto a = clk::now();
          rp.run(tr, static_cast<Mode>(m), m == 0 ? ref : out, c[m]);
          ms[m] += std::chrono::duration<double, std::milli>(clk::now() - a).count();
          if (m > 0) {
            for (size_t q = 0; q < ref.size(); ++q) {mismatches += ref[q] != out[q];}
          }
        }
      }
      const Counts & e = c[0];
      const Counts & f = c[4];
      const double fp = std::max<long>(1, f.footprint);
      std::printf("%s %s | %.0f%% | %.1f%% %.1f%% %.1f%% | exact-hit %.1f%% of exact calls | "
        "%.3f %.3f %.3f %.3f %.3f | %.1f %.1f | box free %.1f%% | collided traj %.1f%% | "
        "mismatches %ld | band 4/8/16: %.3f %.3f %.3f ms, band free %.1f%% %.1f%% %.1f%%, "
        "band plane build %.1f us | table: %.3f ms, free %.1f%%, build %.1f us\n",
        robot.name, kScenes[scene], 100.0 * e.footprint / e.poses,
        100.0 * c[2].mask_free / std::max<long>(1, c[2].footprint),
        100.0 * c[2].must_hit / std::max<long>(1, c[2].footprint),
        100.0 * c[2].exact / std::max<long>(1, c[2].footprint),
        100.0 * c[1].exact_hit / std::max<long>(1, c[1].exact),
        ms[0] / reps, ms[1] / reps, ms[2] / reps, ms[3] / reps, ms[4] / reps, plane_us, sat_us,
        100.0 * f.box_free / fp,
        100.0 * e.collided / (reps * 2000.0), mismatches, ms[5] / reps, ms[6] / reps, ms[7] / reps,
        100.0 * c[5].box_free / std::max<long>(1, c[5].footprint),
        100.0 * c[6].box_free / std::max<long>(1, c[6].footprint),
        100.0 * c[7].box_free / std::max<long>(1, c[7].footprint), band_us, ms[8] / reps,
        100.0 * c[8].box_free / std::max<long>(1, c[8].footprint), table_us);
      std::fflush(stdout);
    }
  }
  rclcpp::shutdown();
  return 0;
}
