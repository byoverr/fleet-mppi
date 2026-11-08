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

// Closed-loop driving with the Nav2 MPPI optimizer: quality versus CPU.
// A 0.7 x 0.5 m AMR follows a straight-ish global plan through five scenes (slalom in an aisle,
// a box on the plan, a door, a box that appears ahead, an L-turn). The robot tracks the command
// with the optimizer's acceleration limits. Collisions are checked on the exact geometry.
//
// Usage: closed_loop_benchmark batch lowpass_hz bias seed0 seeds scenes [stock]
//   bias bits: 1 = stop sample, 2 = plan sample;  scenes: e.g. "01234"
//   stock = 1 switches off all exact speedups (masks, fused generation, fast paths)
// Environment: VX_MAX (default 0.5, the Nav2 default), LP_ORDER (default 2), TRACE=1 prints poses,
//   LOCAL (rolling local costmap side in m, default 5; 0 = the whole 30 m map is the costmap),
//   REGEN=1 (regenerate_noises), BANK=rows (noise bank), BANK_STEP=rows per cycle (0 = random block), TEMP=MPPI temperature, OFF="mask fuse reg fast pa" (switch single speedups off).
// Prints one JSON line per episode.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/inflation_layer_interface.hpp>
#include "nav2_ros_common/tf2_factories.hpp"

#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

#include "utils.hpp"

namespace
{

constexpr double kRes = 0.05, kHalfL = 0.35, kHalfW = 0.25;
constexpr double kPeriod = 0.05;  // controller at 20 Hz
const char * kFootprint = "[[0.35,0.25],[-0.35,0.25],[-0.35,-0.25],[0.35,-0.25]]";

struct Rect {double x0, y0, x1, y1;};
struct Scene
{
  std::vector<Rect> obs;
  std::vector<std::pair<double, double>> way;  // global plan corners
  double sx, sy, syaw;
  Rect drop{0, 0, 0, 0};
  double drop_x = 1e9;  // box appears when the robot passes this x
};

Scene makeScene(int id, unsigned seed)
{
  std::mt19937 rng(seed * 7919u + static_cast<unsigned>(id));
  auto U = [&](double a, double b) {return std::uniform_real_distribution<double>(a, b)(rng);};
  Scene s;
  const double cy = 15.0;
  if (id == 0) {  // slalom: 2.4 m aisle, boxes alternate from both walls, 1.0 m into the aisle
    s.obs = {{2, cy + 1.2, 28, cy + 1.25}, {2, cy - 1.25, 28, cy - 1.2}};
    double x = 7.0 + U(-0.5, 0.5);
    for (int k = 0; k < 4; ++k) {
      const double d = U(0.85, 1.05);
      if (k % 2 == 0) {s.obs.push_back({x, cy + 1.2 - d, x + 0.6, cy + 1.2});} else {
        s.obs.push_back({x, cy - 1.2, x + 0.6, cy - 1.2 + d});
      }
      x += U(3.2, 4.2);
    }
    s.way = {{3, cy}, {27, cy}};
    s.sx = 3; s.sy = cy; s.syaw = 0;
  } else if (id == 1) {  // a 0.6 m box sits on the plan in the open, with a wall on one side
    const double by = cy + U(-0.2, 0.2);
    s.obs = {{12, by - 0.3, 12.6, by + 0.3}, {6, cy - 2.0, 20, cy - 1.95}};
    s.way = {{3, cy}, {22, cy}};
    s.sx = 3; s.sy = cy; s.syaw = 0;
  } else if (id == 2) {  // door: wall at x = 14 with a 0.9 m gap, approached at an angle
    const double gy = cy + U(-0.4, 0.4);
    s.obs = {{14, 2, 14.1, gy - 0.45}, {14, gy + 0.45, 14.1, 28}};
    s.way = {{3, cy - 4}, {12.8, gy}, {15.3, gy}, {24, cy + 4}};
    s.sx = 3; s.sy = cy - 4; s.syaw = std::atan2(gy - (cy - 4), 9.8);
  } else if (id == 3) {  // a box appears 1.6-2.0 m ahead in a 2.4 m aisle
    s.obs = {{2, cy + 1.2, 28, cy + 1.25}, {2, cy - 1.25, 28, cy - 1.2}};
    const double side = U(0, 1) < 0.5 ? 1.0 : -1.0, ahead = U(1.6, 2.0), w = U(0.9, 1.1);
    s.drop_x = 9.0 + U(-1, 1);
    const double bx = s.drop_x + ahead;
    s.drop = side > 0 ? Rect{bx, cy + 1.2 - w, bx + 0.5, cy + 1.2} :
      Rect{bx, cy - 1.2, bx + 0.5, cy - 1.2 + w};
    s.way = {{3, cy}, {25, cy}};
    s.sx = 3; s.sy = cy; s.syaw = 0;
  } else {  // L-turn in a 1.6 m corridor
    const double cx = 16.0 + U(-0.5, 0.5);
    s.obs = {{2, cy + 0.8, cx - 0.8, cy + 0.85}, {2, cy - 0.85, cx + 0.8, cy - 0.8},
      {cx + 0.8, cy - 0.85, cx + 0.85, 28}, {cx - 0.85, cy + 0.8, cx - 0.8, 28}};
    s.way = {{3, cy}, {cx, cy}, {cx, 27}};
    s.sx = 3; s.sy = cy; s.syaw = 0;
  }
  return s;
}

// Distance between the robot rectangle and an axis-aligned obstacle rectangle (0 if overlapping)
double polyDist(const std::vector<std::pair<double, double>> & a,
  const std::vector<std::pair<double, double>> & b)
{
  auto overlap = [](const auto & p, const auto & q) {
      for (size_t i = 0; i < p.size(); ++i) {
        const auto & u = p[i]; const auto & v = p[(i + 1) % p.size()];
        const double nx = -(v.second - u.second), ny = v.first - u.first;
        double pmin = 1e18, pmax = -1e18, qmin = 1e18, qmax = -1e18;
        for (auto & w : p) {const double d = nx * w.first + ny * w.second; pmin = std::min(pmin, d); pmax = std::max(pmax, d);}
        for (auto & w : q) {const double d = nx * w.first + ny * w.second; qmin = std::min(qmin, d); qmax = std::max(qmax, d);}
        if (pmax < qmin || qmax < pmin) {return false;}
      }
      return true;
    };
  if (overlap(a, b) && overlap(b, a)) {return 0.0;}
  auto ptSeg = [](auto p, auto u, auto v) {
      const double dx = v.first - u.first, dy = v.second - u.second;
      double t = ((p.first - u.first) * dx + (p.second - u.second) * dy) / (dx * dx + dy * dy);
      t = std::clamp(t, 0.0, 1.0);
      return std::hypot(p.first - u.first - t * dx, p.second - u.second - t * dy);
    };
  double d = 1e18;
  for (size_t i = 0; i < a.size(); ++i) {
    for (auto & p : b) {d = std::min(d, ptSeg(p, a[i], a[(i + 1) % a.size()]));}
  }
  for (size_t i = 0; i < b.size(); ++i) {
    for (auto & p : a) {d = std::min(d, ptSeg(p, b[i], b[(i + 1) % b.size()]));}
  }
  return d;
}

double clearance(double x, double y, double yaw, const std::vector<Rect> & obs)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::vector<std::pair<double, double>> r;
  for (auto [lx, ly] : {std::pair{kHalfL, kHalfW}, {-kHalfL, kHalfW}, {-kHalfL, -kHalfW},
      {kHalfL, -kHalfW}})
  {
    r.emplace_back(x + c * lx - s * ly, y + s * lx + c * ly);
  }
  double d = 1e18;
  for (auto & o : obs) {
    if (o.x0 > x + 1.5 || o.x1 < x - 1.5 || o.y0 > y + 1.5 || o.y1 < y - 1.5) {
      d = std::min(d, 1.0);
      continue;
    }
    d = std::min(d, polyDist(r, {{o.x0, o.y0}, {o.x1, o.y0}, {o.x1, o.y1}, {o.x0, o.y1}}));
  }
  return d;
}

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> makeCostmapRos(int size_m)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"width", size_m}, {"height", size_m}, {"resolution", kRes},
    {"footprint", kFootprint},
    {"footprint_padding", 0.0},
    {"plugins", std::vector<std::string>{"inflation_layer"}},
    {"inflation_layer.plugin", "nav2_costmap_2d::InflationLayer"},
    {"inflation_layer.inflation_radius", 0.7},
    {"inflation_layer.cost_scaling_factor", 3.0},
  });
  auto ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  ros->on_configure(rclcpp_lifecycle::State{});
  auto * cm = ros->getCostmap();
  ros->getLayeredCostmap()->resizeMap(
    cm->getSizeInCellsX(), cm->getSizeInCellsY(), kRes, 0.0, 0.0);
  return ros;
}

void paint(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros, nav2_costmap_2d::Costmap2D * cm,
  const std::vector<Rect> & rects)
{
  auto infl = nav2_costmap_2d::InflationLayerInterface::getInflationLayer(ros, "inflation_layer");
  std::vector<std::pair<int, int>> cells;
  for (auto & r : rects) {
    for (double x = r.x0; x <= r.x1 + 1e-9; x += kRes / 2) {
      for (double y = r.y0; y <= r.y1 + 1e-9; y += kRes / 2) {
        unsigned int mx, my;
        if (cm->worldToMap(x, y, mx, my)) {cells.emplace_back(mx, my);}
      }
    }
  }
  std::sort(cells.begin(), cells.end());
  cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
  const int rc = static_cast<int>(std::ceil(infl->getInflationRadius() / kRes));
  const int sx = cm->getSizeInCellsX(), sy = cm->getSizeInCellsY();
  for (auto [ox, oy] : cells) {
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

// Rolling local costmap like Nav2's: a size x size window centred on the robot, copied from
// the global truth map (outside the timed controller call).
void rollWindow(nav2_costmap_2d::Costmap2D * local, const nav2_costmap_2d::Costmap2D & world,
  double x, double y)
{
  const double half = local->getSizeInMetersX() / 2;
  local->updateOrigin(x - half, y - half);
  const double ox = local->getOriginX(), oy = local->getOriginY();
  for (unsigned int j = 0; j < local->getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < local->getSizeInCellsX(); ++i) {
      unsigned int gx, gy;
      const double wx = ox + (i + 0.5) * kRes, wy = oy + (j + 0.5) * kRes;
      local->setCost(i, j, world.worldToMap(wx, wy, gx, gy) ? world.getCost(gx, gy) :
        nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

// What the noise thread does per cycle with regenerate_noises: draw 2 x K x 56 normals
double noiseGenUs(int batch)
{
  std::default_random_engine gen;
  std::normal_distribution<float> nv(0.0f, 0.2f), nw(0.0f, 0.4f);
  Eigen::ArrayXXf a, b;
  std::vector<double> t;
  float sink = 0.0f;
  for (int r = 0; r < 30; ++r) {
    const auto c0 = std::chrono::steady_clock::now();
    a = Eigen::ArrayXXf::NullaryExpr(batch, 56, [&]() {return nv(gen);});
    b = Eigen::ArrayXXf::NullaryExpr(batch, 56, [&]() {return nw(gen);});
    t.push_back(std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - c0).count());
    sink += a(0, 0) + b(0, 0);
  }
  std::sort(t.begin(), t.end());
  return t.at(t.size() / 2) + 0.0 * sink;
}

struct PathPt {double x, y, yaw, s;};

std::vector<PathPt> densePath(const Scene & sc)
{
  std::vector<PathPt> p;
  double s = 0;
  for (size_t i = 0; i + 1 < sc.way.size(); ++i) {
    auto [x0, y0] = sc.way[i];
    auto [x1, y1] = sc.way[i + 1];
    const double L = std::hypot(x1 - x0, y1 - y0), yaw = std::atan2(y1 - y0, x1 - x0);
    const int n = static_cast<int>(L / kRes);
    for (int k = 0; k < n; ++k) {
      p.push_back({x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, yaw, s});
      s += L / n;
    }
  }
  auto [xe, ye] = sc.way.back();
  p.push_back({xe, ye, p.back().yaw, s});
  return p;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 7) {
    std::fprintf(stderr, "usage: batch lowpass_hz bias seed0 seeds scenes [stock]\n");
    return 1;
  }
  const int batch = std::atoi(argv[1]);
  const double lp = std::atof(argv[2]);
  const int bias = std::atoi(argv[3]);
  const int seed0 = std::atoi(argv[4]), nseeds = std::atoi(argv[5]);
  const std::string scenes = argv[6];
  const bool stock = argc > 7 && std::atoi(argv[7]) != 0;
  const double vx_max = std::getenv("VX_MAX") ? std::atof(std::getenv("VX_MAX")) : 0.5;
  const int lp_order = std::getenv("LP_ORDER") ? std::atoi(std::getenv("LP_ORDER")) : 2;
  const bool trace = std::getenv("TRACE") != nullptr;
  const bool regen = std::getenv("REGEN") != nullptr;  // fresh noise every cycle
  const int bank = std::getenv("BANK") ? std::atoi(std::getenv("BANK")) : 0;  // noise bank rows
  const int bank_step = std::getenv("BANK_STEP") ? std::atoi(std::getenv("BANK_STEP")) : 0;
  const double temp = std::getenv("TEMP") ? std::atof(std::getenv("TEMP")) : 0.0;  // 0 = Nav2 default 0.3
  // OFF="mask fuse reg fast pa" switches single speedups off (for the ablation)
  const std::string off = std::getenv("OFF") ? std::getenv("OFF") : "";
  auto on = [&](const char * k) {return !stock && off.find(k) == std::string::npos;};
  const int local_m = std::getenv("LOCAL") ? std::atoi(std::getenv("LOCAL")) : 5;  // 0: whole map
  const double noise_us = noiseGenUs(batch);
  rclcpp::init(0, nullptr);

  for (char sch : scenes) {
    const int id = sch - '0';
    for (int seed = seed0; seed < seed0 + nseeds; ++seed) {
      Scene sc = makeScene(id, static_cast<unsigned>(seed));
      auto costmap_ros = makeCostmapRos(local_m > 0 ? local_m : 30);
      nav2_costmap_2d::Costmap2D world(600, 600, kRes, 0.0, 0.0);
      auto * truth = local_m > 0 ? &world : costmap_ros->getCostmap();
      paint(costmap_ros, truth, sc.obs);
      std::vector<Rect> live = sc.obs;
      const auto plan = densePath(sc);

      const std::string n = "dummy.";
      std::vector<rclcpp::Parameter> params{
        {n + "batch_size", batch}, {n + "time_steps", 56}, {n + "iteration_count", 1},
        {n + "model_dt", 0.05}, {n + "motion_model", std::string("diff_drive")},
        {n + "diff_drive.plugin", std::string("mppi::DiffDriveMotionModel")},
        {"controller_frequency", 1.0 / kPeriod},
        {n + "vx_max", vx_max}, {n + "vx_min", -0.35}, {n + "wz_max", 1.9},
        {n + "critics", std::vector<std::string>{"ConstraintCritic", "CostCritic", "GoalCritic",
            "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
            "PreferForwardCritic"}},
        {n + "CostCritic.consider_footprint", true},
        {n + "CostCritic.inflation_layer_name", std::string("inflation_layer")},
        {n + "noise_seed", 1000 * seed + id},
        {n + "lowpass_cutoff", lp}, {n + "lowpass_order", lp_order},
        {n + "bias_stop_sample", (bias & 1) != 0}, {n + "bias_plan_sample", (bias & 2) != 0},
        {n + "regenerate_noises", regen}, {n + "noise_bank_rows", bank}, {n + "noise_bank_step", bank_step},
        {n + "CostCritic.use_footprint_bitmask", on("mask")},
        {n + "fuse_generation", on("fuse")}, {n + "fuse_generation_registers", on("reg")},
        {n + "fast_reductions", on("fast")}, {n + "PathAlignCritic.packet_evaluation", on("pa")},
      };
      if (temp > 0.0) {params.emplace_back(n + "temperature", temp);}
      rclcpp::NodeOptions options;
      options.parameter_overrides(params);
      auto node = getDummyNode(options);
      std::string name = "test";
      auto ph = std::make_unique<mppi::ParametersHandler>(node, name);
      auto tf_buffer = nav2::create_transform_buffer(node);
      auto optimizer = std::make_shared<mppi::Optimizer>();
      nav2::LifecycleNode::WeakPtr weak_node{node};
      optimizer->initialize(weak_node, node->get_name(), costmap_ros, tf_buffer, ph.get());

      double x = sc.sx, y = sc.sy, yaw = sc.syaw, v = 0, w = 0, t = 0;
      const double goal_x = plan.back().x, goal_y = plan.back().y;
      const double t_max = plan.back().s / vx_max * 2.5 + 15.0;
      size_t idx = 0;
      const double prune = std::max(2.0, 1.2 * vx_max * 56 * 0.05);
      bool dropped = false;
      std::string status = "timeout";
      double min_clear = 1e9, len = 0, sum_dw2 = 0, sum_dv2 = 0, prev_cv = 0, prev_cw = 0;
      int novalid = 0, cycles = 0;
      std::vector<double> cpu;
      while (t < t_max) {
        if (!dropped && x > sc.drop_x) {
          paint(costmap_ros, truth, {sc.drop});
          live.push_back(sc.drop);
          dropped = true;
        }
        // Local plan: from the closest point forward, 1.2 x the horizon length (PathHandler-like)
        size_t best = idx;
        double bd = 1e18;
        for (size_t k = idx; k < plan.size() && plan[k].s < plan[idx].s + 3.0; ++k) {
          const double d = std::hypot(plan[k].x - x, plan[k].y - y);
          if (d < bd) {bd = d; best = k;}
        }
        idx = best;
        auto path = getDummyPath(node);
        for (size_t k = idx; k < plan.size() && plan[k].s <= plan[idx].s + prune; ++k) {
          auto p = getDummyPointStamped(node, TestPose{plan[k].x, plan[k].y});
          p.pose.orientation.z = std::sin(plan[k].yaw / 2);
          p.pose.orientation.w = std::cos(plan[k].yaw / 2);
          path.poses.push_back(p);
        }
        if (local_m > 0) {rollWindow(costmap_ros->getCostmap(), world, x, y);}
        auto pose = getDummyPointStamped(node, TestPose{x, y});
        pose.pose.orientation.z = std::sin(yaw / 2);
        pose.pose.orientation.w = std::cos(yaw / 2);
        auto goal = getDummyPointStamped(node, TestPose{goal_x, goal_y}).pose;
        goal.orientation.z = std::sin(plan.back().yaw / 2);
        goal.orientation.w = std::cos(plan.back().yaw / 2);
        geometry_msgs::msg::Twist vel;
        vel.linear.x = v;
        vel.angular.z = w;
        double cv = 0, cw = 0;
        const auto c0 = std::chrono::steady_clock::now();
        try {
          auto [cmd, traj] = optimizer->evalControl(pose, vel, path, goal, nullptr);
          cv = cmd.twist.linear.x;
          cw = cmd.twist.angular.z;
        } catch (const nav2_core::NoValidControl &) {
          ++novalid;  // the controller server would stop and retry; we stop for one period
        }
        cpu.push_back(std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - c0).count());
        ++cycles;
        sum_dv2 += (cv - prev_cv) * (cv - prev_cv);
        sum_dw2 += (cw - prev_cw) * (cw - prev_cw);
        prev_cv = cv;
        prev_cw = cw;
        // Plant: follows the command within the acceleration limits, 5 substeps
        for (int k = 0; k < 5; ++k) {
          const double h = kPeriod / 5;
          v += std::clamp(cv - v, -3.0 * h, 3.0 * h);
          w += std::clamp(cw - w, -3.5 * h, 3.5 * h);
          x += v * std::cos(yaw) * h;
          y += v * std::sin(yaw) * h;
          yaw += w * h;
          len += std::abs(v) * h;
        }
        t += kPeriod;
        if (trace) {std::fprintf(stderr, "%.2f %.3f %.3f %.3f %.2f %.2f\n", t, x, y, yaw, v, w);}
        const double cl = clearance(x, y, yaw, live);
        min_clear = std::min(min_clear, cl);
        if (cl <= 0.0) {status = "collision"; break;}
        if (std::hypot(x - goal_x, y - goal_y) < 0.25) {status = "success"; break;}
        if (novalid > 40) {status = "novalid"; break;}
      }
      optimizer->shutdown();
      std::sort(cpu.begin(), cpu.end());
      double mean = 0;
      for (double c : cpu) {mean += c;}
      mean /= std::max<size_t>(1, cpu.size());
      std::printf("{\"scene\": %d, \"seed\": %d, \"batch\": %d, \"lp\": %g, \"lp_order\": %d, "
        "\"bias\": %d, \"stock\": %d, \"regen\": %d, \"bank\": %d, \"bank_step\": %d, \"temp\": %g, \"off\": \"%s\", \"local\": %d, \"noise_us\": %.1f, \"vx_max\": %g, \"status\": \"%s\", \"time\": %.2f, "
        "\"min_clear\": %.3f, \"len\": %.2f, \"novalid\": %d, \"dv_rms\": %.4f, \"dw_rms\": %.4f, "
        "\"cpu_med\": %.1f, \"cpu_mean\": %.1f, \"cpu_p95\": %.1f, \"cycles\": %d}\n",
        id, seed, batch, lp, lp_order, bias, stock ? 1 : 0, regen ? 1 : 0, bank, bank_step, temp > 0.0 ? temp : 0.3, off.c_str(), local_m, noise_us, vx_max, status.c_str(), t, min_clear,
        len, novalid, std::sqrt(sum_dv2 / std::max(1, cycles)),
        std::sqrt(sum_dw2 / std::max(1, cycles)), cpu.at(cpu.size() / 2), mean,
        cpu.at(cpu.size() * 95 / 100), cycles);
      std::fflush(stdout);
    }
  }
  rclcpp::shutdown();
  return 0;
}
