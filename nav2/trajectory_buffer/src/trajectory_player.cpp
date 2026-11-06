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

// Robot-side player of the MPPI optimal trajectory (publish_optimal_trajectory: true).
//
// Nav2 stamps the trajectory with the robot pose the controller planned from, and point i with
// stamp + i * model_dt. That pose stamp comes from the robot's own odometry, so the player can
// index the trajectory with its own clock: the command for "now" is the velocity of the last
// point whose stamp is not in the future. Link delay is compensated without clock sync, and a
// lost packet only means the robot keeps playing the previous plan. When the plan runs out (or
// is older than max_age) the robot stops. Safety (e.g. nav2_collision_monitor) stays downstream.

#include <algorithm>
#include <memory>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

class TrajectoryPlayer : public rclcpp::Node
{
public:
  TrajectoryPlayer()
  : Node("trajectory_player")
  {
    rate_ = declare_parameter("rate", 50.0);
    max_age_ = declare_parameter("max_age", 1.0);
    frame_ = declare_parameter("base_frame", std::string("base_link"));
    sub_ = create_subscription<nav_msgs::msg::Trajectory>(
      "optimal_trajectory", rclcpp::QoS(1).best_effort(),
      [this](nav_msgs::msg::Trajectory::ConstSharedPtr msg) {onTrajectory(msg);});
    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_), [this]() {onTimer();});
  }

private:
  void onTrajectory(nav_msgs::msg::Trajectory::ConstSharedPtr msg)
  {
    if (msg->points.empty()) {return;}
    // a reordered (older) plan never replaces a newer one
    if (traj_ && rclcpp::Time(msg->header.stamp) < rclcpp::Time(traj_->header.stamp)) {return;}
    traj_ = msg;
  }

  void onTimer()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = frame_;
    if (traj_) {
      const rclcpp::Time t = now();
      const double age = (t - rclcpp::Time(traj_->header.stamp)).seconds();
      const auto & pts = traj_->points;
      // last point whose stamp is not in the future
      auto it = std::upper_bound(
        pts.begin(), pts.end(), t, [](const rclcpp::Time & tt, const auto & p) {
          return tt < rclcpp::Time(p.header.stamp);
        });
      const bool inside = it != pts.begin() && it != pts.end();
      if (age <= max_age_ && inside) {
        cmd.twist = std::prev(it)->velocity;
      }
    }
    pub_->publish(cmd);
  }

  double rate_, max_age_;
  std::string frame_;
  nav_msgs::msg::Trajectory::ConstSharedPtr traj_;
  rclcpp::Subscription<nav_msgs::msg::Trajectory>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPlayer>());
  rclcpp::shutdown();
  return 0;
}
