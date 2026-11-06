"""End-to-end check of trajectory_player: a "server" publishes MPPI-like trajectories that are
stamped with an old robot pose (as if they crossed a slow link), sometimes drops them, and stops
sending altogether. The player's cmd_vel must follow the trajectory at the right index."""

import math
import sys
import time

import rclpy
import rclpy.time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Trajectory, TrajectoryPoint
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

DT, N = 0.05, 56


def make_traj(node, stamp):
    tr = Trajectory()
    tr.header.stamp = stamp.to_msg()
    tr.header.frame_id = "odom"
    for i in range(N):
        p = TrajectoryPoint()
        p.header.stamp = (stamp + Duration(seconds=i * DT)).to_msg()
        p.velocity.linear.x = 0.01 * i  # the index is readable from the command
        p.velocity.angular.z = 0.1
        tr.points.append(p)
    return tr


def main():
    rclpy.init()
    node = rclpy.create_node("fake_server")
    pub = node.create_publisher(Trajectory, "optimal_trajectory",
                                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    got = []
    node.create_subscription(TwistStamped, "cmd_vel",
                             lambda m: got.append((rclpy.time.Time.from_msg(m.header.stamp), m.twist.linear.x)), 10)
    errors, samples, ages = [], 0, []
    delay = 0.3
    stamps = []
    t_end = time.time() + 6.0
    k = 0
    while time.time() < t_end:
        now = node.get_clock().now()
        phase = 6.0 - (t_end - time.time())
        # 10 Hz plans, planned from a pose that is `delay` old, 20 % of the packets lost,
        # silence after 3 s
        if phase < 3.0 and k % 2 == 0 and (k // 2) % 5 != 3:
            stamps.append(now - Duration(seconds=delay))
            pub.publish(make_traj(node, stamps[-1]))
        k += 1
        t0 = time.time()
        while time.time() - t0 < 0.05:
            rclpy.spin_once(node, timeout_sec=0.01)
        for tt, vx in got:
            if not stamps or phase < 0.5 or phase > 3.0:
                continue
            samples += 1
            # vx = 0.01 * i tells which point was played: the plan stamp must lie in
            # (t - (i + 1) DT, t - i DT] for one of the plans actually sent
            i = round(vx / 0.01)
            lo, hi = tt - Duration(seconds=(i + 1) * DT), tt - Duration(seconds=i * DT)
            used = [s for s in stamps if lo.nanoseconds - 1e6 < s.nanoseconds <= hi.nanoseconds + 1e6]
            if not used:
                errors.append((round(phase, 2), i))
            else:
                ages.append((tt - used[-1]).nanoseconds * 1e-9)
        got.clear()
    # after the silence the plan (2.8 s long, 1.0 s max age) must have expired: commands are zero
    rclpy.spin_once(node, timeout_sec=0.1)
    got.clear()
    t0 = time.time()
    while time.time() - t0 < 0.3:
        rclpy.spin_once(node, timeout_sec=0.01)
    stopped = all(vx == 0.0 for _, vx in got) and len(got) > 5
    print(f"samples {samples}, index errors {len(errors)}, plan age played: "
          f"min {min(ages):.3f} max {max(ages):.3f} s, stopped after silence {stopped}")
    if errors:
        print(errors[:10])
    rclpy.shutdown()
    sys.exit(0 if not errors and stopped and samples > 50 else 1)


if __name__ == "__main__":
    main()
