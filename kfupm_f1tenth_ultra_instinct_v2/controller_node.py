#!/usr/bin/env python3
"""
CIE-KFUPM F1TENTH Autonomous Racing Competition - Stage 2
Team: Ultra Instinct 2.0
Algorithm: Wide-Radius Momentum Tracker (WRMT)
Track: Levine Obstacle Circuit (levine_obs)

Features:
- Wide-radius cornering dynamics with effective curvature scaling
- Momentum preservation speed profiling (5.80 m/s straights, 2.35 m/s apex floor)
- Zero steering angle braking penalty to eliminate corner over-braking
- Stanley front-axle heading control with dynamic speed-dependent preview
- Active gyro yaw damping (-k_yaw * yaw_rate) for high-speed corridor stability
- Precomputed backwards deceleration and forward acceleration profile
- Forward LiDAR obstacle safety supervisor
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

try:
    from .slalom_waypoints import LEVINE_SLALOM_WAYPOINTS
except (ImportError, ValueError):
    from kfupm_f1tenth_ultra_instinct_v2.slalom_waypoints import LEVINE_SLALOM_WAYPOINTS


class UltraInstinct2Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Vehicle & Kinematic Parameters
        self.wheelbase = 0.33
        self.max_steer = 0.4189  # ~24 degrees (max servo limit)

        # High-Speed Momentum Tracking Limits
        self.v_max_straight = 5.80   # Boosted straightaway cruising speed (m/s)
        self.v_corner_floor = 2.35   # Momentum cornering floor (m/s)
        self.a_lat_max = 5.80        # Physical lateral acceleration limit (m/s^2)
        self.a_brake = 4.20          # Smooth kinetic-energy preserving deceleration (m/s^2)
        self.a_accel = 5.50          # Punchy corner-exit acceleration (m/s^2)

        # Calibrated Stanley & Yaw Damping Gains
        self.k_h = 0.85              # Heading error gain
        self.k_e = 2.80              # Cross-track error gain
        self.k_soft = 0.50           # Low-speed softening constant
        self.k_yaw = 0.018           # Active gyro yaw damping gain

        # Load isolated Levine Slalom Waypoints
        self.wps = np.array(LEVINE_SLALOM_WAYPOINTS, dtype=np.float32)
        self.num_pts = len(self.wps)
        self.headings, self.curvatures = self._compute_nominal_geometry()
        self.v_profile = self._build_momentum_velocity_profile()

        self.last_closest_idx = 0
        self.current_speed = 0.0
        self.last_steer = 0.0
        self.min_forward_dist = float('inf')

        # ROS 2 Subscriptions and Publishers
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            qos_profile=qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_cb,
            10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.get_logger().info(
            f"[Ultra Instinct 2.0] Wide-Radius Momentum Tracker Initialized "
            f"({self.num_pts} waypoints, max speed: {self.v_max_straight} m/s)."
        )

    def _compute_nominal_geometry(self):
        """Calculates waypoint headings and local 3-point circumscribed curvatures."""
        n = self.num_pts
        headings = np.zeros(n, dtype=np.float32)
        curvatures = np.zeros(n, dtype=np.float32)

        for i in range(n):
            p_prev = self.wps[(i - 1) % n]
            p_curr = self.wps[i]
            p_next = self.wps[(i + 1) % n]

            dx = p_next[0] - p_curr[0]
            dy = p_next[1] - p_curr[1]
            headings[i] = math.atan2(dy, dx)

            a = math.hypot(p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
            b = math.hypot(p_next[0] - p_curr[0], p_next[1] - p_curr[1])
            c = math.hypot(p_next[0] - p_prev[0], p_next[1] - p_prev[1])
            s = (a + b + c) / 2.0
            area = math.sqrt(max(0.0, s * (s - a) * (s - b) * (s - c)))
            curvatures[i] = (4.0 * area) / max(1e-5, a * b * c)

        return headings, curvatures

    def _build_momentum_velocity_profile(self):
        """
        Builds a momentum-preserving speed profile with backward deceleration
        and forward acceleration passes calibrated for Levine obstacle slalom.
        """
        n = self.num_pts
        ds = [math.hypot(self.wps[(i + 1) % n, 0] - self.wps[i, 0],
                         self.wps[(i + 1) % n, 1] - self.wps[i, 1]) for i in range(n)]

        v_targets = np.zeros(n, dtype=np.float32)

        for i in range(n):
            k = abs(self.curvatures[i])
            x, y = self.wps[i]

            k_eff = max(0.035, k * 0.70)
            v_corner = math.sqrt(self.a_lat_max / k_eff)
            v_target = max(self.v_corner_floor, min(self.v_max_straight, v_corner))

            # Calibrated safety zones for Levine Stage 2 obstacle corridors
            if y < 2.0:
                if -1.0 <= x <= 3.2:
                    v_target = min(v_target, 3.25)  # Obstacles 8, 11, 7 slalom
                elif 5.5 <= x <= 8.8:
                    v_target = min(v_target, 2.50)  # Obstacles 10, 6, 13 south pinch
                elif -8.8 <= x <= -5.8:
                    v_target = min(v_target, 3.45)  # Obstacle 9
                elif -12.0 <= x <= -9.0:
                    v_target = min(v_target, 3.40)  # South straight entry approach
                elif x > 8.8 or x < -12.0:
                    v_target = min(v_target, 2.35)  # Corner apexes
            elif y > 7.5:
                if -11.5 <= x <= -8.8:
                    v_target = min(v_target, 3.35)  # Obstacle 4 chicane
                elif -6.0 <= x <= 5.5:
                    v_target = self.v_max_straight  # Open North straight (Full 5.80 m/s!)
                elif x < -12.0 or x > 8.5:
                    v_target = min(v_target, 2.35)  # Corner apexes
            elif x > 9.5:
                v_target = min(v_target, 5.25)      # East straight
            elif x < -12.5:
                v_target = min(v_target, 5.35)      # West straight

            v_targets[i] = v_target

        # Backward deceleration pass (a_brake = 4.2 preserves kinetic energy)
        for _ in range(3):
            for i in range(n - 1, -1, -1):
                next_i = (i + 1) % n
                v_lim = math.sqrt(v_targets[next_i]**2 + 2.0 * self.a_brake * ds[i])
                if v_targets[i] > v_lim:
                    v_targets[i] = v_lim

        # Forward acceleration pass onto straightaways
        for _ in range(3):
            for i in range(n):
                next_i = (i + 1) % n
                v_lim = math.sqrt(v_targets[i]**2 + 2.0 * self.a_accel * ds[i])
                if v_targets[next_i] > v_lim:
                    v_targets[next_i] = v_lim

        return v_targets

    def scan_cb(self, scan: LaserScan):
        """LiDAR safety supervisor monitoring the forward cone."""
        ranges = np.array(scan.ranges, dtype=np.float32)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment

        fov = math.radians(7)
        mid = int((0.0 - angle_min) / angle_inc)
        span = int(fov / angle_inc)
        fwd = ranges[max(0, mid - span):min(len(ranges), mid + span + 1)]
        valid = fwd[(fwd > 0.1) & (~np.isnan(fwd)) & (~np.isinf(fwd))]
        self.min_forward_dist = float(np.min(valid)) if len(valid) > 0 else 5.0

    def odom_cb(self, odom: Odometry):
        # 1. Vehicle Pose, Heading, and Angular Velocity
        xc = odom.pose.pose.position.x
        yc = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaw_rate = odom.twist.twist.angular.z
        self.current_speed = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)

        # 2. Dynamic Front Axle Preview (scales smoothly with velocity)
        preview_dist = max(0.12, min(0.20, 0.10 + 0.02 * self.current_speed))
        xf = xc + (self.wheelbase + preview_dist) * math.cos(yaw)
        yf = yc + (self.wheelbase + preview_dist) * math.sin(yaw)

        # 3. Local Window Search for Closest Waypoint
        search_radius = min(self.num_pts // 2, 80)
        idx_win = [(self.last_closest_idx + offset) % self.num_pts for offset in range(-search_radius, search_radius)]
        wps_win = self.wps[idx_win]
        dists_sq = (wps_win[:, 0] - xf)**2 + (wps_win[:, 1] - yf)**2
        min_win_idx = int(np.argmin(dists_sq))
        idx = idx_win[min_win_idx]
        self.last_closest_idx = idx

        # 4. Path Geometry at Preview Waypoint
        psi_p = self.headings[idx]
        wx, wy = self.wps[idx]

        dx = xf - wx
        dy = yf - wy

        # Cross-track error (positive if vehicle is to the left of path)
        e_y = -dx * math.sin(psi_p) + dy * math.cos(psi_p)

        # Heading error wrapped to [-pi, pi]
        theta_e = math.atan2(math.sin(psi_p - yaw), math.cos(psi_p - yaw))

        # 5. Kinematic Control Law with Active Gyro Yaw Damping
        steer_fb = math.atan2(-self.k_e * e_y, self.current_speed + self.k_soft)
        steer_damp = -self.k_yaw * yaw_rate

        total_steer = self.k_h * theta_e + steer_fb + steer_damp
        steer_cmd = float(np.clip(total_steer, -self.max_steer, self.max_steer))
        self.last_steer = steer_cmd

        # 6. Momentum Preservation Speed Lookup
        # Lookahead 2-5 points accounts for actuator delay without premature braking
        brake_pts = max(2, min(5, int(self.current_speed * 0.15 / 0.20) + 1))
        target_v = min(self.v_profile[(idx + k) % self.num_pts] for k in range(min(brake_pts, self.num_pts)))
        target_v = min(self.v_max_straight, target_v)

        # Enforce minimum rolling floor
        target_v = max(1.20, target_v)

        # Forward LiDAR Safety Proximity Override
        if self.min_forward_dist < 0.45:
            target_v = min(target_v, 1.80)

        # 7. Publish Drive Command
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_racecar/base_link'
        msg.drive.steering_angle = steer_cmd
        msg.drive.speed = float(target_v)
        self.drive_pub.publish(msg)


def main():
    rclpy.init()
    node = UltraInstinct2Controller()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        if "rcl_shutdown" not in str(e) and "context is not valid" not in str(e):
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
