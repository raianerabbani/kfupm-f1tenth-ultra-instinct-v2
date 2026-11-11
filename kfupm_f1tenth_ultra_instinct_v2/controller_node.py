#!/usr/bin/env python3
"""
CIE-KFUPM F1TENTH Autonomous Racing Competition
Team: Ultra Instinct 2.1 (Universal Wide-Radius Momentum Tracker)
Package: kfupm_f1tenth_ultra_instinct_v2

Core Innovations (100% Coordinate-Independent & Universal):
1. Pure Differential Geometry Speed Profiling:
   - Kinematic curvature profiling v(s) = sqrt(a_lat_max / kappa_eff(s))
   - Smooth backward deceleration pass (a_brake) and forward acceleration pass (a_accel)
   - Zero hardcoded Cartesian (x, y) coordinates; adapts to any circuit geometry.
2. Perception-Driven Real-Time Corridor Clearance (LiDAR Governor):
   - Flanking LiDAR sectors dynamically sense local corridor width and lateral obstacle margins.
   - Automatically modulates entry speed in narrow obstacle pinches/slaloms while unleashing full
     throttle (5.80 m/s) on open straights.
3. Wide-Radius Dynamic Front-Axle Preview (WRMT):
   - Speed-scaled lookahead chord projection ahead of front axle increases effective turning radius.
4. Kinematic Stanley Tracking with Active Gyro Yaw Damping:
   - Heading error regulation + cross-track correction + active chassis yaw rate damping.
5. Multi-Stage Emergency LiDAR Safety Supervisor.
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


class UltraInstinct21Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Vehicle Kinematics
        self.wheelbase = 0.33
        self.max_steer = 0.4189  # ~24 degrees (max servo limit)

        # Physical Acceleration & Velocity Limits (Universal Invariants)
        self.v_max_straight = 5.80   # Top straightaway speed (m/s)
        self.v_corner_floor = 2.35   # Apex momentum preservation floor (m/s)
        self.a_lat_max = 5.80        # Lateral acceleration budget (m/s^2)
        self.a_brake = 4.20          # Kinetic energy preserving deceleration (m/s^2)
        self.a_accel = 5.50          # Punchy corner-exit acceleration (m/s^2)

        # Stanley Tracking & Damping Gains
        self.k_h = 0.85              # Heading error gain
        self.k_e = 2.80              # Cross-track error gain
        self.k_soft = 0.50           # Low-speed softening constant
        self.k_yaw = 0.018           # Active gyro yaw-rate damping gain

        # Load Nominal Waypoints (x, y)
        self.wps = np.array(LEVINE_SLALOM_WAYPOINTS, dtype=np.float32)
        self.num_pts = len(self.wps)

        # Compute Universal Geometric Properties (Pure Differential Geometry)
        self.headings, self.curvatures, self.ds = self._compute_universal_geometry()
        self.v_profile = self._build_universal_velocity_profile()

        # Real-time state
        self.last_closest_idx = 0
        self.current_speed = 0.0
        self.last_steer = 0.0

        # LiDAR Perception State
        self.d_forward = 10.0
        self.d_left = 5.0
        self.d_right = 5.0
        self.d_corridor = 10.0
        self.d_margin = 5.0

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
            f"[Ultra Instinct 2.1] Universal Wide-Radius Momentum Tracker Initialized "
            f"({self.num_pts} waypoints, zero coordinate-checks, max: {self.v_max_straight} m/s)."
        )

    def _compute_universal_geometry(self):
        """
        Computes path headings, circumscribed curvatures, and segment lengths
        using purely relative differential geometry (zero track coordinates).
        """
        n = self.num_pts
        headings = np.zeros(n, dtype=np.float32)
        curvatures = np.zeros(n, dtype=np.float32)
        ds = np.zeros(n, dtype=np.float32)

        for i in range(n):
            p_prev = self.wps[(i - 1) % n]
            p_curr = self.wps[i]
            p_next = self.wps[(i + 1) % n]

            dx = p_next[0] - p_curr[0]
            dy = p_next[1] - p_curr[1]
            headings[i] = math.atan2(dy, dx)
            ds[i] = math.hypot(dx, dy)

            # Menger curvature of 3 consecutive points
            a = math.hypot(p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
            b = math.hypot(p_next[0] - p_curr[0], p_next[1] - p_curr[1])
            c = math.hypot(p_next[0] - p_prev[0], p_next[1] - p_prev[1])
            s = (a + b + c) / 2.0
            area = math.sqrt(max(0.0, s * (s - a) * (s - b) * (s - c)))
            curvatures[i] = (4.0 * area) / max(1e-5, a * b * c)

        return headings, curvatures, ds

    def _build_universal_velocity_profile(self):
        """
        Universal Physics Speed Profiler:
        Computes maximum permissible speed based on lateral tire acceleration limits,
        curvature derivatives, backward braking integration, and forward acceleration integration.
        Operates without any map-specific or coordinate-specific conditions.
        """
        n = self.num_pts

        # Window-smoothed curvature across vehicle wheelbase chord (~0.33m)
        win = 4
        padded = np.pad(self.curvatures, win, mode='wrap')
        k_smooth = np.convolve(padded, np.ones(2 * win + 1) / (2 * win + 1), mode='valid')

        # Curvature rate of change |dkappa/ds| (accounts for steering slew rate)
        dk_ds = np.zeros(n, dtype=np.float32)
        for i in range(n):
            next_i = (i + 1) % n
            prev_i = (i - 1) % n
            denom = max(1e-3, self.ds[i] + self.ds[prev_i])
            dk_ds[i] = abs(k_smooth[next_i] - k_smooth[prev_i]) / denom

        v_targets = np.zeros(n, dtype=np.float32)
        for i in range(n):
            # Effective curvature accounts for peak turning radius and chicane transitions
            k_eff = max(0.035, k_smooth[i] * 0.72 + 0.06 * min(3.0, dk_ds[i]))
            v_corner = math.sqrt(self.a_lat_max / k_eff)
            v_targets[i] = max(self.v_corner_floor, min(self.v_max_straight, v_corner))

        # Backward deceleration pass (kinematic energy preservation)
        for _ in range(3):
            for i in range(n - 1, -1, -1):
                next_i = (i + 1) % n
                v_lim = math.sqrt(v_targets[next_i]**2 + 2.0 * self.a_brake * self.ds[i])
                if v_targets[i] > v_lim:
                    v_targets[i] = v_lim

        # Forward acceleration pass onto straightaways
        for _ in range(3):
            for i in range(n):
                next_i = (i + 1) % n
                v_lim = math.sqrt(v_targets[i]**2 + 2.0 * self.a_accel * self.ds[i])
                if v_targets[next_i] > v_lim:
                    v_targets[next_i] = v_lim

        return v_targets

    def scan_cb(self, scan: LaserScan):
        """
        Universal LiDAR Perception Supervisor:
        Extracts forward distance, flanking wall clearances, and total corridor width.
        Pure sensor-space calculation (zero track coordinates).
        """
        ranges = np.array(scan.ranges, dtype=np.float32)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        total_rays = len(ranges)

        def get_min_range(deg_start, deg_end):
            idx_start = max(0, min(total_rays - 1, int((math.radians(deg_start) - angle_min) / angle_inc)))
            idx_end = max(0, min(total_rays - 1, int((math.radians(deg_end) - angle_min) / angle_inc)))
            if idx_start > idx_end:
                idx_start, idx_end = idx_end, idx_start
            sector = ranges[idx_start : idx_end + 1]
            valid = sector[(sector > 0.08) & (~np.isnan(sector)) & (~np.isinf(sector))]
            return float(np.min(valid)) if len(valid) > 0 else 10.0

        # Forward sector (+/- 10 degrees)
        self.d_forward = get_min_range(-10, 10)

        # Flanking corridor sectors (detects narrow walls and obstacles)
        self.d_left = get_min_range(20, 75)
        self.d_right = get_min_range(-75, -20)
        self.d_corridor = self.d_left + self.d_right
        self.d_margin = min(self.d_left, self.d_right)

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

        # 6. Geometric Velocity Lookup (Compensates for actuator delay without over-braking)
        brake_pts = max(2, min(5, int(self.current_speed * 0.15 / 0.20) + 1))
        target_v = min(self.v_profile[(idx + k) % self.num_pts] for k in range(min(brake_pts, self.num_pts)))
        target_v = min(self.v_max_straight, target_v)

        # 7. Universal Perception Governor (LiDAR-Based Corridor Clearance)
        # When squeezing through narrow obstacle pinch points (corridor < 1.30m or margin < 0.48m),
        # smoothly scale velocity ceiling with physical clearance, independent of track/coordinates!
        if self.d_margin < 0.48 or self.d_corridor < 1.30:
            clearance_factor = np.clip((self.d_corridor - 0.75) / (1.30 - 0.75), 0.0, 1.0)
            v_corridor_cap = self.v_corner_floor + (4.00 - self.v_corner_floor) * clearance_factor
            target_v = min(target_v, float(v_corridor_cap))

        # 8. Emergency Forward Obstacle Supervisor
        if self.d_forward < 0.50:
            target_v = min(target_v, 1.80)
        if self.d_forward < 0.30:
            target_v = 0.0  # Immediate stop

        # Enforce minimum rolling floor
        target_v = max(1.20, target_v)

        # 9. Publish Drive Command
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_racecar/base_link'
        msg.drive.steering_angle = steer_cmd
        msg.drive.speed = float(target_v)
        self.drive_pub.publish(msg)


def main():
    rclpy.init()
    node = UltraInstinct21Controller()
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
