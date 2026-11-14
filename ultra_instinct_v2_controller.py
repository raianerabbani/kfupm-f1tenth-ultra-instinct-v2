

import math
import time
import numpy as np
from scipy.ndimage import label, distance_transform_edt, gaussian_filter1d
from scipy.interpolate import splprep, splev

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class UltraInstinct21Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Vehicle Kinematics
        self.wheelbase = 0.33
        self.max_steer = 0.4189  # ~24 degrees (max servo limit)

        # Physical Acceleration & Velocity Limits (Universal Invariants)
        self.v_max_straight = 5.80   # Full throttle on all open straightaways (m/s)
        self.v_corner_floor = 2.35   # Apex momentum preservation floor (m/s)
        self.a_lat_max = 5.40        # Physical lateral tire acceleration budget (m/s^2)
        self.a_brake = 4.20          # Kinetic energy preserving deceleration (m/s^2)
        self.a_accel = 5.50          # Punchy corner-exit acceleration (m/s^2)

        # Stanley Tracking & Active Gyro Damping Gains
        self.k_h = 0.85              # Heading error gain
        self.k_e = 2.80              # Cross-track error gain
        self.k_soft = 0.50           # Low-speed softening constant
        self.k_yaw = 0.018           # Active gyro yaw-rate damping gain

        # Dynamic Raceline State (populated automatically from /map + /ego_racecar/odom)
        self.wps = None
        self.num_pts = 0
        self.headings = None
        self.curvatures = None
        self.ds = None
        self.v_profile = None
        self.raceline_ready = False
        self.map_msg = None
        self.init_pose = None

        # Real-time tracking & perception state
        self.last_closest_idx = 0
        self.current_speed = 0.0
        self.last_steer = 0.0
        self.d_forward_clearance = 10.0

        # QoS for /map (nav2_map_server uses TRANSIENT_LOCAL + RELIABLE)
        map_qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        map_qos_volatile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ROS 2 Subscriptions and Publishers
        self.map_sub_latched = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_cb,
            qos_profile=map_qos_latched,
        )
        self.map_sub_volatile = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_cb,
            qos_profile=map_qos_volatile,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            qos_profile=qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_cb,
            10,
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10,
        )

        self.get_logger().info(
            "[Ultra Instinct 2.1] Universal Map-to-Raceline Controller Initialized. "
            "Waiting for /map and /ego_racecar/odom..."
        )

    def map_cb(self, msg: OccupancyGrid):
        """Receives the OccupancyGrid map and triggers raceline generation once pose is known."""
        if self.map_msg is None:
            self.map_msg = msg
            self.get_logger().info(
                f"[Ultra Instinct 2.1] Received /map ({msg.info.width}x{msg.info.height}, "
                f"res={msg.info.resolution:.3f}m)."
            )
            if self.init_pose is not None and not self.raceline_ready:
                self._initialize_raceline()

    @staticmethod
    def _trace_outer_contour(mask_bool: np.ndarray) -> np.ndarray:

        ih_labels, _ = label(mask_bool, structure=np.ones((3, 3), dtype=int))
        ih_sizes = np.bincount(ih_labels.ravel())
        ih_sizes[0] = 0
        blob = ih_labels == int(np.argmax(ih_sizes))
        rs, cs = np.where(blob)
        r_start, c_start = int(rs[0]), int(cs[0])

        dirs = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]
        contour = [(c_start, r_start)]
        cr, cc = r_start, c_start
        back_dir = 6  # Came from North (-1, 0) since top-left pixel has no North neighbor
        H, W = blob.shape

        max_steps = len(rs) * 2 + 1000
        for _ in range(max_steps):
            found = False
            for k in range(1, 9):
                d_idx = (back_dir + k) % 8
                nr, nc = cr + dirs[d_idx][0], cc + dirs[d_idx][1]
                if 0 <= nr < H and 0 <= nc < W and blob[nr, nc]:
                    cr, cc = nr, nc
                    back_dir = (d_idx + 4) % 8
                    found = True
                    break
            if not found or (cr == r_start and cc == c_start and len(contour) > 20):
                break
            contour.append((cc, cr))

        return np.array(contour, dtype=np.float64)

    def _generate_raceline_from_map(self, msg: OccupancyGrid, sx: float, sy: float, stheta: float) -> np.ndarray:

        w_full = int(msg.info.width)
        h_full = int(msg.info.height)
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)

        occ_grid = np.array(msg.data, dtype=np.int8).reshape((h_full, w_full))

        # 1. Crop tightly to the occupied map bounding box (+20px margin) for ~10x speedup
        occ_r, occ_c = np.where(occ_grid >= 50)
        if len(occ_r) > 0:
            r_min = max(0, int(occ_r.min()) - 20)
            r_max = min(h_full, int(occ_r.max()) + 21)
            c_min = max(0, int(occ_c.min()) - 20)
            c_max = min(w_full, int(occ_c.max()) + 21)
        else:
            r_min, r_max, c_min, c_max = 0, h_full, 0, w_full

        occ_sub = occ_grid[r_min:r_max, c_min:c_max]
        ox_sub = ox + c_min * res
        oy_sub = oy + r_min * res
        H, W = occ_sub.shape

        # 2. Euclidean Distance Transform (EDT) & gradient on real obstacle/wall grid
        walls = (occ_sub >= 50) | (occ_sub < 0)
        dist_m = distance_transform_edt(~walls) * res
        grad_r, grad_c = np.gradient(dist_m)
        grad_norm = np.hypot(grad_r, grad_c) + 1e-8
        grad_x = grad_c / grad_norm
        grad_y = grad_r / grad_norm

        # 3. Seal open SLAM doorways into exterior voids using local corridor width
        c0 = int(np.clip(round((sx - ox_sub) / res), 0, W - 1))
        r0 = int(np.clip(round((sy - oy_sub) / res), 0, H - 1))
        local_r = float(dist_m[max(0, r0 - 15):min(H, r0 + 16), max(0, c0 - 15):min(W, c0 + 16)].max())
        max_track_half_width = max(3.5, local_r * 2.0)

        ext_core = dist_m > max_track_half_width
        if np.any(ext_core):
            exterior_void = (distance_transform_edt(~ext_core) * res) <= (max_track_half_width + 0.05)
        else:
            exterior_void = np.zeros_like(walls)

        # 4. Identify and remove small isolated obstacle islands (< 3.5 m^2) to extract clean track topology
        non_track = walls | exterior_void
        non_track[0, :] = True
        non_track[-1, :] = True
        non_track[:, 0] = True
        non_track[:, -1] = True

        wall_labels, _ = label(non_track, structure=np.ones((3, 3), dtype=int))
        comp_sizes = np.bincount(wall_labels.ravel())
        max_obs_pixels = int(round(3.5 / (res ** 2)))
        is_small_obs = (wall_labels > 0) & (comp_sizes[wall_labels] < max_obs_pixels)

        clean_walls = non_track & (~is_small_obs)
        clean_dist_m = distance_transform_edt(~clean_walls) * res

        drivable_clean = clean_dist_m >= (res * 1.2)
        labeled, _ = label(drivable_clean, structure=np.ones((3, 3), dtype=int))
        cid = int(labeled[r0, c0])
        if cid == 0:
            win = max(3, int(round(0.6 / res)))
            sub_l = labeled[max(0, r0 - win):min(H, r0 + win + 1), max(0, c0 - win):min(W, c0 + win + 1)]
            nz = sub_l[sub_l > 0]
            if len(nz) > 0:
                cid = int(np.bincount(nz).argmax())
        track_mask = labeled == cid

        # 5. Jordan Curve Inner/Outer Boundary Separation + Morphological Opening of Wall-Attached Obstacles
        boundary_walls = ~track_mask
        boundary_walls[0, :] = True
        boundary_walls[-1, :] = True
        boundary_walls[:, 0] = True
        boundary_walls[:, -1] = True

        bw_labels, _ = label(boundary_walls, structure=np.ones((3, 3), dtype=int))
        outer_cid = int(bw_labels[0, 0])
        bw_sizes = np.bincount(bw_labels.ravel())
        bw_sizes[0] = 0
        bw_sizes[outer_cid] = 0
        inner_cid = int(np.argmax(bw_sizes))

        r_open = min(0.65, local_r * 0.80)
        outer_raw = bw_labels == outer_cid
        inner_raw = bw_labels == inner_cid

        outer_core = (distance_transform_edt(outer_raw) * res) >= r_open
        outer_smooth = (distance_transform_edt(~outer_core) * res) <= r_open if np.any(outer_core) else outer_raw
        inner_core = (distance_transform_edt(inner_raw) * res) >= r_open
        inner_smooth = (distance_transform_edt(~inner_core) * res) <= r_open if np.any(inner_core) else inner_raw

        d_outer = distance_transform_edt(~outer_smooth) * res
        d_inner = distance_transform_edt(~inner_smooth) * res

        # 6. Zero-Level-Set Reference Centerline Extraction (d_outer == d_inner)
        inner_half_bool = (d_outer >= d_inner) & (~outer_smooth)
        main_contour = self._trace_outer_contour(inner_half_bool)
        raw_pts = np.column_stack([ox_sub + main_contour[:, 0] * res, oy_sub + main_contour[:, 1] * res])

        # Align index 0 to vehicle starting position and orient along initial heading stheta
        idx0 = int(np.argmin((raw_pts[:, 0] - sx) ** 2 + (raw_pts[:, 1] - sy) ** 2))
        raw_pts = np.roll(raw_pts, -idx0, axis=0)
        tang = raw_pts[min(5, len(raw_pts) - 1)] - raw_pts[0]
        if tang[0] * math.cos(stheta) + tang[1] * math.sin(stheta) < 0:
            raw_pts = np.vstack([raw_pts[0:1], raw_pts[1:][::-1]])

        sub_step = max(1, int(round(0.12 / res)))
        raw_pts = gaussian_filter1d(raw_pts, sigma=0.35 / res, axis=0, mode='wrap')[::sub_step]
        N = len(raw_pts)

        # Compute reference tangents, normals, and curvatures
        p_prev = np.roll(raw_pts, 1, axis=0)
        p_next = np.roll(raw_pts, -1, axis=0)
        t_vec = p_next - p_prev
        tangents = t_vec / (np.linalg.norm(t_vec, axis=1, keepdims=True) + 1e-6)
        area = 0.5 * (
            p_prev[:, 0] * (raw_pts[:, 1] - p_next[:, 1])
            + raw_pts[:, 0] * (p_next[:, 1] - p_prev[:, 1])
            + p_next[:, 0] * (p_prev[:, 1] - raw_pts[:, 1])
        )
        d123 = (
            np.linalg.norm(p_prev - raw_pts, axis=1)
            * np.linalg.norm(raw_pts - p_next, axis=1)
            * np.linalg.norm(p_next - p_prev, axis=1)
        )
        curvatures = np.where(d123 > 1e-6, 4.0 * area / d123, 0.0)
        normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])

        # 7. Vectorized 2-Pass Periodic Dynamic Programming (DP) Corridor Slalom
        max_dev = min(2.2, max(0.95, local_r * 1.3))
        states = np.linspace(-max_dev, max_dev, 91)
        K = len(states)
        col_thresh = 0.24
        safety_margin = 0.36

        cand_x = raw_pts[:, 0, None] + states[None, :] * normals[:, 0, None]
        cand_y = raw_pts[:, 1, None] + states[None, :] * normals[:, 1, None]
        cpx = np.clip(np.round((cand_x - ox_sub) / res).astype(int), 0, W - 1)
        cpy = np.clip(np.round((cand_y - oy_sub) / res).astype(int), 0, H - 1)
        in_bounds = (
            (cand_x >= ox_sub)
            & (cand_x < ox_sub + W * res)
            & (cand_y >= oy_sub)
            & (cand_y < oy_sub + H * res)
        )
        valid_cell = (
            in_bounds
            & track_mask[cpy, cpx]
            & (~walls[cpy, cpx])
            & ((1.0 - curvatures[:, None] * states[None, :]) >= 0.25)
        )
        c_grid = dist_m[cpy, cpx]
        u2 = 0.05 * (states[None, :] ** 2)

        cost_grid = np.where(
            ~valid_cell,
            1e8,
            np.where(
                c_grid < col_thresh,
                1e7 * (col_thresh - c_grid) + 1e5,
                np.where(
                    c_grid < safety_margin,
                    2500.0 * ((safety_margin - c_grid) ** 2) + u2,
                    u2 - 2.0 * np.minimum(c_grid, safety_margin + 0.06),
                ),
            ),
        )

        w_smooth = 250.0
        diff = states[None, :] - states[:, None]
        trans = np.where(np.abs(diff) > 0.20, 1e8, w_smooth * (diff ** 2))

        # Pass 1: find optimal periodic boundary state at index 0
        dp1 = np.zeros((N, K), dtype=np.float64)
        dp1[0] = cost_grid[0]
        for i in range(1, N):
            dp1[i] = np.min(dp1[i - 1, :, None] + trans + cost_grid[i, None, :], axis=0)
        best_k = int(np.argmin(dp1[-1] + w_smooth * (states ** 2)))

        # Pass 2: backtrack optimal periodic offset trajectory u*(s)
        dp2 = np.full((N, K), 1e8, dtype=np.float64)
        parent2 = np.zeros((N, K), dtype=int)
        dp2[0, best_k] = cost_grid[0, best_k]
        for i in range(1, N):
            total = dp2[i - 1, :, None] + trans + cost_grid[i, None, :]
            parent2[i] = np.argmin(total, axis=0)
            dp2[i] = np.min(total, axis=0)

        opt_offsets = np.zeros(N, dtype=np.float64)
        curr_k = best_k
        opt_offsets[-1] = states[curr_k]
        for i in range(N - 2, -1, -1):
            curr_k = parent2[i + 1, curr_k]
            opt_offsets[i] = states[curr_k]

        # 8. Frenet 1D Offset Smoothing + Gentle 2D Corner Rounding + Periodic B-Spline Resampling
        smoothed_u = opt_offsets.copy()
        cpx0 = np.clip(np.round(((raw_pts[:, 0] + smoothed_u * normals[:, 0]) - ox_sub) / res).astype(int), 0, W - 1)
        cpy0 = np.clip(np.round(((raw_pts[:, 1] + smoothed_u * normals[:, 1]) - oy_sub) / res).astype(int), 0, H - 1)
        req_c = np.minimum(0.35, np.maximum(0.24, dist_m[cpy0, cpx0] - 0.01))

        for stride in [8, 4, 2, 1]:
            for _ in range(35):
                mid_u = 0.5 * (np.roll(smoothed_u, stride) + np.roll(smoothed_u, -stride))
                cand_u = smoothed_u + 0.35 * (mid_u - smoothed_u)
                cx_u = raw_pts[:, 0] + cand_u * normals[:, 0]
                cy_u = raw_pts[:, 1] + cand_u * normals[:, 1]
                cpx = np.clip(np.round((cx_u - ox_sub) / res).astype(int), 0, W - 1)
                cpy = np.clip(np.round((cy_u - oy_sub) / res).astype(int), 0, H - 1)
                valid = track_mask[cpy, cpx] & (~walls[cpy, cpx]) & (dist_m[cpy, cpx] >= req_c)
                smoothed_u[valid] = cand_u[valid]

        safe_pts = raw_pts + normals * gaussian_filter1d(smoothed_u, sigma=1.5, mode='wrap')[:, None]

        cpx0 = np.clip(np.round((safe_pts[:, 0] - ox_sub) / res).astype(int), 0, W - 1)
        cpy0 = np.clip(np.round((safe_pts[:, 1] - oy_sub) / res).astype(int), 0, H - 1)
        local_max_c = np.minimum(0.36, np.maximum(0.24, dist_m[cpy0, cpx0] - 0.01))
        opt_pts = safe_pts.copy()

        for stride in [3, 2, 1]:
            for _ in range(20):
                mid = 0.5 * (np.roll(opt_pts, stride, axis=0) + np.roll(opt_pts, -stride, axis=0))
                cand_pts = opt_pts + 0.30 * (mid - opt_pts)
                cpx = np.clip(np.round((cand_pts[:, 0] - ox_sub) / res).astype(int), 0, W - 1)
                cpy = np.clip(np.round((cand_pts[:, 1] - oy_sub) / res).astype(int), 0, H - 1)
                valid = track_mask[cpy, cpx] & (~walls[cpy, cpx]) & (dist_m[cpy, cpx] >= local_max_c)
                opt_pts[valid] = cand_pts[valid]

        # Enforce minimum 0.25m obstacle clearance prior to B-spline fit to preserve C2 continuity
        for _ in range(3):
            cpx = np.clip(np.round((opt_pts[:, 0] - ox_sub) / res).astype(int), 0, W - 1)
            cpy = np.clip(np.round((opt_pts[:, 1] - oy_sub) / res).astype(int), 0, H - 1)
            c_val = dist_m[cpy, cpx]
            need = c_val < 0.25
            if not np.any(need):
                break
            opt_pts[need, 0] += (0.26 - c_val[need]) * grad_x[cpy[need], cpx[need]]
            opt_pts[need, 1] += (0.26 - c_val[need]) * grad_y[cpy[need], cpx[need]]

        tck, _ = splprep([opt_pts[:, 0], opt_pts[:, 1]], s=0.005, per=True)
        u_dense = np.linspace(0.0, 1.0, 5000)
        xd, yd = splev(u_dense, tck)
        cum_s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xd), np.diff(yd)))])
        total_len = float(cum_s[-1])
        num_wps = max(50, int(round(total_len / 0.10)))
        u_target = np.interp(np.linspace(0.0, total_len, num_wps, endpoint=False), cum_s, u_dense)
        rx, ry = splev(u_target, tck)
        wps = np.column_stack([rx, ry]).astype(np.float32)

        return wps

    def _initialize_raceline(self):
        """Generates the raceline and velocity profile from /map and initial pose."""
        if self.raceline_ready or self.map_msg is None or self.init_pose is None:
            return

        t0 = time.time()
        sx, sy, stheta = self.init_pose
        self.wps = self._generate_raceline_from_map(self.map_msg, sx, sy, stheta)
        self.num_pts = len(self.wps)
        self.headings, self.curvatures, self.ds = self._compute_universal_geometry()
        self.v_profile = self._build_universal_velocity_profile()
        self.last_closest_idx = 0
        self.raceline_ready = True
        dt_ms = (time.time() - t0) * 1000.0

        self.get_logger().info(
            f"[Ultra Instinct 2.1] Generated {self.num_pts} universal raceline waypoints from /map "
            f"in {dt_ms:.1f} ms (v_min={self.v_profile.min():.2f} m/s, v_max={self.v_profile.max():.2f} m/s)."
        )

    def _compute_universal_geometry(self):

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

            # Menger circumscribed curvature of 3 consecutive points
            a = math.hypot(p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
            b = math.hypot(p_next[0] - p_curr[0], p_next[1] - p_curr[1])
            c = math.hypot(p_next[0] - p_prev[0], p_next[1] - p_prev[1])
            s = (a + b + c) / 2.0
            area = math.sqrt(max(0.0, s * (s - a) * (s - b) * (s - c)))
            curvatures[i] = (4.0 * area) / max(1e-5, a * b * c)

        return headings, curvatures, ds

    def _build_universal_velocity_profile(self):

        n = self.num_pts

        # Light 3-point noise filter (preserves true peak curvature in slaloms)
        win = 1
        padded = np.pad(self.curvatures, win, mode='wrap')
        k_clean = np.convolve(padded, np.ones(2 * win + 1) / (2 * win + 1), mode='valid')

        v_targets = np.zeros(n, dtype=np.float32)
        for i in range(n):
            k_eff = max(0.038, float(k_clean[i]))
            v_corner = math.sqrt(self.a_lat_max / k_eff)
            v_targets[i] = max(self.v_corner_floor, min(self.v_max_straight, v_corner))

        # Backward deceleration pass (kinematic energy preservation)
        for _ in range(4):
            for i in range(n - 1, -1, -1):
                next_i = (i + 1) % n
                v_lim = math.sqrt(v_targets[next_i] ** 2 + 2.0 * self.a_brake * self.ds[i])
                if v_targets[i] > v_lim:
                    v_targets[i] = v_lim

        # Forward acceleration pass onto straightaways
        for _ in range(4):
            for i in range(n):
                next_i = (i + 1) % n
                v_lim = math.sqrt(v_targets[i] ** 2 + 2.0 * self.a_accel * self.ds[i])
                if v_targets[next_i] > v_lim:
                    v_targets[next_i] = v_lim

        return v_targets

    def scan_cb(self, scan: LaserScan):

        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment

        valid = (ranges > 0.08) & (ranges < 15.0) & (~np.isnan(ranges)) & (~np.isinf(ranges))
        r_val = ranges[valid]
        th_val = angles[valid]

        # Vehicle Cartesian coordinates
        x_pts = r_val * np.cos(th_val)
        y_pts = r_val * np.sin(th_val)

        # Forward travel corridor (vehicle width 0.30m -> half-width 0.22m with buffer)
        corridor_half_width = 0.22
        in_travel_path = (x_pts > 0.10) & (np.abs(y_pts) <= corridor_half_width)

        if np.any(in_travel_path):
            self.d_forward_clearance = float(np.min(x_pts[in_travel_path]))
        else:
            self.d_forward_clearance = 10.0

    def odom_cb(self, odom: Odometry):
        # 1. Vehicle Pose, Heading, and Angular Velocity
        xc = odom.pose.pose.position.x
        yc = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaw_rate = odom.twist.twist.angular.z
        self.current_speed = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)

        # Capture initial pose and synthesize raceline if /map has arrived
        if self.init_pose is None:
            self.init_pose = (xc, yc, yaw)
            if self.map_msg is not None and not self.raceline_ready:
                self._initialize_raceline()

        if not self.raceline_ready:
            return

        # 2. Dynamic Front Axle Preview (scales smoothly with velocity)
        preview_dist = max(0.12, min(0.20, 0.10 + 0.02 * self.current_speed))
        xf = xc + (self.wheelbase + preview_dist) * math.cos(yaw)
        yf = yc + (self.wheelbase + preview_dist) * math.sin(yaw)

        # 3. Local Window Search for Closest Waypoint
        search_radius = min(self.num_pts // 2, 80)
        idx_win = [(self.last_closest_idx + offset) % self.num_pts for offset in range(-search_radius, search_radius)]
        wps_win = self.wps[idx_win]
        dists_sq = (wps_win[:, 0] - xf) ** 2 + (wps_win[:, 1] - yf) ** 2
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

        # 6. Geometric Velocity Lookup
        # When path ahead is clear, accelerates to full straightaway speed (5.80 m/s)
        brake_pts = max(2, min(5, int(self.current_speed * 0.15 / 0.20) + 1))
        target_v = min(self.v_profile[(idx + k) % self.num_pts] for k in range(min(brake_pts, self.num_pts)))
        target_v = min(self.v_max_straight, target_v)

        # 7. Direction-of-Travel Collision Safety Override (Active LiDAR AEB)
        # Decelerates ONLY if an obstacle or wall is directly in the forward travel corridor
        if self.d_forward_clearance < 0.50:
            target_v = min(target_v, 1.80)
        if self.d_forward_clearance < 0.32:
            target_v = 0.0  # Immediate full stop

        # Enforce minimum rolling floor
        target_v = max(1.20, target_v)

        # 8. Publish Drive Command
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
