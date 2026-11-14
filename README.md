# KFUPM F1TENTH - Team Ultra Instinct 2.1

Official Stage 2 controller submission for the **CIE-KFUPM F1TENTH Autonomous Racing Competition**.

## Controller Submission Information
- **Team Name:** Ultra Instinct 2.1
- **Package Name:** `kfupm_f1tenth_ultra_instinct_v2`
- **Node Name:** `controller`
- **Standalone File:** `controller.py` (identical to `kfupm_f1tenth_ultra_instinct_v2/controller_node.py`)
- **Execution Command:** `ros2 run kfupm_f1tenth_ultra_instinct_v2 controller` (or `python3 controller.py`)
- **Target Stage:** Stage 2 (Obstacle Avoidance & High-Speed Autonomous Racing)
- **Max Straight Speed:** 5.80 m/s
- **Corner Momentum Floor:** 2.35 m/s
- **Track Record (`levine_obs`):** **16.020 s** best lap (`16.07 s`, `16.06 s`, `16.02 s`) with **0 collisions**
- **Design Paradigm:** **100% Universal, Standalone Map-to-Raceline Synthesis** (Zero embedded waypoints, zero map/CSV files, zero coordinate checks, active LiDAR forward collision envelope)

---

## Overview & Architecture: Universal Map-to-Raceline Momentum Tracker (WRMT)

**Ultra Instinct 2.1** is a **100% standalone single-file controller** with zero hardcoded waypoint arrays or external `.csv` dependencies. Upon receiving `/map` (`nav_msgs/msg/OccupancyGrid`) and the initial pose from `/ego_racecar/odom` (`nav_msgs/msg/Odometry`), it automatically synthesizes a globally optimal obstacle-avoiding raceline and physics velocity profile in **~170 ms**:

```
┌─────────────────────────────────────────────────────────────────────────┐
│  1. Real-Time Map-to-Raceline Synthesis (/map + /ego_racecar/odom)      │
│     • Euclidean Distance Transform (EDT) & SLAM doorway void sealing    │
│     • Isolated obstacle island removal & Jordan Curve wall separation   │
│     • Zero-level-set centerline tracing (d_outer == d_inner)            │
│     • Vectorized 2-pass periodic Dynamic Programming (DP) slalom        │
│     • Frenet 1D offset smoothing + corner rounding + cubic B-spline     │
└────────────────────────────────────┬────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  2. Pure Differential Geometry Speed Profiler                           │
│     • Circumscribed Menger curvature: κ(s)                              │
│     • Friction limit: v(s) = √(a_lat_max / κ(s))                        │
│     • Bidirectional kinematic braking (a_brake) & acceleration (a_accel)│
└────────────────────────────────────┬────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────┐ Relative Error (e_y, θ_e)  ┌──────────────────────┐
│  3. Active Sensors  │───────────────────────────▶│  4. Kinematic Control│
│  • /scan (LiDAR)    │                            │  • WRMT Preview Chord│
│  • /ego_racecar/odom│───────────────────────────▶│  • Kinematic Stanley │
│    (Pose & Gyro)    │  Forward Envelope (x, |y|) │  • Gyro Yaw Damping  │
└─────────────────────┘                            └──────────────────────┘
```

### 1. Real-Time Map-to-Raceline Synthesis (`~170 ms`)
- **Topology Extraction:** Crops the `OccupancyGrid` to the track bounding box, computes the exact Euclidean Distance Transform (`scipy.ndimage.distance_transform_edt`), seals open SLAM doorways, and separates the inner and outer track boundaries.
- **Pure-Python Contour Tracing:** Extracts the zero-level-set reference centerline ($d_{\text{outer}} = d_{\text{inner}}$) using an 8-connected Moore-Neighbor boundary tracer (requiring zero OpenCV dependency).
- **Periodic Dynamic Programming (DP) Slalom:** Evaluates 91 lateral offset states $u \in [-w_{\max}, +w_{\max}]$ along the track normals against the true obstacle distance field to compute the globally optimal periodic obstacle-avoidance path.
- **Frenet Smoothing & $C^2$ Cubic B-Spline:** Smooths 1D Frenet offsets $u(s)$ (keeping straightaways centered while straightening slalom transitions), rounds corner apexes, enforces $\ge 0.25\,\text{m}$ clearance, and resamples a periodic cubic B-spline at uniform $0.10\,\text{m}$ arc-length steps.

### 2. Pure Differential Geometry Speed Profiling
- **Coordinate-Free Formulation:** Velocity bounds are computed strictly from local curvature $\kappa(s)$:
  $$v_{\text{corner}}(s) = \sqrt{\frac{a_{\text{lat\_max}}}{\kappa(s)}}$$
- **Kinematic Passes:** Backward deceleration integration ($a_{\text{brake}} = 4.20\,\text{m/s}^2$) and forward acceleration integration ($a_{\text{accel}} = 5.50\,\text{m/s}^2$) guarantee optimal straightaway launch and precise corner entry braking points on any circuit.

### 3. Active LiDAR Perception & Forward Collision Envelope (`/scan`)
- **Direction-of-Travel Collision Envelope:** Projects `/scan` (`sensor_msgs/msg/LaserScan`) rays into vehicle Cartesian coordinates $(x_{\text{fwd}}, y_{\text{lat}})$ and monitors the forward travel corridor ($x > 0.10\,\text{m}, |y| \le 0.22\,\text{m}$).
- Walls parallel to the car (to the left or right) **never** trigger deceleration, allowing full acceleration to maximum speed ($5.80\,\text{m/s}$) whenever the path ahead is clear.
- Multi-stage forward AEB override: brakes to $1.80\,\text{m/s}$ if $d_{\text{fwd}} < 0.50\,\text{m}$ and commands a full stop ($0.0\,\text{m/s}$) if $d_{\text{fwd}} < 0.32\,\text{m}$.

### 4. Kinematic Stanley Tracking with Active Gyro Yaw Damping
- Front-axle kinematic cross-track and heading error regulation with speed-scaled lookahead preview:
  $$\delta(t) = k_h \cdot \theta_e + \arctan\left(\frac{-k_e \cdot e_y}{v + k_{\text{soft}}}\right) - k_{\text{yaw}} \cdot \dot{\psi}$$

---

## Stage 2 Official Evaluation Results (`levine_obs`)
Validated live on ROS 2 Jazzy with the official competition `race_monitor`:
```text
============================================================
STAGE 2 EVALUATION: LEVINE OBSTACLES TRACK (UNIVERSAL V2.1)
Map-to-Raceline Synthesis: 625 waypoints in 171.3 ms
Total Laps Completed: 3
Total Recorded Collisions: 0
  Lap 1: 16.070 s
  Lap 2: 16.060 s
  Lap 3: 16.020 s
Best Lap Time: 16.020 s
Average Lap Time: 16.050 s
Top Speed: 5.79 m/s
LiDAR Usage (/scan): Active (+10/10 pts)
Safety Score: 60/60 pts (0 collisions)
============================================================
```

---

## Dependencies
- ROS 2 Jazzy Jalisco (`rclpy`, `sensor_msgs`, `nav_msgs`, `ackermann_msgs`)
- `numpy`
- `scipy`

---

## Build and Run Instructions

### Option A: Run as a ROS 2 Package
```bash
cd ~/controller_ws
colcon build --symlink-install --packages-select kfupm_f1tenth_ultra_instinct_v2
source install/setup.bash
ros2 run kfupm_f1tenth_ultra_instinct_v2 controller
```

### Option B: Run Standalone Single File (`controller.py`)
```bash
python3 controller.py
```
