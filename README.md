# KFUPM F1TENTH - Team Ultra Instinct 2.1

## Official run
- Package: kfupm_f1tenth_ultra_instinct_v2
- Command: ros2 run kfupm_f1tenth_ultra_instinct_v2 controller
- Extra arguments: none
- Map it is built for: levine_obs
- Uses LiDAR: yes, subscribes to /scan
- Branch or tag to race: main
- Startup time before the car moves: about 1 s

---

## Architecture Overview
- **Zero Hardcoded Waypoints or Track Coordinates:** Dynamically extracts the track topology and synthesizes an obstacle-avoiding $C^2$ cubic B-spline raceline from `/map` (`nav_msgs/msg/OccupancyGrid`) and `/ego_racecar/odom` (`nav_msgs/msg/Odometry`) in ~0.17 s using Euclidean Distance Transforms and periodic Dynamic Programming.
- **Universal Physics Speed Profiler:** Computes curvature-limited corner velocities ($v = \sqrt{a_{\text{lat}} / \kappa}$) with bidirectional kinematic braking/acceleration integration passes (`v_max_straight = 5.80 m/s`, `v_corner_floor = 2.35 m/s`).
- **Active LiDAR Forward Collision Envelope (`/scan`):** Monitors the forward travel corridor ($x > 0.10\,\text{m}, |y| \le 0.22\,\text{m}$) without slowing down next to parallel side walls.
- **Kinematic Stanley + Active Gyro Damping:** Front-axle preview tracking with yaw-rate damping (`k_h = 0.85`, `k_e = 2.80`, `k_yaw = 0.018`).

## Local Verification (Clean Empty Workspace)
```bash
colcon build --symlink-install --packages-select kfupm_f1tenth_ultra_instinct_v2
source install/setup.bash
ros2 run kfupm_f1tenth_ultra_instinct_v2 controller
```
