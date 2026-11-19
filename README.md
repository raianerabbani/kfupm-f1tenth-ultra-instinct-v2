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
- **Universal Physics Speed Profiler:** Computes curvature-limited corner velocities ($v = \sqrt{a_{\text{lat}} / \kappa}$) with combined friction-circle and powertrain acceleration/braking integration (`v_max_straight = 18.0 m/s`, `v_corner_floor = 3.80 m/s`, `a_lat_max = 12.0 m/s^2`).
- **Active LiDAR Forward Collision Envelope (`/scan`):** Monitors the forward travel corridor ($x > 0.08\,\text{m}, |y| \le 0.13\,\text{m}$) without slowing down next to parallel side walls.
- **Asymmetric Feedforward + Stanley + Active Gyro Damping:** Kinematic Ackermann feedforward with asymmetric turn-in anticipation, short async pose lead, front-axle preview Stanley control, and yaw-rate damping (`k_h = 1.08`, `k_e = 4.20`, `k_yaw = 0.026`).

## Local Verification (Clean Empty Workspace)
```bash
colcon build --symlink-install --packages-select kfupm_f1tenth_ultra_instinct_v2
source install/setup.bash
ros2 run kfupm_f1tenth_ultra_instinct_v2 controller
```
