# KFUPM F1TENTH - Team Ultra Instinct 2.0

Official Stage 2 submission package for the **CIE-KFUPM F1TENTH Autonomous Racing Competition**.

## Controller Submission Information
- **Team Name:** Ultra Instinct 2.0
- **Package Name:** `kfupm_f1tenth_ultra_instinct_v2`
- **Node Name:** `controller`
- **Execution Command:** `ros2 run kfupm_f1tenth_ultra_instinct_v2 controller`
- **Target Stage:** Stage 2 (High-Speed Obstacle Slalom Racing on `levine_obs`)
- **Max Straight Speed:** 5.80 m/s
- **Corner Momentum Floor:** 2.35 m/s
- **Track Record:** 18.070 s best lap with 0 collisions

---

## Overview & Algorithm: Wide-Radius Momentum Tracker (WRMT)
**Ultra Instinct 2.0** implements the **Wide-Radius Momentum Tracker (WRMT)**, engineered specifically for high-speed obstacle navigation and cornering momentum preservation on the Levine Obstacle circuit:

1. **Dynamic Wide-Radius Preview & High Turning Radius ($R_{\text{eff}}$):**
   - Replaces tight apex hugging with speed-dependent lookahead preview ($L_{\text{prev}} = \text{clip}(0.10 + 0.02 v, 0.12, 0.20)\,\text{m}$) projected along the front axle.
   - Smooths the transition into corner entries and increases effective turning radius ($R_{\text{eff}}$), reducing peak lateral acceleration demand ($a_{\text{lat}} = v^2 / R_{\text{eff}}$).

2. **Momentum Preservation Speed Profile:**
   - Completely eliminates steering-angle braking penalties that plague conventional pure pursuit controllers.
   - Maintains a high rolling corner momentum floor ($2.35\,\text{m/s}$) rather than over-braking to $< 1.5\,\text{m/s}$.
   - Precomputes backward deceleration ($a_{\text{brake}} = 4.20\,\text{m/s}^2$) and forward acceleration ($a_{\text{accel}} = 5.50\,\text{m/s}^2$) passes to maximize straightaway exit speed.

3. **Active Gyro Yaw Damping & Kinematic Stanley Tracking:**
   - Front-axle kinematic cross-track and heading error regulation:
     $$\delta(t) = k_h \cdot \theta_e + \arctan\left(\frac{-k_e \cdot e_y}{v + k_{\text{soft}}}\right) - k_{\text{yaw}} \cdot \dot{\psi}$$
   - Active yaw-rate damping ($-k_{\text{yaw}} \cdot \dot{\psi}$) stabilizes the chassis at high velocities ($> 5.5\,\text{m/s}$) through narrow obstacle corridors.

4. **Calibrated Obstacle Slalom Corridor Management:**
   - Fine-tuned sector speed caps across Levine obstacle zones (Obstacles 8, 11, 7 slalom; Obstacles 10, 6, 13 south pinch point; Obstacle 4 chicane).
   - Allows full-throttle acceleration ($5.80\,\text{m/s}$) along the unblocked North straightaway.

5. **Forward LiDAR Safety Supervisor:**
   - Continuously monitors a narrow forward LiDAR cone ($7^\circ$) to provide non-intrusive emergency braking if an obstacle directly impedes the vehicle path within $0.45\,\text{m}$.

6. **Zero-Dependency Self-Contained Portability:**
   - All 627 optimal obstacle slalom waypoints are embedded directly in-memory, requiring no external file paths or configuration dependencies.

---

## Stage 2 Evaluation Results
Validated live on ROS 2 Jazzy with the official competition `race_monitor` on `levine_obs`:
```text
============================================================
STAGE 2 EVALUATION: LEVINE OBSTACLES TRACK
Total Laps Completed: 3
Total Recorded Collisions: 0
  Lap 1: 18.080 s
  Lap 2: 18.070 s
  Lap 3: 18.110 s
Best Lap Time: 18.070 s
Average Lap Time: 18.087 s
Qualification Status: 100% CLEAN / 0 COLLISIONS
============================================================
```

---

## Dependencies
- ROS 2 Jazzy Jalisco
- `rclpy`
- `sensor_msgs`
- `nav_msgs`
- `ackermann_msgs`
- `numpy`

---

## Build and Run Instructions (For Organizers)

### 1. Build
```bash
source /opt/ros/jazzy/setup.bash
source <competition-repo>/starter_ws/.venv/bin/activate
source <competition-repo>/starter_ws/install/setup.bash

cd ~/controller_ws
colcon build --symlink-install --packages-select kfupm_f1tenth_ultra_instinct_v2
source install/setup.bash
```

### 2. Run
**Terminal 1 (Simulation & Race Monitor on Levine Obstacles):**
```bash
cd <competition-repo>
# Ensure map is set to levine_obs
./scripts/launch_sim.sh
```

**Terminal 2 (Ultra Instinct 2.0 Controller):**
```bash
source /opt/ros/jazzy/setup.bash
source <competition-repo>/starter_ws/.venv/bin/activate
source <competition-repo>/starter_ws/install/setup.bash
source ~/controller_ws/install/setup.bash

ros2 run kfupm_f1tenth_ultra_instinct_v2 controller
```

---

## Parameters Summary
| Parameter | Value | Description |
|---|---|---|
| `v_max_straight` | `5.80 m/s` | Maximum cruising velocity on straightaways |
| `v_corner_floor` | `2.35 m/s` | Minimum rolling speed preserved through corner apexes |
| `a_lat_max` | `5.80 m/s²` | Maximum permissible lateral acceleration |
| `a_brake` | `4.20 m/s²` | Momentum-preserving deceleration limit |
| `a_accel` | `5.50 m/s²` | Forward acceleration capability |
| `k_h` | `0.85` | Stanley heading error gain |
| `k_e` | `2.80` | Stanley cross-track error gain |
| `k_yaw` | `0.018` | Active gyro yaw-rate damping gain |
| `max_steer` | `0.4189 rad` | Maximum steering angle limit (~24°) |
| `wheelbase` | `0.33 m` | Vehicle wheelbase |
