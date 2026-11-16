# 🏎️ Reactive Follow the Gap (FTG) Controller with Disparity Extender — Sao Paulo Circuit (F1Tenth)

[![ROS 2](https://img.shields.io/badge/ROS%202-HUMBLE-brightgreen.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/PYTHON-3.10%2B-blue.svg)](https://www.python.org/)
[![F1Tenth](https://img.shields.io/badge/F1TENTH-GYM%20ROS-red.svg)](https://f1tenth.org/)
[![Validation](https://img.shields.io/badge/VALIDATION-10%20COLLISION--FREE%20LAPS-success.svg)](data/champion_10_laps.csv)
[![License: MIT](https://img.shields.io/badge/LICENSE-MIT-yellow.svg)](LICENSE)

* **Author:** Prantor Imtiar Rashid
* **Course:** Autonomous Vehicles — II PAO 2026
* **Assessment:** First Midterm Project (Part 1) — Reactive Controllers Competition
* **Assigned Track:** Sao Paulo (Group 5)

---

## 📑 Table of Contents

1. [Project Overview and Objectives](#1-project-overview-and-objectives)
2. [Geometric Analysis of the Sao Paulo Circuit](#2-geometric-analysis-of-the-sao-paulo-circuit)
3. [ROS 2 System Architecture](#3-ros-2-system-architecture)
4. [Mathematical Formulation and Algorithmic Approach](#4-mathematical-formulation-and-algorithmic-approach)
5. [Telemetry System: Lap Counter, Lap Timer, Distance Tracker, and 3D RViz Trajectory](#5-telemetry-system-lap-counter-lap-timer-distance-tracker-and-3d-rviz-trajectory)
6. [Detailed Code Structure](#6-detailed-code-structure)
7. [Hyperparameter Table and Tuning Strategy](#7-hyperparameter-table-and-tuning-strategy)
8. [Installation, Configuration, and Execution Instructions](#8-installation-configuration-and-execution-instructions)
9. [Experimental Results, Lap Distance, and Video Evidence](#9-experimental-results-lap-distance-and-video-evidence)

---

## 1. Project Overview and Objectives

This project implements an advanced **Reactive Autonomous Racing Controller** for the official F1Tenth Gym ROS simulation environment, customized specifically for **Group 5** on the **Sao Paulo (Interlagos)** Grand Prix circuit.

The controller addresses two core competitive benchmarks:
* **Challenge 1 (Endurance & Robustness):** Complete **10 consecutive laps** autonomously with zero collisions, maintaining dynamic stability through continuous racing corridors.
* **Challenge 2 (Sprint & Optimization):** Maximize longitudinal racing velocity to achieve the **fastest single lap time** on the assigned track while strictly obeying the non-holonomic Ackerman vehicle dynamics and friction boundaries.

In addition to autonomous control, the system features a complete, self-contained telemetry and observability architecture including an odometry-based lap counter with spatial hysteresis, a microsecond-precision lap timer, real-time Euclidean distance integration, and dynamic 3D RViz visualization overlays.

---

## 2. Geometric Analysis of the Sao Paulo Circuit

The Sao Paulo circuit is characterized by demanding geometry that exposes the failure modes of naive reactive steering controllers:

* **Corridor Width ($W_{track} \approx 2.22\text{ m}$):** Relatively narrow confines requiring precise path-centering to avoid barrier strikes.
* **High-Speed Main Straight:** A long sweeping straightaway where standard Follow-the-Gap algorithms suffer from severe oscillatory weaving ("zigzagging") due to sensor noise jumping across distant walls.
* **Complex Multi-Apex Chicanes (Senna 'S' & Junção):** Sharp sequential turns where blind reactive controllers clip inside apexes with their rear chassis (tail-strike) or understeer excessively on corner entry due to lingering straight-line momentum.

To overcome these challenges, this controller deploys a 7-stage reactive pipeline incorporating the **UNC Chapel Hill Disparity Extender**, **anti-zigzag dynamic apex blending**, **lateral tail-strike safety guards**, and a **kinematic lookahead constant-deceleration braking curve**.

---

## 3. ROS 2 System Architecture

The controller operates inside a decoupled publish/subscribe ROS 2 architecture where the node `dynamic_apex_gap_node` processes 2D LiDAR scans at high frequency ($\approx 40\text{ Hz}$) while concurrently updating odometry metrics and 3D RViz markers:

```
+-------------------------------------------------------+
|              F1Tenth Gym ROS Bridge                   |
|             (Physics Simulator + 2D Map)              |
+-------------------------------------------------------+
               |                       |
               | /scan                 | /ego_racecar/odom
               | (LaserScan)           | (Odometry)
               v                       v
+-------------------------------------------------------+
|  Node: dynamic_apex_gap_node                          |
|  (f1tenth_saopaulo_grand_prix)                        |
|                                                       |
|  1. LiDAR Filtering + Disparity Ext.                  |
|  2. Dynamic Safety Bubble                             |
|  3. Max Gap & Hybrid Target Selection                 |
|  4. Adaptive PD Steering + Speed Law                  |
|  5. Lap Counter, Timer, Dist & Path                   |
+-------------------------------------------------------+
     |             |             |             |
     | /drive      | /ftg_hud    | /ftg_...    | /ftg_path &
     | (Ack.)      | _marker     | target      | /ftg_trajectory_marker
     v             v             v             v
+----------+  +-----------------------------------------+
| Ego      |  |                 RViz 2                  |
| Racecar  |  |       (3D HUD + Trajectory Line)        |
+----------+  +-----------------------------------------+
```

### ROS 2 Communication Interfaces (Topics)

| Topic | Message Type | Direction | System Role |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Subscribed | Receives the 2D LiDAR range array ($1080$ beams). |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | Subscribed | Provides $(x, y)$ pose for lap counting, lap timing, distance integration, and path drawing. |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Published | Sends steering angle (`steering_angle`) and longitudinal velocity (`speed`) commands. |
| `/ftg_hud_marker` | `visualization_msgs/msg/Marker` | Published | Renders a 3D floating HUD above the vehicle displaying lap, time, lap distance, and speed. |
| `/ftg_target_marker` | `visualization_msgs/msg/Marker` | Published | Renders a 3D green sphere at the selected target point inside the navigable gap. |
| `/ftg_trajectory_marker` | `visualization_msgs/msg/Marker` | Published | Draws a continuous 3D orange line (`LINE_STRIP`) showing the trajectory driven on the circuit. |
| `/ftg_path` | `nav_msgs/msg/Path` | Published | Publishes the historical pose sequence compatible with the native RViz `Path` display. |

---

## 4. Mathematical Formulation and Algorithmic Approach

Unlike a baseline *Follow the Gap* implementation—which frequently clips inner corners on hairpins or oscillates on high-speed straights—this controller executes a 7-stage pipeline on every LiDAR scan:

### Stage 1: Field-of-View (FOV) Cropping and Spatial Smoothing
The raw LiDAR scan is cropped to retain only the frontal sector $[-\theta_{fov}, +\theta_{fov}]$ with $\theta_{fov} = 67.5^\circ$ ($135^\circ$ total cone), discarding rear and lateral reflections. Non-finite values (`NaN`, $\pm\infty$) are clamped to the maximum horizon $R_{max} = 15.0\text{ m}$, and a 5-point moving-average convolution filter is applied:

$$r_{smooth}[i] = \frac{1}{2M+1} \sum_{m=-M}^M r[i+m], \quad M=2$$

### Stage 2: Disparity Extender
When approaching corners or chicanes in Sao Paulo, sharp depth discontinuities occur between a beam hitting an inner wall edge ($d_{near}$) and the adjacent beam looking into the open turn. Whenever $|r[i+1] - r[i]| > \delta_{disp}$ ($\delta_{disp} = 0.38\text{ m}$), the vehicle's virtual half-width ($W_{half} = 0.31\text{ m}$) is projected onto the closer reading to mask unsafe rays:

$$\Delta \theta = \arctan\left(\frac{W_{half}}{d_{near}}\right), \quad N_{samples} = \left\lceil \frac{\Delta \theta}{\Delta \alpha} \right\rceil$$

where $\Delta \alpha$ is the LiDAR angular resolution (`angle_increment`).

### Stage 3: Dynamic Safety Bubble
If the closest obstacle lies within a critical proximity threshold ($d_{min} \le 2.5\text{ m}$), its index $i_{min}$ is identified and all rays within a safety radius $R_{bubble} = 0.45\text{ m}$ are zeroed out ($0.0\text{ m}$):

$$N_{bubble} = \left\lceil \frac{\arcsin\left(\min\left(1.0, \frac{R_{bubble}}{d_{min}}\right)\right)}{\Delta \alpha} \right\rceil$$

### Stage 4: Adaptive Threshold Max-Gap Detection
To avoid selecting narrow false gaps, a dynamic free-space threshold is computed relative to the current maximum range:

$$T_{gap} = \max\left(1.15\text{ m}, 0.38 \cdot \max_i(r_{proc}[i])\right)$$

All continuous sequences of LiDAR indices where $r_{proc}[i] \ge T_{gap}$ are identified as candidate gaps, and the widest gap $[g_{start}, g_{end}]$ is extracted.

### Stage 5: Dynamic Apex & Hybrid Target Selection (Anti-Zigzag Blending)
To prevent high-speed oscillatory weaving down straights while maintaining aggressive corner entry, the steering index is blended between the geometric center of the widest gap and the deepest LiDAR point:

$$i_{center} = \frac{g_{start} + g_{end}}{2}, \quad i_{deep} = \arg\max_{i \in [g_{start}, g_{end}]} r_{proc}[i]$$

$$i_{target} = \text{round}\Big(\alpha \cdot i_{center} + (1 - \alpha) \cdot i_{deep}\Big), \quad \alpha = 0.65$$

$$\delta_{raw} = \theta[i_{target}]$$

### Stage 6: Lateral Tail-Strike Guard & Steering Slew Rate Limiter
To protect the rear chassis during rapid chicane transitions, lateral clearances at $\theta \in [\pm 65^\circ, \pm 85^\circ]$ are continuously checked. If clearance drops below $0.28\text{ m}$, steering is attenuated by $55\%$:

$$\delta_{guarded} = \begin{cases} 0.45 \cdot \delta_{raw} & \text{if } \delta_{raw} > 0.12\text{ and } d_{left} < 0.28\text{ m} \\ 0.45 \cdot \delta_{raw} & \text{if } \delta_{raw} < -0.12\text{ and } d_{right} < 0.28\text{ m} \\ \delta_{raw} & \text{otherwise} \end{cases}$$

The steering command is then rate-limited ($\Delta \delta_{max} = 0.15\text{ rad/step}$):

$$\delta_{cmd} = \delta_{prev} + \text{clip}\left(\delta_{guarded} - \delta_{prev}, -\Delta \delta_{max}, +\Delta \delta_{max}\right)$$

### Stage 7: Kinematic Lookahead Braking Curve & Curvature Speed Profile
Longitudinal speed is governed by the minimum of a curvature-governed limit $v_{curv}(\delta)$ and the kinematic constant-deceleration stopping curve:

$$v_{brake} = \begin{cases} v_{min} & \text{if } d_{clearance} \le d_{margin} \\ \sqrt{v_{min}^2 + 2 a_{brake} (d_{clearance} - d_{margin})} & \text{if } d_{clearance} > d_{margin} \end{cases}$$

$$v_{cmd} = \text{clip}\left(\min(v_{curv}, v_{brake}), v_{min}, v_{max}\right)$$

where $v_{max} = 8.8\text{ m/s}$ ($31.68\text{ km/h}$), $v_{min} = 2.35\text{ m/s}$, $a_{brake} = 5.8\text{ m/s}^2$, and $d_{margin} = 1.85\text{ m}$.

---

## 5. Telemetry System: Lap Counter, Lap Timer, Distance Tracker, and 3D RViz Trajectory

Synchronized with `/ego_racecar/odom`, the telemetry subsystem provides full observability during execution:

* **Automatic Finish-Line Detection with Spatial Hysteresis:** Upon receiving the first odometry message, the initial coordinates $(x_0, y_0)$ are stored as the finish line. A new lap is registered only after the vehicle leaves the start zone ($d > 15.0\text{ m}$), completes the circuit ($t \ge 10.0\text{ s}$), and re-enters the finish radius ($d \le 2.2\text{ m}$).
* **Euclidean Lap Distance Integration:** On every odometry update $(x_k, y_k)$, the incremental displacement $\Delta s_k = \sqrt{(x_k - x_{k-1})^2 + (y_k - y_{k-1})^2}$ is accumulated to compute the exact distance traveled per lap:

$$D_{lap} = \sum_{k=1}^K \sqrt{(x_k - x_{k-1})^2 + (y_k - y_{k-1})^2}$$

* **Real-Time Trajectory Rendering in RViz:** Every $0.20\text{ m}$ of travel, the vehicle's $(x, y)$ coordinates in the `map` frame are appended to `/ftg_trajectory_marker` (`Marker.LINE_STRIP`) and `/ftg_path` (`nav_msgs/Path`), drawing the complete racing line followed by the vehicle around Sao Paulo.

---

## 6. Detailed Code Structure

```
f1tenth_saopaulo_grand_prix/
├── CMakeLists.txt / setup.py           # ROS 2 Python package build configuration
├── package.xml                         # ROS 2 dependency manifest
├── README.md                           # Master engineering report & tutorial
├── LICENSE                             # MIT License
├── config/
│   ├── controller_params.yaml          # Tuned controller hyperparameters
│   └── sim_saopaulo.yaml               # F1Tenth Gym simulator environment config
├── controllers/
│   ├── __init__.py
│   └── dynamic_apex_gap_node.py        # 7-stage controller, telemetry & HUD node
├── launch/
│   └── race_saopaulo.launch.py         # Launch file for bridge, RViz & controller
├── data/
│   ├── champion_10_laps.csv            # Official 10-lap verification telemetry record
│   └── champion_lap_times.csv          # Real-time CSV logging output
├── docs/
│   └── ORAL_DEFENSE_GUIDE.md           # 1-on-1 interview defense preparation guide
├── media/
│   ├── SaoPaulo_GrandPrix_Champion_Evidence.mp4  # 720p HD broadcast video evidence
│   ├── champion_telemetry_screenshot.png         # Telemetry HUD evidence screenshot
│   └── 10_laps_completed_proof_screenshot.png    # 10 clean laps proof screenshot
└── test/
    ├── test_chicane.py                 # Automated 10-lap simulation test harness
    ├── test_controller.py              # Unit tests for controller stages
    ├── run_10_lap_grand_prix.py        # Headless verification runner
    └── generate_champion_video.py      # Video rendering pipeline
```

---

## 7. Hyperparameter Table and Tuning Strategy

| Parameter | Value | Units | Engineering Justification |
| :--- | :---: | :---: | :--- |
| `max_speed` | `8.8` | $\text{m/s}$ | Maximum straight-line speed ($31.68\text{ km/h}$) tuned to extract maximum pace down the Sao Paulo straight. |
| `min_speed` | `2.35` | $\text{m/s}$ | Cornering base speed ensuring sufficient centripetal tire grip through the narrowest chicanes. |
| `a_brake` | `5.8` | $\text{m/s}^2$ | Maximum kinematic deceleration before entering braking zones without tire friction saturation. |
| `d_margin` | `1.85` | $\text{m}$ | Longitudinal safety buffer distance maintained ahead of inside barriers. |
| `fov_degrees` | `135.0` | $\text{deg}$ | Forward scanning sector ($[-\frac{135^\circ}{2}, +\frac{135^\circ}{2}]$) eliminating false rear/side reflections. |
| `bubble_radius` | `0.45` | $\text{m}$ | Dynamic obstacle safety inflation radius protecting vehicle footprint. |
| `car_width_half`| `0.31` | $\text{m}$ | Virtual half-width safety margin utilized by Disparity Extender angle calculation. |
| `disparity_threshold`| `0.38`| $\text{m}$ | Minimum depth step between adjacent rays that triggers disparity bubble masking. |
| `gap_distance_threshold`| `1.15`| $\text{m}$ | Minimum radial distance to qualify as free navigable track space. |
| `alpha_blend` | `0.65` | — | Midpoint ($65\%$) vs deepest point ($35\%$) blending weight eliminating straight-line zigzags. |
| `steer_rate_limit` | `0.15` | $\text{rad/step}$ | Steering slew rate limiter preventing sharp steering jerks and destabilization. |

---

## 8. Installation, Configuration, and Execution Instructions

### Prerequisites
* **OS & Middleware:** Ubuntu 22.04 LTS with ROS 2 Humble.
* **Simulator:** Official F1Tenth Gym ROS installed inside your workspace (`~/ros2_ws` or `~/f1tenth_ws`).

### Step 1: Copy the Sao Paulo Map and Configure `sim.yaml`
1. Copy the Sao Paulo map files into the simulator's `maps/` directory:
   ```bash
   cp maps/SaoPaulo_map.* ~/ros2_ws/src/f1tenth_gym_ros/maps/
   ```
2. Update `~/ros2_ws/src/f1tenth_gym_ros/config/sim.yaml` with the Sao Paulo map path and initial heading:
   ```yaml
   map_path: '/home/user/ros2_ws/src/f1tenth_gym_ros/maps/SaoPaulo_map'
   map_img_ext: '.png'
   num_agent: 1
   sx: 0.0
   sy: 0.0
   stheta: 0.0
   ```

### Step 2: Clone and Build the Package
```bash
cd ~/ros2_ws/src
git clone https://github.com/rashidimtiar73-pixel/f1tenth_saopaulo_grand_prix.git
cd ~/ros2_ws
colcon build --packages-select f1tenth_saopaulo_grand_prix --symlink-install
source install/setup.bash
```

### Step 3: Launch the Simulator, RViz Trajectory, and Controller
1. **Terminal 1 — Launch the F1Tenth Simulator and RViz:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```
2. **Terminal 2 — Launch the Reactive Controller:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch f1tenth_saopaulo_grand_prix race_saopaulo.launch.py
   ```

---

## 9. Experimental Results, Lap Distance, and Video Evidence

### Performance Summary on Sao Paulo (10-Lap Run)

| Evaluation Metric | Recorded Value | Status |
| :--- | :---: | :---: |
| **Test 1 — Consecutive Collision-Free Laps** | **10 / 10 laps** | 🥇 **Completed** |
| **Test 2 — Fastest Lap Time** | **87.720 s** | 🥇 **Completed** |
| **Distance Traveled per Lap (Racing Line)** | **$\approx 642.50\text{ m / lap}$** (measured via `/ego_racecar/odom`) | ✅ **Verified** |
| **Peak Velocity on Main Straight** | **8.8 m/s (31.68 km/h)** | ✅ **Verified** |
| **RViz Trajectory Visualization** | Active via `/ftg_trajectory_marker` (`LINE_STRIP`) & `/ftg_path` | ✅ **Verified** |

### Official 10-Lap Telemetry Record (`data/champion_10_laps.csv`)

| Lap | Lap Time (s) | Best Lap Time (s) | Top Speed (km/h) | Collisions | Status |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | 88.9100 | 88.9100 | 31.6 | 0 | Clean |
| **2** | 88.0700 | 88.0700 | 31.6 | 0 | Clean |
| **3** | 88.4800 | 88.0700 | 31.6 | 0 | Clean |
| **4** | 88.2300 | 88.0700 | 31.6 | 0 | Clean |
| **5** | 88.4200 | 88.0700 | 31.6 | 0 | Clean |
| **6** | **87.7200** | **87.7200** | **31.6** | **0** | **Clean (Fastest Record)** |
| **7** | 88.5100 | 87.7200 | 31.6 | 0 | Clean |
| **8** | 88.2000 | 87.7200 | 31.6 | 0 | Clean |
| **9** | 88.2700 | 87.7200 | 31.6 | 0 | Clean |
| **10** | 88.1300 | 87.7200 | 31.6 | 0 | Clean |

### Visual Telemetry Proof

![Champion Telemetry Screenshot](media/champion_telemetry_screenshot.png)

### 🎥 Video Evidence

* **Demonstration Video (Vehicle navigating Sao Paulo + Visible Lap Counter + Live Lap Timer + Distance & RViz Trajectory):**  
  [▶️ Watch / Download SaoPaulo_GrandPrix_Champion_Evidence.mp4](media/SaoPaulo_GrandPrix_Champion_Evidence.mp4)

---

## 📄 License
This project is open-source under the [MIT License](LICENSE).
