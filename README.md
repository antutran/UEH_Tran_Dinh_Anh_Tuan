# UEH Creative Robot Contest 2026 – Simulation Round

```
NAME:    [Your Full Name]
SCHOOL:  University of Economics Ho Chi Minh City (UEH)
FACULTY: [Your Faculty Name]
```

---

## Environment Requirements

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo Classic | 11 |
| Python | 3.10+ |
| OpenCV | 4.5+ |
| TurtleBot3 | waffle model |

---

## Dependency Installation

```bash
# ROS dependencies (run once)
sudo apt-get update
sudo apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-turtlebot3-description \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-numpy

# Python analysis dependencies
pip3 install matplotlib pandas
```

---

## Build Instructions

```bash
# From the workspace root (where src/ lives):
cd /path/to/UEH_CRC_2026_Simulation_Pack

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Simulation

### Step 1 – Start the Simulator (terminal 1)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch crc_sim sim.launch.py
```

### Step 2 – Start the Solution (terminal 2)

> **This is the single competition command.**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ueh_solution run.launch.py
```

---

## Changing Parameters Live (for video demo)

```bash
# Example: increase base speed live
ros2 param set /behavior_node base_speed 0.22

# Example: decrease base speed live
ros2 param set /behavior_node base_speed 0.13
```

---

## Architecture Overview

```
Camera (/camera/image_raw)
  ├── lane_node     → /lane/error         (pixel error from lane centre)
  ├── sign_node     → /sign/detection     ("STOP" | "crosswalk" | "none")
  ├── light_node    → /light/state        ("RED" | "YELLOW" | "GREEN" | "NONE")
  └── ped_node      → /pedestrian/blocking (True/False)

LiDAR (/scan)
  ├── lidar_node    → /obstacle/front_dist  (metres)
  │                 → /obstacle/status      (0=clear, 1=slow, 2=stop)
  └── ped_node      → (also uses /scan for pedestrian detection)

All perception topics → behavior_node → /cmd_vel (Twist)

/odom /cmd_vel → data_logger → /tmp/crc_logs/run_*.csv
```

### Behavior FSM Priority

| Priority | State | Trigger |
|---|---|---|
| 1 (highest) | EMERGENCY_STOP | LiDAR front < 0.25 m |
| 2 | PEDESTRIAN_STOP | Pedestrian blocking road |
| 3 | RED_LIGHT_STOP | Camera sees RED or YELLOW |
| 4–6 | STOP_SIGN_SEQUENCE | STOP sign detected; hold 2 s |
| 7 (lowest) | LANE_FOLLOWING | Normal driving |

---

## Key ROS Parameters

All parameters are in [`src/ueh_solution/config/params.yaml`](src/ueh_solution/config/params.yaml).

| Node | Parameter | Default | Description |
|---|---|---|---|
| behavior_node | `base_speed` | 0.18 | Normal cruise speed (m/s) — **video demo param** |
| behavior_node | `kp_steer` | 0.0042 | Proportional steering gain |
| behavior_node | `kd_steer` | 0.0008 | Derivative steering gain |
| behavior_node | `emergency_stop_dist` | 0.25 | LiDAR stop threshold (m) |
| behavior_node | `stop_hold_seconds` | 2.2 | STOP sign hold duration |
| lane_node | `roi_top_frac` | 0.55 | ROI top boundary (fraction from top) |
| lane_node | `clahe_clip` | 2.5 | CLAHE clip limit (normal) |
| lane_node | `clahe_clip_dark` | 4.5 | CLAHE clip limit (tunnel) |

---

## Prohibited Practices (verified)

This solution does NOT:
- Subscribe to `/traffic_lights`, `/traffic_light/*`, `/automobile/semaphores`
- Subscribe to `/sky_cam/*`, `/model_states`, `/link_states`
- Call Gazebo services (`/set_entity_state`, `/get_entity_state`, etc.)
- Read simulation world/config files at runtime
- Hard-code any track object coordinates

---

## Analysis

```bash
# After a run, generate figures from the CSV log
python3 analysis/plot_run.py /tmp/crc_logs/run_<timestamp>.csv

# Compare two runs (e.g. before/after parameter change)
python3 analysis/plot_steering.py run_slow.csv run_fast.csv
```

---

## Docker (alternative)

```bash
cd docker
docker compose run --rm crc
# Inside container:
build && source /ws_build/install/setup.bash
ros2 launch crc_sim sim.launch.py gui:=false &
sleep 30
ros2 launch ueh_solution run.launch.py
```
