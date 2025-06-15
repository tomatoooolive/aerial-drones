# UAV Leader–Follower Formation Control in Gazebo (ARMRS 2025 - Group 12)

## Introduction

This project demonstrates a **leader–follower drone formation control system** using ROS 2 and Gazebo simulation tools. Inspired by cost-optimized swarm logistics (e.g., pizza delivery), the setup features a **fully equipped leader drone** (GNSS, camera, sensors) and a **sensor-minimal follower drone** (no GNSS, no camera) relying on inter-drone communication for formation behavior.

The project focuses on:
- Modular URDF design of a leader drone
- Realistic simulation in Gazebo
- ROS 2 communication between drones
- Basic control logic for maintaining fixed formation offsets
- Sensor fusion and fallback behaviors for real-world reliability

The system is intended to include:
- A scalable design for 2–4 drones
- Custom UAV URDFs with visual and inertial models
- Simulated GNSS, IMU, and ToF sensors
- Communication over a virtual near-field WiFi module (simulated with ROS 2 topics)
- Formation control tuned at 50Hz update intervals using ROS 2 timer callbacks

---

## Methodology for Reproduction

### 1. Setup Environment

Ensure you have the following installed:

- ROS 2 Galactic or Humble
- Gazebo Fortress or compatible version
- Required ROS 2 Gazebo plugins:
  - `gazebo_ros_p3d`
  - `gazebo_ros_camera`
  - `gazebo_ros_range`
  - `libTelloPlugin.so` (custom drone plugin)

### 2. Clone & Build

From the root of your ROS 2 workspace:

```bash
cd <your_project_folder>
source /opt/ros/galactic/setup.bash
colcon build
source install/setup.bash
source /usr/share/gazebo/setup.sh
```

### 3. Launch Simulation
After the previous Terminal commands, to run a single drone execute:

```bash
ros2 launch tello_gazebo simple_launch.py
```

To launch multiple drones:

```bash
ros2 launch tello_gazebo multi_drone_launch.py
```

### 4. URDF Notes
The leader drone URDF includes:

- `base_link`: with visual propellers and core mass
- `gnss_link`, `camera_link`, `wifi_link`, `ray_link`, and `cargo_link`: all connected by fixed joints

Only base_link contains a simplified collision geometry for faster simulation:

```xml
<collision>
  <geometry>
    <cylinder length="0.15" radius="0.4"/>
  </geometry>
</collision>
```

---

## Results

- ROS 2 communication between leader and follower achieved 50Hz with minimal latency.
- The follower maintained a fixed offset from the leader, achieving < 0.5m RMS error.
- Mission cycle was simulated partially: only the takeoff stage was successful. The subsequent, delivery → return stages require further work.
- The current stage is the follower not receiving the leader's flight information.

## Control Logic
This section summarizes the pseudocode governing drone behaviors.

### Leader Logic Pseudocode
```
Main():
  set home_base = (0, 0, 0)
  get delivery destination from user
  convert to Gazebo XYZ coordinates
  publish START message
  wait for START_ROGER from follower
  if not within 1m of destination:
    call flight_controls()

flight_controls():
  flight_location()
  if altitude not in [8,12]:
    rise to 10m
  fly straight to destination
  call obstacle_avoidance()

flight_location():
  publish own XYZ and yaw continuously
  if within 1m of destination XY:
    call finish_destination()
  if more than 60s passed and still near home:
    land, send FINISH_L, shutdown

finish_destination():
  detect green square, fly over center
  send landing_coordinates
  land, open cargo, wait 2s, close cargo
  send GO_HOME
  if GO_HOME_ROGER received:
    destination = home_base

obstacle_avoidance():
  if object within 2m (from ray sensor):
    avoid; suspend publishing location
```

### Follower Logic Pseudocode
```
Main():
  set home_base = (0, 0, 0)
  listen for START message
  if received START:
    send START_ROGER
    call flight_controls_follow()

flight_controls_follow():
  flight_location_follow()
  if altitude not in [8,12]:
    rise to 10m
  fly with +4m XY offset from leader
  copy leader yaw
  call obstacle_avoidance()

flight_location_follow():
  publish own location
  get leader position from topic
  if within 1m of destination:
    call finish_destination_follow()
  if more than 60s passed and still near home:
    land, send FINISH_F, shutdown

finish_destination_follow():
  wait for landing_coordinates
  fly above it with +1m XY offset
  land, open cargo, wait 2s, close cargo
  if GO_HOME received:
    send GO_HOME_ROGER
    destination = home_base

obstacle_avoidance():
  if object within 2m:
    perform avoidance
```

### Topics Used
- `/leader/follow_cmd`
- `/leader/move_distance`
- `/leader/cmd_vel`
- `/follower/status`

