# Robot Obstacle Detection and Avoidance

## Stage 1 - Object detection: 
- use LiDAR or OAK-D camera sensors 
- read incoming info 
- we make a histogram where each bin is a range of a few degrees 
- decide if a detected object is an obstacle 

## Stage 2 - obstacle avoidance: 
- if the obstacle is close, turn in a different direction

## Workspace Setup 
```bash
colcon build --symlink-install
```

# Parameters

Both `controller` and `depth_controller` accept these ROS parameters (`--ros-args -p name:=value`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `safe_distance` | `1.0` | Distance in metres below which a front obstacle triggers a turn |
| `fov_deg` | `30.0` | Half-width in degrees of the front window checked for obstacles |
| `forward_speed` | `0.2` | Forward speed in m/s when the path is clear |
| `turn_speed` | `0.5` | Turning speed in rad/s when avoiding an obstacle |
| `turn_choice` | `space` | Side selection: `space` (mean side distance) or `clearance` (closest side bin) |

`safe_distance`, `fov_deg`, `forward_speed` and `turn_speed` must be positive.

`controller` (LiDAR) also accepts `angle_offset_deg` (default `90.0`), added to every LiDAR angle so 0° points forward on the real rover (the LiDAR front arrow points to the rover's left). Use `angle_offset_deg:=0.0` if the scan is already aligned (e.g. in Gazebo). Always pass float values (`0.0`, not `0`).

# One-command launch on the real UGV Rover (in the docker container)

`scripts/run_lidar.sh` and `scripts/run_depth.sh` source ROS, the UGV workspace and this workspace, start `ugv_driver` and the sensor if they are not already running, wait until sensor data arrives, then start the controller.

```bash
ssh -p 23 root@ROVER_IP
cd ~/robot_vision_obstacle_avoidance
./scripts/run_lidar.sh safe_distance:=0.5
./scripts/run_depth.sh safe_distance:=0.5 camera_height:=0.15
```

Arguments are ROS parameters, given as `name:=value`, `--name value` or `--name=value`. Run with `--help` for the full list.

The rover drives as soon as the controller starts. Ctrl-C stops the controller, publishes a zero velocity, then stops the sensor and driver started by the script (SIGINT, then SIGTERM/SIGKILL only if a node hangs). Nodes that were already running are left untouched. Background node logs are in `/tmp/obstacle_avoidance_logs`. `ROS_DOMAIN_ID` defaults to 18.

# Sensors

## LiDAR
```bash
ros2 run object_detection_pkg controller
```

```bash
ros2 run object_detection_pkg controller --ros-args -p safe_distance:=0.5
```

### LiDAR on the real UGV Rover (in the docker container)

Each terminal: `ssh -p 23 root@ROVER_IP`, then:
```bash
export ROS_DOMAIN_ID=GROUP_ID
source /home/ws/ugv_ws/install/setup.bash
```

```bash
# T1: motor driver
ros2 run ugv_bringup ugv_driver

# T2: LiDAR driver (not bringup_lidar.launch.py, it starts a second ugv_driver)
ros2 launch ldlidar ldlidar.launch.py

# T3: check scan data (~10 Hz)
ros2 topic hz /scan

# T4: controller
cd ~/robot_vision_obstacle_avoidance
source install/setup.bash
ros2 run object_detection_pkg controller --ros-args -p safe_distance:=0.5
```

Do not set `use_sim_time:=true` on the real robot. The `_ARRAY_API not found` NumPy traceback at startup can be ignored.

### Visualize the LiDAR scan (on the laptop)
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=GROUP_ID
rviz2 -f base_lidar_link
```

In rviz: Add → By topic → `/scan` → LaserScan, then click Zero in the Views panel. The default `map` fixed frame does not exist on the rover and gives `No tf data`.

## Depth Camera
```bash
ros2 run object_detection_pkg depth_controller
```

```bash
ros2 run object_detection_pkg depth_controller --ros-args \
    -p depth_topic:=/SOME_DEPTH_TOPIC \
    -p camera_info_topic:=/SOME_CAMERA_INFO_TOPIC
```

### Depth Camera in Gazebo (with tunable parameters)
```bash
PYTHONNOUSERSITE=1 ros2 run object_detection_pkg depth_controller --ros-args \
    -p depth_topic:=/camera/depth/image_raw \
    -p camera_info_topic:=/camera/depth/camera_info \
    -p use_sim_time:=true \
    -p safe_distance:=0.8 -p forward_speed:=0.15 -p turn_speed:=0.4 -p fov_deg:=25.0 \
    -p turn_choice:=space
```