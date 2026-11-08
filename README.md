# Obstacle Avoidance for the UGV Rover

ROS 2 (Humble) obstacle detection and avoidance for the Waveshare UGV Rover, using either the LD19 LiDAR or the OAK-D Lite depth camera.

```
/scan  or  /oak/stereo/*  ──►  detector  ──► /obj_detect ──►  avoidance  ──► /cmd_vel
                                                 /odom, /imu ──┘
```

| Package | Executables | What it does |
|---|---|---|
| `object_detection_pkg` | `detector`, `depth_detector` | Turns LiDAR or depth data into a polar histogram (nearest distance per angular bin) and publishes it on `/obj_detect` |
| `obstacle_avoidance_pkg` | `avoidance` | State machine that follows a straight reference line and drives around obstacles. See [STATE-MACHINE.md](src/obstacle_avoidance_pkg/STATE-MACHINE.md) |

`/obj_detect` is a `Float32MultiArray` laid out as `[distance, angle, distance, angle, ...]` (metres, radians, `NaN` = no reading in that bin). Bins between two close obstacles are filled in when the gap is narrower than the rover.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run on the rover

Inside the rover's docker container (`ssh -p 23 root@ROVER_IP`):

```bash
cd ~/robot_vision_obstacle_avoidance
./scripts/run_avoidance.sh lidar
./scripts/run_avoidance.sh depth camera_height:=0.12
./scripts/run_avoidance.sh lidar safe_distance:=0.6   # override any config.yaml value
```

The script starts the driver, sensor, odometry, detector and avoidance node in order and waits for each topic before moving on. Ctrl-C stops the rover and shuts everything down. **The rover drives as soon as the avoidance node starts, so lift it first.**

Step-by-step manual startup, laptop monitoring and checks are in [STARTUP.md](STARTUP.md).

## Parameters

All parameters live in [`config.yaml`](config.yaml) and are loaded with `--ros-args --params-file config.yaml`. The main ones:

| Node | Parameter | Default | Meaning |
|---|---|---|---|
| detector | `safe_distance` | 1.0 m | Obstacles within `safe_distance * danger_zone_factor` are grouped into objects |
| detector | `hist_fov_deg` / `bin_width_deg` | 120 / 5 | Histogram coverage and bin width (coverage must be a multiple of the width) |
| depth_detector | `camera_height` | 0.10 m | Camera height above the floor, measure it on the rover |
| avoidance | `safe_distance` | 0.5 m | Something closer than this inside the rover's corridor blocks the front |
| avoidance | `half_width` | 0.2 m | Half-width of the corridor in front of the rover |
| avoidance | `v_max` / `v_slow` / `v_turn` | 0.5 / 0.2 / 0.1 m/s | Cruise, detour and turning speed |
| avoidance | `omega_max` | 0.5 rad/s | Max angular speed |

The full list of avoidance parameters and their defaults is in `obstacle_avoidance_pkg/utils.py`.

The LiDAR angles are rotated by +90° in `lidar_subscribers.py`, because the LD19 front arrow points to the rover's left.

## Gazebo

```bash
PYTHONNOUSERSITE=1 ros2 run object_detection_pkg depth_detector --ros-args \
    --params-file config.yaml \
    -p depth_topic:=/camera/depth/image_raw \
    -p camera_info_topic:=/camera/depth/camera_info \
    -p use_sim_time:=true
ros2 run obstacle_avoidance_pkg avoidance --ros-args --params-file config.yaml -p use_sim_time:=true
```

Never set `use_sim_time:=true` on the real rover.
