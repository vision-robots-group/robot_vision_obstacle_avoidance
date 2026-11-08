# Startup on the UGV Rover

Everything to do after connecting to the rover, terminal by terminal.

- **D terminals** run inside the rover's docker container. They start the driver, the sensors and our nodes.
- **L terminals** run on the laptop. They only watch topics and never send commands.

`ROS_DOMAIN_ID` is `18` below. Use the same value on the rover and on the laptop. If you use another value, change it everywhere.

## 0. Once per session: update and build on the rover

```bash
ssh -p 23 root@ROVER_IP
cd ~/robot_vision_obstacle_avoidance
git pull

# an old checkout may still have the old package, which has the same name and breaks colcon build
[ -d src/object_detection_pkg_old ] && touch src/object_detection_pkg_old/COLCON_IGNORE

source /opt/ros/humble/setup.bash
source /home/ws/ugv_ws/install/setup.bash
colcon build --symlink-install
```

## One command (instead of section 1)

`scripts/run_avoidance.sh` starts in one terminal on the rover what D1–D5 below start by hand: driver, sensor, odometry, detector, avoidance. It waits for each sensor topic before starting the next node.

```bash
ssh -p 23 root@ROVER_IP
cd ~/robot_vision_obstacle_avoidance
./scripts/run_avoidance.sh lidar                      # or: depth camera_height:=0.12
./scripts/run_avoidance.sh lidar safe_distance:=0.5   # override any config.yaml parameter
./scripts/run_avoidance.sh --help
```

**Lift the rover first: it drives as soon as the avoidance starts.**

Ctrl-C stops the avoidance (zero velocity), then the detector, odometry, sensor and driver started by the script, in reverse order. Nodes that were already running are left untouched. Node logs are in `/tmp/obstacle_avoidance_logs`. `ROS_DOMAIN_ID` defaults to 18 (`ROS_DOMAIN_ID=5 ./scripts/run_avoidance.sh lidar` to change it).

If it stops with `ugv_bringup ugv_bringup not found`, the odometry node names differ on this rover: see [Odometry](#odometry-d3).

The laptop terminals (section 2) still work the same for watching.

## 1. Rover (docker) terminals

Open each one with `ssh -p 23 root@ROVER_IP`, then paste:

```bash
export ROS_DOMAIN_ID=18
source /opt/ros/humble/setup.bash
source /home/ws/ugv_ws/install/setup.bash
cd ~/robot_vision_obstacle_avoidance && source install/setup.bash
```

| Terminal | Purpose | Command |
|---|---|---|
| D1 | Motor driver | `ros2 run ugv_bringup ugv_driver` |
| D2 | LiDAR (LiDAR detector) | `ros2 launch ldlidar ldlidar.launch.py` |
| D2 | OAK-D camera (depth detector) | `ros2 launch ugv_vision oak_d_lite.launch.py` |
| D3 | Odometry | see [Odometry](#odometry-d3) |
| D4 | LiDAR detector | `ros2 run object_detection_pkg detector --ros-args --params-file config.yaml` |
| D4 | Depth detector | `ros2 run object_detection_pkg depth_detector --ros-args --params-file config.yaml -p camera_height:=0.10` |
| D5 | Avoidance (**moves the rover**) | `ros2 run obstacle_avoidance_pkg avoidance --ros-args --params-file config.yaml` |

For D2 and D4, run either the LiDAR pair or the depth camera pair, not both. For the depth detector, measure `camera_height` on the rover.

Do not use `bringup_lidar.launch.py` for D2: it starts a second `ugv_driver`.

### Odometry (D3)

The avoidance node needs `/odom` (wheel speed) and `/imu/data_raw` (gyro). `ugv_driver` publishes neither. If either one is missing or older than 0.5 s, the avoidance node only publishes zero velocity and the rover never moves.

At start-up the avoidance node measures the gyro bias for 3 s (`calib_time`). Keep the rover still during that time; it publishes zero velocity until the calibration is done.

The odometry nodes come from the UGV workspace. Check their names on the rover the first time:

```bash
ros2 pkg executables ugv_bringup
ros2 pkg executables | grep -i -E "base|odom"
grep -rn "odom" /home/ws/ugv_ws/src/*/ugv_bringup/launch/*.py | head
```

On the Waveshare workspace this is usually two nodes, each in its own terminal:

```bash
# D3a: reads encoders and IMU from the board, publishes /imu/data_raw
ros2 run ugv_bringup ugv_bringup
# D3b: turns odom_raw + imu into /odom
ros2 run ugv_base_node base_node
```

If the launch file found by `grep` starts an IMU filter before `base_node`, start that one as well.

## 2. Laptop terminals

No build needed: all messages are standard ROS types. In each terminal:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=18
```

| Terminal | Command | Expected |
|---|---|---|
| L1 | `ros2 topic hz /odom` | ~10–20 Hz |
| L1 | `ros2 topic hz /imu/data_raw` | non-zero rate |
| L2 | `ros2 topic hz /obj_detect` | ~10 Hz |
| L2 | `ros2 topic echo /obj_detect --once --field data` | `[distance, angle, distance, angle, ...]`, metres and radians |
| L3 | `ros2 topic echo /cmd_vel` | see [Checks](#4-checks-rover-lifted) |
| L4 | `rviz2 -f base_lidar_link` | Optional. Add → By topic → `/scan` → LaserScan, then Zero in the Views panel |

If `ros2 topic list` on the laptop shows only `/parameter_events` and `/rosout`, the network blocks ROS discovery. Run L1–L3 in extra docker terminals instead.

## 3. Start order

1. D1 driver
2. D2 sensor, then check `ros2 topic hz /scan` (LiDAR, ~10 Hz) or `ros2 topic hz /oak/stereo/image_raw` (camera)
3. D3 odometry, then check L1
4. D4 detector, then check L2
5. **Lift the rover** (wheels off the floor)
6. D5 avoidance, then watch L3

## 4. Checks (rover lifted)

| Situation | Expected on `/cmd_vel` |
|---|---|
| First 3 s after D5 starts (gyro calibration) | all zero |
| Nothing in front | `linear.x` between 0.2 and 0.5, small `angular.z` |
| Hand within 0.5 m (`safe_distance`) straight in front | `linear.x: 0.1` (or 0.0 if both sides are blocked), `angular.z` non-zero |
| Ctrl-C on D5 | last message all zero, log `Shutting down in state ...` |
| Ctrl-C on D4 (detector) | all zero within 0.5 s |
| Ctrl-C on D3 (odometry) | all zero within 0.5 s |

Only put the rover on the floor once all checks pass.

## 5. Stop

1. Ctrl-C on D5 first (avoidance, stops the rover)
2. Then D4, D3, D2, D1 in any order

## Troubleshooting

- **`_ARRAY_API not found` NumPy traceback at startup**: harmless, ignore it.
- **`/cmd_vel` always zero**: check `/odom`, `/imu/data_raw` (L1) and `/obj_detect` (L2). The avoidance node stops if any of them is missing or older than 0.5 s.
- **Rover does not move although `/cmd_vel` is non-zero**: D1 (`ugv_driver`) is not running, or it runs on another `ROS_DOMAIN_ID`.
- **Unknown nodes in `ros2 node list`**: another group may be on the same `ROS_DOMAIN_ID`.
- **Do not set `use_sim_time:=true` on the real rover.**
