#!/usr/bin/env bash

EXTRA_USAGE="
Sensor (first argument, default lidar):
  lidar   LD19 LiDAR + object_detection_pkg detector
  depth   OAK-D Lite + object_detection_pkg depth_detector

Depth camera parameters:
  camera_height        (0.10, metres, measure it on the real rover)
  min_obstacle_height  (0.03)
"
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

SENSOR=lidar
if [[ $# -gt 0 && "$1" != -* && "$1" != *:=* ]]; then
    SENSOR="$1"; shift
fi
[[ "$SENSOR" == lidar || "$SENSOR" == depth ]] || die "unknown sensor '$SENSOR', use lidar or depth"

parse_params "$@"
PARAMS=(--ros-args --params-file "$PROJECT_DIR/config.yaml" "${ROS_PARAMS[@]}")

setup_env
check_no_controller
start_driver

if [[ "$SENSOR" == lidar ]]; then
    start_bg ldlidar "ldlidar" ros2 launch ldlidar ldlidar.launch.py
    wait_for_topic /scan sensor_msgs/msg/LaserScan ldlidar
else
    start_bg oak_d_lite "oak_d_lite.launch.py" ros2 launch ugv_vision oak_d_lite.launch.py
    wait_for_topic /oak/stereo/camera_info sensor_msgs/msg/CameraInfo oak_d_lite
    wait_for_topic /oak/stereo/image_raw sensor_msgs/msg/Image oak_d_lite
fi

start_odometry
start_imu_filter
wait_for_topic /imu/data sensor_msgs/msg/Imu imu_filter
wait_for_topic /odom nav_msgs/msg/Odometry base_node

DETECTOR=detector
[[ "$SENSOR" == depth ]] && DETECTOR=depth_detector
start_bg "$DETECTOR" "object_detection_pkg/$DETECTOR" ros2 run object_detection_pkg "$DETECTOR" "${PARAMS[@]}"
wait_for_topic /obj_detect std_msgs/msg/Float32MultiArray "$DETECTOR"

run_avoidance "${PARAMS[@]}"
