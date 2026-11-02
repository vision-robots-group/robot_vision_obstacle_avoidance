#!/usr/bin/env bash

EXTRA_USAGE="
Depth camera parameters:
  camera_height        (0.10, metres, measure it on the real rover)
  min_obstacle_height  (0.03)
  depth_topic          (/oak/stereo/image_raw)
  camera_info_topic    (/oak/stereo/camera_info)
"
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

parse_params "$@"

DEPTH_TOPIC=/oak/stereo/image_raw
INFO_TOPIC=/oak/stereo/camera_info
for ((i = 0; i < ${#ROS_PARAMS[@]}; i++)); do
    case "${ROS_PARAMS[$i]}" in
        depth_topic:=*) DEPTH_TOPIC="${ROS_PARAMS[$i]#*:=}" ;;
        camera_info_topic:=*) INFO_TOPIC="${ROS_PARAMS[$i]#*:=}" ;;
    esac
done

setup_env
check_no_controller
start_driver
start_bg oak_d_lite "oak_d_lite.launch.py" ros2 launch ugv_vision oak_d_lite.launch.py
wait_for_topic "$INFO_TOPIC" sensor_msgs/msg/CameraInfo oak_d_lite
wait_for_topic "$DEPTH_TOPIC" sensor_msgs/msg/Image oak_d_lite
run_controller depth_controller
