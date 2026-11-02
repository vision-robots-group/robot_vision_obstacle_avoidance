#!/usr/bin/env bash

EXTRA_USAGE=""
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

parse_params "$@"
setup_env
check_no_controller
start_driver
start_bg ldlidar "ldlidar" ros2 launch ldlidar ldlidar.launch.py
wait_for_topic /scan sensor_msgs/msg/LaserScan ldlidar
run_controller controller
