#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
UGV_WS="${UGV_WS:-/home/ws/ugv_ws}"
LOG_DIR="${LOG_DIR:-/tmp/obstacle_avoidance_logs}"
SENSOR_TIMEOUT="${SENSOR_TIMEOUT:-30}"
STOP_TIMEOUT="${STOP_TIMEOUT:-10}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-18}"

STARTED_PIDS=()
STARTED_NAMES=()
ROS_PARAMS=()
CLEANED_UP=0

set -m

log() { echo "[$(basename "$0")] $*"; }
die() { log "ERROR: $*" >&2; exit 1; }

usage() {
    cat <<EOF
Usage: $(basename "$0") [name:=value | --name value | --name=value] ...

Every argument is forwarded to the controller as a ROS parameter (-p name:=value).

Examples:
  $(basename "$0") safe_distance:=0.5
  $(basename "$0") --safe_distance 0.5 --forward_speed 0.15 --turn_choice clearance

Controller parameters:
  safe_distance  (1.0)    fov_deg        (30.0)
  forward_speed  (0.2)    turn_speed     (0.5)
  turn_choice    (space | clearance)
$EXTRA_USAGE
Environment:
  ROS_DOMAIN_ID   (default 18, current: $ROS_DOMAIN_ID)
  UGV_WS          (default /home/ws/ugv_ws)
  SENSOR_TIMEOUT  seconds to wait for sensor data (default 30)
  STOP_TIMEOUT    seconds to wait for each node to stop on Ctrl-C (default 10)
  LOG_DIR         background node logs (default /tmp/obstacle_avoidance_logs)

Ctrl-C stops the controller (zero velocity), then the sensor and driver started by this script.
EOF
}

parse_params() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help) usage; exit 0 ;;
            --*=*)
                local kv="${1#--}"
                ROS_PARAMS+=(-p "${kv%%=*}:=${kv#*=}"); shift ;;
            --*)
                [[ $# -ge 2 ]] || die "missing value for $1"
                ROS_PARAMS+=(-p "${1#--}:=$2"); shift 2 ;;
            *:=*) ROS_PARAMS+=(-p "$1"); shift ;;
            *) die "unrecognised argument '$1' (use name:=value or --name value, see --help)" ;;
        esac
    done
}

setup_env() {
    source /opt/ros/humble/setup.bash || die "cannot source ROS Humble"
    source "$UGV_WS/install/setup.bash" || die "cannot source $UGV_WS/install/setup.bash"
    if [[ ! -f "$PROJECT_DIR/install/setup.bash" ]]; then
        log "workspace not built, running colcon build in $PROJECT_DIR"
        (cd "$PROJECT_DIR" && colcon build --symlink-install) || die "colcon build failed"
    fi
    source "$PROJECT_DIR/install/setup.bash" || die "cannot source $PROJECT_DIR/install/setup.bash"
    mkdir -p "$LOG_DIR"
    log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID, logs in $LOG_DIR"
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM HUP
}

start_bg() {
    local name="$1" pattern="$2"; shift 2
    if pgrep -f "$pattern" >/dev/null; then
        log "$name already running, leaving it untouched"
        return
    fi
    log "starting $name: $*"
    "$@" >"$LOG_DIR/$name.log" 2>&1 &
    STARTED_PIDS+=("$!")
    STARTED_NAMES+=("$name")
}

wait_for_topic() {
    local topic="$1" name="$2"
    log "waiting up to ${SENSOR_TIMEOUT}s for data on $topic"
    if ! timeout "$SENSOR_TIMEOUT" ros2 topic echo "$topic" --once >/dev/null 2>&1; then
        die "no data on $topic, check $LOG_DIR/$name.log"
    fi
    log "$topic is publishing"
}

group_alive() {
    kill -0 -- "-$1" 2>/dev/null
}

wait_group() {
    local pgid="$1" seconds="$2" i
    for ((i = 0; i < seconds * 10; i++)); do
        group_alive "$pgid" || return 0
        sleep 0.1
    done
    return 1
}

stop_group() {
    local pgid="$1" name="$2"
    group_alive "$pgid" || return
    log "stopping $name"
    kill -INT -- "-$pgid" 2>/dev/null
    wait_group "$pgid" "$STOP_TIMEOUT" && return
    log "$name did not stop after SIGINT, sending SIGTERM"
    kill -TERM -- "-$pgid" 2>/dev/null
    wait_group "$pgid" 5 && return
    log "$name still running, sending SIGKILL"
    kill -KILL -- "-$pgid" 2>/dev/null
}

send_zero_velocity() {
    if pgrep -f "ugv_bringup/ugv_driver" >/dev/null; then
        log "publishing zero velocity"
        timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
    fi
}

cleanup() {
    [[ $CLEANED_UP -eq 1 ]] && return
    CLEANED_UP=1
    trap '' INT TERM HUP
    send_zero_velocity
    local i
    for ((i = ${#STARTED_PIDS[@]} - 1; i >= 0; i--)); do
        stop_group "${STARTED_PIDS[$i]}" "${STARTED_NAMES[$i]}"
    done
    wait 2>/dev/null
    log "shutdown complete"
}

check_no_controller() {
    if pgrep -f "object_detection_pkg/(depth_)?controller" >/dev/null; then
        die "another obstacle avoidance controller is already running, stop it first"
    fi
}

start_driver() {
    start_bg ugv_driver "ugv_bringup/ugv_driver" ros2 run ugv_bringup ugv_driver
}

run_controller() {
    local executable="$1"
    log "LIFT THE ROVER OR CLEAR THE FLOOR: it drives as soon as the controller starts"
    log "running: ros2 run object_detection_pkg $executable --ros-args ${ROS_PARAMS[*]}"
    log "press Ctrl-C to stop the rover and shut everything down"
    if [[ ${#ROS_PARAMS[@]} -gt 0 ]]; then
        ros2 run object_detection_pkg "$executable" --ros-args "${ROS_PARAMS[@]}"
    else
        ros2 run object_detection_pkg "$executable"
    fi
}
