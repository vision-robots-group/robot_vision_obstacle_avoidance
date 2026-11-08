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
Usage: $(basename "$0") [lidar | depth] [name:=value | --name value | --name=value] ...

Parameters are loaded from config.yaml, every argument overrides one of them (-p name:=value).

Examples:
  $(basename "$0") lidar safe_distance:=0.5
  $(basename "$0") depth --safe_distance 0.5 --v_max 0.15 --camera_height 0.12

Main parameters (see config.yaml for all of them):
  safe_distance  (1.0)    fov_deg        (30.0)
  v_max          (0.2)    v_slow         (0.1)
  omega_max      (0.5)    near_dist      (1.5)
$EXTRA_USAGE
Environment:
  ROS_DOMAIN_ID   (default 18, current: $ROS_DOMAIN_ID)
  UGV_WS          (default /home/ws/ugv_ws)
  SENSOR_TIMEOUT  seconds to wait for sensor data (default 30)
  STOP_TIMEOUT    seconds to wait for each node to stop on Ctrl-C (default 10)
  LOG_DIR         background node logs (default /tmp/obstacle_avoidance_logs)
  SPEECH          0 disables the spoken state changes (default 1)
  AUDIO_DEVICE    ALSA device for speech (default: first USB card, else default)

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
    local topic="$1" msg_type="$2" name="$3"
    local deadline=$((SECONDS + SENSOR_TIMEOUT))
    log "waiting up to ${SENSOR_TIMEOUT}s for data on $topic"
    while ((SECONDS < deadline)); do
        if timeout 5 ros2 topic echo --once --no-arr "$topic" "$msg_type" >/dev/null 2>&1; then
            log "$topic is publishing"
            return
        fi
        sleep 1
    done
    die "no data on $topic, check $LOG_DIR/$name.log"
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
    if pgrep -f "object_detection_pkg/(depth_)?(controller|detector)|obstacle_avoidance_pkg/avoidance" >/dev/null; then
        die "another obstacle avoidance controller is already running, stop it first"
    fi
}

start_driver() {
    start_bg ugv_driver "ugv_bringup/ugv_driver" ros2 run ugv_bringup ugv_driver
}

start_odometry() {
    if ros2 topic info /odom 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        log "/odom already published, leaving it untouched"
        return
    fi
    ros2 pkg executables ugv_bringup | grep -q " ugv_bringup$" \
        || die "ugv_bringup ugv_bringup not found, see STARTUP.md (Odometry)"
    ros2 pkg executables ugv_base_node | grep -q " base_node$" \
        || die "ugv_base_node base_node not found, see STARTUP.md (Odometry)"
    start_bg ugv_bringup "ugv_bringup/ugv_bringup" ros2 run ugv_bringup ugv_bringup
    start_bg base_node "ugv_base_node/base_node" ros2 run ugv_base_node base_node
}

setup_speech() {
    if [[ "${SPEECH:-1}" == 0 ]]; then
        log "speech disabled (SPEECH=0)"
        return
    fi
    if ! command -v espeak-ng >/dev/null && ! command -v espeak >/dev/null || ! command -v aplay >/dev/null; then
        log "installing espeak-ng and alsa-utils"
        local sudo=""
        [[ $EUID -ne 0 ]] && sudo="sudo -n"
        if ! { $sudo apt-get update -qq && DEBIAN_FRONTEND=noninteractive $sudo apt-get install -y -qq espeak-ng alsa-utils; } >/dev/null 2>&1; then
            log "WARNING: cannot install espeak-ng / alsa-utils, running without speech"
            return
        fi
    fi
    if [[ ! -d /dev/snd ]]; then
        log "WARNING: no /dev/snd in the container (start it with --device /dev/snd or --privileged), running without speech"
        return
    fi
    if [[ -z "${AUDIO_DEVICE:-}" ]]; then
        local card
        card=$(aplay -l 2>/dev/null | awk '/^card [0-9]+:/ && tolower($0) ~ /usb/ {sub(":", "", $2); print $2; exit}')
        AUDIO_DEVICE="${card:+plughw:$card,0}"
    fi
    export AUDIO_DEVICE="${AUDIO_DEVICE:-default}"
    local tts
    tts=$(command -v espeak-ng || command -v espeak)
    if timeout 5 bash -c '"$0" --stdout "speech ready" | aplay -q -D "$1"' "$tts" "$AUDIO_DEVICE" 2>"$LOG_DIR/speech.log"; then
        log "speech on $AUDIO_DEVICE"
    else
        log "WARNING: speech test failed on $AUDIO_DEVICE, see $LOG_DIR/speech.log (set AUDIO_DEVICE, list cards with aplay -l)"
    fi
}

run_avoidance() {
    setup_speech
    log "LIFT THE ROVER OR CLEAR THE FLOOR: it drives as soon as the avoidance starts"
    log "running: ros2 run obstacle_avoidance_pkg avoidance $*"
    log "press Ctrl-C to stop the rover and shut everything down"
    ros2 run obstacle_avoidance_pkg avoidance "$@"
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
