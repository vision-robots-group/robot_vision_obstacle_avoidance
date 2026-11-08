from enum import Enum, auto
import math

class State(Enum):
    CRUISE = auto()
    TURN_AWAY = auto()
    SIDESTEP = auto()
    PASS = auto()
    CORNER = auto()

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

DEFAULTS = {
    'odom_topic': '/odom',
    'imu_topic': '/imu/data_raw',
    'calib_time': 3.0,
    'cmd_vel_topic': '/cmd_vel',
    'safe_distance': 0.5,
    'half_width': 0.2,
    'fov_deg': 30.0,
    'side_start_deg': 10.0,
    'side_limit_deg': 45.0,
    'v_max': 0.2,
    'v_slow': 0.1,
    'v_turn': 0.0,
    'omega_max': 0.5,
    'theta_avoid_deg': 45.0,
    'theta_tol_deg': 5.0,
    'e_min': 0.4,
    'pass_min': 0.5,
    'clear_ticks': 3,
    'near_dist': 1.5,
    'ramp': 1.0,
    'odom_max_age': 0.5,
    'k_theta': 1.5,
    'k_e': 1.0,
    'rate': 10.0,
}