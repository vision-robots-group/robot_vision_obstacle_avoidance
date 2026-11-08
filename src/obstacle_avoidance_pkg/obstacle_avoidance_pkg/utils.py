from enum import Enum, auto
import math
import os
import shutil
import signal
import subprocess

class State(Enum):
    CRUISE = auto()
    TURN_AWAY = auto()
    SIDESTEP = auto()
    PASS = auto()
    CORNER = auto()

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

class Speaker:
    """Says short texts with espeak-ng (or espeak) piped to aplay, without blocking. ``SPEECH=0`` disables it."""
    def __init__(self):
        enabled = os.environ.get('SPEECH', '1') != '0'
        self.cmd = (shutil.which('espeak-ng') or shutil.which('espeak')) if enabled else None
        self.device = os.environ.get('AUDIO_DEVICE', 'default')
        self.proc = None

    def say(self, text):
        if self.cmd is None: return
        if self.proc is not None and self.proc.poll() is None:
            try: os.killpg(self.proc.pid, signal.SIGTERM)
            except ProcessLookupError: pass
        self.proc = subprocess.Popen(['sh', '-c', '"$0" --stdout "$1" | aplay -q -D "$2"', self.cmd, text, self.device],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

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