import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from .utils import State, wrap
from .histogram_subscriber import ObjectDetectionSubscriber

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

class Agent(Node):
    """
    This agent will read the histogram from the topic ``/obj_detect`` and the pose from ``/odom``, and publish on the ``/cmd_vel`` topic.
    It works as a state machine: each state has its own callback that returns the linear and angular velocity.

    At start we save a reference line (``origin`` and ``angle``), which is the path we want to follow.
    From the pose we always know how far we went along the line (``s``) and how far we are from it (``e``),
    so we don't need to time the rotations, we just check the odometry.

    Object avoidance case:
    - We are going straight on the line (CRUISE)
    - An object is detected in front, we pick the side whose obstacle edge is closer and rotate towards it (TURN_AWAY)
    - We keep going at that angle till the front is clear and we are far enough from the line (SIDESTEP)
    - We go parallel to the line till we passed the object (PASS)
    - We go back to CRUISE, whose line-following terms bring us back on the line

    If both sides are blocked we rotate in place till the front is clear (CORNER).
    If a new object shows up during the avoidance we just start again from TURN_AWAY, the line stays the same.
    """
    def __init__(self, hist_sub: ObjectDetectionSubscriber):
        super().__init__('agent')
        self.hist_sub = hist_sub
        self.origin = None # (x, y), set on first pose
        self.angle = 0.0
        self.hist = []
        self.pose = None # (x, y, theta)
        self.odom_stamp = None
        self.imu_stamp = None
        self.calib_start = None
        self.gyro = []
        self.bias = None
        self.yaw = 0.0
        self.x, self.y = 0.0, 0.0
        self.heading = 0.0
        self.state = State.CRUISE
        self.sigma = 1
        self.clear_count = 0
        self.s_pass = 0.0

        self.callback_dict = {
            State.CORNER: self.__corner,
            State.CRUISE: self.__cruise,
            State.TURN_AWAY: self.__turn_away,
            State.SIDESTEP: self.__clear_front,
            State.PASS: self.__drive_by_obstacle
        }

        self.__load_config()

        self.create_subscription(Odometry, self.odom_topic, self.__odom_callback, 10)
        self.create_subscription(Imu, self.imu_topic, self.__imu_callback, 50)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.step)

    def __load_config(self):
        """
        Load the parameters (``--params-file config.yaml``).
        """
        for k, default in DEFAULTS.items():
            v = self.declare_parameter(k, default).value
            if k.endswith('_deg'): setattr(self, k[:-4], math.radians(v)) # convert to radians if in degrees
            else: setattr(self, k, v)

    # ROS I/O ==============================================

    def __odom_callback(self, msg):
        """
        Odometry callback, integrates the wheel speed along the gyro yaw into the pose (x, y, theta).
        """
        now = self.get_clock().now()
        if self.bias is not None and self.odom_stamp is not None:
            ds = msg.twist.twist.linear.x * (now - self.odom_stamp).nanoseconds / 1e9
            self.x += ds * math.cos(self.yaw)
            self.y += ds * math.sin(self.yaw)
            self.pose = (self.x, self.y, self.yaw)
        self.odom_stamp = now

    def __imu_callback(self, msg):
        now = self.get_clock().now()
        gz = msg.angular_velocity.z
        if self.bias is None:
            if self.calib_start is None: self.calib_start = now
            self.gyro.append(gz)
            if (now - self.calib_start).nanoseconds / 1e9 >= self.calib_time:
                self.bias = sum(self.gyro) / len(self.gyro)
        elif self.imu_stamp is not None:
            self.yaw = wrap(self.yaw + (gz - self.bias) * (now - self.imu_stamp).nanoseconds / 1e9)
        self.imu_stamp = now


    def publish_cmd(self, v, w):
        """Used to publish to ``/cmd_vel``."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def step(self):
        """Control loop. Execute on each timer tick."""
        hist = self.hist_sub.read()
        now = self.get_clock().now()
        fresh = all(t is not None and 0 <= (now - t).nanoseconds / 1e9 <= self.odom_max_age
                    for t in (self.odom_stamp, self.imu_stamp))
        if hist is None or self.pose is None or not fresh:
            self.publish_cmd(0.0, 0.0) # no fresh data, stop
            return
        self.hist = hist

        x, y, th = self.pose
        if self.origin is None: # first pose, save the reference line
            self.origin, self.angle = (x, y), th
        s, e = self.__errors(x, y)
        front_blocked = any(self.__in_front(d, a) for d, a in self.hist if abs(a) <= self.fov)

        # new object in front: start the avoidance again, the line stays the same
        if front_blocked and self.state not in (State.CORNER, State.TURN_AWAY):
            if self.state == State.CRUISE:
                self.sigma = self.__pick_side(e)
                self.state = State.TURN_AWAY
            else:
                self.state = State.CORNER

        v, w = self.callback_dict[self.state](th, s, e, front_blocked)
        self.publish_cmd(v, max(-self.omega_max, min(self.omega_max, w)))

    def __errors(self, x, y):
        """
        Progress along the line (``s``) and distance from it (``e``, > 0 if left of the line).
        """
        dx, dy = x - self.origin[0], y - self.origin[1]
        s = dx * math.cos(self.angle) + dy * math.sin(self.angle)
        e = -dx * math.sin(self.angle) + dy * math.cos(self.angle)
        return s, e

    def __hold(self, th, target):
        """
        Used to turn the robot towards the target angle and keep it there.
        It uses a proportional controller and bounds the angular velocity to omega_max.
        We want to slow down when close to the target angle, and this is a simple way to do it.
        """
        w = self.k_theta * wrap(target - th)
        return max(-self.omega_max, min(self.omega_max, w))

    def __sector(self, side):
        return [(d, a) for d, a in self.hist if self.side_start <= side * a <= self.side_limit]

    def __side_clear(self, side):
        return all(not d < self.safe_distance for d, _ in self.__sector(side))

    def __in_front(self, d, a):
        return d * math.cos(a) < self.safe_distance and abs(d * math.sin(a)) < self.half_width

    def __pick_side(self, e):
        bins = sorted(self.hist, key=lambda b: b[1])
        blocked = [not math.isfinite(d) or d < self.near_dist for d, _ in bins]
        seeds = [k for k, (d, a) in enumerate(bins) if abs(a) <= self.fov and self.__in_front(d, a)]
        towards_line = -1 if e > 0 else 1
        if not seeds: return towards_line
        i = j = min(seeds, key=lambda k: abs(bins[k][1]))
        while i > 0 and blocked[i - 1]: i -= 1
        while j < len(bins) - 1 and blocked[j + 1]: j += 1
        left = math.pi if j == len(bins) - 1 else bins[j][1]
        right = math.pi if i == 0 else -bins[i][1]
        if abs(left - right) < 1e-6: return towards_line
        return 1 if left < right else -1

    def __speed(self, v_min):
        front = [d * math.cos(a) if abs(d * math.sin(a)) < self.half_width else math.inf
                 for d, a in self.hist if math.isfinite(d) and abs(a) <= self.fov]
        if not front: return v_min
        k = max(0.0, min(1.0, (min(front) - self.safe_distance) / self.ramp))
        return v_min + k * (self.v_max - v_min)

    # Callbacks ============================================
    # Each of them returns a linear and angular velocity based on
    # the current state and the current position of the robot.

    def __cruise(self, th, s, e, front_blocked):
        """Cruise: just follow the line."""
        return self.__speed(self.v_slow), -self.k_e * e + self.k_theta * wrap(self.angle - th)

    def __turn_away(self, th, s, e, front_blocked):
        """Turn away: spin toward the free side."""
        if not self.__side_clear(1) and not self.__side_clear(-1):
            self.state = State.CORNER
            return 0.0, 0.0
        target = self.angle + self.sigma * self.theta_avoid
        if abs(wrap(target - th)) < self.theta_tol:
            self.state = State.CORNER if front_blocked else State.SIDESTEP
            self.clear_count, self.heading = 0, target
        return self.v_turn, self.__hold(th, target)

    def __clear_front(self, th, s, e, front_blocked):
        """Sidestep: slide sideways till the front is free."""
        self.clear_count = 0 if front_blocked else self.clear_count + 1
        if self.clear_count >= self.clear_ticks and abs(e) >= self.e_min:
            self.state, self.s_pass = State.PASS, s
        return self.v_slow, self.__hold(th, self.heading)

    def __drive_by_obstacle(self, th, s, e, front_blocked):
        """Pass: go straight next to the obstacle."""
        if not self.__side_clear(-self.sigma):
            self.s_pass = s
        elif s - self.s_pass >= self.pass_min:
            self.state = State.CRUISE
        return self.__speed(self.v_slow), self.__hold(th, self.angle)

    def __corner(self, th, s, e, front_blocked):
        """Corner: stuck, spin in place till something opens up."""
        if not front_blocked: self.state, self.clear_count, self.heading = State.SIDESTEP, 0, th
        return 0.0, self.sigma * self.omega_max
