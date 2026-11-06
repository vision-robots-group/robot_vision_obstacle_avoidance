import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from .utils import State, wrap
from .histogram_subscriber import ObjectDetectionSubscriber

DEFAULTS = {
    'odom_topic': '/odom',
    'cmd_vel_topic': '/cmd_vel',
    'safe_distance': 0.5,
    'fov_deg': 30.0,
    'side_start_deg': 10.0,
    'side_limit_deg': 45.0,
    'turn_choice': 'safe',
    'v_max': 0.2,
    'v_slow': 0.1,
    'v_turn': 0.0,
    'omega_max': 0.5,
    'theta_avoid_deg': 45.0,
    'theta_return_deg': 30.0,
    'theta_tol_deg': 5.0,
    'e_min': 0.4,
    'e_tol': 0.05,
    'pass_min': 0.5,
    'clear_ticks': 3,
    'free_dist': 3.0,
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
    - An object is detected in front, we pick the freer side and rotate towards it (TURN_AWAY)
    - We keep going at that angle till the front is clear and we are far enough from the line (SIDESTEP)
    - We go parallel to the line till we passed the object (PASS)
    - We go back towards the line till we are on it again (RETURN)
    - We rotate back to the original heading (REALIGN) and go straight again

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
            State.PASS: self.__drive_by_obstacle,
            State.RETURN: self.__return_to_path,
            State.REALIGN: self.__realign
        }

        self.__load_config()

        self.create_subscription(Odometry, self.odom_topic, self.__odom_callback, 10)
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
        Odometry callback, saves the pose as (x, y, theta).
        p is the position, q is the quaternion orientation, theta is the yaw angle.
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, 2.0 * math.atan2(q.z, q.w))
        self.odom_stamp = self.get_clock().now()


    def publish_cmd(self, v, w):
        """Used to publish to ``/cmd_vel``."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def step(self):
        """Control loop. Execute on each timer tick."""
        hist = self.hist_sub.read()
        odom_age = None if self.odom_stamp is None else (self.get_clock().now() - self.odom_stamp).nanoseconds / 1e9
        if hist is None or odom_age is None or not 0 <= odom_age <= self.odom_max_age:
            self.publish_cmd(0.0, 0.0) # no fresh data, stop
            return
        self.hist = hist

        x, y, th = self.pose
        if self.origin is None: # first pose, save the reference line
            self.origin, self.angle = (x, y), th
        s, e = self.__errors(x, y)
        front_blocked = any(d < self.safe_distance for d, a in self.hist if abs(a) <= self.fov)

        # new object in front: start the avoidance again, the line stays the same
        if front_blocked and self.state not in (State.CORNER, State.TURN_AWAY):
            self.sigma = self.__pick_side()
            self.state = State.TURN_AWAY

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

    def __pick_side(self):
        def score(side):
            sector = [(d if math.isfinite(d) else self.free_dist, a) for d, a in self.__sector(side)]
            if self.turn_choice == 'clearance': return sum(d * (1 - abs(a) / self.side_limit) for d, a in sector)
            return sum(d for d, _ in sector)
        return 1 if score(1) >= score(-1) else -1

    # Callbacks ============================================
    # Each of them returns a linear and angular velocity based on
    # the current state and the current position of the robot.

    def __cruise(self, th, s, e, front_blocked):
        """Cruise: just follow the line."""
        return self.v_max, -self.k_e * e + self.k_theta * wrap(self.angle - th)

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
            self.state = State.RETURN
        return self.v_slow, self.__hold(th, self.angle)

    def __return_to_path(self, th, s, e, front_blocked):
        """Return: head back to the line."""
        if abs(e) < self.e_tol:
            self.state = State.REALIGN
        return self.v_slow, self.__hold(th, self.angle - self.sigma * self.theta_return)

    def __realign(self, th, s, e, front_blocked):
        """Realign: face forward again."""
        if abs(wrap(self.angle - th)) < self.theta_tol:
            self.state = State.CRUISE
        return self.v_turn, self.__hold(th, self.angle)

    def __corner(self, th, s, e, front_blocked):
        """Corner: stuck, spin in place till something opens up."""
        if not front_blocked: self.state, self.clear_count, self.heading = State.SIDESTEP, 0, th
        return 0.0, self.sigma * self.omega_max
