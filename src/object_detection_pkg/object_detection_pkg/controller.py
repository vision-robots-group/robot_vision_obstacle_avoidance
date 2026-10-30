
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from object_detection_pkg.velocity_publisher import VelocityPublisher
from object_detection_pkg.lidar_subscribers import LidarSubscriber
from object_detection_pkg.depth_camera_subscriber import DepthCameraSubscriber


class Controller(Node):
    def __init__(self, sensors, actuators, safe_distance=1.0, fov=math.radians(30)):
        super().__init__('controller')
        self.sensors = sensors
        self.stop_reason = None
        self.actuators = actuators
        self.safe_distance = self.positive_param('safe_distance', safe_distance)
        # half-width of the front window, given in degrees
        self.fov = math.radians(self.positive_param('fov_deg', math.degrees(fov)))
        self.forward_speed = self.positive_param('forward_speed', 0.2)
        self.turn_speed = self.positive_param('turn_speed', 0.5)
        self.escape_turn = None
        # side choice: 'space' (mean distance of side bins) or 'clearance' (closest side bin)
        self.turn_choice = self.declare_parameter('turn_choice', 'space').value
        if self.turn_choice not in ('space', 'clearance'):
            raise ValueError(f"turn_choice must be 'space' or 'clearance', got '{self.turn_choice}'")
        self.timer = self.create_timer(0.1, self.step)

    def positive_param(self, name, default):
        value = float(self.declare_parameter(name, float(default)).value)
        if value <= 0: raise ValueError(f'{name} must be positive, got {value}')
        return value

    def compute_command(self, distances, angles):
        if distances is None or angles is None:
            status = getattr(self.sensors, 'status', 'unknown')
            return self.stop(f'no sensor data ({status})')

        # norm angles to [-pi, pi]
        all_bins = [(d, math.atan2(math.sin(a), math.cos(a))) for d,a in zip(distances, angles)]

        readings = [(d, a) for d, a in all_bins if math.isfinite(d)]

        front_bins = [(d, a) for d, a in all_bins if abs(a) <= self.fov] 

        front = [(d, a) for d, a in readings if abs(a) <= self.fov]  

        # stop robot if there is insufficient forward information
        if not front_bins or len(front) < 0.7 * len(front_bins): 
            return self.stop(f'insufficient front data ({len(front)}/{len(front_bins)} bins)')

        blocked = [(d, a) for d, a in front if d < self.safe_distance]  

        # No obstacle: continue forward
        if not blocked:
            self.stop_reason = None
            self.escape_turn = None
            return self.forward_speed, 0.0

        # Obstacle: compare left and right clearances
        side_limit = math.radians(45)
        side_start = math.radians(10)

        left = [d for d, a in readings if side_start <= a <= side_limit]

        right = [d for d, a in readings if -side_limit <= a <= -side_start]

        left_clearance = min(left) if left else 0.0
        right_clearance = min(right) if right else 0.0

        # turn toward the side with more space (mean distance of its bins)
        left_space = sum(left) / len(left) if left else 0.0
        right_space = sum(right) / len(right) if right else 0.0

        if self.turn_choice == 'clearance':
            left_score, right_score = left_clearance, right_clearance
        else:
            left_score, right_score = left_space, right_space

        # rotate in place when blocked
        if max(left_clearance, right_clearance) < self.safe_distance and self.escape_turn is None:
            self.escape_turn = self.turn_speed if left_score >= right_score else -self.turn_speed

        # Positive angular velocity = left turn, negative = right
        if self.escape_turn is not None:
            turn = self.escape_turn
            direction = 'left (escaping)' if turn > 0 else 'right (escaping)'
        elif left_score >= right_score:
            turn = self.turn_speed
            direction = 'left'
        else:
            turn = -self.turn_speed
            direction = 'right'

        obstacle_bins = ', '.join(f'{math.degrees(a):+.1f}°' for _, a in blocked)

        self.get_logger().info(
            f'Obstacle in bins [{obstacle_bins}]; turning {direction}',
            throttle_duration_sec=1.0
        )

        self.stop_reason = None
        return 0.0, turn

    def stop(self, reason):
        # log immediately when the reason changes, otherwise once per second
        if reason != self.stop_reason:
            self.get_logger().warn(f'Stopping: {reason}')
        else:
            self.get_logger().warn(f'Stopping: {reason}', throttle_duration_sec=1.0)
        self.stop_reason = reason
        return 0.0, 0.0

    def step(self):
        distances = self.sensors.read()
        angles = self.sensors.get_angles()
        self.actuators.apply(self.compute_command(distances, angles))


def _run(sensor_class, args=None):
    rclpy.init(args=args)

    sensor = sensor_class()
    actuator = VelocityPublisher()
    controller = Controller(sensor, actuator, safe_distance=1.0)

    executor = SingleThreadedExecutor()
    for node in (controller, sensor, actuator):
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # stops the rover before shutting down
        controller.get_logger().warn(f'Shutting down; last stop reason: {controller.stop_reason or "none, robot was moving"}')
        actuator.apply((0.0, 0.0))
        actuator.timer_callback()

        for node in (controller, sensor, actuator):
            node.destroy_node()

        rclpy.shutdown()


def main(args=None):
    _run(LidarSubscriber, args)


def main_depth(args=None):
    _run(DepthCameraSubscriber, args)


if __name__ == '__main__':
    main()
