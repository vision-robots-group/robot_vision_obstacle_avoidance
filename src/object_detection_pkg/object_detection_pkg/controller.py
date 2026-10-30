
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
        self.actuators = actuators
        self.safe_distance = safe_distance
        self.fov = fov
        self.forward_speed = 0.2
        self.turn_speed = 0.5
        self.timer = self.create_timer(0.1, self.step)

    def compute_command(self, distances, angles):
        if distances is None or angles is None:
            return 0.0, 0.0

        # norm angles to [-pi, pi]
        all_bins = [(d, math.atan2(math.sin(a), math.cos(a))) for d,a in zip(distances, angles)]

        readings = [(d, a) for d, a in all_bins if math.isfinite(d)]

        front_bins = [(d, a) for d, a in all_bins if abs(a) <= self.fov] 

        front = [(d, a) for d, a in readings if abs(a) <= self.fov]  

        # stop robot if there is insufficient forward information
        if not front_bins or len(front) < 0.7 * len(front_bins): 
            return 0.0, 0.0

        blocked = [(d, a) for d, a in front if d < self.safe_distance]  

        # No obstacle: continue forward
        if not blocked:
            return self.forward_speed, 0.0

        # Obstacle: compare left and right clearances
        side_limit = math.radians(45)
        side_start = math.radians(10)

        left = [d for d, a in readings if side_start <= a <= side_limit]

        right = [d for d, a in readings if -side_limit <= a <= -side_start]

        left_clearance = min(left) if left else 0.0
        right_clearance = min(right) if right else 0.0

        # No sufficiently clear turning direction
        if max(left_clearance, right_clearance) < self.safe_distance:
            return 0.0, 0.0

        # Positive angular velocity = left turn, negative = right
        if left_clearance >= right_clearance:
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

        return 0.0, turn

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
