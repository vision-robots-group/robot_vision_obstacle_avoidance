import rclpy
from rclpy.executors import SingleThreadedExecutor

from object_detection_pkg.velocity_publisher import VelocityPublisher
from object_detection_pkg.lidar_subscribers import LidarSubscriber

class Controller():
    def __init__(self, sensors, actuators, safe_distance = 1.0):
        self.sensors = sensors
        self.actuators = actuators
        self.safe_distance = safe_distance
        self.angle = 0.0

    def compute_command(self, distances, angle):
        if distances is None:
            return 0.0, 0.0  # No command if no distances are available

        min_distance = min(distances)
        if min_distance < self.safe_distance:
            # If an obstacle is too close, stop or turn
            linear_velocity = 0.0
            angular_velocity = -0.5
            self.get_logger().info(f'Obstacle detected! Min distance: {min_distance:.2f}. Stopping or turning.')
        else:
            # If the path is clear, move forward
            linear_velocity = self.angle
            angular_velocity = 0.5
            self.get_logger().info(f'Path clear. Min distance: {min_distance:.2f}. Moving forward.')

        return linear_velocity, angular_velocity

    def step(self):
        distances = self.sensors.read()
        angles = self.sensors.get_angles()
        command = self.compute_command(distances, angles)
        self.actuators.apply(command)


def main(args=None):
    rclpy.init(args=args)
    sensor = LidarSubscriber()
    actuator = VelocityPublisher()
    controller = Controller(sensors=sensor, actuators=actuator, safe_distance=1.0)

    executor = SingleThreadedExecutor()
    executor.add_node(controller.lidar_subscriber)
    executor.add_node(controller.velocity_publisher)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.lidar_subscriber.destroy_node()
        controller.velocity_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
