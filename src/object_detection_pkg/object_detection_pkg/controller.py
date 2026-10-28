import rclpy
from rclpy.executors import SingleThreadedExecutor

from object_detection_pkg.velocity_publisher import VelocityPublisher
from object_detection_pkg.lidar_subscribers import LidarSubscriber

class Controller():
    def __init__(self):
        self.lidar_subscriber = LidarSubscriber()
        self.velocity_publisher = VelocityPublisher()
    def step(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()

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
