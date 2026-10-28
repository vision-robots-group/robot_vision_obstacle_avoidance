import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Lidar data received: {msg.ranges}')


def main(args=None):
    rclpy.init(args=args)

    lidar_sub = LidarSubscriber()

    rclpy.spin(lidar_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()