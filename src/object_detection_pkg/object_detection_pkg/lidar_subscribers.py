import math

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
        self.subscription
        self.scan = None

    def listener_callback(self, msg):
        self.scan = msg

    def read(self):
        """Valid (finite, in-range) ranges, or None if no scan yet."""
        return [r for r, _ in self._valid()] if self.scan else None

    def get_angles(self):
        return [a for _, a in self._valid()] if self.scan else None

    def _valid(self):
        s = self.scan
        for i, r in enumerate(s.ranges):
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                yield r, s.angle_min + i * s.angle_increment


def main(args=None):
    rclpy.init(args=args)

    lidar_sub = LidarSubscriber()

    rclpy.spin(lidar_sub)

    lidar_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()