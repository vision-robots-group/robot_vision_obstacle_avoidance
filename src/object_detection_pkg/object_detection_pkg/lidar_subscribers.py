import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
PI = math.pi


class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription( LaserScan, '/scan', self.listener_callback, 10)
        self.subscription
        self.scan = None
        self.last_scan_time = None
        self.status = 'no scan received yet'

        self.max_age = float( self.declare_parameter('lidar_max_age', 0.5).value)

        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('lidar_max_age must be finite and positive')

    def listener_callback(self, msg):
        self.scan = msg
        self.last_scan_time = self.get_clock().now()
        self.status = 'ok'

    def _fresh(self):
        if self.scan is None or self.last_scan_time is None:
            return False

        age = ( self.get_clock().now() - self.last_scan_time ).nanoseconds / 1e9

        if age < 0 or age > self.max_age:
            self.status = f'lidar data old ({age:.2f} s)'
            return False

        return True


    def read(self):
        if not self._fresh():
            return None

        return [r for r, _ in self._valid()]


    def get_angles(self):
        if not self._fresh():
            return None

        return [a for _, a in self._valid()]

    def _valid(self):
        s = self.scan
        for i, r in enumerate(s.ranges):
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                yield r, s.angle_min + i * s.angle_increment + PI/2


def main(args=None):
    rclpy.init(args=args)

    lidar_sub = LidarSubscriber()

    rclpy.spin(lidar_sub)

    lidar_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()