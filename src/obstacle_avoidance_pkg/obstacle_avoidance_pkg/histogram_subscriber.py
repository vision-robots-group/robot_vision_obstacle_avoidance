"""
This class subscribes to the topic ``/obj_detect`` (published by ``object_detection_pkg``).
The Agent class (``controller.py``) uses this class to get the distances and angles
of the detected objects in order to avoid them.

Message layout (``Float32MultiArray``): ``[d0, a0, d1, a1, ...]``,
one row per bin, distance in meters (NaN = unknown bin) and angles in rad.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ObjectDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('histogram_subscriber')
        topic = self.declare_parameter('hist_topic', '/obj_detect').value
        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic,
            self.listener_callback,
            10)
        self.hist = None # list of (distance, angle) bins
        self.stamp = None # receive time of the last message

    def listener_callback(self, msg):
        data = msg.data
        self.hist = list(zip(data[0::2], data[1::2]))
        self.stamp = self.get_clock().now()

    def read(self, max_age=0.5):
        """Last histogram as ``[(distance, angle), ...]``, or None if none yet or older than ``max_age`` s."""
        if self.hist is None: return None
        age = (self.get_clock().now() - self.stamp).nanoseconds / 1e9
        if age < 0 or age > max_age: return None
        return self.hist


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
