"""
This class will subscribing to the topic ``/object_detect``.
The Agent class (``controller.py``) will use this class
to get the distances and angles of the detected objects in order to avoid them.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

PI = math.pi

class ObjectDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/obj_detect',
            self.listener_callback,
            10)
        self.subscription
        self.boolean_polar_hist = None

    def listener_callback(self, msg):
        self.scan = msg