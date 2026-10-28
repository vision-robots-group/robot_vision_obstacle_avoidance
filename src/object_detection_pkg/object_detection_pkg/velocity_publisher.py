import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class VelocityPublisher(Node):

    def __init__(self, linear_velocity=Vector3(0.0, 0.0, 0.0), angular_velocity=Vector3(0.0, 0.0, 0.0)):
        super().__init__('velocity_publisher')
        self.i = 0
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

    def timer_callback(self):
        msg = Twist()
        msg.linear = self.linear_velocity
        msg.angular = self.angular_velocity
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg} velocity')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    velocity_pub = VelocityPublisher()

    rclpy.spin(velocity_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()