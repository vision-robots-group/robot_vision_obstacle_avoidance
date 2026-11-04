"""
Main file spinning ROS node for obstacle avoidance.
"""

import rclpy

from .controller import Agent

def main(args=None):
    rclpy.init(args=args)
    agent = Agent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()