"""
Main file spinning ROS nodes for obstacle avoidance.
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .controller import Agent
from .histogram_subscriber import ObjectDetectionSubscriber


def main(args=None):
    rclpy.init(args=args)

    hist_sub = ObjectDetectionSubscriber()
    agent = Agent(hist_sub)

    executor = SingleThreadedExecutor()
    for node in (agent, hist_sub):
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        for node in (agent, hist_sub):
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()