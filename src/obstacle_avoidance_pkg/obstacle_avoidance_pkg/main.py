"""
Main file spinning ROS nodes for obstacle avoidance.
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .controller import Agent
from .histogram_subscriber import ObjectDetectionSubscriber


def main(args=None):
    # no rclpy signal handler, so the context is still valid to stop the rover on Ctrl-C
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

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
        agent.get_logger().warn(f'Shutting down in state {agent.state.name}')
        agent.publish_cmd(0.0, 0.0)
        executor.shutdown()
        for node in (agent, hist_sub):
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()