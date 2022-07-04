import rclpy
from choirbot.utils.visualizer_agent import Visualizer_Agent


def main():

    rclpy.init()
    visualizer_agent = Visualizer_Agent(pose_handler='pubsub', pose_topic='odom')

    rclpy.spin(visualizer_agent)

    rclpy.shutdown()
