import rclpy
from choirbot.utils.visualizer_goal import Visualizer_Task

def main():
    rclpy.init()

    visualizer_task = Visualizer_Task()

    rclpy.spin(visualizer_task)


    rclpy.shutdown()
