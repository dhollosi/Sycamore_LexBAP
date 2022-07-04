from rclpy.node import Node
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point
import numpy as np

class Visualizer_Task(Node):

    def __init__(self, visualization_topic: str = '/visualization_marker', update_frequency: int = 25,
                 pose_handler: str = None, pose_topic: str = None):
        super().__init__('task_visualizer', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.agent_id = self.get_parameter('agent_id').value
        self.publisher_ = self.create_publisher(Marker, visualization_topic, 1)

        self.goal_pose_x = None
        self.goal_pose_y = None
        # self.goal_pose_z = None
        self.goal_pose_theta = None

        self.timer_task = self.create_timer(1.0/update_frequency, self.publish_goal_msg)

        qos = QoSProfile(depth=10)

        self.goal_sub = self.create_subscription(
            Point,
            '/agent_{}/goal'.format(self.agent_id),
            self.goal_callback, qos)

    def goal_callback(self, msg):
        # Print terminal message and get inputs
        goal = np.array([msg.x, msg.y])
        self.get_logger().info('Goal callback: {}'.format(goal))
        self.goal_pose_x = msg.x
        self.goal_pose_y = msg.y
        # self.goal_pose_z = msg.z
        self.goal_pose_theta = 0.0


    def publish_goal_msg(self):

        if self.goal_pose_x is not None:

            goal_msg = Marker()

            goal_msg.pose.position.x = self.goal_pose_x
            goal_msg.pose.position.y = self.goal_pose_y
            goal_msg.pose.position.z = 0.0

            goal_msg.pose.orientation.x = 0.0
            goal_msg.pose.orientation.y = 0.0
            goal_msg.pose.orientation.z = 0.0
            goal_msg.pose.orientation.w = 1.0

            goal_msg.ns = 'Tasks'
            goal_msg.id = self.agent_id

            goal_msg.header.frame_id = 'my_frame'

            goal_msg.type = Marker.SPHERE

            goal_msg.scale.x = 0.2
            goal_msg.scale.y = 0.2
            goal_msg.scale.z = 0.2

            goal_msg.color.r = 0.0
            goal_msg.color.g = 1.0
            goal_msg.color.b = 0.0
            goal_msg.color.a = 1.0

            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.action = Marker.ADD

            self.publisher_.publish(goal_msg)