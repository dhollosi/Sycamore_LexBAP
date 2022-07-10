import rclpy
from rclpy.node import Node
from choirbot import Pose
from sycamore.LexBAP_to_ChoiRbot import ChoirBotLexicoBAP
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker
import os
import pickle
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt


class Analyser(Node):

    def __init__(self, visualization_topic: str = '/visualization_marker', update_frequency: int= 1,
            pose_handler: str ='pubsub', pose_topic: str=None):
        super().__init__('LexBAP_Analyser', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)

        self.agent_dim = self.get_parameter('N').value
        self.task_dim = self.agent_dim
        self.current_agent_pose_dict = {}
        self.last_all_agents_pose_dict = {}
        self.counter = 100000
        self.counter_robust = 0

        reassigned_task_data = 'reassigned_tasks_lex_format.pk'
        task_table_dir = get_package_share_directory('sycamore')
        task_table_file = os.path.join(task_table_dir, reassigned_task_data)

        with open(task_table_file, 'rb') as fi:
            assigned_tasks_lexy = pickle.load(fi)

        save_dir = self.init_store_directory()

        self.Lexy = ChoirBotLexicoBAP(assigned_tasks_lexy, save_dir, agent_dim=self.agent_dim)

        self.Lexy.task_pos = assigned_tasks_lexy

        self.current_agent_id = None

        self.current_pose = Pose(None, None)

        qos = QoSProfile(depth=10)

        self.viz_sub = self.create_subscription(
            Marker,
            visualization_topic,
            self.robustness_analysis,
            qos)


    def init_store_directory(self):

        path = os.getcwd()

        path_parent = os.path.abspath(os.path.join(path))


        directory = 'LexicoBAP_results'

        if not os.path.exists(os.path.join(path_parent, directory)):
            os.mkdir(os.path.join(path_parent, directory))

        save_dir = os.path.join(path_parent + '/' + directory)

        self.get_logger().info('\n\n *** INITIALISING SAVE DIRECTORY TO : \n {}'.format(save_dir))


        if os.path.exists(save_dir + '/Robustness') is False:
            # If there are no sub directories, create them (stores the different plots)
            robustness = '/Robustness'
            history = '/Assignments'
            safe_set = '/Safe_set'
            safe_set_history = '/Safe_sets_history'
            graph_view = '/Graph'
            os.mkdir(os.path.join(save_dir + robustness))
            os.mkdir(os.path.join(save_dir + history))
            os.mkdir(os.path.join(save_dir + safe_set))
            os.mkdir(os.path.join(save_dir + safe_set_history))
            os.mkdir(os.path.join(save_dir + graph_view))

        return save_dir


    def robustness_analysis(self, msg):

        self.viz_callback(msg)

        if len(self.last_all_agents_pose_dict) == self.agent_dim: #wait until odometry from all agents has been received
            self.get_logger().info('\n\n **** DEBUG **** Counter is \n {}'.format(self.counter))

            self.Lexy.get_agent_pos_from_dict(self.last_all_agents_pose_dict)

            self.Lexy.optimise()
            x_match, y_match = self.Lexy.get_agent_to_task_match()

            if self.counter_robust == 0:
                self.Lexy.init_cost = self.Lexy.cost
                self.Lexy.init_agent_pos = self.Lexy.agent_pos
                self.Lexy.init_task_pos = self.Lexy.task_pos

            # Update variables for plots
            self.Lexy.mu_k_array.append(self.Lexy.mu_k)
            self.Lexy.get_ak()
            self.Lexy.x_agent_array.append(x_match[0])
            self.Lexy.y_agent_array.append(y_match[0])

            # Generate plots
            self.Lexy.plot_solution(self.counter)
            self.Lexy.plot_robustness_margins(self.counter_robust)
            self.Lexy.plot_solution_history(self.counter, self.counter_robust)
            self.Lexy.plot_solution_robust(self.counter, self.counter_robust)
            self.Lexy.graph_vis(self.counter, clean = True)


            # Re-initialise agent positions
            self.last_all_agents_pose_dict = {}

            self.get_logger().info('\n\n **** DEBUG **** Mu_k is \n {}'.format(self.Lexy.mu_k))
            self.get_logger().info('\n\n **** DEBUG **** optimass is \n {}'.format(self.Lexy.optimal_ass))
            # self.get_logger().info('\n\n **** DEBUG **** Akk is \n {}'.format(self.Lexy.a_k))

            self.counter += 1
            self.counter_robust += 1


    def viz_callback(self, msg):
        # Print terminal message and get inputs
        if msg.ns == 'Agents':
            current_agent_id = msg.id

            pose_x = msg.pose.position.x
            pose_y = msg.pose.position.y

            current_agent_data = {'agent_{}'.format(current_agent_id) : {'pose_x' : [pose_x], 'pose_y' : [pose_y]}}
            self.current_agent_pose_dict.update(current_agent_data)

            # Only update agent dictionary once all agent positions have been received
            if len(self.current_agent_pose_dict) == self.agent_dim:
                self.last_all_agents_pose_dict = self.current_agent_pose_dict
                self.current_agent_pose_dict = {}

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Analyser()


    plt.show()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()
#
