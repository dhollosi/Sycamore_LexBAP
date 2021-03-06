from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from disropt.utils.graph_constructor import binomial_random_graph
import numpy as np
import sys
import argparse
import os
from sycamore.AssignmentProblem import AssignmentProblem
from sycamore.LexicoBAP import LexicoBAP
import pickle

'''###############################################################
############### SET INITIAL PARAMETERS UNDER #####################
 #################################################################'''
# Number of Tasks, Agents and safety distance, Can Copy Paste from ap_generator.py
task_dim = 6
agent_dim = 6

# Set to RobustSetup True if robustness margin below is to set the be the minimum mu_k
rob_margin = 0
RobustSetup = True

lower_bound = -3
upper_bound = 3

true_heading = True
LexBAP_analysis = True # set to True to perform LexBAP robustness analysis. Creates LexBAP_Analyser node which outputs graphs

'''###############################################################
############### SET INITIAL PARAMETERS ABOVE #####################
 #################################################################'''

use_LSAP = False
use_lexBAP = True

__location__ = os.path.realpath(
                os.path.join(os.getcwd(), os.path.dirname(__file__)))

reassigned_agent_data = 'reassigned_agent_lex_format.pk'
reassigned_task_data = 'reassigned_tasks_lex_format.pk'


def generate_launch_description():
    ap = argparse.ArgumentParser(prog='ros2 launch sycamore lexBAP.launch.py')
    ap.add_argument("-n", "--number", help="number of robots", default = agent_dim, type=int)
    ap.add_argument("-s", "--seed", help="seed for initial positions", default=3, type=int)

    # parse arguments (exception thrown on error)
    args, _ = ap.parse_known_args(sys.argv)
    N = args.number

    # generate communication graph (this function also sets the seed)
    Adj = binomial_random_graph(N, 0.2, seed=args.seed)

    Problem = AssignmentProblem(agent_dim, task_dim, rob_margin, lower_bound=lower_bound,
                                upper_bound=upper_bound, isRobust=RobustSetup)


    with open(os.path.join(__location__,reassigned_agent_data), 'rb') as fi:
        Problem.agent_pos_lex = pickle.load(fi)
    with open(os.path.join(__location__,reassigned_task_data), 'rb') as fi:
        Problem.task_pos_lex = pickle.load(fi)
    Problem.make_choirbot_compatible()


    LexBAP = LexicoBAP(Problem)
    LexBAP.optimise()

    print('\n \n Optimal Task Assignment is: \n', LexBAP.optimal_ass, '\n \n muk is: \n', LexBAP.mu_k)

    LexBAP.plot_solution()

    LexBAP.order_assignment()

    T_new = LexBAP.task_pos_ordered_cb
    P_new = LexBAP.agent_pos_ordered_cb


    # initialize launch description
    robot_launch = []  # launched after 10 sec (to let Gazebo open)

    rviz_config_dir = get_package_share_directory('sycamore')
    rviz_config_file = os.path.join(rviz_config_dir, 'rvizconf.rviz')

    launch_description = [Node(package='rviz2', node_executable='rviz2', output='screen',
                               arguments=['-d', rviz_config_file])]

    if LexBAP_analysis is True:
        robot_launch.append(Node(
            package='sycamore', node_executable='sycamore_lexBAP_analysis', output='screen',
            parameters=[{'N': N}]))

    robot_launch.append(Node(
        package='sycamore', node_executable='sycamore_lexBAP_table', output='screen',
        prefix=['xterm -hold -e'],
        parameters=[{'N': N, 'use_lexBAP': use_lexBAP, 'use_LSAP': use_LSAP}]))

    # add executables for each robot
    for i in range(N):
        in_neighbors = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        position = P_new[i, :].tolist()
        task_positions = T_new[i, :].tolist()

        # guidance
        robot_launch.append(Node(
            package='sycamore', node_executable='sycamore_lexBAP_guidance', output='screen',
            prefix=['xterm -hold -e'],
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors,
                        'out_neigh': out_neighbors}]))

        # planner
        robot_launch.append(Node(
            package='sycamore', node_executable='sycamore_lexBAP_planner', output='screen',
            node_namespace='agent_{}'.format(i),
            # parameters=[{'agent_id': i}]))
            parameters=[{'agent_id': i, 'task_position': task_positions}]))

        # controller
        robot_launch.append(Node(
            package='sycamore', node_executable='sycamore_lexBAP_controller', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))

        # turtlebot spawner
        launch_description.append(Node(
            package='sycamore', node_executable='sycamore_turtlebot_spawner', output='screen',
            parameters=[{'namespace': 'agent_{}'.format(i), 'position': position,
                         'task_position': task_positions, 'true_heading': true_heading,}]))

        launch_description.append(Node(
            package='sycamore', node_executable='sycamore_lexBAP_rviz', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))

        launch_description.append(Node(
            package='sycamore', node_executable='sycamore_lexBAP_rviz_task', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))

    # include launcher for gazebo
    gazebo_launcher = os.path.join(get_package_share_directory('sycamore'), 'gazebo.launch.py')
    launch_description.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launcher)))

    # include delayed robot executables
    timer_action = TimerAction(period=10.0, actions=[LaunchDescription(robot_launch)])
    launch_description.append(timer_action)

    return LaunchDescription(launch_description)
