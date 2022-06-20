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



def generate_launch_description():
    ap = argparse.ArgumentParser(prog='ros2 launch sycamore lexBAP.launch.py')
    ap.add_argument("-n", "--number", help="number of robots", default=6, type=int)
    ap.add_argument("-s", "--seed", help="seed for initial positions", default=3, type=int)

    # parse arguments (exception thrown on error)
    args, _ = ap.parse_known_args(sys.argv)
    N = args.number

    # generate communication graph (this function also sets the seed)
    Adj = binomial_random_graph(N, 0.2, seed=args.seed)

    # generate initial positions in [-3, 3] with z = 0

    ''' ############ SET INITIAL PARAMETERS UNDER ################ '''

    task_dim = 6
    agent_dim = 6
    rob_margin = 0.3 #set here minimum desired bottleneck margin (mu_k)

    RobustSetup = True
    lower_bound = -3
    upper_bound = 3

    use_data = True

    ''' ############ SET INITIAL PARAMETERS ABOVE ################ '''
    __location__ = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))

    agent_data = 'previous_agent.pk'
    task_data = 'previous_task.pk'

    Problem = AssignmentProblem(agent_dim, task_dim, rob_margin, lower_bound = lower_bound,
                                upper_bound = upper_bound, isRobust = RobustSetup)


    with open(os.path.join(__location__,agent_data), 'rb') as fi:
        Problem.agent_pos_conv  = pickle.load(fi)
        # print(Problem.agent_pos_conv)
    with open(os.path.join(__location__,task_data), 'rb') as fi:
        Problem.task_pos_conv  = pickle.load(fi)
        # print(Problem.task_pos_conv)



    LexBAP = LexicoBAP(Problem)
    LexBAP.optimise()

    while LexBAP.robustsetup is not False:

        if use_data is True:
            LexBAP = LexicoBAP(Problem)

            LexBAP.run_coll_avoidance(Problem)
        else:

            Problem = AssignmentProblem(agent_dim, task_dim, rob_margin, lower_bound=lower_bound,
                                        upper_bound=upper_bound, isRobust=RobustSetup)
            LexBAP = LexicoBAP(Problem)

            LexBAP.run_coll_avoidance(Problem)


    print('\n \n Optimal Task Assignment is: \n', LexBAP.optimal_ass, '\n \n muk is: \n', LexBAP.mu_k)

    # LexBAP.plot_solution()
    # LexBAP.graph_vis(clean = True)


    #store data
    with open(os.path.join(__location__,agent_data), 'wb') as fi:
        pickle.dump(LexBAP.agent_pos, fi)
    with open(os.path.join(__location__,task_data), 'wb') as fi:
        pickle.dump(LexBAP.task_pos, fi)

    # P = np.zeros((N, 3))
    # P[:, 0:2] = np.random.randint(0, 6, (N, 2))


    # T = np.zeros((N, 3))
    # T[:, 0:2] = np.random.randint(0, 6, (N, 2))
    # T = Problem.task_pos_cb

    Problem.make_choirbot_compatible()
    P = Problem.agent_pos_cb
    T = Problem.task_pos_cb

    task_sequence = []
    agent_sequence = []
    for i in range(0, agent_dim):
        agent_idx, task_idx = np.where(LexBAP.optimal_ass == i + 1)
        task_sequence.append(int(task_idx))
        agent_sequence.append((int(agent_idx)))

    print('task sequence {}'.format(task_sequence))
    print('agent sequence {}'.format(agent_sequence))


    T_new = T[task_sequence, :]
    P_new = T[:, agent_sequence]


    # initialize launch description
    robot_launch = []  # launched after 10 sec (to let Gazebo open)
    launch_description = []  # launched immediately (will contain robot_launch)

    rviz_config_dir = get_package_share_directory('sycamore')
    rviz_config_file = os.path.join(rviz_config_dir, 'rvizconf.rviz')

    launch_description = [Node(package='rviz2', node_executable='rviz2', output='screen',
                               arguments=['-d', rviz_config_file])]

    robot_launch.append(Node(
        package='sycamore', node_executable='choirbot_lexBAP_analysis', output='screen',
        parameters=[{'N': N}]))

    # add task table executable
    robot_launch.append(Node(
        package='sycamore', node_executable='choirbot_lexBAP_table', output='screen',
        prefix=['xterm -hold -e'],
        parameters=[{'N': N}]))

    # add executables for each robot
    for i in range(N):
        in_neighbors = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        position = P_new[i, :].tolist()
        task_positions = T_new[i, :].tolist()
        # guidance
        robot_launch.append(Node(
            package='sycamore', node_executable='choirbot_lexBAP_guidance', output='screen',
            prefix=['xterm -hold -e'],
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors,
                        'out_neigh': out_neighbors}]))

        # planner
        robot_launch.append(Node(
            package='sycamore', node_executable='choirbot_lexBAP_planner', output='screen',
            node_namespace='agent_{}'.format(i),
            # parameters=[{'agent_id': i}]))
            parameters=[{'agent_id': i, 'task_position': task_positions}]))

        # controller
        robot_launch.append(Node(
            package='sycamore', node_executable='choirbot_lexBAP_controller', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))

        # turtlebot spawner
        launch_description.append(Node(
            package='sycamore', node_executable='choirbot_turtlebot_spawner', output='screen',
            parameters=[{'namespace': 'agent_{}'.format(i), 'position': position}]))

        launch_description.append(Node(
            package='sycamore', node_executable='choirbot_lexBAP_rviz', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))

        launch_description.append(Node(
            package='sycamore', node_executable='choirbot_lexBAP_rviz_task', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))

    # include launcher for gazebo
    gazebo_launcher = os.path.join(get_package_share_directory('sycamore'), 'gazebo.launch.py')
    launch_description.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launcher)))

    # include delayed robot executables
    timer_action = TimerAction(period=10.0, actions=[LaunchDescription(robot_launch)])
    launch_description.append(timer_action)

    return LaunchDescription(launch_description)
