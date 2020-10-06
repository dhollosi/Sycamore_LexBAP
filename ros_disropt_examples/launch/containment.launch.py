from launch import LaunchDescription
from launch_ros.actions import Node
from disropt.utils.graph_constructor import path_graph
import numpy as np
import sys
import argparse
import os

def generate_launch_description():
    ap = argparse.ArgumentParser(prog='ros2 launch ros_disropt planner.launch.py')
    ap.add_argument("-n", "--nodes", help="number of nodes", default=3, type=int)

    try:
        args = vars(ap.parse_args(sys.argv[4:])) # skip "ros2 launch ros_disropt main.launch.py"
    except:
        return None
    
    #######################
    N = args['nodes']
    Adj = np.array([[0, 1, 0, 1, 0, 1],
                    [1, 0, 1, 0, 1, 0],
                    [0, 1, 0, 1, 0, 1],
                    [1, 0, 1, 0, 1, 0],
                    [0, 1, 0, 1, 0, 1],
                    [1, 0, 1, 0, 1, 0]])
    
    P = np.array([
        [2, 2, 0],
        [2, 0, 0],     
        [0, 2, 0],
        [0, 3.5, 0],
        [3.5, 0, 0],
        [-.5, -2 ,0]])

    #######################

    list_description = []

    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        if i<3:
            is_leader = True
        else:
            is_leader = False
        
        initial_pos = P[i, :].tolist()

        list_description.append(Node(
            package='ros_disropt_examples', node_executable='ros_disropt_singleintegrator', output='screen',
            node_namespace='agent_{}'.format(i),
            prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i, 'init_pos': initial_pos}]))

        list_description.append(Node(
            package='ros_disropt_examples', node_executable='ros_disropt_containment', output='screen',
            node_namespace='agent_{}'.format(i),
            #prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors, 'is_leader': is_leader}]))

        list_description.append(Node(
            package='ros_disropt_examples', node_executable='ros_disropt_rviz', output='screen',
            node_namespace='agent_{}'.format(i),
            # prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i}]))

    return LaunchDescription(list_description)