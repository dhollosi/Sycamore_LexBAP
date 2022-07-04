import os
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Set the path to this package.
    # pkg_share = FindPackageShare(package='sycamore').find('resource')
    # world_file_name = 'road.world'
    # world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # launch gazebo
    list_description = [ExecuteProcess(cmd=['gazebo', '-s', 'libgazebo_ros_factory.so'], output='screen')]

    list_description.append(DeclareLaunchArgument(
        'world',
        default_value=[os.path.join('/home/dimitri/ChoirBot/install/sycamore/share/sycamore', 'road.world'), ''],
        description='SDF world file'))

    # set use_sim_time = true after 5 seconds
    action = ExecuteProcess(cmd=['ros2', 'param', 'set', '/gazebo', 'headless', 'true','use_sim_time', 'true'], output='screen')
    list_description.append(TimerAction(period=5.0, actions=[action]))

    # pkg_road_world = get_package_share_directory('resource')



    return LaunchDescription(list_description)
