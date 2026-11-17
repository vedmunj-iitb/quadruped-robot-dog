"""The robot dog walks a figure-8 round two posts: sim.launch.py with the course world, plus
the path node that steers it.

    ros2 launch robot_dog_sim path.launch.py              walk
    ros2 launch robot_dog_sim path.launch.py gait:=trot   trot
The waypoints are in config/path.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_dog_sim')

    sim = IncludeLaunchDescription(
        os.path.join(pkg, 'launch', 'sim.launch.py'),
        launch_arguments={'gait': LaunchConfiguration('gait'), 'gui': LaunchConfiguration('gui'),
                          'world': os.path.join(pkg, 'worlds', 'course.sdf')}.items())

    path = Node(package='robot_dog_sim', executable='path_node',
                parameters=[os.path.join(pkg, 'config', 'path.yaml'), {'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('gait', default_value='walk', choices=['walk', 'trot']),
        DeclareLaunchArgument('gui', default_value='true', description='show the Gazebo window'),
        sim, path,
    ])
