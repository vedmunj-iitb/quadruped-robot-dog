"""Starts Gazebo with the robot dog, its two controllers and the gait node.

    ros2 launch robot_dog_sim sim.launch.py              walk, with the Gazebo window
    ros2 launch robot_dog_sim sim.launch.py gait:=trot   trot, diagonal legs together
    ros2 launch robot_dog_sim sim.launch.py gui:=false   no window
The settings of each gait (cycle time, speed, step height) are in config/trot.yaml and walk.yaml.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_dog_sim')
    robot = xacro.process_file(os.path.join(pkg, 'urdf', 'robot_dog.urdf.xacro')).toxml()
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')

    # Gazebo with its window, or only the simulation server (-s) when gui is false
    gazebo = IncludeLaunchDescription(
        gz_launch, condition=IfCondition(gui),
        launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items())
    gazebo_server = IncludeLaunchDescription(
        gz_launch, condition=UnlessCondition(gui),
        launch_arguments={'gz_args': ['-r -s ', world], 'on_exit_shutdown': 'true'}.items())

    # publishes robot_description for the spawner, and the TF tree of the legs
    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                           parameters=[{'robot_description': robot, 'use_sim_time': True}])

    # the legs start straight: 67 + 75 mm of leg and an 8 mm ball reach 150 mm below the body
    # centre, so at z = 0.16 m the feet start 10 mm above the floor
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'robot_dog', '-z', '0.16'])

    # the Gazebo clock, the IMU and the body's true position, into ROS 2
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                             '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                             '/model/robot_dog/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                  remappings=[('/model/robot_dog/odometry', '/odom')])

    joint_states = Node(package='controller_manager', executable='spawner',
                        arguments=['joint_state_broadcaster'])
    legs = Node(package='controller_manager', executable='spawner', arguments=['leg_controller'])

    # gait:=walk loads config/walk.yaml
    settings = PathJoinSubstitution([pkg, 'config', [LaunchConfiguration('gait'), '.yaml']])
    gait = Node(package='robot_dog_sim', executable='gait_node',
                parameters=[settings, {'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('gait', default_value='walk', choices=['walk', 'trot']),
        DeclareLaunchArgument('gui', default_value='true', description='show the Gazebo window'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'floor.sdf')),
        gazebo, gazebo_server, state_publisher, spawn, bridge,
        # the controllers start once the dog is in Gazebo, the gait node once they run
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_states, legs])),
        RegisterEventHandler(OnProcessExit(target_action=legs, on_exit=[gait])),
    ])
