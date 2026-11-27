"""
display_robot.launch.py
Starts Gazebo (gz sim) and spawns my_robot from my_robot.urdf.xacro.

Package: my_robot_description  (ROS 2 Jazzy)
Run:     ros2 launch my_robot_description display_robot.launch.py

After launching, start the clock bridge and spawn the controllers
(joint_state_broadcaster, joint_position_controller) in other terminals.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')

    # Turn the xacro file into a plain URDF string
    robot_description = xacro.process_file(xacro_file).toxml()

    # 1) Start Gazebo with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2) Publish the robot model on /robot_description and the TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # 3) Spawn the robot into Gazebo from /robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot'],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
