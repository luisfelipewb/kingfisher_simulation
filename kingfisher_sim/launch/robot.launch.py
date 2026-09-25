# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""The Kingfisher's robot nodes in sim, the twin of kingfisher_bringup's robot.launch.py.

kingfisher_twist and kingfisher_viz run with the boat's config;
cmd_drive_translate stands in for the MCU. Start a world that spawns <name> first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name = LaunchConfiguration('name')
    sim_config = os.path.join(
        get_package_share_directory('kingfisher_sim'), 'config', 'cmd_drive_translate.yaml')
    boat_config = os.path.join(
        get_package_share_directory('kingfisher_bringup'), 'config', 'kingfisher.yaml')
    sim_time = {'use_sim_time': True}

    cmd_drive = Node(
        package='kingfisher_sim',
        executable='cmd_drive_translate',
        namespace=name,
        output='screen',
        parameters=[sim_config, sim_time])

    twist = Node(
        package='kingfisher_twist',
        executable='kingfisher_twist_node',
        name='kingfisher_twist',
        namespace=name,
        output='screen',
        parameters=[boat_config, sim_time])

    # base_frame: the boat's frames are unprefixed until it runs robot_state_publisher
    viz = Node(
        package='kingfisher_viz',
        executable='kingfisher_viz_node',
        name='kingfisher_viz',
        namespace=name,
        output='screen',
        parameters=[boat_config, sim_time, {'base_frame': [name, '/base_link']}])

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='kingfisher',
                              description='Model name and ROS namespace of the vehicle'),
        cmd_drive,
        twist,
        viz,
    ])
