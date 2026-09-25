# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""The Kingfisher's robot nodes in sim, the twin of kingfisher_bringup's robot.launch.py.

kingfisher_twist and kingfisher_viz run with the boat's config;
cmd_drive_translate stands in for the MCU. The sail takes sail/cmd_angle
through sail_relay, and sail/cmd_rate through sail_jog. Start a world that
spawns <name> first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name = LaunchConfiguration('name')
    sim_config = os.path.join(get_package_share_directory('kingfisher_sim'), 'config')
    boat_config = os.path.join(
        get_package_share_directory('kingfisher_bringup'), 'config', 'kingfisher.yaml')
    sim_time = {'use_sim_time': True}

    cmd_drive = Node(
        package='kingfisher_sim',
        executable='cmd_drive_translate',
        namespace=name,
        output='screen',
        parameters=[os.path.join(sim_config, 'cmd_drive_translate.yaml'), sim_time])

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

    # thrusters/sail/pos is sim-internal, like thrusters/*/thrust. The relay
    # takes its type from the first publisher it sees.
    sail_relay = Node(
        package='topic_tools',
        executable='relay',
        name='sail_relay',
        namespace=name,
        output='screen',
        parameters=[sim_time, {'input_topic': 'sail/cmd_angle',
                               'output_topic': 'thrusters/sail/pos'}])

    # sail/cmd_rate -> sail/cmd_angle; may be native on the boat's sail controller
    sail_jog = Node(
        package='kingfisher_sim',
        executable='sail_jog',
        namespace=name,
        output='screen',
        parameters=[os.path.join(sim_config, 'sail_jog.yaml'), sim_time])

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='kingfisher',
                              description='Model name and ROS namespace of the vehicle'),
        cmd_drive,
        twist,
        viz,
        sail_relay,
        sail_jog,
    ])
