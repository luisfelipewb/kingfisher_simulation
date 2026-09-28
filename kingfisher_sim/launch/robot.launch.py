# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""The Kingfisher's robot nodes in sim, the twin of kingfisher_bringup's robot.launch.py.

kingfisher_twist and kingfisher_viz run with the boat's config;
cmd_drive_translate stands in for the MCU. The sail takes sail/cmd_angle
through sail_relay, and sail/cmd_rate through sail_jog. wind_bridge and
anemometer_relay expose ApparentWind's sail/wrench and sensors/anemometer/wind.
Start a world that spawns <name> first.
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

    # The URDF frames, prefixed; the boat uses static_tfs' until it runs robot_state_publisher
    viz = Node(
        package='kingfisher_viz',
        executable='kingfisher_viz_node',
        name='kingfisher_viz',
        namespace=name,
        output='screen',
        parameters=[boat_config, sim_time, {
            'base_frame': [name, '/base_link'],
            'left_thruster_frame': [name, '/left_thruster_link'],
            'right_thruster_frame': [name, '/right_thruster_link'],
        }])

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

    # ApparentWind's gz topics are absolute (/<name>/...); vrx_gz only bridges
    # the plugins it knows
    wind_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wind_bridge',
        namespace=name,
        output='screen',
        parameters=[sim_time],
        arguments=[
            ['/', name, '/sail/wrench@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'],
            ['/', name, '/sensors/anemometer/twist@geometry_msgs/msg/TwistStamped[gz.msgs.Twist'],
        ])

    # ros_gz_bridge has no Vector3Stamped; --wait-for-start since the bridge may not be up yet
    anemometer_relay = Node(
        package='topic_tools',
        executable='relay_field',
        name='anemometer_relay',
        namespace=name,
        output='screen',
        parameters=[sim_time],
        arguments=['sensors/anemometer/twist', 'sensors/anemometer/wind',
                   'geometry_msgs/msg/Vector3Stamped',
                   '{header: m.header, vector: m.twist.linear}', '--wait-for-start'])

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='kingfisher',
                              description='Model name and ROS namespace of the vehicle'),
        cmd_drive,
        twist,
        viz,
        sail_relay,
        sail_jog,
        wind_bridge,
        anemometer_relay,
    ])
