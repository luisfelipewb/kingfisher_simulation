# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""The Kingfisher's robot nodes in sim, the twin of kingfisher_bringup's robot.launch.py.

kingfisher_twist, kingfisher_viz and kingfisher_sail's sail_controller run
with the boat's config; cmd_drive_translate stands in for the MCU, and
sail_stepper_sim for the sail's stepper. wind_bridge and
anemometer_relay expose ApparentWind's sail/wrench and sensors/anemometer/wind.
ground_truth_tf stands in for localization. joy_teleop runs as on the boat, with
kingfisher_teleop's config. Start a world that spawns <name> first.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name = LaunchConfiguration('name')
    sim_config = os.path.join(get_package_share_directory('kingfisher_sim'), 'config')
    boat_config = os.path.join(
        get_package_share_directory('kingfisher_bringup'), 'config', 'kingfisher.yaml')
    teleop_config = os.path.join(
        get_package_share_directory('kingfisher_teleop'), 'config', 'teleop.yaml')
    sim_time = {'use_sim_time': True}
    with open(boat_config) as f:
        stepper = yaml.safe_load(f)['/**/phidgets_stepper']['ros__parameters']

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

    # The URDF frames, prefixed, as base.launch.py does on the boat
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

    # sail/cmd_position and sail/cmd_velocity -> phidgets_stepper/command, as on the boat
    sail_controller = Node(
        package='kingfisher_sail',
        executable='sail_controller',
        namespace=name,
        output='screen',
        parameters=[boat_config, sim_time])

    # The stepper: phidgets_stepper/command -> thrusters/sail/pos, sim-internal
    # like thrusters/*/thrust. Same acceleration as the boat's driver.
    sail_stepper = Node(
        package='kingfisher_sim',
        executable='sail_stepper_sim',
        namespace=name,
        output='screen',
        parameters=[sim_time, {'acceleration': float(stepper['acceleration'])}])

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

    # The gz ground truth as map -> <name>/base_link, until localization publishes map -> odom
    ground_truth_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_tf',
        namespace=name,
        output='screen',
        parameters=[sim_time],
        arguments=[['/model/', name, '/ground_truth/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']],
        remappings=[(['/model/', name, '/ground_truth/tf'], '/tf')],
        condition=IfCondition(LaunchConfiguration('ground_truth_tf')))

    # joy -> cmd_vel, cmd_drive, sail commands; the joystick node
    # (kingfisher_teleop joy.launch.py) runs separately
    teleop = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        namespace=name,
        output='screen',
        parameters=[teleop_config, sim_time],
        condition=IfCondition(LaunchConfiguration('teleop')))

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='kingfisher',
                              description='Model name and ROS namespace of the vehicle'),
        DeclareLaunchArgument('ground_truth_tf', default_value='True',
                              description='Publish the gz ground truth as map -> <name>/base_link'),
        DeclareLaunchArgument('teleop', default_value='True',
                              description='Start joy_teleop; the joystick node runs separately'),
        cmd_drive,
        twist,
        viz,
        sail_controller,
        sail_stepper,
        wind_bridge,
        anemometer_relay,
        ground_truth_tf,
        teleop,
    ])
