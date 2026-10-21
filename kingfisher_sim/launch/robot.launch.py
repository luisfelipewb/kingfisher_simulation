# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""The Kingfisher's robot nodes in sim, the twin of kingfisher_bringup's robot.launch.py.

kingfisher_twist, drive_mux, kingfisher_viz and kingfisher_sail's sail_controller
run with the boat's config; cmd_drive_translate stands in for the MCU, and
sail_stepper_sim for the sail's stepper. wind_bridge and
anemometer_relay expose ApparentWind's sail/wrench and sensors/anemometer/wind.
The gz ground truth is the odom (the boat's RTK INS is close to it), with map -> odom
and the site frames as kingfisher_localization does on the boat. joy_teleop runs as on
the boat, with kingfisher_teleop's config. Start a world that spawns <name> first.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    name = LaunchConfiguration('name')
    sim_config = os.path.join(get_package_share_directory('kingfisher_sim'), 'config')
    boat_config = os.path.join(
        get_package_share_directory('kingfisher_bringup'), 'config', 'kingfisher.yaml')
    teleop_config = os.path.join(
        get_package_share_directory('kingfisher_teleop'), 'config', 'teleop.yaml')
    site = LaunchConfiguration('site')
    site_file = PathJoinSubstitution([get_package_share_directory('kingfisher_localization'),
                                      'config', 'sites', [site, '.yaml']])
    localization = IfCondition(LaunchConfiguration('localization'))
    has_site = IfCondition(PythonExpression([
        "'", LaunchConfiguration('localization'), "'.lower() in ('true', '1')",
        " and '", site, "' != ''"]))
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
        parameters=[boat_config, sim_time],
        remappings=[('cmd_drive', 'teleop/cmd_drive')])

    # teleop/cmd_drive (default) or control/cmd_drive -> cmd_drive, as on the boat
    drive_mux = Node(
        package='topic_tools',
        executable='mux',
        name='drive_mux',
        namespace=name,
        output='screen',
        parameters=[sim_time],
        arguments=['cmd_drive', 'teleop/cmd_drive', 'control/cmd_drive'])

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

    # Localization: the gz ground truth (in odom) as odom and odom -> <name>/base_link
    ground_truth_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_tf',
        namespace=name,
        output='screen',
        parameters=[sim_time],
        arguments=[['/model/', name, '/ground_truth/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']],
        remappings=[(['/model/', name, '/ground_truth/tf'], '/tf')],
        condition=localization)

    odom = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        namespace=name,
        output='screen',
        parameters=[sim_time],
        arguments=['sensors/position/ground_truth_odometry', 'odom'],
        condition=localization)

    # map = odom, as on the boat
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        parameters=[sim_time],
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        condition=localization)

    # map -> <site point>, as on the boat; none when site is empty
    geo_tf = Node(
        package='kingfisher_localization',
        executable='geo_tf_publisher',
        namespace=name,
        output='screen',
        parameters=[sim_time, {'site_file': site_file}],
        condition=has_site)

    # joy -> cmd_vel, teleop/cmd_drive, drive_mux, sail commands; the joystick node
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
        DeclareLaunchArgument('localization', default_value='True',
                              description='Publish odom, map -> odom -> <name>/base_link '
                                          'and the site frames'),
        DeclareLaunchArgument('site', default_value='lac_symphonie',
                              description="kingfisher_localization's config/sites/<site>.yaml; "
                                          'empty for no site frames'),
        DeclareLaunchArgument('teleop', default_value='True',
                              description='Start joy_teleop; the joystick node runs separately'),
        cmd_drive,
        twist,
        drive_mux,
        viz,
        sail_controller,
        sail_stepper,
        wind_bridge,
        anemometer_relay,
        ground_truth_tf,
        odom,
        map_to_odom,
        geo_tf,
        teleop,
    ])
