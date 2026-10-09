# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Launch a world with a Kingfisher, plus its robot nodes (robot.launch.py).

world:=lac_symphonie (the default) spawns from the lake's own spawn config, so
name must match the model there (kingfisher), and the wind_* args set its
wind. Any other VRX world spawns one Kingfisher called <name> and keeps the
wind in its SDF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration


def generate_launch_description():
    name = LaunchConfiguration('name')
    world = LaunchConfiguration('world')
    is_lake = EqualsSubstitution(world, 'lac_symphonie')
    gz_args = {
        'headless': LaunchConfiguration('headless'),
        'paused': LaunchConfiguration('paused'),
        'extra_gz_args': LaunchConfiguration('extra_gz_args'),
    }

    wind_args = {
        'wind_speed': LaunchConfiguration('wind_speed'),
        'wind_direction': LaunchConfiguration('wind_direction'),
        'wind_gust': LaunchConfiguration('wind_gust'),
    }

    # Each include sits in a GroupAction so its launch arguments stay scoped
    lake = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('lac_symphonie'), 'launch',
                'lac_symphonie.launch.py')),
            launch_arguments={**gz_args, **wind_args}.items()),
    ], condition=IfCondition(is_lake))

    vrx_world = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('vrx_gz'), 'launch', 'competition.launch.py')),
            launch_arguments={
                'model': 'usv',
                'urdf': os.path.join(get_package_share_directory('kingfisher_gazebo'),
                                     'urdf', 'kingfisher_gazebo.urdf.xacro'),
                'name': name,
                'world': world,
                **gz_args,
            }.items()),
    ], condition=UnlessCondition(is_lake))

    robot = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('kingfisher_sim'), 'launch', 'robot.launch.py')),
            launch_arguments={
                'name': name,
                'ground_truth_tf': LaunchConfiguration('ground_truth_tf'),
                'teleop': LaunchConfiguration('teleop'),
            }.items()),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='kingfisher',
                              description='Model name and ROS namespace of the vehicle; '
                                          'with the lake, the model name in its spawn config'),
        DeclareLaunchArgument('world', default_value='lac_symphonie',
                              description='lac_symphonie or the name of a VRX world'),
        DeclareLaunchArgument('headless', default_value='False',
                              description='True to run gz sim without the GUI'),
        DeclareLaunchArgument('paused', default_value='False',
                              description='True to start the simulation paused'),
        DeclareLaunchArgument('extra_gz_args', default_value='',
                              description='Additional arguments passed to gz sim'),
        DeclareLaunchArgument('ground_truth_tf', default_value='True',
                              description='Publish the gz ground truth as map -> <name>/base_link'),
        DeclareLaunchArgument('teleop', default_value='True',
                              description='Start joy_teleop; the joystick node runs separately'),
        DeclareLaunchArgument('wind_speed', default_value='0.0',
                              description='Lake only: mean wind speed, m/s'),
        DeclareLaunchArgument('wind_direction', default_value='240',
                              description='Lake only: where the wind goes, '
                                          'degrees counter-clockwise from east'),
        DeclareLaunchArgument('wind_gust', default_value='0.0',
                              description='Lake only: gust gain, about the standard '
                                          'deviation of the speed, m/s'),
        lake,
        vrx_world,
        robot,
    ])
