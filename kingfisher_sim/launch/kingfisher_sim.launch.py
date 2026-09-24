# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Launch the VRX simulation with a Kingfisher and the cmd_drive translator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name = LaunchConfiguration('name')
    sim_pkg = get_package_share_directory('kingfisher_sim')

    vrx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vrx_gz'), 'launch', 'competition.launch.py')),
        launch_arguments={
            'model': 'usv',
            'urdf': os.path.join(get_package_share_directory('kingfisher_gazebo'),
                                 'urdf', 'kingfisher_gazebo.urdf.xacro'),
            'name': name,
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'paused': LaunchConfiguration('paused'),
            'extra_gz_args': LaunchConfiguration('extra_gz_args'),
        }.items())

    cmd_drive = Node(
        package='kingfisher_sim',
        executable='cmd_drive_translate',
        namespace=name,
        output='screen',
        parameters=[os.path.join(sim_pkg, 'config', 'cmd_drive_translate.yaml'),
                    {'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='kingfisher',
                              description='Model name and ROS namespace of the vehicle'),
        DeclareLaunchArgument('world', default_value='sydney_regatta',
                              description='Name of the VRX world to load'),
        DeclareLaunchArgument('headless', default_value='False',
                              description='True to run gz sim without the GUI'),
        DeclareLaunchArgument('paused', default_value='False',
                              description='True to start the simulation paused'),
        DeclareLaunchArgument('extra_gz_args', default_value='',
                              description='Additional arguments passed to gz sim'),
        vrx_launch,
        cmd_drive,
    ])
