#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '3.1415926', '3.1415926', 'base_footprint', 'laser']
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/home/jetson/nav2.rviz'],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('image_proc'), 'launch', 'slam_launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'params_file': '/home/jetson/nav2.yaml',
                    'log_level': 'off'
                }.items()
        ),
        
        # Node(
        #     package='mbot_python',
        #     executable='rviz_conversion_node',
        #     name='rviz_conversion_node',
        #     output='log'
        # ),

        Node(
            package='image_proc',
            executable='sync',
            name='sync',
            output='log'
        ),

        Node(
            package='image_proc',
            executable='image_pub',
            name='image_pub',
            output='log'
        ),

        Node(
            package='image_proc',
            executable='manipulator_ros',
            name='manipulator_ros',
            output='log'
        ),

        Node(
            package='image_proc',
            executable='ultrasonic_ros',
            name='ultrasonic_ros',
            output='log'
        ),

        # Node(
        #     package='image_proc',
        #     executable='amr_nav',
        #     name='amr_nav',
        #     output='log'
        # )


    ])