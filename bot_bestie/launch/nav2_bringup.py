# Copyright 2019 Open Source Robotics Foundation
# Licensed under the Apache License, Version 2.0, etc.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # This is used if you also want to load an initial map or do pure localization
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))

    # Nav2 param file for TurtleBot3 (e.g. burger.yaml, waffle.yaml, etc.)
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name))


    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load (if doing map-based localization)'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to Nav2 param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Path to Cartographer's config files
        cartographer_config_dir = os.path.join(
            get_package_share_directory('turtlebot3_cartographer'),
            'config'
        ),


        # 2) Nav2 bringup (runs map_server, planner, controller, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir
            }.items(),
        ),

        # cartographer_node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]
        ),

        # 3) RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
