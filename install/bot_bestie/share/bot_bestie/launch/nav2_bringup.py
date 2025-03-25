from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=['config/nav2_params.yaml']
        ),
        Node(
            package='nav2_costmap',
            executable='global_costmap',
            name='global_costmap',
            output='screen',
            parameters=['config/nav2_params.yaml']
        )
    ])
