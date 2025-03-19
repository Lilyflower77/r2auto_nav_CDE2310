from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = os.path.join(
        os.getenv('COLCON_PREFIX_PATH', '').split(':')[0], 'share', 'bot_bestie')

    # Include Planner (Nav2) launch file
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2_bringup.py')
        )
    )

    # Include global controller launch file
    global_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'global_controller_bringup.py')
        )
    )

    # Lifecycle Manager to coordinate states for planner
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'planner_server',   # Only manage planner node
                'global_costmap'
            ]
        }]
    )

    return LaunchDescription([
        planner_launch,
        global_controller_launch,
        lifecycle_manager
    ])
