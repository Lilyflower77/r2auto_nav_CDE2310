from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('bot_bestie')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the argument (this is where you define the default)
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # or 'false' depending on your needs
        description='Use simulation (Gazebo) clock if true'
    )

    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2_bringup.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()  # correctly passes it along
    )

    global_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'global_controller_bringup.py')
        )
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        planner_launch,
        global_controller_launch,
    ])
