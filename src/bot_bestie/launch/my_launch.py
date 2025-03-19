from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bot_bestie',
            executable='my_node',
            name='my_node',
            output='screen'
        )
    ])
