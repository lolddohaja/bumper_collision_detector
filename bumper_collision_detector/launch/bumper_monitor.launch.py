from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bumper_collision_detector',
            executable='bumper_monitor_node',
            name='bumper_monitor_node',
            parameters=[
                {'collision_value': 100},
                {'end_collision_value': -50},
                {'end_positive_count': 10}
            ]
        )
    ])