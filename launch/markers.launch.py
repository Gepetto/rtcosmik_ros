from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtcosmik_ros',
            executable='marker_bridge',
            name='rtcosmik_marker_bridge',
            output='screen',
        ),
    ])
