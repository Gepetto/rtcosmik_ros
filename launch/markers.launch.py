from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    cam_calib_path = LaunchConfiguration('cam_calib_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 together with the marker bridge.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('rtcosmik_ros'),
                'rviz',
                'markers.rviz',
            ]),
            description='RViz2 config file path.',
        ),
        DeclareLaunchArgument(
            'cam_calib_path',
            default_value='',
            description=(
                'Optional camera calibration/config path passed to RT-COSMIK via '
                'RTCOSMIK_CAM_CALIB_PATH environment variable. '
                'If empty, the default RT-COSMIK settings are used.'
            ),
        ),
        Node(
            package='rtcosmik_ros',
            executable='marker_bridge',
            name='rtcosmik_marker_bridge',
            output='screen',
            additional_env={
                'RTCOSMIK_CAM_CALIB_PATH': cam_calib_path,
            },
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rtcosmik_rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])

