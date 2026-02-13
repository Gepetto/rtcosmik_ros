import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_urdf_text():
    share_dir = get_package_share_directory('rtcosmik_ros')
    urdf_path = os.path.join(share_dir, 'urdf', 'human.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        return f.read()


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    cam_calib_path = LaunchConfiguration('cam_calib_path')
    enable_markers = LaunchConfiguration('enable_markers')
    queue_timeout_s = LaunchConfiguration('queue_timeout_s')
    use_robot_model = LaunchConfiguration('use_robot_model')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 together with the bridge.',
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
                'RTCOSMIK_CAM_CALIB_PATH environment variable.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_markers',
            default_value='true',
            description='Publish MarkerArray on /rtcosmik/markers.',
        ),
        DeclareLaunchArgument(
            'queue_timeout_s',
            default_value='0.03',
            description='Blocking queue timeout in seconds for bridge consumer thread.',
        ),
        DeclareLaunchArgument(
            'use_robot_model',
            default_value='true',
            description='Start robot_state_publisher for the human URDF model.',
        ),
        Node(
            package='rtcosmik_ros',
            executable='marker_bridge',
            name='rtcosmik_bridge',
            output='screen',
            parameters=[{
                'enable_markers': enable_markers,
                'queue_timeout_s': queue_timeout_s,
            }],
            additional_env={
                'RTCOSMIK_CAM_CALIB_PATH': cam_calib_path,
            },
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rtcosmik_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': _load_urdf_text()}],
            remappings=[('joint_states', '/rtcosmik/joint_states')],
            condition=IfCondition(use_robot_model),
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
