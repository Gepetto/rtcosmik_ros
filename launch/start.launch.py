from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

INIT_DONE_TOKEN = 'RTCOSMIK_INIT_DONE'


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    cam_calib_path = LaunchConfiguration('cam_calib_path')
    scaled_urdf_output_path = PathJoinSubstitution([
        FindPackageShare('rtcosmik_ros'),
        'urdf',
        'human_scaled.urdf',
    ])

    bridge_node = Node(
        package='rtcosmik_ros',
        executable='marker_bridge',
        name='rtcosmik_bridge',
        output='screen',
        additional_env={
            'RTCOSMIK_CAM_CALIB_PATH': cam_calib_path,
        },
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rtcosmik_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', scaled_urdf_output_path]),
                value_type=str,
            ),
        }],
        remappings=[('joint_states', '/rtcosmik/joint_states')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rtcosmik_rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('rtcosmik_ros'),
            'rviz',
            'markers.rviz',
        ])],
        condition=IfCondition(use_rviz),
    )

    startup_gate = {'started': False, 'buffer': ''}

    def _start_visualization_after_init(event, *args, **kwargs):
        del args, kwargs
        if startup_gate['started']:
            return []

        text = event.text
        if isinstance(text, (bytes, bytearray)):
            text = text.decode(errors='ignore')
        else:
            text = str(text)

        startup_gate['buffer'] += text
        if INIT_DONE_TOKEN not in startup_gate['buffer']:
            return []

        startup_gate['started'] = True
        return [
            LogInfo(msg='RT-COSMIK initialization completed, starting robot model and RViz.'),
            robot_state_publisher_node,
            rviz_node,
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 together with the bridge.',
        ),
        DeclareLaunchArgument(
            'cam_calib_path',
            default_value='',
            description=(
                'Optional camera calibration/config path passed to RT-COSMIK via '
                'RTCOSMIK_CAM_CALIB_PATH environment variable.'
            ),
        ),
        bridge_node,
        RegisterEventHandler(
            OnProcessIO(
                target_action=bridge_node,
                on_stdout=_start_visualization_after_init,
                on_stderr=_start_visualization_after_init,
            )
        ),
    ])
