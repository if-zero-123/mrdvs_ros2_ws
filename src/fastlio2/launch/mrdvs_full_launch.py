import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_rviz = LaunchConfiguration('enable_rviz')
    camera_ip = LaunchConfiguration('camera_ip')
    fastlio_delay = LaunchConfiguration('fastlio_delay')
    config_file = LaunchConfiguration('config_file')

    lidar_launch = PathJoinSubstitution(
        [FindPackageShare('lx_camera_ros'), 'launch', 'lx_lidar_ros.launch.py']
    )
    fastlio_launch = PathJoinSubstitution(
        [FindPackageShare('fastlio2'), 'launch', 'mrdvs_lio_launch.py']
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_rviz',
                default_value='true',
                description='Whether to launch rviz2 with the FAST-LIO2 config',
            ),
            DeclareLaunchArgument(
                'camera_ip',
                default_value='192.168.100.82',
                description='Fixed MRDVS camera IP passed to lx_camera_node',
            ),
            DeclareLaunchArgument(
                'fastlio_delay',
                default_value='3.0',
                description='Seconds to wait after starting the camera before FAST-LIO2',
            ),
            DeclareLaunchArgument(
                'config_file',
                default_value='mrdvs_refined.yaml',
                description='FAST-LIO2 config file under the fastlio2 config directory',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch),
                launch_arguments={
                    'enable_rviz': 'false',
                    'ip': camera_ip,
                }.items(),
            ),
            TimerAction(
                period=fastlio_delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(fastlio_launch),
                        launch_arguments={
                            'enable_rviz': enable_rviz,
                            'config_file': config_file,
                        }.items(),
                    ),
                ],
            ),
        ]
    )
