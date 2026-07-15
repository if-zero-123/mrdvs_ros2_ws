import launch
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_rviz = LaunchConfiguration('enable_rviz')
    camera_ip = LaunchConfiguration('camera_ip')
    fastlio_delay = LaunchConfiguration('fastlio_delay')

    lidar_launch = PathJoinSubstitution(
        [FindPackageShare('lx_camera_ros'), 'launch', 'lx_lidar_ros.launch.py']
    )
    fastlio_launch = PathJoinSubstitution(
        [FindPackageShare('fastlio2'), 'launch', 'mrdvs_lio_launch.py']
    )
    pgo_config = PathJoinSubstitution(
        [FindPackageShare('pgo'), 'config', 'mrdvs.yaml']
    )
    pgo_rviz = PathJoinSubstitution(
        [FindPackageShare('pgo'), 'rviz', 'mrdvs_pgo_optimized.rviz']
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_rviz',
                default_value='true',
                description='Whether to launch RViz with the PGO configuration',
            ),
            DeclareLaunchArgument(
                'camera_ip',
                default_value='192.168.100.82',
                description='Fixed MRDVS camera IP passed to lx_camera_node',
            ),
            DeclareLaunchArgument(
                'fastlio_delay',
                default_value='3.0',
                description='Seconds to wait before starting FAST-LIO2 and PGO',
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(lidar_launch),
                        launch_arguments={
                            'enable_rviz': 'false',
                            'ip': camera_ip,
                        }.items(),
                    ),
                ],
            ),
            TimerAction(
                period=fastlio_delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(fastlio_launch),
                        launch_arguments={
                            'enable_rviz': 'false',
                            'config_file': 'mrdvs_pgo.yaml',
                        }.items(),
                    ),
                    Node(
                        package='pgo',
                        namespace='pgo',
                        executable='pgo_node',
                        name='pgo_node',
                        output='screen',
                        parameters=[{'config_path': pgo_config}],
                    ),
                    Node(
                        package='rviz2',
                        namespace='pgo',
                        executable='rviz2',
                        name='rviz2',
                        output='screen',
                        arguments=['-d', pgo_rviz],
                        condition=IfCondition(enable_rviz),
                    ),
                ],
            ),
        ]
    )
