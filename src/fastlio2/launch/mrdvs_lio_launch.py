import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_rviz = LaunchConfiguration('enable_rviz')
    config_file = LaunchConfiguration('config_file')

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare('fastlio2'), 'rviz', 'fastlio2.rviz']
    )
    config_path = PathJoinSubstitution(
        [FindPackageShare('fastlio2'), 'config', config_file]
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_rviz',
                default_value='false',
                description='Whether to launch rviz2 with the FAST-LIO2 config',
            ),
            DeclareLaunchArgument(
                'config_file',
                default_value='mrdvs.yaml',
                description='FAST-LIO2 config file under the fastlio2 config directory',
            ),
            launch_ros.actions.Node(
                package='fastlio2',
                namespace='fastlio2',
                executable='lio_node',
                name='lio_node',
                output='screen',
                parameters=[{'config_path': config_path}],
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='mrdvs_imu_to_tof_tf',
                output='screen',
                arguments=[
                    '--x', '0.014569',
                    '--y', '-0.002738',
                    '--z', '0.022567',
                    '--roll', '0.0',
                    '--pitch', '0.0',
                    '--yaw', '0.0',
                    '--frame-id', 'mrdvs_imu',
                    '--child-frame-id', 'mrdvs_tof',
                ],
            ),
            launch_ros.actions.Node(
                package='rviz2',
                namespace='fastlio2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_cfg],
                condition=IfCondition(enable_rviz),
            ),
        ]
    )
