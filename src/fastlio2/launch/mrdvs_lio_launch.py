import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_rviz = LaunchConfiguration('enable_rviz')

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare('fastlio2'), 'rviz', 'fastlio2.rviz']
    )
    config_path = PathJoinSubstitution(
        [FindPackageShare('fastlio2'), 'config', 'mrdvs.yaml']
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_rviz',
                default_value='false',
                description='Whether to launch rviz2 with the FAST-LIO2 config',
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
