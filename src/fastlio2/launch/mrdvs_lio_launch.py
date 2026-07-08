import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_rviz = LaunchConfiguration('enable_rviz')
    config_file = LaunchConfiguration('config_file')
    static_tf_x = LaunchConfiguration('static_tf_x')
    static_tf_y = LaunchConfiguration('static_tf_y')
    static_tf_z = LaunchConfiguration('static_tf_z')
    static_tf_roll = LaunchConfiguration('static_tf_roll')
    static_tf_pitch = LaunchConfiguration('static_tf_pitch')
    static_tf_yaw = LaunchConfiguration('static_tf_yaw')

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
            DeclareLaunchArgument(
                'static_tf_x',
                default_value='0.014569',
                description='LiDAR-to-IMU static TF x used for mrdvs_imu -> mrdvs_tof',
            ),
            DeclareLaunchArgument(
                'static_tf_y',
                default_value='-0.002738',
                description='LiDAR-to-IMU static TF y used for mrdvs_imu -> mrdvs_tof',
            ),
            DeclareLaunchArgument(
                'static_tf_z',
                default_value='0.022567',
                description='LiDAR-to-IMU static TF z used for mrdvs_imu -> mrdvs_tof',
            ),
            DeclareLaunchArgument(
                'static_tf_roll',
                default_value='0.0',
                description='LiDAR-to-IMU static TF roll used for mrdvs_imu -> mrdvs_tof',
            ),
            DeclareLaunchArgument(
                'static_tf_pitch',
                default_value='0.0',
                description='LiDAR-to-IMU static TF pitch used for mrdvs_imu -> mrdvs_tof',
            ),
            DeclareLaunchArgument(
                'static_tf_yaw',
                default_value='0.0',
                description='LiDAR-to-IMU static TF yaw used for mrdvs_imu -> mrdvs_tof',
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
                    '--x', static_tf_x,
                    '--y', static_tf_y,
                    '--z', static_tf_z,
                    '--roll', static_tf_roll,
                    '--pitch', static_tf_pitch,
                    '--yaw', static_tf_yaw,
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
