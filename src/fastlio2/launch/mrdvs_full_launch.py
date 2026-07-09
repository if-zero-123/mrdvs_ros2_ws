import launch
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_rviz = LaunchConfiguration('enable_rviz')
    camera_ip = LaunchConfiguration('camera_ip')
    fastlio_delay = LaunchConfiguration('fastlio_delay')
    config_file = LaunchConfiguration('config_file')
    static_tf_x = LaunchConfiguration('static_tf_x')
    static_tf_y = LaunchConfiguration('static_tf_y')
    static_tf_z = LaunchConfiguration('static_tf_z')
    static_tf_roll = LaunchConfiguration('static_tf_roll')
    static_tf_pitch = LaunchConfiguration('static_tf_pitch')
    static_tf_yaw = LaunchConfiguration('static_tf_yaw')

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
                default_value='mrdvs.yaml',
                description='FAST-LIO2 config file under the fastlio2 config directory',
            ),
            DeclareLaunchArgument(
                'static_tf_x',
                default_value='0.014569',
                description='Manufacturer LiDAR-to-IMU static TF x',
            ),
            DeclareLaunchArgument(
                'static_tf_y',
                default_value='-0.002738',
                description='Manufacturer LiDAR-to-IMU static TF y',
            ),
            DeclareLaunchArgument(
                'static_tf_z',
                default_value='0.022567',
                description='Manufacturer LiDAR-to-IMU static TF z',
            ),
            DeclareLaunchArgument(
                'static_tf_roll',
                default_value='0.0',
                description='Manufacturer LiDAR-to-IMU static TF roll',
            ),
            DeclareLaunchArgument(
                'static_tf_pitch',
                default_value='0.0',
                description='Manufacturer LiDAR-to-IMU static TF pitch',
            ),
            DeclareLaunchArgument(
                'static_tf_yaw',
                default_value='0.0',
                description='Manufacturer LiDAR-to-IMU static TF yaw',
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
                            'enable_rviz': enable_rviz,
                            'config_file': config_file,
                            'static_tf_x': static_tf_x,
                            'static_tf_y': static_tf_y,
                            'static_tf_z': static_tf_z,
                            'static_tf_roll': static_tf_roll,
                            'static_tf_pitch': static_tf_pitch,
                            'static_tf_yaw': static_tf_yaw,
                        }.items(),
                    ),
                ],
            ),
        ]
    )
