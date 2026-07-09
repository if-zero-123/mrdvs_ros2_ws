#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_ip = LaunchConfiguration("camera_ip")
    fastlivo_delay = LaunchConfiguration("fastlivo_delay")
    use_rviz = LaunchConfiguration("use_rviz")
    mrdvs_params_file = LaunchConfiguration("mrdvs_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    tof_tf_x = LaunchConfiguration("tof_tf_x")
    tof_tf_y = LaunchConfiguration("tof_tf_y")
    tof_tf_z = LaunchConfiguration("tof_tf_z")
    tof_tf_roll = LaunchConfiguration("tof_tf_roll")
    tof_tf_pitch = LaunchConfiguration("tof_tf_pitch")
    tof_tf_yaw = LaunchConfiguration("tof_tf_yaw")

    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    mrdvs_config = os.path.join(config_file_dir, "mrdvs.yaml")
    camera_config = os.path.join(config_file_dir, "camera_mrdvs.yaml")

    lx_launch = os.path.join(
        get_package_share_directory("lx_camera_ros"),
        "launch",
        "lx_lidar_ros.launch.py",
    )
    fastlivo_launch = os.path.join(
        get_package_share_directory("fast_livo"),
        "launch",
        "mapping_mrdvs.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_ip", default_value="192.168.100.82", description="MRDVS camera IP"),
        DeclareLaunchArgument("fastlivo_delay", default_value="3.0", description="Delay before starting FAST-LIVO2"),
        DeclareLaunchArgument("use_rviz", default_value="False", description="Whether to launch FAST-LIVO2 RViz"),
        DeclareLaunchArgument(
            "mrdvs_params_file",
            default_value=mrdvs_config,
            description="FAST-LIVO2 parameter file for MRDVS",
        ),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=camera_config,
            description="Camera parameter file loaded into parameter_blackboard",
        ),
        DeclareLaunchArgument(
            "tof_tf_x",
            default_value="0.014569",
            description="Manufacturer LiDAR-to-IMU static TF x",
        ),
        DeclareLaunchArgument(
            "tof_tf_y",
            default_value="-0.002738",
            description="Manufacturer LiDAR-to-IMU static TF y",
        ),
        DeclareLaunchArgument(
            "tof_tf_z",
            default_value="0.022567",
            description="Manufacturer LiDAR-to-IMU static TF z",
        ),
        DeclareLaunchArgument(
            "tof_tf_roll",
            default_value="0.0",
            description="Manufacturer LiDAR-to-IMU static TF roll",
        ),
        DeclareLaunchArgument(
            "tof_tf_pitch",
            default_value="0.0",
            description="Manufacturer LiDAR-to-IMU static TF pitch",
        ),
        DeclareLaunchArgument(
            "tof_tf_yaw",
            default_value="0.0",
            description="Manufacturer LiDAR-to-IMU static TF yaw",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lx_launch),
            launch_arguments={
                "ip": camera_ip,
                "enable_rviz": "false",
            }.items(),
        ),
        TimerAction(
            period=fastlivo_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(fastlivo_launch),
                    launch_arguments={
                        "use_rviz": use_rviz,
                        "mrdvs_params_file": mrdvs_params_file,
                        "camera_params_file": camera_params_file,
                        "tof_tf_x": tof_tf_x,
                        "tof_tf_y": tof_tf_y,
                        "tof_tf_z": tof_tf_z,
                        "tof_tf_roll": tof_tf_roll,
                        "tof_tf_pitch": tof_tf_pitch,
                        "tof_tf_yaw": tof_tf_yaw,
                    }.items(),
                )
            ],
        ),
    ])
