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
                    }.items(),
                )
            ],
        ),
    ])
