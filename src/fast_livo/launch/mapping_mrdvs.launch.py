#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")

    mrdvs_config = os.path.join(config_file_dir, "mrdvs.yaml")
    camera_config = os.path.join(config_file_dir, "camera_mrdvs.yaml")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to launch RViz2",
    )
    mrdvs_config_arg = DeclareLaunchArgument(
        "mrdvs_params_file",
        default_value=mrdvs_config,
        description="FAST-LIVO2 parameter file for MRDVS",
    )
    camera_config_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=camera_config,
        description="Camera parameter file loaded into parameter_blackboard",
    )

    mrdvs_params_file = LaunchConfiguration("mrdvs_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    tof_tf_x = LaunchConfiguration("tof_tf_x")
    tof_tf_y = LaunchConfiguration("tof_tf_y")
    tof_tf_z = LaunchConfiguration("tof_tf_z")
    tof_tf_roll = LaunchConfiguration("tof_tf_roll")
    tof_tf_pitch = LaunchConfiguration("tof_tf_pitch")
    tof_tf_yaw = LaunchConfiguration("tof_tf_yaw")

    return LaunchDescription([
        use_rviz_arg,
        mrdvs_config_arg,
        camera_config_arg,
        DeclareLaunchArgument(
            "tof_tf_x",
            default_value="0.014569",
            description="LiDAR-to-IMU static TF x used for aft_mapped -> mrdvs_tof",
        ),
        DeclareLaunchArgument(
            "tof_tf_y",
            default_value="-0.002738",
            description="LiDAR-to-IMU static TF y used for aft_mapped -> mrdvs_tof",
        ),
        DeclareLaunchArgument(
            "tof_tf_z",
            default_value="0.022567",
            description="LiDAR-to-IMU static TF z used for aft_mapped -> mrdvs_tof",
        ),
        DeclareLaunchArgument(
            "tof_tf_roll",
            default_value="0.0",
            description="LiDAR-to-IMU static TF roll used for aft_mapped -> mrdvs_tof",
        ),
        DeclareLaunchArgument(
            "tof_tf_pitch",
            default_value="0.0",
            description="LiDAR-to-IMU static TF pitch used for aft_mapped -> mrdvs_tof",
        ),
        DeclareLaunchArgument(
            "tof_tf_yaw",
            default_value="0.0",
            description="LiDAR-to-IMU static TF yaw used for aft_mapped -> mrdvs_tof",
        ),
        Node(
            package="demo_nodes_cpp",
            executable="parameter_blackboard",
            name="parameter_blackboard",
            parameters=[camera_params_file],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_camera_init_tf",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "-1.57079632679",
                "--pitch", "0.0",
                "--yaw", "-1.57079632679",
                "--frame-id", "map",
                "--child-frame-id", "camera_init",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="aft_mapped_to_mrdvs_tof_tf",
            output="screen",
            arguments=[
                "--x", tof_tf_x,
                "--y", tof_tf_y,
                "--z", tof_tf_z,
                "--roll", tof_tf_roll,
                "--pitch", tof_tf_pitch,
                "--yaw", tof_tf_yaw,
                "--frame-id", "aft_mapped",
                "--child-frame-id", "mrdvs_tof",
            ],
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="fast_livo",
                    executable="fastlivo_mapping",
                    name="laserMapping",
                    parameters=[mrdvs_params_file],
                    output="screen",
                )
            ],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
        ),
    ])
