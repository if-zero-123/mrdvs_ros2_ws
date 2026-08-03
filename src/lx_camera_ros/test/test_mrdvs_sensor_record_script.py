from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


def test_sensor_record_script_records_only_required_topics_with_mcap():
    script_source = (WORKSPACE_ROOT / "record_mrdvs_sensor_bag.sh").read_text()

    assert 'bag_root="${MRDVS_BAG_ROOT:-/home/zero/MRDVS_bags}"' in script_source
    assert 'ip:=192.168.100.82' in script_source
    assert "ros2 bag record" in script_source
    assert "--storage mcap" in script_source
    assert "/lx_camera_node/LxCamera_Cloud" in script_source
    assert "/lx_camera_node/LxCamera_Rgb" in script_source
    assert "/lx_camera_node/LxCamera_Imu" in script_source
    assert "-a" not in script_source


def test_sensor_recording_processes_are_isolated_from_terminal_interrupts():
    script_source = (WORKSPACE_ROOT / "record_mrdvs_sensor_bag.sh").read_text()

    assert "setsid ros2 bag record" in script_source
    assert "setsid ros2 launch lx_camera_ros lx_lidar_ros.launch.py" in script_source


def test_lidar_launch_explicitly_disables_2d_undistortion():
    launch_source = (
        WORKSPACE_ROOT / "src/lx_camera_ros/launch/lx_lidar_ros.launch.py"
    ).read_text()

    assert '{"LX_BOOL_ENABLE_2D_UNDISTORT": 0}' in launch_source
