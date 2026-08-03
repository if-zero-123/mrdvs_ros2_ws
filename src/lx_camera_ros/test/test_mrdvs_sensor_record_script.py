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
    assert "record -a" not in script_source


def test_sensor_recording_processes_are_isolated_from_terminal_interrupts():
    script_source = (WORKSPACE_ROOT / "record_mrdvs_sensor_bag.sh").read_text()

    assert "run_in_new_session ros2 bag record" in script_source
    assert "run_in_new_session ros2 launch lx_camera_ros lx_lidar_ros.launch.py" in script_source


def test_cleanup_finalizes_rosbag_before_stopping_driver():
    script_source = (WORKSPACE_ROOT / "record_mrdvs_sensor_bag.sh").read_text()

    assert script_source.index('if [[ -n "$bag_pid" ]]') < script_source.index(
        'if [[ -n "$driver_pid" ]]'
    )


def test_cleanup_stale_sessions_runs_before_new_recording():
    script_source = (WORKSPACE_ROOT / "record_mrdvs_sensor_bag.sh").read_text()

    assert "cleanup_stale_sessions" in script_source
    assert script_source.index("cleanup_stale_sessions") < script_source.index(
        "run_in_new_session ros2 bag record"
    )
    assert "ros2 bag record --storage mcap" in script_source
    assert "ros2 launch lx_camera_ros lx_lidar_ros.launch.py" in script_source


def test_new_processes_restore_interrupt_signal_defaults_and_force_stale_cleanup():
    script_source = (WORKSPACE_ROOT / "record_mrdvs_sensor_bag.sh").read_text()

    assert "trap - INT TERM" in script_source
    assert "kill -KILL" in script_source


def test_lidar_launch_explicitly_disables_2d_undistortion():
    launch_source = (
        WORKSPACE_ROOT / "src/lx_camera_ros/launch/lx_lidar_ros.launch.py"
    ).read_text()

    assert '{"LX_BOOL_ENABLE_2D_UNDISTORT": 0}' in launch_source
