import math
import struct

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from mrdvs_web_console.models import AppConfig
from mrdvs_web_console.ros_bridge import (
    RosBridge,
    decode_cloud_xyzi,
    encode_pointcloud,
    sample_xyzi,
)


def make_cloud() -> object:
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.UINT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
        PointField(name="row_pos", offset=24, datatype=PointField.UINT16, count=1),
        PointField(name="col_pos", offset=26, datatype=PointField.UINT16, count=1),
    ]
    header = Header(stamp=Time(sec=10, nanosec=20), frame_id="mrdvs_tof")
    return point_cloud2.create_cloud(
        header,
        fields,
        [
            (1.0, 2.0, 3.0, 4, 1_000_000.0, 10, 20),
            (5.0, 6.0, 7.0, 8, 1_000_001.0, 11, 21),
        ],
        point_step=28,
    )


def make_imu(seconds: int, nanoseconds: int, value: float) -> Imu:
    message = Imu()
    message.header.stamp = Time(sec=seconds, nanosec=nanoseconds)
    message.linear_acceleration.x = value
    message.linear_acceleration.y = value + 1
    message.linear_acceleration.z = value + 2
    message.angular_velocity.x = value + 3
    message.angular_velocity.y = value + 4
    message.angular_velocity.z = value + 5
    return message


def test_pointcloud_sampling_is_bounded_and_does_not_mutate_source():
    source = np.arange(400_000, dtype=np.float32).reshape(100_000, 4)
    source_copy = source.copy()

    sampled = sample_xyzi(source, max_points=50_000)

    assert sampled.shape == (50_000, 4)
    np.testing.assert_array_equal(source, source_copy)


def test_pointcloud_sampling_drops_nonfinite_coordinates():
    source = np.array(
        [[1, 2, 3, 4], [math.nan, 2, 3, 5], [4, 5, 6, 7]], dtype=np.float32
    )

    sampled = sample_xyzi(source, max_points=50_000)

    np.testing.assert_array_equal(sampled, [[1, 2, 3, 4], [4, 5, 6, 7]])


def test_pointcloud_binary_frame_contains_magic_count_and_xyzi():
    points = np.array([[1.0, 2.0, 3.0, 4.0]], dtype="<f4")

    frame = encode_pointcloud(points)

    assert frame[:4] == b"MPC1"
    assert struct.unpack_from("<I", frame, 4)[0] == 1
    assert struct.unpack_from("<ffff", frame, 8) == (1.0, 2.0, 3.0, 4.0)


def test_decode_cloud_reads_xyzi_without_touching_point_timestamp():
    cloud = make_cloud()
    original_data = bytes(cloud.data)

    decoded = decode_cloud_xyzi(cloud)

    np.testing.assert_array_equal(decoded, [[1, 2, 3, 4], [5, 6, 7, 8]])
    assert bytes(cloud.data) == original_data
    assert any(field.name == "timestamp" for field in cloud.fields)


def test_bridge_throttles_display_frames_but_counts_raw_cloud_rate(tmp_path):
    now = [100.0]
    config = AppConfig(
        deploy_root=tmp_path,
        bag_root=tmp_path / "bags",
        state_root=tmp_path / "state",
        pointcloud_max_hz=5,
    )
    bridge = RosBridge(config, monotonic_clock=lambda: now[0])
    cloud = make_cloud()

    bridge.handle_cloud(cloud)
    first_sequence, first_frame = bridge.latest_pointcloud()
    now[0] += 0.1
    bridge.handle_cloud(cloud)
    throttled_sequence, _ = bridge.latest_pointcloud()
    now[0] += 0.1
    bridge.handle_cloud(cloud)
    final_sequence, _ = bridge.latest_pointcloud()

    assert first_frame is not None
    assert throttled_sequence == first_sequence
    assert final_sequence == first_sequence + 1
    assert bridge.topic_status().cloud_hz == pytest.approx(10.0)


def test_bridge_retains_only_configured_imu_window(tmp_path):
    now = [200.0]
    config = AppConfig(
        deploy_root=tmp_path,
        bag_root=tmp_path / "bags",
        state_root=tmp_path / "state",
        imu_max_hz=20,
        imu_window_seconds=0.1,
    )
    bridge = RosBridge(config, monotonic_clock=lambda: now[0])

    for index in range(5):
        bridge.handle_imu(make_imu(20, index, float(index)))
        now[0] += 0.05

    sequence, latest = bridge.latest_imu()
    history = bridge.imu_history()

    assert sequence == 5
    assert latest["linear_acceleration"]["x"] == 4.0
    assert latest["angular_velocity"]["z"] == 9.0
    assert len(history) == 3
    assert history[0]["linear_acceleration"]["x"] == 2.0


def test_bridge_start_and_stop_are_idempotent(tmp_path):
    config = AppConfig(
        deploy_root=tmp_path,
        bag_root=tmp_path / "bags",
        state_root=tmp_path / "state",
    )
    bridge = RosBridge(config)

    bridge.start()
    bridge.start()
    assert bridge.running is True

    bridge.stop()
    bridge.stop()
    assert bridge.running is False
