from pathlib import Path
import struct

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from mrdvs_web_console.timestamp_analysis import (
    DEFAULT_CLOUD_TOPIC,
    DEFAULT_IMU_TOPIC,
    analyze_deserialized_messages,
    write_analysis_outputs,
)


def make_cloud(seconds: int, nanoseconds: int, point_times_us: list[float]):
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.UINT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
        PointField(name="row_pos", offset=24, datatype=PointField.UINT16, count=1),
        PointField(name="col_pos", offset=26, datatype=PointField.UINT16, count=1),
    ]
    points = [
        (float(index), 0.0, 0.0, index, point_time_us, 0, index)
        for index, point_time_us in enumerate(point_times_us)
    ]
    return point_cloud2.create_cloud(
        Header(stamp=Time(sec=seconds, nanosec=nanoseconds), frame_id="mrdvs_tof"),
        fields,
        points,
        point_step=28,
    )


def make_imu(seconds: int, nanoseconds: int) -> Imu:
    message = Imu()
    message.header.stamp = Time(sec=seconds, nanosec=nanoseconds)
    return message


def make_padded_multiline_cloud(
    seconds: int, nanoseconds: int, point_times_us: list[float]
) -> PointCloud2:
    """Create a 2×2 PointCloud2 whose rows contain trailing padding bytes."""

    message = PointCloud2()
    message.header = Header(stamp=Time(sec=seconds, nanosec=nanoseconds))
    message.height = 2
    message.width = 2
    message.fields = [
        PointField(name="timestamp", offset=8, datatype=PointField.FLOAT64, count=1)
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = 40  # 2 * point_step plus 8 padding bytes per row
    payload = bytearray(message.row_step * message.height)
    for index, point_time_us in enumerate(point_times_us):
        row, column = divmod(index, message.width)
        struct.pack_into(
            "<d",
            payload,
            row * message.row_step + column * message.point_step + 8,
            point_time_us,
        )
    message.data = list(payload)
    return message


def test_analyze_messages_keeps_sensor_point_and_record_times_separate():
    cloud_first = make_cloud(100, 0, [100_000_000.0, 100_091_730.0])
    cloud_second = make_cloud(100, 100_000_000, [100_100_000.0, 100_191_730.0])
    imu = make_imu(100, 25_000_000)

    report = analyze_deserialized_messages(
        [
            (DEFAULT_CLOUD_TOPIC, cloud_first, 100_500_000_000),
            (DEFAULT_IMU_TOPIC, imu, 100_501_000_000),
            (DEFAULT_CLOUD_TOPIC, cloud_second, 100_510_000_000),
        ]
    )

    cloud_rows = [row for row in report.rows if row.topic == DEFAULT_CLOUD_TOPIC]
    imu_rows = [row for row in report.rows if row.topic == DEFAULT_IMU_TOPIC]

    assert len(cloud_rows) == 2
    assert len(imu_rows) == 1

    first = cloud_rows[0]
    assert first.header_stamp_ns == 100_000_000_000
    assert first.recorded_stamp_ns == 100_500_000_000
    assert first.record_minus_header_ms == 500.0
    assert first.point_timestamp_min_ns == 100_000_000_000
    assert first.point_timestamp_max_ns == 100_091_730_000
    assert first.point_time_span_ms == 91.73
    assert first.point_min_offset_ms == 0.0
    assert first.point_max_offset_ms == 91.73

    second = cloud_rows[1]
    assert second.header_delta_ms == 100.0
    assert second.record_delta_ms == 10.0
    assert second.point_timestamp_min_ns == 100_100_000_000

    assert imu_rows[0].header_stamp_ns == 100_025_000_000
    assert imu_rows[0].point_timestamp_min_ns is None
    assert report.summary.message_counts == {
        DEFAULT_CLOUD_TOPIC: 2,
        DEFAULT_IMU_TOPIC: 1,
    }


def test_microsecond_point_timestamp_keeps_epoch_precision_without_float_ns_rounding():
    cloud = make_cloud(
        1_784_713_619,
        169_397_000,
        [1_784_713_619_169_397.0, 1_784_713_619_261_127.0],
    )

    report = analyze_deserialized_messages(
        [(DEFAULT_CLOUD_TOPIC, cloud, 1_784_713_620_000_000_000)]
    )

    row = report.rows[0]
    assert row.point_timestamp_min_ns == 1_784_713_619_169_397_000
    assert row.point_timestamp_max_ns == 1_784_713_619_261_127_000
    assert row.point_min_offset_ms == 0.0


def test_point_timestamp_bounds_follow_row_step_for_padded_multiline_clouds():
    cloud = make_padded_multiline_cloud(
        100,
        0,
        [100_000_000.0, 100_025_000.0, 100_050_000.0, 100_075_000.0],
    )

    report = analyze_deserialized_messages([(DEFAULT_CLOUD_TOPIC, cloud, 100_100_000_000)])

    row = report.rows[0]
    assert row.point_count == 4
    assert row.point_timestamp_min_ns == 100_000_000_000
    assert row.point_timestamp_max_ns == 100_075_000_000


def test_write_outputs_creates_csv_summary_guide_and_svg_charts(tmp_path: Path):
    cloud = make_cloud(100, 0, [100_000_000.0, 100_091_730.0])
    imu = make_imu(100, 25_000_000)
    report = analyze_deserialized_messages(
        [
            (DEFAULT_CLOUD_TOPIC, cloud, 100_500_000_000),
            (DEFAULT_IMU_TOPIC, imu, 100_501_000_000),
        ]
    )

    outputs = write_analysis_outputs(report, tmp_path)

    assert outputs.comparison_csv.is_file()
    assert outputs.summary_json.is_file()
    assert outputs.reading_guide.is_file()
    assert outputs.comparison_chart.is_file()
    assert outputs.point_coverage_chart.is_file()
    assert "header_stamp_ns" in outputs.comparison_csv.read_text(encoding="utf-8")
    assert "点云和 IMU 同步" in outputs.reading_guide.read_text(encoding="utf-8")
    assert "MCAP 接收时间" in outputs.comparison_chart.read_text(encoding="utf-8")
    assert "点级 timestamp" in outputs.point_coverage_chart.read_text(encoding="utf-8")

    values = np.loadtxt(
        outputs.comparison_csv,
        delimiter=",",
        skiprows=1,
        usecols=(2, 3),
        dtype=str,
    )
    assert values.shape == (2, 2)
