"""Compare MRDVS sensor timestamps with rosbag receive timestamps.

The tool deliberately keeps three independent clocks separate:

* ``header.stamp`` is the ROS sensor-frame time.
* PointCloud2 ``timestamp`` is the MRDVS per-point device time.
* rosbag's message timestamp is when the recorder received the message.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
from collections import Counter, defaultdict
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2, PointField

DEFAULT_CLOUD_TOPIC = "/lx_camera_node/LxCamera_Cloud"
DEFAULT_IMU_TOPIC = "/lx_camera_node/LxCamera_Imu"
_POINT_TIME_SCALES_TO_NS = {"us": 1_000.0, "ns": 1.0, "s": 1_000_000_000.0}


@dataclass(frozen=True)
class TimestampRow:
    """One recorded cloud or IMU message and the clocks associated with it."""

    topic: str
    message_index: int
    header_stamp_ns: int
    recorded_stamp_ns: int
    header_delta_ms: float | None
    record_delta_ms: float | None
    record_minus_header_ms: float
    point_count: int | None
    point_timestamp_min_ns: int | None
    point_timestamp_max_ns: int | None
    point_time_span_ms: float | None
    point_min_offset_ms: float | None
    point_max_offset_ms: float | None


@dataclass(frozen=True)
class TimestampSummary:
    """Aggregate values that can be read without reopening the CSV file."""

    message_counts: dict[str, int]
    maximum_header_delta_ms: dict[str, float]
    maximum_record_delta_ms: dict[str, float]
    maximum_record_minus_header_ms: dict[str, float]


@dataclass(frozen=True)
class TimestampReport:
    rows: tuple[TimestampRow, ...]
    summary: TimestampSummary
    point_timestamp_unit: str


@dataclass(frozen=True)
class AnalysisOutputs:
    comparison_csv: Path
    summary_json: Path
    reading_guide: Path
    comparison_chart: Path
    point_coverage_chart: Path


def stamp_to_ns(stamp: Any) -> int:
    """Convert a ROS ``builtin_interfaces/Time`` into exact integer nanoseconds."""

    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _timestamp_field(message: PointCloud2) -> PointField:
    field = next((item for item in message.fields if item.name == "timestamp"), None)
    if field is None:
        raise ValueError("点云缺少名为 timestamp 的点级时间字段")
    if field.datatype != PointField.FLOAT64 or field.count != 1:
        raise ValueError("点云 timestamp 字段必须是单个 FLOAT64 值")
    return field


def _point_timestamp_bounds_ns(
    message: PointCloud2, point_timestamp_unit: str
) -> tuple[int, int, int]:
    """Return point count and min/max per-point device timestamps in nanoseconds."""

    field = _timestamp_field(message)
    width = int(message.width)
    height = int(message.height)
    point_count = width * height
    if point_count <= 0:
        raise ValueError("点云不包含任何点")
    dtype = ">f8" if message.is_bigendian else "<f8"
    try:
        values = np.ndarray(
            (height, width),
            dtype=dtype,
            buffer=message.data,
            offset=field.offset,
            strides=(message.row_step, message.point_step),
        )
    except (TypeError, ValueError) as error:
        raise ValueError("点云 data 与 timestamp 字段布局不匹配") from error
    finite = values[np.isfinite(values)]
    if len(finite) == 0:
        raise ValueError("点云不存在有效的 timestamp 值")
    minimum_ns = _point_time_to_ns(float(finite.min()), point_timestamp_unit)
    maximum_ns = _point_time_to_ns(float(finite.max()), point_timestamp_unit)
    return point_count, minimum_ns, maximum_ns


def _point_time_to_ns(value: float, point_timestamp_unit: str) -> int:
    """Convert one point timestamp without losing Epoch-scale microsecond precision."""

    if point_timestamp_unit == "us":
        # MRDVS emits integer microseconds around 1.78e15.  Multiplying that
        # float by 1000 first can lose several nanoseconds, so multiply only
        # after converting to an exact Python integer.
        return int(round(value)) * 1_000
    if point_timestamp_unit == "ns":
        return int(round(value))
    return int(round(value * _POINT_TIME_SCALES_TO_NS[point_timestamp_unit]))


def analyze_deserialized_messages(
    messages: Iterable[tuple[str, object, int]],
    *,
    cloud_topic: str = DEFAULT_CLOUD_TOPIC,
    imu_topic: str = DEFAULT_IMU_TOPIC,
    point_timestamp_unit: str = "us",
) -> TimestampReport:
    """Compare clocks from already deserialized messages.

    ``recorded_stamp_ns`` is supplied separately because it belongs to MCAP
    storage metadata, not to the ROS message body.  This boundary makes the
    analysis easy to test and prevents confusing receive time with sensor time.
    """

    if point_timestamp_unit not in _POINT_TIME_SCALES_TO_NS:
        choices = ", ".join(sorted(_POINT_TIME_SCALES_TO_NS))
        raise ValueError(f"不支持的点级时间单位 {point_timestamp_unit!r}，可选：{choices}")

    last_header_ns: dict[str, int] = {}
    last_record_ns: dict[str, int] = {}
    counts: Counter[str] = Counter()
    rows: list[TimestampRow] = []

    for topic, message, recorded_stamp_ns in messages:
        if topic not in {cloud_topic, imu_topic}:
            continue
        if topic == cloud_topic and not isinstance(message, PointCloud2):
            raise TypeError(f"{cloud_topic} 不是 PointCloud2 消息")
        if topic == imu_topic and not isinstance(message, Imu):
            raise TypeError(f"{imu_topic} 不是 Imu 消息")

        header_stamp_ns = stamp_to_ns(message.header.stamp)
        record_ns = int(recorded_stamp_ns)
        header_delta_ms = (
            None
            if topic not in last_header_ns
            else (header_stamp_ns - last_header_ns[topic]) / 1_000_000.0
        )
        record_delta_ms = (
            None
            if topic not in last_record_ns
            else (record_ns - last_record_ns[topic]) / 1_000_000.0
        )

        point_count: int | None = None
        point_min_ns: int | None = None
        point_max_ns: int | None = None
        point_time_span_ms: float | None = None
        point_min_offset_ms: float | None = None
        point_max_offset_ms: float | None = None
        if topic == cloud_topic:
            point_count, point_min_ns, point_max_ns = _point_timestamp_bounds_ns(
                message, point_timestamp_unit
            )
            point_time_span_ms = (point_max_ns - point_min_ns) / 1_000_000.0
            point_min_offset_ms = (point_min_ns - header_stamp_ns) / 1_000_000.0
            point_max_offset_ms = (point_max_ns - header_stamp_ns) / 1_000_000.0

        counts[topic] += 1
        rows.append(
            TimestampRow(
                topic=topic,
                message_index=counts[topic],
                header_stamp_ns=header_stamp_ns,
                recorded_stamp_ns=record_ns,
                header_delta_ms=header_delta_ms,
                record_delta_ms=record_delta_ms,
                record_minus_header_ms=(record_ns - header_stamp_ns) / 1_000_000.0,
                point_count=point_count,
                point_timestamp_min_ns=point_min_ns,
                point_timestamp_max_ns=point_max_ns,
                point_time_span_ms=point_time_span_ms,
                point_min_offset_ms=point_min_offset_ms,
                point_max_offset_ms=point_max_offset_ms,
            )
        )
        last_header_ns[topic] = header_stamp_ns
        last_record_ns[topic] = record_ns

    return TimestampReport(
        rows=tuple(rows),
        summary=TimestampSummary(
            message_counts=dict(counts),
            maximum_header_delta_ms=_maximum_delta(rows, "header_delta_ms"),
            maximum_record_delta_ms=_maximum_delta(rows, "record_delta_ms"),
            maximum_record_minus_header_ms=_maximum_delta(
                rows, "record_minus_header_ms"
            ),
        ),
        point_timestamp_unit=point_timestamp_unit,
    )


def _maximum_delta(
    rows: Sequence[TimestampRow], attribute: str
) -> dict[str, float]:
    values: dict[str, list[float]] = defaultdict(list)
    for row in rows:
        value = getattr(row, attribute)
        if value is not None:
            values[row.topic].append(float(value))
    return {topic: max(topic_values) for topic, topic_values in values.items()}


def analyze_bag(
    bag_path: Path,
    *,
    cloud_topic: str = DEFAULT_CLOUD_TOPIC,
    imu_topic: str = DEFAULT_IMU_TOPIC,
    point_timestamp_unit: str = "us",
    storage_id: str = "mcap",
) -> TimestampReport:
    """Read the selected topics from an MCAP bag without replaying it."""

    import rosbag2_py

    topic_types = {cloud_topic: PointCloud2, imu_topic: Imu}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    available = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    selected = [topic for topic in topic_types if topic in available]
    if not selected:
        expected = ", ".join(topic_types)
        raise ValueError(f"数据包中未找到要分析的话题：{expected}")
    for topic in selected:
        expected_type = (
            "sensor_msgs/msg/PointCloud2"
            if topic == cloud_topic
            else "sensor_msgs/msg/Imu"
        )
        if available[topic] != expected_type:
            raise ValueError(f"{topic} 类型为 {available[topic]}，预期为 {expected_type}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))

    def deserialized() -> Iterable[tuple[str, object, int]]:
        while reader.has_next():
            topic, data, recorded_stamp_ns = reader.read_next()
            yield topic, deserialize_message(data, topic_types[topic]), recorded_stamp_ns

    return analyze_deserialized_messages(
        deserialized(),
        cloud_topic=cloud_topic,
        imu_topic=imu_topic,
        point_timestamp_unit=point_timestamp_unit,
    )


def _format_ns(value: int | None) -> str:
    if value is None:
        return ""
    seconds, nanoseconds = divmod(int(value), 1_000_000_000)
    return f"{seconds}.{nanoseconds:09d}"


def _csv_value(value: int | float | str | None) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.9f}"
    return str(value)


def write_analysis_outputs(report: TimestampReport, output_dir: Path) -> AnalysisOutputs:
    """Write machine-readable comparison data, SVG charts, and reading guidance."""

    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = AnalysisOutputs(
        comparison_csv=output_dir / "timestamp_comparison.csv",
        summary_json=output_dir / "timestamp_summary.json",
        reading_guide=output_dir / "timestamp_reading_guide.txt",
        comparison_chart=output_dir / "timestamp_comparison.svg",
        point_coverage_chart=output_dir / "point_timestamp_coverage.svg",
    )
    _write_csv(report, outputs.comparison_csv)
    outputs.summary_json.write_text(
        json.dumps(
            {
                "point_timestamp_unit": report.point_timestamp_unit,
                "summary": asdict(report.summary),
                "time_meanings": {
                    "header_stamp": "传感器帧时间，ROS sec + nanosec",
                    "point_timestamp": "点级设备时间，单位由 point_timestamp_unit 指定",
                    "recorded_stamp": "MCAP/rosbag 接收时间，不是传感器时间",
                },
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    outputs.reading_guide.write_text(_reading_guide(report.point_timestamp_unit), encoding="utf-8")
    _write_comparison_chart(report, outputs.comparison_chart)
    _write_point_coverage_chart(report, outputs.point_coverage_chart)
    return outputs


def _write_csv(report: TimestampReport, path: Path) -> None:
    fieldnames = [
        "topic",
        "message_index",
        "header_stamp_ns",
        "recorded_stamp_ns",
        "header_stamp",
        "recorded_stamp",
        "header_delta_ms",
        "record_delta_ms",
        "record_minus_header_ms",
        "point_count",
        "point_timestamp_min_ns",
        "point_timestamp_max_ns",
        "point_timestamp_min",
        "point_timestamp_max",
        "point_time_span_ms",
        "point_min_offset_ms",
        "point_max_offset_ms",
    ]
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in report.rows:
            writer.writerow(
                {
                    "topic": row.topic,
                    "message_index": row.message_index,
                    "header_stamp_ns": row.header_stamp_ns,
                    "recorded_stamp_ns": row.recorded_stamp_ns,
                    "header_stamp": _format_ns(row.header_stamp_ns),
                    "recorded_stamp": _format_ns(row.recorded_stamp_ns),
                    "header_delta_ms": _csv_value(row.header_delta_ms),
                    "record_delta_ms": _csv_value(row.record_delta_ms),
                    "record_minus_header_ms": _csv_value(row.record_minus_header_ms),
                    "point_count": _csv_value(row.point_count),
                    "point_timestamp_min_ns": _csv_value(row.point_timestamp_min_ns),
                    "point_timestamp_max_ns": _csv_value(row.point_timestamp_max_ns),
                    "point_timestamp_min": _format_ns(row.point_timestamp_min_ns),
                    "point_timestamp_max": _format_ns(row.point_timestamp_max_ns),
                    "point_time_span_ms": _csv_value(row.point_time_span_ms),
                    "point_min_offset_ms": _csv_value(row.point_min_offset_ms),
                    "point_max_offset_ms": _csv_value(row.point_max_offset_ms),
                }
            )


def _reading_guide(point_timestamp_unit: str) -> str:
    return f"""MRDVS 时间戳读取说明

1. 点云和 IMU 同步、判断传感器是否跳秒：读取消息 header.stamp。
   它是 ROS 标准时间，使用 sec * 1_000_000_000 + nanosec 转成整数纳秒。

2. 点云帧内去畸变：读取每个点的 data.timestamp。
   当前分析按 {point_timestamp_unit!r} 解释该字段；MRDVS 当前驱动默认是 'us'（微秒）。
   点级相对帧头偏移 = point_timestamp_ns - header_stamp_ns。

3. 录制和回放积压：读取 MCAP recorded_stamp。
   它是 rosbag 接收消息的时间，不在 PointCloud2 或 Imu 消息体中，不能替代传感器时间。

不要用 ros2 topic echo 的终端输出条数判断数据包是否缺帧：大点云在回放突发时可能只漏显示。
"""


def _write_comparison_chart(report: TimestampReport, path: Path) -> None:
    cloud_rows = [row for row in report.rows if row.point_count is not None]
    imu_rows = [row for row in report.rows if row.point_count is None]
    panels = [
        ("点云：header.stamp 与 MCAP 接收间隔", cloud_rows),
        ("IMU：header.stamp 与 MCAP 接收间隔", imu_rows),
    ]
    _write_svg_panels(
        path,
        "三种时间对比：传感器帧时间与 MCAP 接收时间",
        panels,
        [("header.stamp 相邻间隔", "header_delta_ms", "#087f8c"), ("MCAP 接收间隔", "record_delta_ms", "#dc322f")],
        "间隔（ms）",
    )


def _write_point_coverage_chart(report: TimestampReport, path: Path) -> None:
    cloud_rows = [row for row in report.rows if row.point_count is not None]
    _write_svg_panels(
        path,
        "点级 timestamp 相对 PointCloud2.header.stamp 的时间范围",
        [("点云帧内点级时间", cloud_rows)],
        [("最早点偏移", "point_min_offset_ms", "#268bd2"), ("最晚点偏移", "point_max_offset_ms", "#b58900")],
        "相对帧头偏移（ms）",
    )


def _write_svg_panels(
    path: Path,
    title: str,
    panels: Sequence[tuple[str, Sequence[TimestampRow]]],
    series: Sequence[tuple[str, str, str]],
    y_label: str,
) -> None:
    width, height, margin = 1280, 260 + 270 * len(panels), 72
    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:"Noto Sans CJK SC","Droid Sans Fallback",sans-serif;fill:#243b45}.title{font-size:23px;font-weight:700}.panel{font-size:16px;font-weight:700}.label{font-size:13px}.legend{font-size:12px}</style>',
        f'<text x="{margin}" y="38" class="title">{html.escape(title)}</text>',
    ]
    plot_width = width - margin * 2
    for panel_index, (panel_title, rows) in enumerate(panels):
        top = 70 + panel_index * 270
        plot_top, plot_bottom = top + 38, top + 218
        values = [
            float(getattr(row, attribute))
            for _, attribute, _ in series
            for row in rows
            if getattr(row, attribute) is not None and math.isfinite(float(getattr(row, attribute)))
        ]
        maximum = max(values, default=1.0)
        minimum = min(0.0, min(values, default=0.0))
        if maximum <= minimum:
            maximum = minimum + 1.0
        padding = (maximum - minimum) * 0.1
        minimum -= padding
        maximum += padding
        parts.extend(
            [
                f'<text x="{margin}" y="{top + 18}" class="panel">{html.escape(panel_title)}</text>',
                f'<rect x="{margin}" y="{plot_top}" width="{plot_width}" height="{plot_bottom - plot_top}" fill="#f8fbfc" stroke="#aec2ca"/>',
                f'<text x="16" y="{(plot_top + plot_bottom) / 2}" class="label" transform="rotate(-90 16 {(plot_top + plot_bottom) / 2})">{html.escape(y_label)}</text>',
            ]
        )
        for ratio in (0.0, 0.5, 1.0):
            value = minimum + (maximum - minimum) * ratio
            y = plot_bottom - ratio * (plot_bottom - plot_top)
            parts.extend(
                [
                    f'<line x1="{margin}" y1="{y:.2f}" x2="{width - margin}" y2="{y:.2f}" stroke="#dbe7eb"/>',
                    f'<text x="{margin - 8}" y="{y + 4:.2f}" text-anchor="end" class="label">{value:.3f}</text>',
                ]
            )
        total = max(len(rows) - 1, 1)
        for legend_index, (label, attribute, color) in enumerate(series):
            coordinates = []
            for index, row in enumerate(rows):
                value = getattr(row, attribute)
                if value is None or not math.isfinite(float(value)):
                    continue
                x = margin + index / total * plot_width
                y = plot_bottom - (float(value) - minimum) / (maximum - minimum) * (plot_bottom - plot_top)
                coordinates.append(f"{x:.2f},{y:.2f}")
            if coordinates:
                parts.append(f'<polyline points="{" ".join(coordinates)}" fill="none" stroke="{color}" stroke-width="1.35"/>')
            legend_x = width - margin - 250 + legend_index * 125
            parts.extend(
                [
                    f'<line x1="{legend_x}" y1="{top + 21}" x2="{legend_x + 22}" y2="{top + 21}" stroke="{color}" stroke-width="2"/>',
                    f'<text x="{legend_x + 28}" y="{top + 25}" class="legend">{html.escape(label)}</text>',
                ]
            )
        parts.append(f'<text x="{width / 2}" y="{plot_bottom + 28}" text-anchor="middle" class="label">消息序号</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def _terminal_summary(report: TimestampReport, outputs: AnalysisOutputs) -> str:
    lines = ["时间戳分析完成："]
    for topic, count in report.summary.message_counts.items():
        header_max = report.summary.maximum_header_delta_ms.get(topic, 0.0)
        record_max = report.summary.maximum_record_delta_ms.get(topic, 0.0)
        delay_max = report.summary.maximum_record_minus_header_ms.get(topic, 0.0)
        lines.append(
            f"- {topic}: {count} 条，header 最大间隔 {header_max:.3f} ms，"
            f"MCAP 接收最大间隔 {record_max:.3f} ms，最大接收延迟 {delay_max:.3f} ms"
        )
    lines.extend(
        [
            f"- CSV：{outputs.comparison_csv}",
            f"- JSON：{outputs.summary_json}",
            f"- 图表：{outputs.comparison_chart}、{outputs.point_coverage_chart}",
            f"- 读取说明：{outputs.reading_guide}",
        ]
    )
    return "\n".join(lines)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="对比 MRDVS 点云/IMU 传感器时间、点级时间和 MCAP 接收时间。"
    )
    parser.add_argument("bag", type=Path, help="包含 metadata.yaml 的 MCAP 数据包目录")
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="输出目录；默认写入 <bag>/timestamp_analysis",
    )
    parser.add_argument("--cloud-topic", default=DEFAULT_CLOUD_TOPIC)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument(
        "--point-timestamp-unit",
        choices=sorted(_POINT_TIME_SCALES_TO_NS),
        default="us",
        help="点云 data.timestamp 的单位；当前 MRDVS 驱动使用 us（默认）",
    )
    parser.add_argument("--storage-id", default="mcap")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    output_dir = args.output_dir or args.bag / "timestamp_analysis"
    try:
        report = analyze_bag(
            args.bag,
            cloud_topic=args.cloud_topic,
            imu_topic=args.imu_topic,
            point_timestamp_unit=args.point_timestamp_unit,
            storage_id=args.storage_id,
        )
        outputs = write_analysis_outputs(report, output_dir)
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        raise SystemExit(f"时间戳分析失败：{error}") from error
    print(_terminal_summary(report, outputs))
    return 0


if __name__ == "__main__":  # pragma: no cover - package entry point
    raise SystemExit(main())
