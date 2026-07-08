#!/usr/bin/env python3
"""Analyze MRDVS ROS2 bag timing for FAST-LIVO2 debugging."""

from __future__ import annotations

import argparse
import bisect
import math
import statistics
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointField


DEFAULT_CLOUD_TOPIC = "/lx_camera_node/LxCamera_Cloud"
DEFAULT_IMU_TOPIC = "/lx_camera_node/LxCamera_Imu"
DEFAULT_IMAGE_TOPIC = "/lx_camera_node/LxCamera_Rgb"


@dataclass
class CloudTiming:
    index: int
    header_sec: float
    bag_time_sec: float
    width: int
    height: int
    raw_points: int
    valid_points: int
    mode: str
    first_raw: Optional[float]
    min_raw: Optional[float]
    max_raw: Optional[float]
    last_raw: Optional[float]
    first_rel_us: Optional[float]
    min_rel_us: Optional[float]
    max_rel_us: Optional[float]
    last_rel_us: Optional[float]
    negative_rel_count: int
    zero_rel_count: int

    @property
    def span_us(self) -> Optional[float]:
        if self.min_rel_us is None or self.max_rel_us is None:
            return None
        return self.max_rel_us - self.min_rel_us

    @property
    def min_point_sec(self) -> Optional[float]:
        if self.min_rel_us is None:
            return None
        return self.header_sec + self.min_rel_us / 1.0e6

    @property
    def first_minus_header_us(self) -> Optional[float]:
        return self.first_rel_us

    @property
    def min_minus_header_us(self) -> Optional[float]:
        return self.min_rel_us


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * pct / 100.0
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def format_ms(seconds: float) -> str:
    if not math.isfinite(seconds):
        return "nan"
    return f"{seconds * 1000.0:+.3f} ms"


def format_us(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return "nan"
    return f"{value:+.1f} us"


def describe_values(values: Sequence[float], unit: str = "") -> str:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return "n/a"
    suffix = f" {unit}" if unit else ""
    return (
        f"n={len(finite)}, min={min(finite):.6g}{suffix}, "
        f"mean={statistics.fmean(finite):.6g}{suffix}, "
        f"median={statistics.median(finite):.6g}{suffix}, "
        f"p95={percentile(finite, 95):.6g}{suffix}, max={max(finite):.6g}{suffix}"
    )


def nearest_delta(query: float, times: Sequence[float]) -> Optional[float]:
    if not times:
        return None
    pos = bisect.bisect_left(times, query)
    candidates = []
    if pos < len(times):
        candidates.append(times[pos] - query)
    if pos > 0:
        candidates.append(times[pos - 1] - query)
    return min(candidates, key=lambda x: abs(x))


def infer_storage_id(bag_path: Path) -> str:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return "sqlite3"
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        info = metadata.get("rosbag2_bagfile_information", {})
        return info.get("storage_identifier") or "sqlite3"
    except Exception:
        return "sqlite3"


def datatype_format(datatype: int) -> Optional[str]:
    return {
        PointField.INT8: "b",
        PointField.UINT8: "B",
        PointField.INT16: "h",
        PointField.UINT16: "H",
        PointField.INT32: "i",
        PointField.UINT32: "I",
        PointField.FLOAT32: "f",
        PointField.FLOAT64: "d",
    }.get(datatype)


def read_field(data: bytes, base: int, field: PointField, endian: str) -> float:
    fmt = datatype_format(field.datatype)
    if fmt is None:
        return float("nan")
    try:
        return float(struct.unpack_from(endian + fmt, data, base + field.offset)[0])
    except struct.error:
        return float("nan")


def field_by_name(fields: Iterable[PointField], name: str) -> Optional[PointField]:
    for field in fields:
        if field.name == name:
            return field
    return None


def raw_time_to_relative_us(raw_time: float, header_sec: float) -> Tuple[str, Optional[float]]:
    if not math.isfinite(raw_time):
        return "invalid", None

    header_us = header_sec * 1.0e6
    header_ns = header_sec * 1.0e9

    if abs(raw_time - header_us) < 60.0e6:
        return "absolute_us", raw_time - header_us
    if abs(raw_time - header_ns) < 60.0e9:
        return "absolute_ns", (raw_time - header_ns) / 1.0e3
    if abs(raw_time - header_sec) < 60.0:
        return "absolute_sec", (raw_time - header_sec) * 1.0e6
    if 0.0 <= raw_time < 1.0e6:
        return "offset_us", raw_time
    if 0.0 <= raw_time < 1.0e3:
        return "offset_ms", raw_time * 1.0e3
    return "unknown", None


def analyze_cloud_message(msg: Any, bag_time_sec: float, index: int, point_stride: int) -> CloudTiming:
    fields = msg.fields
    x_field = field_by_name(fields, "x")
    y_field = field_by_name(fields, "y")
    z_field = field_by_name(fields, "z")
    time_field = (
        field_by_name(fields, "timestamp")
        or field_by_name(fields, "time")
        or field_by_name(fields, "offset_time")
    )

    header_sec = stamp_to_sec(msg.header.stamp)
    raw_points = int(msg.width) * int(msg.height)
    if not x_field or not y_field or not z_field or not time_field or raw_points <= 0:
        return CloudTiming(
            index=index,
            header_sec=header_sec,
            bag_time_sec=bag_time_sec,
            width=int(msg.width),
            height=int(msg.height),
            raw_points=raw_points,
            valid_points=0,
            mode="missing_fields",
            first_raw=None,
            min_raw=None,
            max_raw=None,
            last_raw=None,
            first_rel_us=None,
            min_rel_us=None,
            max_rel_us=None,
            last_rel_us=None,
            negative_rel_count=0,
            zero_rel_count=0,
        )

    endian = ">" if msg.is_bigendian else "<"
    stride = max(1, int(point_stride))
    raw_times: List[float] = []
    rel_times: List[float] = []
    mode_counts: Dict[str, int] = {}
    negative_count = 0
    zero_count = 0

    for row in range(int(msg.height)):
        row_base = row * int(msg.row_step)
        for col in range(0, int(msg.width), stride):
            base = row_base + col * int(msg.point_step)
            x = read_field(msg.data, base, x_field, endian)
            y = read_field(msg.data, base, y_field, endian)
            z = read_field(msg.data, base, z_field, endian)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            raw = read_field(msg.data, base, time_field, endian)
            mode, rel_us = raw_time_to_relative_us(raw, header_sec)
            mode_counts[mode] = mode_counts.get(mode, 0) + 1
            if rel_us is None:
                continue
            raw_times.append(raw)
            rel_times.append(rel_us)
            if rel_us < 0.0:
                negative_count += 1
            if abs(rel_us) < 1.0:
                zero_count += 1

    mode = max(mode_counts.items(), key=lambda item: item[1])[0] if mode_counts else "none"
    return CloudTiming(
        index=index,
        header_sec=header_sec,
        bag_time_sec=bag_time_sec,
        width=int(msg.width),
        height=int(msg.height),
        raw_points=raw_points,
        valid_points=len(rel_times),
        mode=mode,
        first_raw=raw_times[0] if raw_times else None,
        min_raw=min(raw_times) if raw_times else None,
        max_raw=max(raw_times) if raw_times else None,
        last_raw=raw_times[-1] if raw_times else None,
        first_rel_us=rel_times[0] if rel_times else None,
        min_rel_us=min(rel_times) if rel_times else None,
        max_rel_us=max(rel_times) if rel_times else None,
        last_rel_us=rel_times[-1] if rel_times else None,
        negative_rel_count=negative_count,
        zero_rel_count=zero_count,
    )


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id=infer_storage_id(bag_path),
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def print_cloud_table(clouds: Sequence[CloudTiming], limit: int) -> None:
    print("\nCloud timing samples:")
    header = (
        "idx  mode          valid/raw     first-header    min-header      "
        "span        neg  zero"
    )
    print(header)
    print("-" * len(header))
    for cloud in clouds[:limit]:
        span = format_us(cloud.span_us)
        print(
            f"{cloud.index:<4} {cloud.mode:<13} "
            f"{cloud.valid_points:>6}/{cloud.raw_points:<7} "
            f"{format_us(cloud.first_minus_header_us):>14} "
            f"{format_us(cloud.min_minus_header_us):>14} "
            f"{span:>11} "
            f"{cloud.negative_rel_count:>4} {cloud.zero_rel_count:>5}"
        )


def print_delta_summary(name: str, deltas: Sequence[float]) -> None:
    finite = [d for d in deltas if d is not None and math.isfinite(d)]
    if not finite:
        print(f"{name}: n/a")
        return
    abs_values = [abs(v) for v in finite]
    print(
        f"{name}: n={len(finite)}, "
        f"median={format_ms(statistics.median(finite))}, "
        f"mean={format_ms(statistics.fmean(finite))}, "
        f"p95_abs={format_ms(percentile(abs_values, 95))}, "
        f"max_abs={format_ms(max(abs_values))}"
    )


def analyze(args: argparse.Namespace) -> int:
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        raise FileNotFoundError(f"bag path does not exist: {bag_path}")

    reader = open_reader(bag_path)
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {
        args.cloud_topic,
        args.imu_topic,
        args.image_topic,
    }
    message_types = {topic: get_message(msg_type) for topic, msg_type in topics.items() if topic in wanted}

    missing = sorted(topic for topic in wanted if topic not in topics)
    if missing:
        print("Missing requested topics:")
        for topic in missing:
            print(f"  - {topic}")
        print()

    cloud_times: List[CloudTiming] = []
    cloud_header_secs: List[float] = []
    cloud_point_start_secs: List[float] = []
    imu_secs: List[float] = []
    image_secs: List[float] = []
    topic_counts: Dict[str, int] = {}

    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if topic not in wanted or topic not in message_types:
            continue

        topic_counts[topic] = topic_counts.get(topic, 0) + 1
        msg = deserialize_message(serialized, message_types[topic])
        bag_time_sec = bag_time_ns * 1.0e-9

        if topic == args.cloud_topic:
            if len(cloud_times) < args.max_clouds:
                timing = analyze_cloud_message(
                    msg,
                    bag_time_sec=bag_time_sec,
                    index=len(cloud_times),
                    point_stride=args.point_stride,
                )
                cloud_times.append(timing)
                cloud_header_secs.append(timing.header_sec)
                if timing.min_point_sec is not None:
                    cloud_point_start_secs.append(timing.min_point_sec)
            else:
                cloud_header_secs.append(stamp_to_sec(msg.header.stamp))
        elif topic == args.imu_topic:
            imu_secs.append(stamp_to_sec(msg.header.stamp))
        elif topic == args.image_topic:
            image_secs.append(stamp_to_sec(msg.header.stamp))

    imu_secs.sort()
    image_secs.sort()
    cloud_header_secs.sort()
    cloud_point_start_secs.sort()

    print(f"Bag: {bag_path}")
    print(f"Storage: {infer_storage_id(bag_path)}")
    print("\nTopic counts:")
    for topic in [args.cloud_topic, args.imu_topic, args.image_topic]:
        print(f"  {topic}: {topic_counts.get(topic, 0)}")

    print("\nHeader rates:")
    if len(cloud_header_secs) > 1:
        cloud_dt = [b - a for a, b in zip(cloud_header_secs, cloud_header_secs[1:])]
        print(f"  cloud dt: {describe_values(cloud_dt, 's')}")
    if len(imu_secs) > 1:
        imu_dt = [b - a for a, b in zip(imu_secs, imu_secs[1:])]
        print(f"  imu dt:   {describe_values(imu_dt, 's')}")
    if len(image_secs) > 1:
        image_dt = [b - a for a, b in zip(image_secs, image_secs[1:])]
        print(f"  image dt: {describe_values(image_dt, 's')}")

    if cloud_times:
        print_cloud_table(cloud_times, args.print_clouds)
        spans = [c.span_us for c in cloud_times if c.span_us is not None]
        first_offsets = [c.first_minus_header_us for c in cloud_times if c.first_minus_header_us is not None]
        min_offsets = [c.min_minus_header_us for c in cloud_times if c.min_minus_header_us is not None]
        negative_counts = [c.negative_rel_count for c in cloud_times]
        print("\nCloud point timestamp summary:")
        print(f"  first point - cloud header: {describe_values(first_offsets, 'us')}")
        print(f"  min point   - cloud header: {describe_values(min_offsets, 'us')}")
        print(f"  point timestamp span:       {describe_values(spans, 'us')}")
        print(f"  negative relative points:   {describe_values(negative_counts)}")

    if cloud_header_secs:
        print("\nNearest-topic timing deltas, signed as other_topic - cloud_time:")
        image_from_header = [
            nearest_delta(t, image_secs) for t in cloud_header_secs if image_secs
        ]
        imu_from_header = [
            nearest_delta(t, imu_secs) for t in cloud_header_secs if imu_secs
        ]
        print_delta_summary("  image - cloud_header", image_from_header)
        print_delta_summary("  imu   - cloud_header", imu_from_header)

        if cloud_point_start_secs:
            image_from_point = [
                nearest_delta(t, image_secs) for t in cloud_point_start_secs if image_secs
            ]
            imu_from_point = [
                nearest_delta(t, imu_secs) for t in cloud_point_start_secs if imu_secs
            ]
            print_delta_summary("  image - point_start", image_from_point)
            print_delta_summary("  imu   - point_start", imu_from_point)

    print("\nInterpretation hints:")
    print("  - first/min point - cloud header should be near 0 us.")
    print("  - A stable several-ms offset means the driver header is not the real frame start.")
    print("  - Negative relative points mean the current header-based de-skew reference is late.")
    print("  - image-cloud median above about 30-50 ms is suspicious for FAST-LIVO2 during fast motion.")
    print("  - Very small point timestamp span means point-level timing may not be usable.")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze MRDVS cloud/RGB/IMU timing inside a ROS2 bag.",
    )
    parser.add_argument("bag", help="Path to a ROS2 bag directory")
    parser.add_argument("--cloud-topic", default=DEFAULT_CLOUD_TOPIC)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--max-clouds", type=int, default=200)
    parser.add_argument("--point-stride", type=int, default=1, help="Inspect every Nth point in each row")
    parser.add_argument("--print-clouds", type=int, default=12, help="Number of cloud sample rows to print")
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    return analyze(args)


if __name__ == "__main__":
    raise SystemExit(main())
