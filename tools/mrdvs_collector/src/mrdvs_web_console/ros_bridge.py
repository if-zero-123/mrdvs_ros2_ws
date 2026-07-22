import copy
import math
import struct
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2

from .models import AppConfig


@dataclass(frozen=True)
class TopicStatus:
    cloud_hz: float
    imu_hz: float
    cloud_age_seconds: float | None
    imu_age_seconds: float | None
    last_error: str


class _RateWindow:
    def __init__(self, window_seconds: float = 2.0):
        self.window_seconds = window_seconds
        self._samples: deque[float] = deque(maxlen=1000)

    def add(self, timestamp: float) -> None:
        self._samples.append(timestamp)
        cutoff = timestamp - self.window_seconds
        while len(self._samples) > 1 and self._samples[0] < cutoff:
            self._samples.popleft()

    def hz(self) -> float:
        if len(self._samples) < 2:
            return 0.0
        elapsed = self._samples[-1] - self._samples[0]
        return (len(self._samples) - 1) / elapsed if elapsed > 0 else 0.0


def sample_xyzi(points: np.ndarray, max_points: int) -> np.ndarray:
    values = np.asarray(points)
    if values.ndim != 2 or values.shape[1] != 4:
        raise ValueError("点云显示数组必须为 N×4 的 XYZI")
    finite = values[np.isfinite(values[:, :3]).all(axis=1)]
    if len(finite) > max_points:
        stride = math.ceil(len(finite) / max_points)
        finite = finite[::stride][:max_points]
    return np.ascontiguousarray(finite, dtype="<f4")


def encode_pointcloud(points: np.ndarray) -> bytes:
    values = np.ascontiguousarray(points, dtype="<f4")
    if values.ndim != 2 or values.shape[1] != 4:
        raise ValueError("点云显示数组必须为 N×4 的 XYZI")
    return b"MPC1" + struct.pack("<I", len(values)) + values.tobytes()


def decode_cloud_xyzi(message: PointCloud2) -> np.ndarray:
    available = {field.name for field in message.fields}
    missing = {"x", "y", "z"} - available
    if missing:
        raise ValueError(f"PointCloud2 缺少字段：{', '.join(sorted(missing))}")
    selected = ["x", "y", "z"]
    has_intensity = "intensity" in available
    if has_intensity:
        selected.append("intensity")
    decoded = point_cloud2.read_points(
        message,
        field_names=selected,
        skip_nans=False,
        reshape_organized_cloud=False,
    )
    if decoded.dtype.names:
        columns = [np.asarray(decoded[name], dtype=np.float32) for name in selected]
        values = np.column_stack(columns)
    else:
        values = np.asarray(decoded, dtype=np.float32).reshape(-1, len(selected))
    if not has_intensity:
        values = np.column_stack(
            [values, np.zeros((len(values),), dtype=np.float32)]
        )
    return np.ascontiguousarray(values, dtype=np.float32)


class RosBridge:
    """Keeps bounded display copies of MRDVS topics for browser clients."""

    def __init__(
        self,
        config: AppConfig,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ):
        self.config = config
        self._clock = monotonic_clock
        self._lock = threading.RLock()
        self._latest_cloud: bytes | None = None
        self._cloud_sequence = 0
        self._latest_imu: dict[str, Any] | None = None
        self._imu_sequence = 0
        history_size = max(
            1, math.ceil(config.imu_window_seconds * config.imu_max_hz) + 1
        )
        self._imu_history: deque[dict[str, Any]] = deque(maxlen=history_size)
        self._cloud_rate = _RateWindow()
        self._imu_rate = _RateWindow()
        self._last_cloud_received: float | None = None
        self._last_imu_received: float | None = None
        self._last_cloud_display: float | None = None
        self._last_imu_display: float | None = None
        self._last_error = ""
        self._node: _BridgeNode | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._thread: threading.Thread | None = None
        self._owns_rclpy_context = False

    @property
    def running(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def handle_cloud(self, message: PointCloud2) -> None:
        received_at = self._clock()
        with self._lock:
            self._cloud_rate.add(received_at)
            self._last_cloud_received = received_at
            minimum_period = 1.0 / self.config.pointcloud_max_hz
            if (
                self._last_cloud_display is not None
                and received_at - self._last_cloud_display + 1e-9 < minimum_period
            ):
                return
        try:
            decoded = decode_cloud_xyzi(message)
            sampled = sample_xyzi(decoded, self.config.pointcloud_max_points)
            encoded = encode_pointcloud(sampled)
        except Exception as error:
            with self._lock:
                self._last_error = f"点云显示解码失败：{error}"
            return
        with self._lock:
            self._latest_cloud = encoded
            self._cloud_sequence += 1
            self._last_cloud_display = received_at
            self._last_error = ""

    def handle_imu(self, message: Imu) -> None:
        received_at = self._clock()
        with self._lock:
            self._imu_rate.add(received_at)
            self._last_imu_received = received_at
            minimum_period = 1.0 / self.config.imu_max_hz
            if (
                self._last_imu_display is not None
                and received_at - self._last_imu_display + 1e-9 < minimum_period
            ):
                return
            sample = {
                "stamp": message.header.stamp.sec
                + message.header.stamp.nanosec / 1_000_000_000,
                "linear_acceleration": {
                    "x": message.linear_acceleration.x,
                    "y": message.linear_acceleration.y,
                    "z": message.linear_acceleration.z,
                },
                "angular_velocity": {
                    "x": message.angular_velocity.x,
                    "y": message.angular_velocity.y,
                    "z": message.angular_velocity.z,
                },
            }
            self._latest_imu = sample
            self._imu_history.append(sample)
            self._imu_sequence += 1
            self._last_imu_display = received_at

    def latest_pointcloud(self) -> tuple[int, bytes | None]:
        with self._lock:
            return self._cloud_sequence, self._latest_cloud

    def reconfigure(self, config: AppConfig) -> None:
        with self._lock:
            history_size = max(
                1, math.ceil(config.imu_window_seconds * config.imu_max_hz) + 1
            )
            newest_samples = list(self._imu_history)[-history_size:]
            self.config = config
            self._imu_history = deque(newest_samples, maxlen=history_size)

    def latest_imu(self) -> tuple[int, dict[str, Any] | None]:
        with self._lock:
            return self._imu_sequence, copy.deepcopy(self._latest_imu)

    def imu_history(self) -> tuple[dict[str, Any], ...]:
        with self._lock:
            return tuple(copy.deepcopy(list(self._imu_history)))

    def topic_status(self) -> TopicStatus:
        now = self._clock()
        with self._lock:
            return TopicStatus(
                cloud_hz=self._cloud_rate.hz(),
                imu_hz=self._imu_rate.hz(),
                cloud_age_seconds=(
                    None
                    if self._last_cloud_received is None
                    else max(0.0, now - self._last_cloud_received)
                ),
                imu_age_seconds=(
                    None
                    if self._last_imu_received is None
                    else max(0.0, now - self._last_imu_received)
                ),
                last_error=self._last_error,
            )

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_rclpy_context = True
            self._node = _BridgeNode(self)
            self._executor = MultiThreadedExecutor(num_threads=2)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(
                target=self._executor.spin,
                name="mrdvs-ros-bridge",
                daemon=True,
            )
            self._thread.start()

    def stop(self) -> None:
        with self._lock:
            executor = self._executor
            node = self._node
            thread = self._thread
            self._executor = None
            self._node = None
            self._thread = None
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
            if node is not None:
                executor.remove_node(node)
        if node is not None:
            node.destroy_node()
        if thread is not None:
            thread.join(timeout=2.0)
        if self._owns_rclpy_context and rclpy.ok():
            rclpy.shutdown()
            self._owns_rclpy_context = False


class _BridgeNode(Node):
    def __init__(self, bridge: RosBridge):
        super().__init__("mrdvs_web_console_bridge")
        self._bridge = bridge
        self._cloud_subscription = self.create_subscription(
            PointCloud2,
            "/lx_camera_node/LxCamera_Cloud",
            bridge.handle_cloud,
            qos_profile_sensor_data,
        )
        self._imu_subscription = self.create_subscription(
            Imu,
            "/lx_camera_node/LxCamera_Imu",
            bridge.handle_imu,
            qos_profile_sensor_data,
        )
