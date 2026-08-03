import asyncio
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Protocol, Sequence

from .bags import BagManager, BagState
from .models import AppConfig


class CollectorConflict(RuntimeError):
    """Raised for an operation that is invalid in the current state."""


class DriverState(str, Enum):
    STOPPED = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    ERROR = "error"


class RecordingState(str, Enum):
    STOPPED = "stopped"
    STARTING = "starting"
    RECORDING = "recording"
    STOPPING = "stopping"
    ERROR = "error"


class RgbRecordingMode(str, Enum):
    RAW = "raw"
    COMPRESSED = "compressed"


class TopicRecordingMode(str, Enum):
    ALL = "all"
    SELECTED = "selected"


RGB_RAW_TOPIC = "/lx_camera_node/LxCamera_Rgb"
RGB_COMPRESSED_TOPIC = f"{RGB_RAW_TOPIC}/compressed"
FIXED_TOPICS: tuple[str, ...] = ("/tf", "/tf_static")
PRESET_TOPICS: tuple[str, ...] = (
    RGB_RAW_TOPIC,
    "/lx_camera_node/LxCamera_RgbInfo",
    "/lx_camera_node/LxCamera_Cloud",
    "/lx_camera_node/LxCamera_Depth",
    "/lx_camera_node/LxCamera_Amp",
    "/lx_camera_node/LxCamera_TofInfo",
    "/lx_camera_node/LxCamera_Imu",
    "/lx_camera_node/LxCamera_Error",
    "/lx_camera_node/LxCamera_Message",
    "/lx_camera_node/LxCamera_FrameRate",
    "/lx_camera_node/LxCamera_Obstacle",
    "/lx_camera_node/LxCamera_Pallet",
    "/lx_camera_node/LxCamera_TF",
)


class ProcessHandle(Protocol):
    name: str

    @property
    def alive(self) -> bool: ...

    @property
    def returncode(self) -> int | None: ...

    @property
    def logs(self) -> tuple[str, ...]: ...

    async def stop(self, timeout: float = 10.0) -> int: ...

    async def wait(self) -> int: ...


class ProcessRunner(Protocol):
    async def start(self, name: str, command: list[str]) -> ProcessHandle: ...


@dataclass(frozen=True)
class RuntimeSnapshot:
    driver_state: DriverState
    recording_state: RecordingState
    active_bag_name: str | None
    driver_returncode: int | None
    recording_returncode: int | None
    last_error: str
    last_warning: str


def driver_command(config: AppConfig) -> list[str]:
    return [
        "ros2",
        "launch",
        "lx_camera_ros",
        "lx_lidar_ros.launch.py",
        f"ip:={config.radar_ip}",
        "enable_rviz:=false",
        f"imu_angular_range_level:={config.imu_range_level}",
    ]


def resolve_topics(
    rgb_mode: RgbRecordingMode,
    topic_mode: TopicRecordingMode,
    selected_topics: Sequence[str] | None = None,
) -> tuple[str, ...]:
    """Resolve the safe, known MRDVS topic set for one recording session."""

    if topic_mode is TopicRecordingMode.ALL:
        source = list(PRESET_TOPICS)
    else:
        if not selected_topics:
            raise CollectorConflict("选择话题模式至少要勾选一个话题")
        unknown = sorted(set(selected_topics) - set(PRESET_TOPICS))
        if unknown:
            raise CollectorConflict(f"包含未预置的话题：{', '.join(unknown)}")
        source = list(dict.fromkeys(selected_topics))

    if rgb_mode is RgbRecordingMode.COMPRESSED:
        if RGB_COMPRESSED_TOPIC in source:
            source.remove(RGB_COMPRESSED_TOPIC)
        if RGB_RAW_TOPIC in source:
            source[source.index(RGB_RAW_TOPIC)] = RGB_COMPRESSED_TOPIC
        elif topic_mode is TopicRecordingMode.ALL:
            source.insert(0, RGB_COMPRESSED_TOPIC)
    elif RGB_COMPRESSED_TOPIC in source:
        raise CollectorConflict("原始 RGB 模式不能选择 compressed 话题")

    return tuple(dict.fromkeys((*source, *FIXED_TOPICS)))


def bag_command(path: Path, topics: tuple[str, ...] | None = None) -> list[str]:
    command = [
        "ros2",
        "bag",
        "record",
    ]
    if topics is None:
        command.extend(["--all"])
    else:
        command.extend(["--topics", *topics])
    command.extend(
        [
            "--include-hidden-topics",
            "--storage",
            "mcap",
            "--output",
            str(path),
        ]
    )
    return command


def compression_command() -> list[str]:
    return [
        "ros2",
        "run",
        "image_transport",
        "republish",
        "raw",
        "compressed",
        "--ros-args",
        "-r",
        f"in:={RGB_RAW_TOPIC}",
        "-r",
        f"out:={RGB_RAW_TOPIC}",
        "-p",
        "jpeg_quality:=100",
    ]


class CollectorController:
    """Serializes driver and rosbag lifecycle transitions."""

    def __init__(
        self,
        config: AppConfig,
        bags: BagManager,
        runner: ProcessRunner,
        startup_grace_seconds: float = 0.5,
    ):
        self.config = config
        self._bags = bags
        self._runner = runner
        self._startup_grace_seconds = startup_grace_seconds
        self._lock = asyncio.Lock()
        self._driver: ProcessHandle | None = None
        self._recording: ProcessHandle | None = None
        self._driver_state = DriverState.STOPPED
        self._recording_state = RecordingState.STOPPED
        self._active_bag_name: str | None = None
        self._driver_returncode: int | None = None
        self._recording_returncode: int | None = None
        self._last_error = ""
        self._last_warning = ""
        self._monitor_tasks: set[asyncio.Task[None]] = set()
        self._shutdown_event = asyncio.Event()
        self._archived_logs: list[str] = []

    @property
    def active_bag_name(self) -> str | None:
        return self._active_bag_name

    def snapshot(self) -> RuntimeSnapshot:
        return RuntimeSnapshot(
            driver_state=self._driver_state,
            recording_state=self._recording_state,
            active_bag_name=self._active_bag_name,
            driver_returncode=self._driver_returncode,
            recording_returncode=self._recording_returncode,
            last_error=self._last_error,
            last_warning=self._last_warning,
        )

    def logs(self) -> tuple[str, ...]:
        lines = list(self._archived_logs)
        if self._driver is not None:
            lines.extend(f"[driver] {line}" for line in self._driver.logs)
        if self._recording is not None:
            lines.extend(f"[rosbag] {line}" for line in self._recording.logs)
        return tuple(lines[-500:])

    async def start_driver(self, record: bool, bag_name: str | None) -> RuntimeSnapshot:
        async with self._lock:
            if self._driver is not None or self._driver_state in {
                DriverState.STARTING,
                DriverState.RUNNING,
                DriverState.STOPPING,
            }:
                raise CollectorConflict("驱动已经启动或正在切换状态")
            self._last_error = ""
            self._last_warning = ""
            if record:
                if not bag_name:
                    raise CollectorConflict("同时录制时必须填写数据包名称")
                await self._start_recording_locked(bag_name, require_driver=False)
            self._driver_state = DriverState.STARTING
            try:
                process = await self._runner.start("driver", driver_command(self.config))
                await self._startup_grace(process)
                if not process.alive:
                    raise RuntimeError(f"驱动启动后立即退出：{process.returncode}")
            except Exception as error:
                self._driver_state = DriverState.ERROR
                self._last_error = str(error)
                if self._recording is not None:
                    await self._stop_recording_locked(
                        BagState.ERROR, "驱动启动失败，录制已停止"
                    )
                raise
            self._driver = process
            self._driver_state = DriverState.RUNNING
            self._watch(self._monitor_driver(process))
            return self.snapshot()

    async def reconfigure(self, config: AppConfig, bags: BagManager) -> None:
        async with self._lock:
            if self._driver is not None or self._recording is not None:
                raise CollectorConflict("驱动或录制运行时不能修改采集配置")
            self.config = config
            self._bags = bags

    async def start_recording(self, bag_name: str) -> RuntimeSnapshot:
        async with self._lock:
            await self._start_recording_locked(bag_name, require_driver=True)
            return self.snapshot()

    async def stop_recording(self, reason: str = "用户停止录制") -> RuntimeSnapshot:
        async with self._lock:
            if self._recording is None:
                raise CollectorConflict("当前没有正在录制的数据包")
            await self._stop_recording_locked(BagState.COMPLETE, reason)
            if reason != "用户停止录制":
                self._last_warning = reason
            return self.snapshot()

    async def stop_driver(self) -> RuntimeSnapshot:
        async with self._lock:
            if self._driver is None:
                raise CollectorConflict("驱动尚未启动")
            process = self._driver
            self._driver_state = DriverState.STOPPING
            returncode = await process.stop()
            self._archive("driver", process)
            self._driver = None
            self._driver_returncode = returncode
            self._driver_state = DriverState.STOPPED
            if returncode not in {0, -2, -6}:
                self._last_warning = f"驱动停止返回代码 {returncode}"
            if self._recording is not None:
                await self._stop_recording_locked(BagState.COMPLETE, "驱动会话已停止")
            return self.snapshot()

    async def monitor_disk(self, interval_seconds: float = 1.0) -> None:
        while not self._shutdown_event.is_set():
            if self._recording is not None and self._bags.disk_status().must_stop:
                try:
                    await self.stop_recording("剩余空间低于 5GB，录制已自动停止")
                except CollectorConflict:
                    pass
            try:
                await asyncio.wait_for(
                    self._shutdown_event.wait(), timeout=interval_seconds
                )
            except TimeoutError:
                continue

    async def shutdown(self) -> None:
        self._shutdown_event.set()
        async with self._lock:
            if self._driver is not None:
                process = self._driver
                self._driver_state = DriverState.STOPPING
                self._driver_returncode = await process.stop()
                self._archive("driver", process)
                self._driver = None
                self._driver_state = DriverState.STOPPED
            if self._recording is not None:
                await self._stop_recording_locked(BagState.COMPLETE, "网页服务已停止")
        pending = [task for task in self._monitor_tasks if not task.done()]
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)

    async def _start_recording_locked(
        self, bag_name: str, require_driver: bool
    ) -> None:
        if require_driver and self._driver_state is not DriverState.RUNNING:
            raise CollectorConflict("必须先启动驱动才能开始录制")
        if self._recording is not None:
            raise CollectorConflict("当前已经在录制数据包")
        if self._bags.disk_status().must_stop:
            raise CollectorConflict("剩余空间低于 5GB，不能开始录制")
        path = self._bags.prepare(bag_name)
        self._recording_state = RecordingState.STARTING
        try:
            process = await self._runner.start("rosbag", bag_command(path))
            await self._startup_grace(process)
            if not process.alive:
                raise RuntimeError(f"rosbag 启动后立即退出：{process.returncode}")
        except Exception as error:
            self._recording_state = RecordingState.ERROR
            self._last_error = str(error)
            self._bags.mark_status(bag_name, BagState.ERROR, str(error))
            raise
        self._recording = process
        self._recording_state = RecordingState.RECORDING
        self._active_bag_name = bag_name
        self._bags.mark_status(bag_name, BagState.RECORDING, "正在完整录制全部话题")
        self._watch(self._monitor_recording(process, bag_name))

    async def _stop_recording_locked(self, final_state: BagState, detail: str) -> None:
        process = self._recording
        bag_name = self._active_bag_name
        if process is None or bag_name is None:
            return
        self._recording_state = RecordingState.STOPPING
        returncode = await process.stop()
        self._archive("rosbag", process)
        self._recording = None
        self._recording_returncode = returncode
        self._recording_state = (
            RecordingState.STOPPED
            if final_state is BagState.COMPLETE
            else RecordingState.ERROR
        )
        self._active_bag_name = None
        self._bags.mark_status(bag_name, final_state, detail)

    async def _monitor_driver(self, process: ProcessHandle) -> None:
        returncode = await process.wait()
        async with self._lock:
            if self._driver is not process or self._driver_state is DriverState.STOPPING:
                return
            self._archive("driver", process)
            self._driver = None
            self._driver_returncode = returncode
            self._driver_state = DriverState.ERROR
            self._last_error = f"驱动异常退出，代码 {returncode}"
            if self._recording is not None:
                await self._stop_recording_locked(
                    BagState.ERROR, self._last_error
                )

    async def _monitor_recording(
        self, process: ProcessHandle, bag_name: str
    ) -> None:
        returncode = await process.wait()
        async with self._lock:
            if (
                self._recording is not process
                or self._recording_state is RecordingState.STOPPING
            ):
                return
            self._archive("rosbag", process)
            self._recording = None
            self._recording_returncode = returncode
            self._recording_state = RecordingState.ERROR
            self._active_bag_name = None
            self._last_error = f"rosbag 异常退出，代码 {returncode}"
            self._bags.mark_status(bag_name, BagState.ERROR, self._last_error)

    async def _startup_grace(self, process: ProcessHandle) -> None:
        if self._startup_grace_seconds > 0:
            await asyncio.sleep(self._startup_grace_seconds)

    def _watch(self, coroutine) -> None:
        task = asyncio.create_task(coroutine)
        self._monitor_tasks.add(task)
        task.add_done_callback(self._monitor_tasks.discard)

    def _archive(self, name: str, process: ProcessHandle) -> None:
        self._archived_logs.extend(f"[{name}] {line}" for line in process.logs)
        self._archived_logs = self._archived_logs[-500:]
