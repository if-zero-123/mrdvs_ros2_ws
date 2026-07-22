import asyncio
from pathlib import Path

import pytest

from mrdvs_web_console.bags import BagManager, BagState, DiskStatus
from mrdvs_web_console.controller import (
    CollectorConflict,
    CollectorController,
    DriverState,
    RecordingState,
    bag_command,
    driver_command,
)
from mrdvs_web_console.models import AppConfig


class FakeProcess:
    def __init__(self, name: str, stop_order: list[str]):
        self.name = name
        self._stop_order = stop_order
        self._returncode: int | None = None
        self._finished = asyncio.Event()
        self.logs: tuple[str, ...] = ()

    @property
    def alive(self) -> bool:
        return self._returncode is None

    @property
    def returncode(self) -> int | None:
        return self._returncode

    async def stop(self, timeout: float = 10.0) -> int:
        self._stop_order.append(self.name)
        self.exit(-6 if self.name == "driver" else 0)
        return int(self._returncode)

    async def wait(self) -> int:
        await self._finished.wait()
        return int(self._returncode)

    def exit(self, returncode: int) -> None:
        if self._returncode is None:
            self._returncode = returncode
            self._finished.set()


class FakeRunner:
    def __init__(self):
        self.commands: list[tuple[str, list[str]]] = []
        self.processes: dict[str, FakeProcess] = {}
        self.stop_order: list[str] = []

    async def start(self, name: str, command: list[str]) -> FakeProcess:
        self.commands.append((name, command))
        process = FakeProcess(name, self.stop_order)
        self.processes[name] = process
        return process


def make_controller(tmp_path: Path) -> tuple[CollectorController, FakeRunner, BagManager]:
    config = AppConfig(
        deploy_root=tmp_path,
        bag_root=tmp_path / "bags",
        state_root=tmp_path / "state",
    )
    bags = BagManager(config.bag_root, config.state_root, config.min_free_bytes)
    runner = FakeRunner()
    controller = CollectorController(
        config=config,
        bags=bags,
        runner=runner,
        startup_grace_seconds=0,
    )
    return controller, runner, bags


@pytest.mark.asyncio
async def test_start_with_recording_prearms_bag_before_driver(tmp_path: Path):
    controller, runner, _ = make_controller(tmp_path)

    await controller.start_driver(record=True, bag_name="完整采集_01")

    assert runner.commands[0][0] == "rosbag"
    assert runner.commands[0][1][:4] == ["ros2", "bag", "record", "--all"]
    assert "--include-hidden-topics" in runner.commands[0][1]
    assert runner.commands[1][0] == "driver"
    assert runner.commands[1][1][:4] == [
        "ros2",
        "launch",
        "lx_camera_ros",
        "lx_lidar_ros.launch.py",
    ]
    snapshot = controller.snapshot()
    assert snapshot.driver_state is DriverState.RUNNING
    assert snapshot.recording_state is RecordingState.RECORDING
    assert snapshot.active_bag_name == "完整采集_01"
    await controller.shutdown()


@pytest.mark.asyncio
async def test_start_recording_requires_running_driver(tmp_path: Path):
    controller, _, _ = make_controller(tmp_path)

    with pytest.raises(CollectorConflict):
        await controller.start_recording("room1")

    await controller.shutdown()


@pytest.mark.asyncio
async def test_stop_driver_stops_driver_before_rosbag(tmp_path: Path):
    controller, runner, _ = make_controller(tmp_path)
    await controller.start_driver(record=True, bag_name="room1")

    await controller.stop_driver()

    assert runner.stop_order == ["driver", "rosbag"]
    snapshot = controller.snapshot()
    assert snapshot.driver_state is DriverState.STOPPED
    assert snapshot.recording_state is RecordingState.STOPPED
    assert snapshot.active_bag_name is None
    await controller.shutdown()


@pytest.mark.asyncio
async def test_snapshot_does_not_start_or_stop_processes(tmp_path: Path):
    controller, runner, _ = make_controller(tmp_path)
    await controller.start_driver(record=False, bag_name=None)
    commands_before = list(runner.commands)
    stop_order_before = list(runner.stop_order)

    snapshot = controller.snapshot()

    assert snapshot.driver_state is DriverState.RUNNING
    assert runner.commands == commands_before
    assert runner.stop_order == stop_order_before
    await controller.shutdown()


@pytest.mark.asyncio
async def test_unexpected_driver_exit_stops_and_marks_active_bag_error(tmp_path: Path):
    controller, runner, bags = make_controller(tmp_path)
    await controller.start_driver(record=True, bag_name="room1")

    runner.processes["driver"].exit(7)
    await asyncio.sleep(0)
    await asyncio.sleep(0)

    snapshot = controller.snapshot()
    assert snapshot.driver_state is DriverState.ERROR
    assert snapshot.recording_state is RecordingState.ERROR
    assert runner.stop_order == ["rosbag"]
    summary_state = bags._load_status("room1")["state"]
    assert summary_state is BagState.ERROR
    await controller.shutdown()


@pytest.mark.asyncio
async def test_disk_monitor_stops_recording_below_threshold(tmp_path: Path, monkeypatch):
    controller, _, bags = make_controller(tmp_path)
    await controller.start_driver(record=True, bag_name="room1")
    monkeypatch.setattr(
        bags,
        "disk_status",
        lambda: DiskStatus(10_000, 6_000, 4_000, 5_000),
    )

    monitor = asyncio.create_task(controller.monitor_disk(interval_seconds=0.001))
    for _ in range(20):
        if controller.snapshot().recording_state is RecordingState.STOPPED:
            break
        await asyncio.sleep(0.001)
    monitor.cancel()
    await asyncio.gather(monitor, return_exceptions=True)

    snapshot = controller.snapshot()
    assert snapshot.driver_state is DriverState.RUNNING
    assert snapshot.recording_state is RecordingState.STOPPED
    assert "剩余空间低于" in snapshot.last_warning
    await controller.shutdown()


def test_exact_command_factories(tmp_path: Path):
    config = AppConfig(
        deploy_root=tmp_path,
        bag_root=tmp_path / "bags",
        state_root=tmp_path / "state",
        radar_ip="192.168.100.82",
        imu_range_level=2,
    )

    assert driver_command(config) == [
        "ros2",
        "launch",
        "lx_camera_ros",
        "lx_lidar_ros.launch.py",
        "ip:=192.168.100.82",
        "enable_rviz:=false",
        "imu_angular_range_level:=2",
    ]
    assert bag_command(tmp_path / "bags" / "room1") == [
        "ros2",
        "bag",
        "record",
        "--all",
        "--include-hidden-topics",
        "--storage",
        "mcap",
        "--output",
        str(tmp_path / "bags" / "room1"),
    ]


@pytest.mark.asyncio
async def test_reconfigure_updates_next_driver_and_bag_settings_only_when_idle(
    tmp_path: Path,
):
    controller, runner, _ = make_controller(tmp_path)
    new_config = AppConfig(
        deploy_root=tmp_path,
        bag_root=tmp_path / "new-bags",
        state_root=tmp_path / "new-state",
        radar_ip="192.168.100.83",
        min_free_bytes=1,
    )
    new_bags = BagManager(
        new_config.bag_root, new_config.state_root, new_config.min_free_bytes
    )

    await controller.reconfigure(new_config, new_bags)
    await controller.start_driver(record=False, bag_name=None)

    assert "ip:=192.168.100.83" in runner.commands[-1][1]
    with pytest.raises(CollectorConflict):
        await controller.reconfigure(new_config, new_bags)
    await controller.shutdown()
