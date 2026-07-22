# MRDVS Web Collection Console Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在鲁班猫的独立目录 `/home/cat/mrdvs_collector` 部署一个可通过 WPA2 热点访问的网页控制台，用于启动 MRDVS 原始驱动、完整录制全部 ROS 2 话题、显示抽样点云和 IMU、下载/删除数据包以及控制下次开机自启。

**Architecture:** 源码作为仓库中的独立 Python 应用保存在 `tools/mrdvs_collector`，部署时同步到鲁班猫的 `/home/cat/mrdvs_collector/app`。FastAPI 负责 HTTP/WebSocket，`rclpy` 只为网页可视化订阅数据，独立 `ros2 bag record --all --include-hidden-topics` 进程负责完整录包；NetworkManager 和 systemd 操作由白名单 root 辅助程序完成。

**Tech Stack:** Python 3.12、FastAPI、Uvicorn、Pydantic 2、PyYAML、NumPy、ROS 2 Jazzy `rclpy`、rosbag2 MCAP、原生 JavaScript、Three.js、Chart.js、NetworkManager、systemd、pytest。

## Global Constraints

- 目标设备固定为 Ubuntu 24.04.4 LTS、ARM64、ROS 2 Jazzy 的 `cat@lubancat`。
- 远端独立部署根目录固定为 `/home/cat/mrdvs_collector`；现有 `/home/cat/mrdvs_ros2_ws` 仅作为 ROS 2 underlay。
- 首版只启动 `lx_camera_ros/lx_lidar_ros.launch.py`，不控制 FAST-LIO2、FAST-LIVO2 或 PGO。
- 录包必须使用 `--all --include-hidden-topics --storage mcap`，不得过滤、抽样或改写驱动消息与时间戳。
- 网页点云最多 5Hz、每帧最多 50000 点；IMU 最多 20Hz并显示最近 10 秒；这些限制只作用于网页副本。
- 数据包名称必须由用户填写；允许 Unicode 字母/数字、中文、`-`、`_`，禁止路径穿越和覆盖同名目录。
- 默认数据目录为 `/home/cat/mrdvs_collector/bags`；剩余空间低于 5GB 时正常停止录包。
- 初始热点为 `MRDVS-Collector` / `12345678`，地址 `10.42.0.1/24`，网页监听 `http://10.42.0.1`，不设置网页登录。
- 热点连接必须 `autoconnect=no`；关闭采集 target 的下次开机自启后，NetworkManager 可恢复普通 Wi-Fi 自动连接。
- 后端不得使用 `shell=True`，不得接受任意 Shell 命令；网页服务以 `cat` 运行。
- 浏览器断开不得停止驱动或录包；正常停止、服务停止和关机必须先用 `SIGINT` 让 rosbag 写完元数据。
- 所有前端资源本地提供，鲁班猫运行时不依赖互联网或 Node.js。
- 每个阶段先写失败测试、确认失败原因、最小实现、运行验证，再通过 Git 管理脚本提交、创建快照标签并推送；不得提交数据包、虚拟环境、运行状态或 NetworkManager 密钥文件。

## File Map

```text
tools/mrdvs_collector/
├── pyproject.toml                         # Python 包、运行依赖和 pytest 配置
├── package.json                           # 仅本机锁定 Three.js/Chart.js 版本
├── package-lock.json                      # 前端依赖锁文件
├── scripts/vendor_frontend.sh             # 将固定版本浏览器资源复制进 static/vendor
├── src/mrdvs_web_console/
│   ├── __init__.py
│   ├── main.py                            # Uvicorn 入口
│   ├── models.py                          # 枚举、配置和 API 共享模型
│   ├── config.py                          # JSON 配置加载、原子保存和更新
│   ├── paths.py                           # 数据包名称及真实路径安全校验
│   ├── bags.py                            # 包列表、状态、磁盘、下载命令和删除
│   ├── processes.py                       # 无 Shell 子进程、进程组和环形日志
│   ├── controller.py                      # 驱动/录包状态机及退出联动
│   ├── ros_bridge.py                      # 点云/IMU 订阅、抽样、二进制编码和频率
│   ├── system_control.py                  # 调用白名单系统辅助程序
│   ├── schemas.py                         # FastAPI 请求/响应模型
│   ├── api.py                             # REST、WebSocket、生命周期和静态页面
│   └── static/
│       ├── index.html
│       ├── styles.css
│       ├── js/api.js
│       ├── js/pointcloud.js
│       ├── js/imu.js
│       ├── js/app.js
│       └── vendor/three.module.min.js、chart.umd.min.js
├── deploy/
│   ├── run_web_console.sh                 # 加载 ROS underlay 后启动 Uvicorn
│   ├── install_lubancat.sh                # 创建独立目录、venv 和系统文件
│   ├── mrdvs_system_helper.py             # root 白名单操作，安装时改为无扩展名命令
│   ├── mrdvs-system-helper.sudoers
│   ├── mrdvs-collector.target
│   ├── mrdvs-hotspot.service
│   └── mrdvs-web-console.service
└── tests/
    ├── conftest.py
    ├── test_config_paths.py
    ├── test_bags.py
    ├── test_controller.py
    ├── test_ros_bridge.py
    ├── test_api.py
    ├── test_static_contract.py
    ├── test_system_helper.py
    └── test_deployment_contract.py

README.md                                  # 用途、操作、部署目录、恢复命令和更新记录
.gitignore                                 # 独立应用本地运行产物
```

---

### Task 1: Standalone package, configuration, and safe paths

**Files:**
- Create: `tools/mrdvs_collector/pyproject.toml`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/__init__.py`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/models.py`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/config.py`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/paths.py`
- Create: `tools/mrdvs_collector/tests/conftest.py`
- Create: `tools/mrdvs_collector/tests/test_config_paths.py`
- Modify: `.gitignore`

**Interfaces:**
- Produces: `AppConfig`, `ConfigStore.load()`, `ConfigStore.update(patch)`, `validate_bag_name(name)`, `resolve_bag_path(root, name)`.
- Consumes: no application interfaces; only Python standard library and Pydantic.

- [ ] **Step 1: Write failing configuration and path tests**

```python
# tests/conftest.py
import sys
from pathlib import Path

APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT / "src"))
```

```python
from pathlib import Path

import pytest

from mrdvs_web_console.config import ConfigStore
from mrdvs_web_console.paths import BagNameError, resolve_bag_path, validate_bag_name


def test_unicode_bag_name_is_preserved():
    assert validate_bag_name("走廊采集_01") == "走廊采集_01"


@pytest.mark.parametrize("name", ["", "  ", "../escape", "a/b", "a\\b", ".", ".."])
def test_unsafe_bag_names_are_rejected(name: str):
    with pytest.raises(BagNameError):
        validate_bag_name(name)


def test_resolved_bag_path_cannot_escape_root(tmp_path: Path):
    assert resolve_bag_path(tmp_path, "room-1") == tmp_path / "room-1"
    with pytest.raises(BagNameError):
        resolve_bag_path(tmp_path, "../outside")


def test_config_store_creates_defaults_and_updates_atomically(tmp_path: Path):
    store = ConfigStore(tmp_path / "config.json")
    config = store.load()
    assert config.bag_root == Path("/home/cat/mrdvs_collector/bags")
    assert config.min_free_bytes == 5 * 1024**3
    updated = store.update({"radar_ip": "192.168.100.83", "imu_range_level": 3})
    assert updated.radar_ip == "192.168.100.83"
    assert store.load().imu_range_level == 3
```

- [ ] **Step 2: Add packaging and exact dependencies**

```toml
[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "mrdvs-web-console"
version = "0.1.0"
requires-python = ">=3.12"
dependencies = [
  "fastapi>=0.115,<1",
  "uvicorn>=0.30,<1",
  "pydantic>=2.7,<3",
  "PyYAML>=6,<7",
  "numpy>=1.26,<3",
]

[project.optional-dependencies]
test = ["pytest>=8,<9", "pytest-asyncio>=0.23,<1", "httpx>=0.27,<1"]

[project.scripts]
mrdvs-web-console = "mrdvs_web_console.main:main"

[tool.setuptools.packages.find]
where = ["src"]

[tool.setuptools.package-data]
mrdvs_web_console = ["static/*", "static/js/*", "static/vendor/*"]

[tool.pytest.ini_options]
pythonpath = ["src"]
asyncio_mode = "auto"
```

- [ ] **Step 3: Create the local venv and confirm the focused test fails for missing implementation**

Run:

```bash
python3 -m venv --system-site-packages tools/mrdvs_collector/.venv
tools/mrdvs_collector/.venv/bin/python -m pip install --upgrade pip
tools/mrdvs_collector/.venv/bin/python -m pip install -e './tools/mrdvs_collector[test]'
tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_config_paths.py -q
```

Expected: dependencies install successfully, then test collection fails because `config.py` and `paths.py` do not exist.

- [ ] **Step 4: Implement validated models, atomic configuration, and path confinement**

```python
# models.py
from pathlib import Path
from pydantic import BaseModel, ConfigDict, Field, IPvAnyAddress


class AppConfig(BaseModel):
    model_config = ConfigDict(validate_assignment=True)
    deploy_root: Path = Path("/home/cat/mrdvs_collector")
    bag_root: Path = Path("/home/cat/mrdvs_collector/bags")
    state_root: Path = Path("/home/cat/mrdvs_collector/state")
    radar_ip: IPvAnyAddress = "192.168.100.82"
    imu_range_level: int = Field(default=2, ge=0, le=4)
    min_free_bytes: int = Field(default=5 * 1024**3, gt=0)
    pointcloud_max_points: int = Field(default=50_000, ge=1, le=200_000)
    pointcloud_max_hz: float = Field(default=5.0, gt=0, le=20)
    imu_max_hz: float = Field(default=20.0, gt=0, le=200)
    imu_window_seconds: float = Field(default=10.0, gt=0, le=120)
    allowed_origins: list[str] = Field(
        default_factory=lambda: ["http://10.42.0.1", "http://localhost"]
    )
```

```python
# paths.py
import unicodedata
from pathlib import Path


class BagNameError(ValueError):
    pass


def validate_bag_name(raw: str) -> str:
    name = unicodedata.normalize("NFC", raw.strip())
    if not name or len(name) > 80:
        raise BagNameError("数据包名称长度必须为 1 到 80 个字符")
    if name in {".", ".."} or any(ch in name for ch in "/\\"):
        raise BagNameError("数据包名称不能包含路径字符")
    if not all(ch.isalnum() or ch in "-_" for ch in name):
        raise BagNameError("数据包名称只能包含中文、字母、数字、短横线和下划线")
    return name


def resolve_bag_path(root: Path, raw_name: str) -> Path:
    root_resolved = root.expanduser().resolve()
    candidate = (root_resolved / validate_bag_name(raw_name)).resolve()
    if candidate.parent != root_resolved:
        raise BagNameError("数据包路径超出保存目录")
    return candidate
```

`ConfigStore` must serialize `Path` and IP values with `model_dump(mode="json")`, write to a same-directory temporary file, call `os.fsync()`, and replace the old file with `os.replace()` before returning a newly validated `AppConfig`.

- [ ] **Step 5: Extend ignore rules for local application artifacts**

```gitignore
# MRDVS collector local runtime
tools/mrdvs_collector/.runtime/
tools/mrdvs_collector/*.egg-info/
```

- [ ] **Step 6: Run tests and packaging checks**

Run:

```bash
tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_config_paths.py -q
tools/mrdvs_collector/.venv/bin/python -m compileall -q tools/mrdvs_collector/src
tools/mrdvs_collector/.venv/bin/python -m pip wheel --no-deps --wheel-dir /tmp/mrdvs-collector-wheel tools/mrdvs_collector
```

Expected: all tests pass, compile command exits `0`, and one `mrdvs_web_console-0.1.0-*.whl` is created.

- [ ] **Step 7: Commit the package foundation**

```bash
python /home/zero/.codex/skills/manage-git-projects/scripts/git_manager.py commit-push \
  --message "feat: scaffold standalone MRDVS collector" \
  --confirm-current-changes --allow-non-main
```

Expected: commit, `snapshot-*` tag, and push succeed; if SSH push fails, retain the local commit and report the exact warning before continuing.

---

### Task 2: Bag catalog, disk guard, download, and deletion

**Files:**
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/bags.py`
- Create: `tools/mrdvs_collector/tests/test_bags.py`

**Interfaces:**
- Consumes: `AppConfig`, `resolve_bag_path()`.
- Produces: `BagManager.prepare(name) -> Path`, `BagManager.list_bags() -> list[BagSummary]`, `BagManager.disk_status() -> DiskStatus`, `BagManager.tar_command(name) -> tuple[str, ...]`, `BagManager.delete(name, confirmation)`, `BagManager.mark_status(name, status, detail)`.

- [ ] **Step 1: Write failing bag safety and disk tests**

```python
from pathlib import Path

import pytest

from mrdvs_web_console.bags import BagConflict, BagManager, BagState


def test_prepare_refuses_existing_directory(tmp_path: Path):
    manager = BagManager(tmp_path, tmp_path / "state", min_free_bytes=1)
    (tmp_path / "room1").mkdir()
    with pytest.raises(BagConflict):
        manager.prepare("room1")


def test_tar_command_has_no_shell_and_is_confined(tmp_path: Path):
    manager = BagManager(tmp_path, tmp_path / "state", min_free_bytes=1)
    (tmp_path / "room1").mkdir()
    command = manager.tar_command("room1")
    assert command == ("tar", "--format=posix", "-C", str(tmp_path.resolve()), "-cf", "-", "room1")


def test_delete_requires_exact_confirmation(tmp_path: Path):
    manager = BagManager(tmp_path, tmp_path / "state", min_free_bytes=1)
    (tmp_path / "走廊_01").mkdir()
    with pytest.raises(ValueError):
        manager.delete("走廊_01", confirmation="走廊_02")
    manager.delete("走廊_01", confirmation="走廊_01")
    assert not (tmp_path / "走廊_01").exists()


def test_disk_guard_reports_stop_when_below_threshold(tmp_path: Path, monkeypatch):
    manager = BagManager(tmp_path, tmp_path / "state", min_free_bytes=5_000)
    monkeypatch.setattr("mrdvs_web_console.bags.shutil.disk_usage", lambda _: (10_000, 6_000, 4_000))
    assert manager.disk_status().must_stop is True
```

- [ ] **Step 2: Confirm tests fail because bag interfaces are absent**

Run: `tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_bags.py -q`

Expected: import fails for `mrdvs_web_console.bags`.

- [ ] **Step 3: Implement bag summaries, status sidecars, and safe commands**

```python
class BagState(str, Enum):
    RECORDING = "recording"
    COMPLETE = "complete"
    ERROR = "error"


@dataclass(frozen=True)
class DiskStatus:
    total_bytes: int
    used_bytes: int
    free_bytes: int
    min_free_bytes: int

    @property
    def must_stop(self) -> bool:
        return self.free_bytes < self.min_free_bytes


class BagManager:
    def prepare(self, name: str) -> Path:
        path = resolve_bag_path(self.bag_root, name)
        if path.exists():
            raise BagConflict(f"数据包已存在：{name}")
        self.bag_root.mkdir(parents=True, exist_ok=True)
        self.state_root.mkdir(parents=True, exist_ok=True)
        return path

    def tar_command(self, name: str) -> tuple[str, ...]:
        path = resolve_bag_path(self.bag_root, name)
        if not path.is_dir() or path.is_symlink():
            raise FileNotFoundError(name)
        return ("tar", "--format=posix", "-C", str(self.bag_root.resolve()), "-cf", "-", path.name)

    def delete(self, name: str, confirmation: str) -> None:
        if validate_bag_name(confirmation) != validate_bag_name(name):
            raise ValueError("二次确认名称不匹配")
        path = resolve_bag_path(self.bag_root, name)
        if path.is_symlink() or not path.is_dir():
            raise FileNotFoundError(name)
        shutil.rmtree(path)
        self.status_path(name).unlink(missing_ok=True)
```

`list_bags()` must enumerate only direct, non-symlink directories and calculate bytes without following symlinks. `mark_status()` must atomically store `name`, `state`, `detail`, `created_at`, and `updated_at` under `state/sessions/<name>.json`.

- [ ] **Step 4: Run focused tests and static checks**

Run:

```bash
tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_bags.py -q
tools/mrdvs_collector/.venv/bin/python -m compileall -q tools/mrdvs_collector/src
```

Expected: all bag tests pass and compilation exits `0`.

- [ ] **Step 5: Commit bag management**

```bash
python /home/zero/.codex/skills/manage-git-projects/scripts/git_manager.py commit-push \
  --message "feat: add safe MRDVS bag management" \
  --confirm-current-changes --allow-non-main
```

---

### Task 3: Process groups and collection state machine

**Files:**
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/processes.py`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/controller.py`
- Create: `tools/mrdvs_collector/tests/test_controller.py`

**Interfaces:**
- Consumes: `AppConfig`, `BagManager`.
- Produces: `ManagedProcess`, `SubprocessRunner`, `CollectorController.start_driver(record, bag_name)`, `start_recording(name)`, `stop_recording()`, `stop_driver()`, `shutdown()`, `snapshot()`.

- [ ] **Step 1: Write failing command and transition tests using a fake runner**

```python
import pytest

from mrdvs_web_console.controller import CollectorController, DriverState, RecordingState


@pytest.mark.asyncio
async def test_start_with_recording_prearms_bag_before_driver(controller, fake_runner):
    await controller.start_driver(record=True, bag_name="完整采集_01")
    assert fake_runner.commands[0][:4] == ["ros2", "bag", "record", "--all"]
    assert "--include-hidden-topics" in fake_runner.commands[0]
    assert fake_runner.commands[1][:4] == ["ros2", "launch", "lx_camera_ros", "lx_lidar_ros.launch.py"]
    snapshot = controller.snapshot()
    assert snapshot.driver_state is DriverState.RUNNING
    assert snapshot.recording_state is RecordingState.RECORDING


@pytest.mark.asyncio
async def test_stop_driver_stops_driver_before_rosbag(controller, fake_runner):
    await controller.start_driver(record=True, bag_name="room1")
    await controller.stop_driver()
    assert fake_runner.stop_order == ["driver", "rosbag"]


@pytest.mark.asyncio
async def test_browser_independent_snapshot_does_not_change_processes(controller, fake_runner):
    await controller.start_driver(record=False, bag_name=None)
    before = list(fake_runner.commands)
    controller.snapshot()
    assert fake_runner.commands == before
```

- [ ] **Step 2: Run the focused tests and verify missing controller failure**

Run: `tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_controller.py -q`

Expected: import fails for `mrdvs_web_console.controller`.

- [ ] **Step 3: Implement no-shell process groups and bounded logs**

```python
process = await asyncio.create_subprocess_exec(
    *command,
    stdout=asyncio.subprocess.PIPE,
    stderr=asyncio.subprocess.STDOUT,
    start_new_session=True,
    env=environment,
)
```

`ManagedProcess.stop()` must call `os.killpg(process.pid, signal.SIGINT)`, wait up to the supplied timeout, then send `SIGTERM`, and finally `SIGKILL` only if the group still exists. The reader task must decode with `errors="replace"` and retain the newest 500 lines in `collections.deque(maxlen=500)`.

- [ ] **Step 4: Implement exact command factories and controller locking**

```python
def driver_command(config: AppConfig) -> list[str]:
    return [
        "ros2", "launch", "lx_camera_ros", "lx_lidar_ros.launch.py",
        f"ip:={config.radar_ip}", "enable_rviz:=false",
        f"imu_angular_range_level:={config.imu_range_level}",
    ]


def bag_command(path: Path) -> list[str]:
    return [
        "ros2", "bag", "record", "--all", "--include-hidden-topics",
        "--storage", "mcap", "--output", str(path),
    ]
```

All public controller mutations must execute under one `asyncio.Lock`. Starting with recording must: validate/prepare the bag, start rosbag, wait 500ms and confirm it is alive, mark the session `recording`, then start the driver. If driver startup fails, stop rosbag and mark the bag `error`. Driver exit monitoring must stop active rosbag and preserve the final exit detail. User-requested driver stop with exit code `-6` is recorded as a warning rather than an active failure when no process remains.

- [ ] **Step 5: Add a disk-monitor coroutine**

```python
async def monitor_disk(self) -> None:
    while not self._shutdown_event.is_set():
        if self._recording is not None and self._bags.disk_status().must_stop:
            await self.stop_recording(reason="剩余空间低于 5GB，录制已自动停止")
        await asyncio.sleep(1.0)
```

The task must be created during application lifespan and cancelled during `shutdown()` after active processes have been stopped gracefully.

- [ ] **Step 6: Run controller tests including cancellation and timeout cases**

Run: `tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_controller.py -q`

Expected: all state, order, timeout, and unexpected-exit tests pass.

- [ ] **Step 7: Commit collection control**

```bash
python /home/zero/.codex/skills/manage-git-projects/scripts/git_manager.py commit-push \
  --message "feat: manage MRDVS driver and complete rosbag sessions" \
  --confirm-current-changes --allow-non-main
```

---

### Task 4: ROS point-cloud and IMU bridge

**Files:**
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/ros_bridge.py`
- Create: `tools/mrdvs_collector/tests/test_ros_bridge.py`

**Interfaces:**
- Consumes: ROS 2 `sensor_msgs/msg/PointCloud2` and `sensor_msgs/msg/Imu`, `AppConfig` sampling limits.
- Produces: `sample_xyzi(points, max_points)`, `encode_pointcloud(points)`, `RosBridge.latest_pointcloud()`, `latest_imu()`, `topic_status()`.

- [ ] **Step 1: Write failing pure sampling and binary-contract tests**

```python
import struct
import numpy as np

from mrdvs_web_console.ros_bridge import encode_pointcloud, sample_xyzi


def test_pointcloud_sampling_is_display_only_and_bounded():
    source = np.arange(400_000, dtype=np.float32).reshape(100_000, 4)
    source_copy = source.copy()
    sampled = sample_xyzi(source, max_points=50_000)
    assert sampled.shape == (50_000, 4)
    np.testing.assert_array_equal(source, source_copy)


def test_pointcloud_binary_frame_contains_magic_count_and_xyzi():
    points = np.array([[1.0, 2.0, 3.0, 4.0]], dtype="<f4")
    frame = encode_pointcloud(points)
    assert frame[:4] == b"MPC1"
    assert struct.unpack_from("<I", frame, 4)[0] == 1
    assert struct.unpack_from("<ffff", frame, 8) == (1.0, 2.0, 3.0, 4.0)
```

- [ ] **Step 2: Confirm tests fail because bridge functions are absent**

Run: `tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_ros_bridge.py -q`

Expected: import fails for `mrdvs_web_console.ros_bridge`.

- [ ] **Step 3: Implement deterministic sampling and binary encoding**

```python
def sample_xyzi(points: np.ndarray, max_points: int) -> np.ndarray:
    finite = points[np.isfinite(points[:, :3]).all(axis=1)]
    if len(finite) <= max_points:
        return np.ascontiguousarray(finite, dtype="<f4")
    stride = math.ceil(len(finite) / max_points)
    return np.ascontiguousarray(finite[::stride][:max_points], dtype="<f4")


def encode_pointcloud(points: np.ndarray) -> bytes:
    body = np.ascontiguousarray(points, dtype="<f4").tobytes()
    return b"MPC1" + struct.pack("<I", len(points)) + body
```

- [ ] **Step 4: Implement the ROS bridge with latest-value backpressure**

The bridge must run a `rclpy.executors.MultiThreadedExecutor` in one daemon thread. It subscribes to `/lx_camera_node/LxCamera_Cloud` and `/lx_camera_node/LxCamera_Imu`, decodes `x/y/z/intensity` with `sensor_msgs_py.point_cloud2.read_points_numpy`, and replaces one thread-safe latest point-cloud frame rather than queueing frames. IMU samples are stored in a bounded deque sized from `imu_window_seconds * imu_max_hz`. Rate counters use monotonic time and expose last-message age.

```python
self.cloud_sub = self.create_subscription(
    PointCloud2,
    "/lx_camera_node/LxCamera_Cloud",
    self._on_cloud,
    rclpy.qos.qos_profile_sensor_data,
)
self.imu_sub = self.create_subscription(
    Imu,
    "/lx_camera_node/LxCamera_Imu",
    self._on_imu,
    rclpy.qos.qos_profile_sensor_data,
)
```

The cloud callback must return immediately when less than `1 / pointcloud_max_hz` has elapsed since the last encoded display frame. It must never publish, mutate the source ROS message, or communicate with the rosbag process.

- [ ] **Step 5: Run pure tests and ROS import smoke test**

Run:

```bash
source /opt/ros/jazzy/setup.bash
tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_ros_bridge.py -q
tools/mrdvs_collector/.venv/bin/python -c "from mrdvs_web_console.ros_bridge import RosBridge; print(RosBridge.__name__)"
```

Expected: all tests pass and smoke test prints `RosBridge`.

- [ ] **Step 6: Commit the visualization bridge**

```bash
python /home/zero/.codex/skills/manage-git-projects/scripts/git_manager.py commit-push \
  --message "feat: bridge sampled MRDVS point cloud and IMU" \
  --confirm-current-changes --allow-non-main
```

---

### Task 5: FastAPI control, file, status, and WebSocket API

**Files:**
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/schemas.py`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/system_control.py`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/api.py`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/main.py`
- Create: `tools/mrdvs_collector/tests/test_api.py`

**Interfaces:**
- Consumes: `ConfigStore`, `BagManager`, `CollectorController`, `RosBridge`.
- Produces: `create_app(runtime) -> FastAPI`, REST routes under `/api`, `/ws/pointcloud`, `/ws/imu`, and `mrdvs-web-console` CLI entry point.

- [ ] **Step 1: Write failing API lifecycle and safety tests**

```python
def test_start_driver_with_recording_requires_bag_name(client):
    response = client.post("/api/driver/start", json={"record": True, "bag_name": None})
    assert response.status_code == 422


def test_start_driver_returns_runtime_snapshot(client):
    response = client.post("/api/driver/start", json={"record": False, "bag_name": None})
    assert response.status_code == 200
    assert response.json()["driver_state"] == "running"


def test_active_bag_cannot_be_downloaded_or_deleted(client, runtime):
    runtime.controller.active_bag_name = "room1"
    assert client.get("/api/bags/room1/download").status_code == 409
    assert client.request("DELETE", "/api/bags/room1", json={"confirmation": "room1"}).status_code == 409


def test_origin_is_restricted(client):
    response = client.get("/api/status", headers={"Origin": "http://evil.example"})
    assert response.headers.get("access-control-allow-origin") is None
```

- [ ] **Step 2: Run tests and verify missing API failure**

Run: `tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_api.py -q`

Expected: import fails for the API factory.

- [ ] **Step 3: Define validated request schemas**

```python
class DriverStartRequest(BaseModel):
    record: bool = False
    bag_name: str | None = None

    @model_validator(mode="after")
    def require_name_for_recording(self):
        if self.record and not self.bag_name:
            raise ValueError("同时录制时必须填写数据包名称")
        return self


class RecordingStartRequest(BaseModel):
    bag_name: str = Field(min_length=1, max_length=80)


class DeleteBagRequest(BaseModel):
    confirmation: str


class AutostartRequest(BaseModel):
    enabled: bool
```

- [ ] **Step 4: Implement the fixed system-control client**

```python
class SystemControl:
    HELPER = "/usr/local/libexec/mrdvs-system-helper"

    async def _run(self, *arguments: str) -> dict:
        process = await asyncio.create_subprocess_exec(
            "sudo", "-n", self.HELPER, *arguments,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await process.communicate()
        if process.returncode != 0:
            raise SystemControlError(stderr.decode(errors="replace").strip())
        return json.loads(stdout.decode())

    async def set_autostart(self, enabled: bool) -> dict:
        return await self._run("set-autostart", "true" if enabled else "false")

    async def configure_hotspot(self, ssid: str, password: str) -> dict:
        return await self._run("configure-hotspot", "--ssid", ssid, "--password", password)
```

Add `get_autostart()` using the `get-autostart` helper subcommand. Never include the password in exceptions, application logs, state snapshots, or settings responses.

- [ ] **Step 5: Implement REST routes and structured errors**

Required routes and status codes:

```text
GET    /api/status                    200
GET    /api/logs                      200
POST   /api/driver/start              200, 409 on illegal state
POST   /api/driver/stop               200, 409 on illegal state
POST   /api/recording/start           200, 409 on illegal state or same name
POST   /api/recording/stop            200, 409 when idle
GET    /api/bags                      200
GET    /api/bags/{name}/download      200, 404, 409 while active
DELETE /api/bags/{name}               204, 404, 409 while active
GET    /api/settings                  200; never returns hotspot password
PUT    /api/settings                  200 with validated radar/storage/display settings
PUT    /api/settings/hotspot          200; applies on next hotspot start
PUT    /api/settings/autostart        200; changes next boot only
```

Download must run `BagManager.tar_command()` with `asyncio.create_subprocess_exec`, stream stdout as `application/x-tar`, use `Content-Disposition: attachment; filename*=UTF-8''<encoded>.tar`, and terminate the tar process if the client disconnects.

Mount the package's `static/` directory at `/static`, serve `index.html` from `/`, and do not add a catch-all route that can mask invalid `/api` paths.

- [ ] **Step 6: Implement WebSockets with latest-frame semantics**

```python
@router.websocket("/ws/pointcloud")
async def pointcloud_socket(websocket: WebSocket):
    verify_websocket_origin(websocket)
    await websocket.accept()
    last_sequence = -1
    while True:
        sequence, frame = runtime.bridge.latest_pointcloud()
        if frame is not None and sequence != last_sequence:
            await websocket.send_bytes(frame)
            last_sequence = sequence
        await asyncio.sleep(0.02)
```

The IMU socket sends JSON with `stamp`, `linear_acceleration{x,y,z}`, `angular_velocity{x,y,z}`, and measured rate. A slow socket sends only the newest snapshot; it does not accumulate an unbounded queue.

- [ ] **Step 7: Implement lifespan and CLI entry point**

Application lifespan must create runtime directories, start the ROS bridge and disk monitor, and on shutdown call `controller.shutdown()` before stopping `rclpy`. `main()` loads `$MRDVS_COLLECTOR_ROOT/config/config.json`, then runs Uvicorn with host and port from environment defaults `10.42.0.1` and `80`.

- [ ] **Step 8: Run API tests and route inspection**

Run:

```bash
tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_api.py -q
tools/mrdvs_collector/.venv/bin/python -c "from mrdvs_web_console.api import create_app; print(create_app)"
```

Expected: all API tests pass and import smoke test exits `0`.

- [ ] **Step 9: Commit the web API**

```bash
python /home/zero/.codex/skills/manage-git-projects/scripts/git_manager.py commit-push \
  --message "feat: expose MRDVS collector control API" \
  --confirm-current-changes --allow-non-main
```

---

### Task 6: Mobile dashboard, point-cloud viewer, and IMU charts

**Files:**
- Create: `tools/mrdvs_collector/package.json`
- Create: `tools/mrdvs_collector/package-lock.json`
- Create: `tools/mrdvs_collector/scripts/vendor_frontend.sh`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/static/index.html`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/static/styles.css`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/static/js/api.js`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/static/js/pointcloud.js`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/static/js/imu.js`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/static/js/app.js`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/static/vendor/three.module.min.js`
- Create: `tools/mrdvs_collector/src/mrdvs_web_console/static/vendor/chart.umd.min.js`
- Create: `tools/mrdvs_collector/tests/test_static_contract.py`

**Interfaces:**
- Consumes: Task 5 REST/WebSocket paths and Task 4 `MPC1` binary frame.
- Produces: offline mobile UI with control, point cloud, IMU, bags, settings, logs, reconnect, and explicit confirmation dialogs.

- [ ] **Step 1: Write failing offline-assets and control-contract tests**

```python
from pathlib import Path

STATIC = Path("tools/mrdvs_collector/src/mrdvs_web_console/static")


def test_static_assets_never_reference_cdn():
    text = "\n".join(path.read_text(errors="ignore") for path in STATIC.rglob("*.html"))
    text += "\n" + "\n".join(path.read_text(errors="ignore") for path in STATIC.rglob("*.js"))
    assert "https://" not in text
    assert "http://" not in text


def test_index_contains_required_mobile_sections():
    html = (STATIC / "index.html").read_text()
    for element_id in ["system-status", "bag-name", "driver-control", "pointcloud-view", "imu-view", "bag-list", "settings-form", "log-view"]:
        assert f'id="{element_id}"' in html
```

- [ ] **Step 2: Confirm static contract fails because assets do not exist**

Run: `tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_static_contract.py -q`

Expected: `FileNotFoundError` for `index.html`.

- [ ] **Step 3: Pin and vendor browser dependencies locally**

```json
{
  "private": true,
  "devDependencies": {
    "chart.js": "4.4.9",
    "three": "0.176.0"
  }
}
```

`vendor_frontend.sh` must use `set -euo pipefail`, resolve its own directory, copy `node_modules/three/build/three.module.min.js` and `node_modules/chart.js/dist/chart.umd.min.js` into `static/vendor`, and fail if either source file is absent. Run `npm install --package-lock-only`, `npm ci`, then the vendor script. Commit the two vendored files and lock file; do not deploy `node_modules`.

- [ ] **Step 4: Build semantic mobile-first HTML and CSS**

The page must provide four tabs: `采集`, `实时数据`, `数据包`, `设置`. Buttons use explicit Chinese labels and disable themselves from server state rather than optimistic local state. The bag-name input has `autocomplete="off"`, `maxlength="80"`, and an adjacent “随驱动完整录制全部话题” checkbox. CSS must keep touch targets at least 44px, use a single-column layout below 760px, and never require horizontal scrolling at 360px width.

- [ ] **Step 5: Implement the API client and reconnect behavior**

```javascript
export async function request(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    ...options,
  });
  if (!response.ok) {
    const body = await response.json().catch(() => ({ detail: `HTTP ${response.status}` }));
    throw new Error(body.detail || `HTTP ${response.status}`);
  }
  return response.status === 204 ? null : response.json();
}

export function reconnectingWebSocket(path, onMessage, onState) {
  let stopped = false;
  const connect = () => {
    const socket = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}${path}`);
    socket.binaryType = "arraybuffer";
    socket.onopen = () => onState("connected");
    socket.onmessage = onMessage;
    socket.onclose = () => { if (!stopped) setTimeout(connect, 1000); };
    socket.onerror = () => socket.close();
  };
  connect();
  return () => { stopped = true; };
}
```

- [ ] **Step 6: Implement Three.js binary decoding and rendering**

`pointcloud.js` must validate the first four bytes as `MPC1`, read the little-endian count at offset 4, require exactly `8 + count * 16` bytes, and update one reusable `THREE.BufferGeometry` with position and intensity-derived color attributes. It must dispose replaced GPU buffers, expose a reset-view button, and show frame rate and displayed point count. It must not retain historical frames.

- [ ] **Step 7: Implement IMU charts and bounded history**

`imu.js` creates two Chart.js line charts, one with acceleration XYZ and one with angular velocity XYZ. Every message appends by timestamp, drops samples older than 10 seconds, updates current numeric values, and calls `chart.update("none")` to avoid animation backlog.

- [ ] **Step 8: Implement state-driven controls, bag actions, settings, and logs**

`app.js` polls `/api/status` once per second, uses the returned states to enable/disable controls, requires a nonempty bag name when recording is selected, and posts fixed JSON requests. Delete must require a modal confirmation containing the exact bag name. Password fields are blank when settings load and are sent only when the user enters a replacement. Turning off autostart must display the recovery command before confirmation.

- [ ] **Step 9: Run static, syntax, and package tests**

Run:

```bash
tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_static_contract.py -q
node --check tools/mrdvs_collector/src/mrdvs_web_console/static/js/api.js
node --check tools/mrdvs_collector/src/mrdvs_web_console/static/js/pointcloud.js
node --check tools/mrdvs_collector/src/mrdvs_web_console/static/js/imu.js
node --check tools/mrdvs_collector/src/mrdvs_web_console/static/js/app.js
tools/mrdvs_collector/.venv/bin/python -m pip wheel --no-deps --wheel-dir /tmp/mrdvs-collector-wheel tools/mrdvs_collector
```

Expected: all tests pass, all JavaScript files parse, and the built wheel contains `static/vendor`, `static/js`, HTML, and CSS.

- [ ] **Step 10: Commit the mobile dashboard**

```bash
python /home/zero/.codex/skills/manage-git-projects/scripts/git_manager.py commit-push \
  --message "feat: add offline MRDVS mobile dashboard" \
  --confirm-current-changes --allow-non-main
```

---

### Task 7: Privileged helper, hotspot, systemd, and installer

**Files:**
- Create: `tools/mrdvs_collector/deploy/mrdvs_system_helper.py`
- Create: `tools/mrdvs_collector/deploy/mrdvs-system-helper.sudoers`
- Create: `tools/mrdvs_collector/deploy/mrdvs-collector.target`
- Create: `tools/mrdvs_collector/deploy/mrdvs-hotspot.service`
- Create: `tools/mrdvs_collector/deploy/mrdvs-web-console.service`
- Create: `tools/mrdvs_collector/deploy/run_web_console.sh`
- Create: `tools/mrdvs_collector/deploy/install_lubancat.sh`
- Create: `tools/mrdvs_collector/tests/test_system_helper.py`
- Create: `tools/mrdvs_collector/tests/test_deployment_contract.py`

**Interfaces:**
- Consumes: `/home/cat/mrdvs_collector/app`, existing ROS underlay, NetworkManager.
- Produces: helper subcommands `configure-hotspot`, `hotspot-up`, `hotspot-down`, `set-autostart`, `get-autostart`; three systemd units and an idempotent installer.

- [ ] **Step 1: Write failing helper validation tests**

```python
import importlib.util
from pathlib import Path

import pytest

HELPER_PATH = Path("tools/mrdvs_collector/deploy/mrdvs_system_helper.py")
SPEC = importlib.util.spec_from_file_location("mrdvs_system_helper", HELPER_PATH)
assert SPEC is not None and SPEC.loader is not None
HELPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HELPER)
validate_password = HELPER.validate_password
validate_ssid = HELPER.validate_ssid


def test_hotspot_defaults_are_valid():
    assert validate_ssid("MRDVS-Collector") == "MRDVS-Collector"
    assert validate_password("12345678") == "12345678"


@pytest.mark.parametrize("password", ["1234567", "contains space", "x" * 64])
def test_invalid_wpa2_password_is_rejected(password: str):
    with pytest.raises(ValueError):
        validate_password(password)
```

- [ ] **Step 2: Write failing deployment contract tests**

```python
from pathlib import Path

DEPLOY = Path("tools/mrdvs_collector/deploy")


def test_web_service_is_unprivileged_and_kills_its_control_group():
    unit = (DEPLOY / "mrdvs-web-console.service").read_text()
    assert "User=cat" in unit
    assert "KillMode=control-group" in unit
    assert "AmbientCapabilities=CAP_NET_BIND_SERVICE" in unit
    assert "After=mrdvs-hotspot.service" in unit


def test_hotspot_profile_is_never_autoconnected_directly():
    helper = (DEPLOY / "mrdvs_system_helper.py").read_text()
    assert '"connection.autoconnect", "no"' in helper
```

- [ ] **Step 3: Confirm both deployment test files fail**

Run:

```bash
tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_system_helper.py tools/mrdvs_collector/tests/test_deployment_contract.py -q
```

Expected: import/file failures because deployment resources are absent.

- [ ] **Step 4: Implement the allow-listed helper without Shell execution**

The helper must use `argparse` subparsers and `subprocess.run(list_args, check=True, text=True, capture_output=True)`. `configure-hotspot` creates or modifies `mrdvs-hotspot` for `wlan0` with WPA-PSK, `ipv4.method shared`, `ipv4.addresses 10.42.0.1/24`, disabled IPv6, and `connection.autoconnect no`. `set-autostart true` runs `systemctl enable mrdvs-collector.target`; `false` runs `systemctl disable mrdvs-collector.target` without `--now`. Every successful command prints one JSON object and never prints the password.

- [ ] **Step 5: Add the exact sudoers boundary**

```sudoers
cat ALL=(root) NOPASSWD: /usr/local/libexec/mrdvs-system-helper
```

The helper file must be root-owned and mode `0755`; the sudoers file must be root-owned, mode `0440`, and validated with `visudo -cf` during install.

- [ ] **Step 6: Add systemd units with explicit dependencies**

```ini
# mrdvs-collector.target
[Unit]
Description=MRDVS handheld collector
Wants=mrdvs-hotspot.service mrdvs-web-console.service
After=NetworkManager.service

[Install]
WantedBy=multi-user.target
```

`mrdvs-hotspot.service` is `Type=oneshot`, `RemainAfterExit=yes`, `PartOf=mrdvs-collector.target`, `After=NetworkManager.service`, and calls the helper's `hotspot-up`/`hotspot-down`. `mrdvs-web-console.service` is `User=cat`, `WorkingDirectory=/home/cat/mrdvs_collector/app`, `Requires/After=mrdvs-hotspot.service`, `PartOf=mrdvs-collector.target`, `Restart=on-failure`, `KillMode=control-group`, `TimeoutStopSec=30`, and grants only `CAP_NET_BIND_SERVICE`.

- [ ] **Step 7: Add a ROS-aware launch wrapper**

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/jazzy/setup.bash
source /home/cat/mrdvs_ros2_ws/install/setup.bash
export LD_LIBRARY_PATH="/opt/MRDVS/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MRDVS_COLLECTOR_ROOT=/home/cat/mrdvs_collector
exec /home/cat/mrdvs_collector/.venv/bin/mrdvs-web-console
```

- [ ] **Step 8: Add an idempotent installer**

The installer must require execution on `aarch64`, verify `/opt/ros/jazzy/setup.bash`, `/home/cat/mrdvs_ros2_ws/install/setup.bash`, `nmcli`, `systemctl`, and the synced `app/pyproject.toml`. It creates `app`, `.venv`, `config`, `state`, and `bags` with owner `cat:cat`; creates the venv with `python3 -m venv --system-site-packages`; installs `app[test]`; installs units/helper/sudoers with `sudo install`; runs `visudo -cf`; creates the default hotspot through the helper; calls `systemctl daemon-reload`; and enables, but does not start, `mrdvs-collector.target`.

- [ ] **Step 9: Run helper and systemd contract checks locally**

Run:

```bash
tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests/test_system_helper.py tools/mrdvs_collector/tests/test_deployment_contract.py -q
systemd-analyze verify tools/mrdvs_collector/deploy/mrdvs-collector.target tools/mrdvs_collector/deploy/mrdvs-hotspot.service tools/mrdvs_collector/deploy/mrdvs-web-console.service
bash -n tools/mrdvs_collector/deploy/run_web_console.sh
bash -n tools/mrdvs_collector/deploy/install_lubancat.sh
```

Expected: tests pass, shell syntax exits `0`; `systemd-analyze` reports no unit syntax errors after providing the target on its verification search path.

- [ ] **Step 10: Commit appliance integration**

```bash
python /home/zero/.codex/skills/manage-git-projects/scripts/git_manager.py commit-push \
  --message "feat: install MRDVS collector hotspot services" \
  --confirm-current-changes --allow-non-main
```

---

### Task 8: Full local verification and user documentation

**Files:**
- Modify: `README.md`
- Modify: tests under `tools/mrdvs_collector/tests/` only when verification exposes a contract gap.

**Interfaces:**
- Consumes: all Tasks 1–7.
- Produces: one documented, locally verified release candidate ready for ARM64 deployment.

- [ ] **Step 1: Run the complete application test suite**

Run:

```bash
source /opt/ros/jazzy/setup.bash
tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests -v
tools/mrdvs_collector/.venv/bin/python -m compileall -q tools/mrdvs_collector/src tools/mrdvs_collector/deploy
```

Expected: `0 failed`, no collection errors, both compile commands exit `0`.

- [ ] **Step 2: Verify ROS CLI options used by the controller**

Run:

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag record --help
ros2 launch lx_camera_ros lx_lidar_ros.launch.py --show-args
```

Expected: help contains `--all`, `--include-hidden-topics`, `--storage`, and `--output`; launch arguments contain `ip`, `enable_rviz`, and `imu_angular_range_level`. If Jazzy spells an option differently, change the command factory and its exact test together, rerun Tasks 3 and 5 tests, and record the verified spelling in README.

- [ ] **Step 3: Run a synthetic full-recording integration test**

Start a short-lived test publisher that emits PointCloud2 and IMU at known counts, start the controller with a unique test bag, wait for 100 IMU and 10 cloud messages, stop through the controller, then run:

```bash
ros2 bag info /tmp/mrdvs-collector-integration/<name>
```

Expected: MCAP storage, both topics present, 100 IMU and 10 cloud messages, and `metadata.yaml` exists. Deserialize one cloud and assert `header.stamp`, `x/y/z/intensity/timestamp` equal the publisher values. Remove only the generated `/tmp/mrdvs-collector-integration/<name>` after assertions.

- [ ] **Step 4: Run a local HTTP/WebSocket smoke test on a nonprivileged port**

Run the app with `MRDVS_WEB_HOST=127.0.0.1 MRDVS_WEB_PORT=18080` and a temporary collector root, then use FastAPI TestClient or `curl` to verify `/`, `/api/status`, illegal recording without a name, bag list, and settings. Connect both WebSockets to the synthetic publisher and verify one valid `MPC1` frame plus IMU JSON.

Expected: HTTP status codes match Task 5, browser assets load without external URLs, and stopping the test server leaves no child process.

- [ ] **Step 5: Update the single project README**

Add concise Chinese sections covering:

```text
- 工具用途：鲁班猫手持 MRDVS 原始数据采集网页控制台
- 核心逻辑：热点、驱动、完整录包、显示副本抽样、数据包管理
- 代码结构：tools/mrdvs_collector 及远端 /home/cat/mrdvs_collector
- 使用方法：http://10.42.0.1、SSID、初始密码、填写包名、开始/停止/下载/删除
- 恢复方法：sudo systemctl enable --now mrdvs-collector.target
- 完整性说明：网页抽样不影响 ros2 bag record --all
- 更新记录：2026-07-22 新增首版网页采集工具
```

Do not copy the implementation plan into README.

- [ ] **Step 6: Run repository-level checks after documentation**

Run:

```bash
git diff --check
tools/mrdvs_collector/.venv/bin/python -m pytest tools/mrdvs_collector/tests -q
git status --short --branch
```

Expected: no whitespace errors, all tests pass, and status contains only intended Task 8 files.

- [ ] **Step 7: Commit the verified release candidate**

```bash
python /home/zero/.codex/skills/manage-git-projects/scripts/git_manager.py commit-push \
  --message "docs: document MRDVS web collector usage" \
  --confirm-current-changes --allow-non-main
```

---

### Task 9: Deploy the release candidate to LubanCat

**Files:**
- Remote create/update: `/home/cat/mrdvs_collector/app/`
- Remote create/update: `/home/cat/mrdvs_collector/.venv/`
- Remote create/update: `/home/cat/mrdvs_collector/config/`
- Remote create/update: `/home/cat/mrdvs_collector/state/`
- Remote create/update: `/home/cat/mrdvs_collector/bags/`
- Remote system files: `/usr/local/libexec/mrdvs-system-helper`, `/etc/sudoers.d/mrdvs-system-helper`, `/etc/systemd/system/mrdvs-*`

**Interfaces:**
- Consumes: committed Task 8 release candidate and existing `/home/cat/mrdvs_ros2_ws/install/setup.bash`.
- Produces: installed but not yet hotspot-switched collector on LubanCat.

- [ ] **Step 1: Invoke the deployment and interactive SSH workflows**

Read and follow `deploy-mrdvs-ros2-lubancat` and `using-ssh-interactive-remotes`; enter `ssh -tt lubancat`, then verify:

```bash
whoami
hostname
uname -m
lsb_release -ds
test -f /home/cat/mrdvs_ros2_ws/install/setup.bash
test -f /opt/MRDVS/lib/libLxCameraApi.so
```

Expected: `cat`, `lubancat`, `aarch64`, Ubuntu 24.04.4, and both file checks exit `0`.

- [ ] **Step 2: Prove the wired SSH fallback before changing wlan0**

From the local workstation open a second PTY session:

```bash
ssh -tt cat@192.168.100.100
```

Run `whoami`, `hostname`, and `ip -brief address show eth1`. Expected: `cat`, `lubancat`, and `192.168.100.100/24`. Keep this wired session open through hotspot tests.

- [ ] **Step 3: Sync only the standalone application into the new directory**

From the local repository:

```bash
ssh lubancat 'mkdir -p /home/cat/mrdvs_collector/app'
rsync -a --delete --info=stats2 \
  --exclude='.venv/' --exclude='node_modules/' --exclude='__pycache__/' --exclude='.pytest_cache/' \
  tools/mrdvs_collector/ lubancat:/home/cat/mrdvs_collector/app/
```

Expected: no webpage source is created under `/home/cat/mrdvs_ros2_ws`; existing remote `config`, `state`, and `bags` are untouched because `--delete` is scoped to `app/`.

- [ ] **Step 4: Run the idempotent installer before hotspot activation**

In the remote interactive session:

```bash
cd /home/cat/mrdvs_collector/app
bash deploy/install_lubancat.sh
```

If sudo prompts, request the password interactively and do not store it. Expected: venv created, package installed, helper/sudoers/units installed, hotspot profile configured, target enabled but inactive.

- [ ] **Step 5: Verify the remote installation and tests**

```bash
/home/cat/mrdvs_collector/.venv/bin/python -m pytest /home/cat/mrdvs_collector/app/tests -q
sudo visudo -cf /etc/sudoers.d/mrdvs-system-helper
systemd-analyze verify mrdvs-hotspot.service mrdvs-web-console.service
systemctl is-enabled mrdvs-collector.target
nmcli -g connection.autoconnect connection show mrdvs-hotspot
```

Expected: all tests pass, sudoers/systemd checks pass, target is `enabled`, hotspot autoconnect is `no`.

- [ ] **Step 6: Record the deployed Git revision**

Write the exact local `git rev-parse HEAD` value to `/home/cat/mrdvs_collector/app/DEPLOYED_GIT_SHA` and verify it matches locally. This file is a deployment artifact and is not committed.

---

### Task 10: LubanCat hotspot, real MRDVS, bag integrity, and reboot acceptance

**Files:**
- Remote runtime only: `/home/cat/mrdvs_collector/state/`, `/home/cat/mrdvs_collector/bags/`
- No project-source edits unless verification finds a reproducible defect; defects return to the relevant TDD task and receive a separate commit before redeployment.

**Interfaces:**
- Consumes: Task 9 installation, MRDVS at `192.168.100.82`, wired SSH fallback.
- Produces: evidence that the deployed appliance meets the approved design.

- [ ] **Step 1: Start the combined target from the wired SSH session**

```bash
sudo systemctl start mrdvs-collector.target
systemctl status mrdvs-hotspot.service --no-pager
systemctl status mrdvs-web-console.service --no-pager
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
curl --fail http://10.42.0.1/api/status
```

Expected: `wlan0` uses `mrdvs-hotspot`, both services are active, and API returns JSON. The Wi-Fi SSH session may close; the wired session must remain usable.

- [ ] **Step 2: Connect the workstation to the hotspot and verify the mobile route**

Save the current local Wi-Fi connection name, connect `wlp8s0` to `MRDVS-Collector` using password `12345678`, then run:

```bash
curl --fail http://10.42.0.1/
curl --fail http://10.42.0.1/api/status
```

Expected: HTML and JSON load through the hotspot. Restore the previous local Wi-Fi profile after hotspot-only browser testing unless the next test requires the hotspot.

- [ ] **Step 3: Start the real driver without recording and verify topics**

Call `POST /api/driver/start` with `{"record": false, "bag_name": null}`. On LubanCat run:

```bash
source /home/cat/mrdvs_ros2_ws/install/setup.bash
ros2 topic hz /lx_camera_node/LxCamera_Cloud
ros2 topic hz /lx_camera_node/LxCamera_Imu
ros2 topic echo --once --no-arr /lx_camera_node/LxCamera_Cloud
```

Expected: finite nonzero point-cloud and IMU rates, PointCloud2 contains `x/y/z/intensity/timestamp/row_pos/col_pos`, and API state is `running`.

- [ ] **Step 4: Verify point-cloud and IMU WebSockets without affecting the driver**

Open the webpage and confirm one rotating/zoomable intensity-colored point cloud, acceleration and angular-velocity XYZ charts, topic rates, and reconnect after a browser refresh. In parallel verify the driver PID remains unchanged and topic rates do not collapse when the browser disconnects.

- [ ] **Step 5: Record a complete named real-device bag**

Stop the driver through the API, then start it with recording enabled and bag name `web_console_smoke_01`. Keep the device still for 10 seconds, move it for 20 seconds, and stop the driver through the API. Run:

```bash
ros2 bag info /home/cat/mrdvs_collector/bags/web_console_smoke_01
```

Expected: storage `mcap`; `/lx_camera_node/LxCamera_Cloud`, `/lx_camera_node/LxCamera_Imu`, RGB and every other driver-created topic are present; duration is at least 25 seconds; session status is `complete`.

- [ ] **Step 6: Prove timestamps and point fields remain original**

During recording, run a temporary reference subscriber that writes the first cloud's `header.stamp`, minimum/maximum point `timestamp`, and the first IMU `header.stamp` to `/tmp/web_console_smoke_01_reference.json`. Read stored PointCloud2 and IMU messages with `rosbag2_py`, find the messages with those header stamps, and assert the stored cloud field list plus minimum/maximum point timestamps equal the reference JSON. Report any rosbag/DDS loss warning instead of claiming perfect capture without evidence.

- [ ] **Step 7: Verify tar download byte-for-byte**

Download `/api/bags/web_console_smoke_01/download` to `/tmp/web_console_smoke_01.tar`, extract into a new temporary directory, and compare SHA-256 for every MCAP and `metadata.yaml` against `/home/cat/mrdvs_collector/bags/web_console_smoke_01`. Expected: all hashes match and no path escapes the extraction directory.

- [ ] **Step 8: Verify protected deletion with a disposable test bag**

Create and complete `web_console_delete_test`, confirm a wrong deletion name returns `400/422`, then send the exact name and expect `204`. Verify only that bag and its state sidecar are gone; keep `web_console_smoke_01` unless the user requests removal.

- [ ] **Step 9: Verify disk guard without filling the disk**

In a test config copy, temporarily set `min_free_bytes` above current free space, start a disposable recording, and verify it stops within two monitor intervals with the low-space reason and valid metadata. Restore `5 * 1024**3` before proceeding.

- [ ] **Step 10: Verify autostart enabled across reboot**

Confirm wired SSH fallback, call the page setting for `enabled=true`, verify `systemctl is-enabled mrdvs-collector.target`, then reboot LubanCat. Reconnect over wired SSH and verify hotspot, web service, and `http://10.42.0.1/api/status` recover without driver auto-starting.

- [ ] **Step 11: Verify disabling next boot restores normal Wi-Fi**

Call the page setting for `enabled=false` and verify the current hotspot/web session remains active. Reboot, then verify `mrdvs-collector.target` is disabled, hotspot and web are inactive, and NetworkManager reconnects the saved `JLG2026625` profile. Re-enable the final desired state with:

```bash
sudo systemctl enable --now mrdvs-collector.target
```

Expected: the recovery command restores hotspot and web service. Leave autostart enabled unless the user explicitly requests the disabled final state.

- [ ] **Step 12: Run final cleanup and completion verification**

```bash
pgrep -af '[r]os2 bag record|[l]x_camera_node' || true
systemctl is-active mrdvs-web-console.service
systemctl is-enabled mrdvs-collector.target
git -C /home/zero/mrdvs_ros2_ws status --short --branch
git -C /home/zero/mrdvs_ros2_ws log -1 --oneline --decorate
```

Expected: no active driver or rosbag process, web service active, target enabled, local worktree clean, and deployed SHA matches the final committed SHA. Report actual test counts, bag topics/duration, download hash evidence, systemd state, Git commits/tags/push status, and any remaining hardware risk.
