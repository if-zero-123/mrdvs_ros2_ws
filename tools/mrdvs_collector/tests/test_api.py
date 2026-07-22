import asyncio
import io
import tarfile
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from mrdvs_web_console.api import (
    ApplicationRuntime,
    create_app,
    imu_socket,
    pointcloud_socket,
)
from mrdvs_web_console.bags import BagManager, BagState
from mrdvs_web_console.config import ConfigStore
from mrdvs_web_console.controller import CollectorController
from mrdvs_web_console.ros_bridge import TopicStatus


class FakeProcess:
    def __init__(self, name: str):
        self.name = name
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
        if self._returncode is None:
            self._returncode = 0
            self._finished.set()
        return self._returncode

    async def wait(self) -> int:
        await self._finished.wait()
        return int(self._returncode)


class FakeRunner:
    def __init__(self):
        self.commands: list[tuple[str, list[str]]] = []

    async def start(self, name: str, command: list[str]) -> FakeProcess:
        self.commands.append((name, command))
        return FakeProcess(name)


class FakeBridge:
    def __init__(self):
        self.running = False
        self.config = None
        self.start_calls = 0
        self.stop_calls = 0

    def start(self) -> None:
        self.running = True
        self.start_calls += 1

    def stop(self) -> None:
        self.running = False
        self.stop_calls += 1

    def reconfigure(self, config) -> None:
        self.config = config

    def latest_pointcloud(self):
        return 1, b"MPC1\x00\x00\x00\x00"

    def latest_imu(self):
        return 1, {
            "stamp": 1.0,
            "linear_acceleration": {"x": 1.0, "y": 2.0, "z": 3.0},
            "angular_velocity": {"x": 4.0, "y": 5.0, "z": 6.0},
        }

    def topic_status(self):
        return TopicStatus(10.0, 100.0, 0.1, 0.01, "")


class FakeSystemControl:
    def __init__(self):
        self.autostart = True
        self.hotspot: tuple[str, str] | None = None

    async def get_autostart(self) -> bool:
        return self.autostart

    async def set_autostart(self, enabled: bool) -> dict:
        self.autostart = enabled
        return {"enabled": enabled}

    async def configure_hotspot(self, ssid: str, password: str) -> dict:
        self.hotspot = (ssid, password)
        return {"ssid": ssid, "applies": "next_hotspot_start"}


class DisconnectingWebSocket:
    def __init__(self):
        self.headers = {"origin": "http://testserver"}
        self.accepted = False
        self._disconnect = asyncio.Event()

    async def accept(self):
        self.accepted = True

    async def close(self, code=1000):
        self._disconnect.set()

    async def receive(self):
        await self._disconnect.wait()
        return {"type": "websocket.disconnect"}

    async def send_bytes(self, _frame):
        self._disconnect.set()

    async def send_json(self, _sample):
        self._disconnect.set()


@pytest.fixture
def runtime(tmp_path: Path) -> ApplicationRuntime:
    store = ConfigStore(tmp_path / "config" / "config.json")
    config = store.update(
        {
            "deploy_root": str(tmp_path),
            "bag_root": str(tmp_path / "bags"),
            "state_root": str(tmp_path / "state"),
            "min_free_bytes": 1,
            "allowed_origins": ["http://10.42.0.1", "http://testserver"],
        }
    )
    bags = BagManager(config.bag_root, config.state_root, config.min_free_bytes)
    controller = CollectorController(config, bags, FakeRunner(), startup_grace_seconds=0)
    bridge = FakeBridge()
    bridge.config = config
    return ApplicationRuntime(
        config_store=store,
        config=config,
        bags=bags,
        controller=controller,
        bridge=bridge,
        system_control=FakeSystemControl(),
    )


@pytest.fixture
def client(runtime: ApplicationRuntime):
    with TestClient(create_app(runtime)) as test_client:
        yield test_client


def test_lifespan_starts_and_stops_bridge(runtime: ApplicationRuntime):
    with TestClient(create_app(runtime)) as test_client:
        assert test_client.get("/api/status").status_code == 200
        assert runtime.bridge.running is True
    assert runtime.bridge.running is False
    assert runtime.bridge.start_calls == 1
    assert runtime.bridge.stop_calls == 1


def test_start_driver_with_recording_requires_bag_name(client: TestClient):
    response = client.post(
        "/api/driver/start", json={"record": True, "bag_name": None}
    )

    assert response.status_code == 422


def test_start_driver_returns_runtime_snapshot(client: TestClient):
    response = client.post(
        "/api/driver/start", json={"record": False, "bag_name": None}
    )

    assert response.status_code == 200
    assert response.json()["driver_state"] == "running"


def test_active_bag_cannot_be_downloaded_or_deleted(client: TestClient):
    started = client.post(
        "/api/driver/start", json={"record": True, "bag_name": "room1"}
    )
    assert started.status_code == 200

    assert client.get("/api/bags/room1/download").status_code == 409
    deleted = client.request(
        "DELETE", "/api/bags/room1", json={"confirmation": "room1"}
    )
    assert deleted.status_code == 409


def test_completed_bag_download_is_a_tar_stream(
    client: TestClient, runtime: ApplicationRuntime
):
    bag = runtime.bags.prepare("room1")
    bag.mkdir()
    (bag / "data.mcap").write_bytes(b"complete-bag")
    (bag / "metadata.yaml").write_text("duration: 1\n", encoding="utf-8")
    runtime.bags.mark_status("room1", BagState.COMPLETE, "录制完成")

    response = client.get("/api/bags/room1/download")

    assert response.status_code == 200
    assert response.headers["content-type"].startswith("application/x-tar")
    with tarfile.open(fileobj=io.BytesIO(response.content), mode="r:") as archive:
        member = archive.extractfile("room1/data.mcap")
        assert member is not None
        assert member.read() == b"complete-bag"


def test_delete_requires_exact_confirmation(client: TestClient, runtime):
    bag = runtime.bags.prepare("delete_me")
    bag.mkdir()

    wrong = client.request(
        "DELETE", "/api/bags/delete_me", json={"confirmation": "wrong"}
    )
    correct = client.request(
        "DELETE", "/api/bags/delete_me", json={"confirmation": "delete_me"}
    )

    assert wrong.status_code == 400
    assert correct.status_code == 204
    assert not bag.exists()


def test_settings_update_next_driver_command_and_bag_root(client, runtime, tmp_path):
    new_root = tmp_path / "new-bags"

    response = client.put(
        "/api/settings",
        json={
            "radar_ip": "192.168.100.83",
            "imu_range_level": 3,
            "bag_root": str(new_root),
        },
    )

    assert response.status_code == 200
    assert str(runtime.controller.config.radar_ip) == "192.168.100.83"
    assert runtime.bags.bag_root == new_root
    assert response.json()["radar_ip"] == "192.168.100.83"


def test_hotspot_password_is_applied_but_never_returned(client, runtime):
    response = client.put(
        "/api/settings/hotspot",
        json={"ssid": "MRDVS-Collector", "password": "12345678"},
    )

    assert response.status_code == 200
    assert runtime.system_control.hotspot == ("MRDVS-Collector", "12345678")
    assert "12345678" not in response.text


def test_autostart_setting_changes_next_boot_only(client, runtime):
    response = client.put("/api/settings/autostart", json={"enabled": False})

    assert response.status_code == 200
    assert response.json() == {"enabled": False, "applies": "next_boot"}
    assert runtime.system_control.autostart is False
    assert runtime.bridge.running is True


def test_cors_does_not_authorize_unknown_origin(client: TestClient):
    response = client.get(
        "/api/status", headers={"Origin": "http://untrusted.example"}
    )

    assert response.headers.get("access-control-allow-origin") is None


def test_websockets_publish_binary_cloud_and_imu_json(client: TestClient):
    with client.websocket_connect(
        "/ws/pointcloud", headers={"origin": "http://testserver"}
    ) as socket:
        assert socket.receive_bytes() == b"MPC1\x00\x00\x00\x00"

    with client.websocket_connect(
        "/ws/imu", headers={"origin": "http://testserver"}
    ) as socket:
        assert socket.receive_json()["angular_velocity"]["z"] == 6.0


@pytest.mark.asyncio
async def test_pointcloud_socket_exits_after_client_disconnect(runtime):
    websocket = DisconnectingWebSocket()

    await asyncio.wait_for(pointcloud_socket(websocket, runtime), timeout=0.25)

    assert websocket.accepted is True


@pytest.mark.asyncio
async def test_imu_socket_exits_after_client_disconnect(runtime):
    websocket = DisconnectingWebSocket()

    await asyncio.wait_for(imu_socket(websocket, runtime), timeout=0.25)

    assert websocket.accepted is True
