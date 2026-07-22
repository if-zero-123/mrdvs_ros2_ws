import asyncio
import os
from contextlib import asynccontextmanager
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any
from urllib.parse import quote

from fastapi import APIRouter, FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .bags import BagConflict, BagManager
from .config import ConfigStore
from .controller import (
    CollectorConflict,
    CollectorController,
    DriverState,
    RecordingState,
)
from .models import AppConfig
from .paths import BagNameError
from .processes import SubprocessRunner
from .ros_bridge import RosBridge
from .schemas import (
    AutostartRequest,
    DeleteBagRequest,
    DriverStartRequest,
    HotspotSettingsRequest,
    RecordingStartRequest,
    SettingsPatch,
)
from .system_control import SystemControl, SystemControlError


@dataclass
class ApplicationRuntime:
    config_store: ConfigStore
    config: AppConfig
    bags: BagManager
    controller: CollectorController
    bridge: Any
    system_control: Any
    disk_task: asyncio.Task[None] | None = field(default=None, init=False)
    settings_lock: asyncio.Lock = field(default_factory=asyncio.Lock, init=False)

    async def start(self) -> None:
        self.bridge.start()
        self.disk_task = asyncio.create_task(self.controller.monitor_disk())

    async def stop(self) -> None:
        await self.controller.shutdown()
        if self.disk_task is not None:
            await asyncio.gather(self.disk_task, return_exceptions=True)
            self.disk_task = None
        self.bridge.stop()

    async def apply_settings(self, patch: dict[str, object]) -> AppConfig:
        async with self.settings_lock:
            current = self.config.model_dump(mode="json")
            current.update(patch)
            candidate = AppConfig.model_validate(current)
            candidate_bags = BagManager(
                candidate.bag_root,
                candidate.state_root,
                candidate.min_free_bytes,
            )
            candidate_bags.disk_status()
            await self.controller.reconfigure(candidate, candidate_bags)
            try:
                persisted = self.config_store.update(patch)
            except Exception:
                await self.controller.reconfigure(self.config, self.bags)
                raise
            self.config = persisted
            self.bags = candidate_bags
            self.bridge.reconfigure(persisted)
            return persisted


def create_runtime(root: Path | None = None) -> ApplicationRuntime:
    deploy_root = root or Path(
        os.environ.get("MRDVS_COLLECTOR_ROOT", "/home/cat/mrdvs_collector")
    )
    config_path = deploy_root / "config" / "config.json"
    config_store = ConfigStore(config_path)
    if config_path.exists():
        config = config_store.load()
    else:
        config = config_store.update(
            {
                "deploy_root": str(deploy_root),
                "bag_root": str(deploy_root / "bags"),
                "state_root": str(deploy_root / "state"),
            }
        )
    bags = BagManager(config.bag_root, config.state_root, config.min_free_bytes)
    controller = CollectorController(config, bags, SubprocessRunner())
    bridge = RosBridge(config)
    return ApplicationRuntime(
        config_store=config_store,
        config=config,
        bags=bags,
        controller=controller,
        bridge=bridge,
        system_control=SystemControl(),
    )


def create_app(runtime: ApplicationRuntime | None = None) -> FastAPI:
    active_runtime = runtime or create_runtime()

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        await active_runtime.start()
        try:
            yield
        finally:
            await active_runtime.stop()

    app = FastAPI(title="MRDVS 采集控制台", version="0.1.0", lifespan=lifespan)
    app.state.runtime = active_runtime
    app.add_middleware(
        CORSMiddleware,
        allow_origins=active_runtime.config.allowed_origins,
        allow_credentials=False,
        allow_methods=["GET", "POST", "PUT", "DELETE"],
        allow_headers=["Content-Type"],
    )
    _install_error_handlers(app)
    app.include_router(_build_router(active_runtime))
    _attach_websockets(app, active_runtime)

    static_root = Path(__file__).resolve().parent / "static"
    app.mount(
        "/static",
        StaticFiles(directory=static_root, check_dir=False),
        name="static",
    )

    @app.get("/", include_in_schema=False)
    async def index():
        index_path = static_root / "index.html"
        if index_path.is_file():
            return FileResponse(index_path)
        return JSONResponse({"service": "mrdvs-web-console", "status": "starting"})

    return app


def _install_error_handlers(app: FastAPI) -> None:
    @app.exception_handler(CollectorConflict)
    @app.exception_handler(BagConflict)
    async def conflict_handler(_: Request, error: Exception):
        return JSONResponse(status_code=409, content={"detail": str(error)})

    @app.exception_handler(BagNameError)
    @app.exception_handler(ValueError)
    async def validation_handler(_: Request, error: Exception):
        return JSONResponse(status_code=400, content={"detail": str(error)})

    @app.exception_handler(FileNotFoundError)
    async def missing_handler(_: Request, error: Exception):
        return JSONResponse(status_code=404, content={"detail": str(error)})

    @app.exception_handler(SystemControlError)
    async def system_handler(_: Request, error: Exception):
        return JSONResponse(status_code=500, content={"detail": str(error)})


def _build_router(runtime: ApplicationRuntime) -> APIRouter:
    router = APIRouter(prefix="/api")

    @router.get("/status")
    async def status():
        snapshot = asdict(runtime.controller.snapshot())
        snapshot.update(
            {
                "disk": asdict(runtime.bags.disk_status()),
                "topics": asdict(runtime.bridge.topic_status()),
                "bridge_running": runtime.bridge.running,
                "autostart_enabled": await runtime.system_control.get_autostart(),
            }
        )
        return snapshot

    @router.get("/logs")
    async def logs():
        return {"lines": runtime.controller.logs()}

    @router.post("/driver/start")
    async def start_driver(request: DriverStartRequest):
        return asdict(
            await runtime.controller.start_driver(request.record, request.bag_name)
        )

    @router.post("/driver/stop")
    async def stop_driver():
        return asdict(await runtime.controller.stop_driver())

    @router.post("/recording/start")
    async def start_recording(request: RecordingStartRequest):
        return asdict(await runtime.controller.start_recording(request.bag_name))

    @router.post("/recording/stop")
    async def stop_recording():
        return asdict(await runtime.controller.stop_recording())

    @router.get("/bags")
    async def bags():
        return [asdict(summary) for summary in runtime.bags.list_bags()]

    @router.get("/bags/{name}/download")
    async def download_bag(name: str):
        if runtime.controller.active_bag_name == name:
            raise CollectorConflict("正在录制的数据包不能下载")
        command = runtime.bags.tar_command(name)
        filename = f"{quote(name, safe='')}.tar"
        return StreamingResponse(
            _stream_process(command),
            media_type="application/x-tar",
            headers={
                "Content-Disposition": f"attachment; filename*=UTF-8''{filename}"
            },
        )

    @router.delete("/bags/{name}", status_code=204)
    async def delete_bag(name: str, request: DeleteBagRequest):
        if runtime.controller.active_bag_name == name:
            raise CollectorConflict("正在录制的数据包不能删除")
        runtime.bags.delete(name, request.confirmation)
        return Response(status_code=204)

    @router.get("/settings")
    async def get_settings():
        payload = runtime.config.model_dump(mode="json")
        payload["autostart_enabled"] = await runtime.system_control.get_autostart()
        payload["hotspot_ssid"] = "MRDVS-Collector"
        return payload

    @router.put("/settings")
    async def update_settings(request: SettingsPatch):
        patch = request.model_dump(mode="json", exclude_none=True)
        return (await runtime.apply_settings(patch)).model_dump(mode="json")

    @router.put("/settings/hotspot")
    async def update_hotspot(request: HotspotSettingsRequest):
        result = await runtime.system_control.configure_hotspot(
            request.ssid, request.password
        )
        return {
            "ssid": result.get("ssid", request.ssid),
            "applies": "next_hotspot_start",
        }

    @router.put("/settings/autostart")
    async def update_autostart(request: AutostartRequest):
        await runtime.system_control.set_autostart(request.enabled)
        return {"enabled": request.enabled, "applies": "next_boot"}

    return router


async def _stream_process(command: tuple[str, ...]):
    process = await asyncio.create_subprocess_exec(
        *command,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.DEVNULL,
    )
    assert process.stdout is not None
    try:
        while chunk := await process.stdout.read(1024 * 1024):
            yield chunk
        returncode = await process.wait()
        if returncode != 0:
            raise RuntimeError(f"数据包打包失败，代码 {returncode}")
    finally:
        if process.returncode is None:
            process.terminate()
            await process.wait()


def _websocket_origin_allowed(websocket: WebSocket, runtime: ApplicationRuntime) -> bool:
    origin = websocket.headers.get("origin")
    return origin is None or origin in runtime.config.allowed_origins


async def _wait_for_websocket_disconnect(websocket: WebSocket) -> None:
    try:
        while True:
            message = await websocket.receive()
            if message["type"] == "websocket.disconnect":
                return
    except (WebSocketDisconnect, RuntimeError):
        return


async def pointcloud_socket(websocket: WebSocket, runtime: ApplicationRuntime) -> None:
    if not _websocket_origin_allowed(websocket, runtime):
        await websocket.close(code=1008)
        return
    await websocket.accept()
    last_sequence = -1
    disconnect_task = asyncio.create_task(_wait_for_websocket_disconnect(websocket))
    try:
        while not disconnect_task.done():
            sequence, frame = runtime.bridge.latest_pointcloud()
            if frame is not None and sequence != last_sequence:
                await websocket.send_bytes(frame)
                last_sequence = sequence
            await asyncio.wait((disconnect_task,), timeout=0.02)
    except (WebSocketDisconnect, RuntimeError):
        return
    finally:
        disconnect_task.cancel()
        await asyncio.gather(disconnect_task, return_exceptions=True)


async def imu_socket(websocket: WebSocket, runtime: ApplicationRuntime) -> None:
    if not _websocket_origin_allowed(websocket, runtime):
        await websocket.close(code=1008)
        return
    await websocket.accept()
    last_sequence = -1
    disconnect_task = asyncio.create_task(_wait_for_websocket_disconnect(websocket))
    try:
        while not disconnect_task.done():
            sequence, sample = runtime.bridge.latest_imu()
            if sample is not None and sequence != last_sequence:
                await websocket.send_json(sample)
                last_sequence = sequence
            await asyncio.wait((disconnect_task,), timeout=0.01)
    except (WebSocketDisconnect, RuntimeError):
        return
    finally:
        disconnect_task.cancel()
        await asyncio.gather(disconnect_task, return_exceptions=True)


def _attach_websockets(app: FastAPI, runtime: ApplicationRuntime) -> None:
    @app.websocket("/ws/pointcloud")
    async def pointcloud_endpoint(websocket: WebSocket):
        await pointcloud_socket(websocket, runtime)

    @app.websocket("/ws/imu")
    async def imu_endpoint(websocket: WebSocket):
        await imu_socket(websocket, runtime)
