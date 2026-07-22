import json
import os
import shutil
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path

import yaml

from .paths import BagNameError, resolve_bag_path, validate_bag_name


class BagConflict(RuntimeError):
    """Raised when a requested bag name already exists."""


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


@dataclass(frozen=True)
class BagSummary:
    name: str
    size_bytes: int
    state: BagState
    detail: str
    created_at: str
    updated_at: str
    duration_seconds: float | None


class BagManager:
    """Owns all filesystem operations below one configured bag root."""

    def __init__(self, bag_root: Path, state_root: Path, min_free_bytes: int):
        self.bag_root = bag_root.expanduser()
        self.state_root = state_root.expanduser()
        self.min_free_bytes = min_free_bytes

    def prepare(self, name: str) -> Path:
        self.bag_root.mkdir(parents=True, exist_ok=True)
        self.state_root.mkdir(parents=True, exist_ok=True)
        path = resolve_bag_path(self.bag_root, name)
        if path.exists() or path.is_symlink():
            raise BagConflict(f"数据包已存在：{validate_bag_name(name)}")
        return path

    def status_path(self, name: str) -> Path:
        safe_name = validate_bag_name(name)
        return self.state_root / "sessions" / f"{safe_name}.json"

    def mark_status(self, name: str, state: BagState, detail: str) -> None:
        status_path = self.status_path(name)
        now = datetime.now(timezone.utc).isoformat()
        created_at = now
        if status_path.is_file():
            try:
                previous = json.loads(status_path.read_text(encoding="utf-8"))
                created_at = str(previous.get("created_at", now))
            except (OSError, ValueError, TypeError):
                created_at = now
        payload = {
            "name": validate_bag_name(name),
            "state": state.value,
            "detail": detail,
            "created_at": created_at,
            "updated_at": now,
        }
        self._write_json(status_path, payload)

    def list_bags(self) -> list[BagSummary]:
        self.bag_root.mkdir(parents=True, exist_ok=True)
        summaries: list[BagSummary] = []
        for entry in sorted(self.bag_root.iterdir(), key=lambda item: item.name):
            if entry.is_symlink() or not entry.is_dir():
                continue
            status = self._load_status(entry.name)
            metadata = self._load_metadata(entry)
            fallback_time = datetime.fromtimestamp(
                entry.stat().st_mtime, timezone.utc
            ).isoformat()
            summaries.append(
                BagSummary(
                    name=entry.name,
                    size_bytes=self._directory_size(entry),
                    state=status["state"],
                    detail=status["detail"],
                    created_at=status["created_at"] or fallback_time,
                    updated_at=status["updated_at"] or fallback_time,
                    duration_seconds=metadata,
                )
            )
        return summaries

    def disk_status(self) -> DiskStatus:
        self.bag_root.mkdir(parents=True, exist_ok=True)
        usage = shutil.disk_usage(self.bag_root)
        return DiskStatus(
            total_bytes=usage.total,
            used_bytes=usage.used,
            free_bytes=usage.free,
            min_free_bytes=self.min_free_bytes,
        )

    def tar_command(self, name: str) -> tuple[str, ...]:
        try:
            path = resolve_bag_path(self.bag_root, name)
        except BagNameError as error:
            raise FileNotFoundError(name) from error
        if path.is_symlink() or not path.is_dir():
            raise FileNotFoundError(name)
        return (
            "tar",
            "--format=posix",
            "-C",
            str(self.bag_root.resolve()),
            "-cf",
            "-",
            path.name,
        )

    def delete(self, name: str, confirmation: str) -> None:
        safe_name = validate_bag_name(name)
        if validate_bag_name(confirmation) != safe_name:
            raise ValueError("二次确认名称不匹配")
        path = resolve_bag_path(self.bag_root, safe_name)
        if path.is_symlink() or not path.is_dir():
            raise FileNotFoundError(name)
        shutil.rmtree(path)
        self.status_path(safe_name).unlink(missing_ok=True)

    def _load_status(self, name: str) -> dict[str, object]:
        status_path = self.status_path(name)
        if not status_path.is_file():
            metadata_exists = (self.bag_root / name / "metadata.yaml").is_file()
            return {
                "state": BagState.COMPLETE if metadata_exists else BagState.ERROR,
                "detail": "已完成" if metadata_exists else "缺少录包元数据",
                "created_at": "",
                "updated_at": "",
            }
        try:
            payload = json.loads(status_path.read_text(encoding="utf-8"))
            return {
                "state": BagState(payload["state"]),
                "detail": str(payload.get("detail", "")),
                "created_at": str(payload.get("created_at", "")),
                "updated_at": str(payload.get("updated_at", "")),
            }
        except (OSError, ValueError, KeyError, TypeError):
            return {
                "state": BagState.ERROR,
                "detail": "状态文件损坏",
                "created_at": "",
                "updated_at": "",
            }

    @staticmethod
    def _load_metadata(bag_path: Path) -> float | None:
        metadata_path = bag_path / "metadata.yaml"
        if not metadata_path.is_file():
            return None
        try:
            payload = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
            info = payload.get("rosbag2_bagfile_information", payload)
            duration = info.get("duration")
            if isinstance(duration, dict) and "nanoseconds" in duration:
                return float(duration["nanoseconds"]) / 1_000_000_000
            if isinstance(duration, (int, float)):
                return float(duration)
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            return None
        return None

    @classmethod
    def _directory_size(cls, directory: Path) -> int:
        total = 0
        with os.scandir(directory) as entries:
            for entry in entries:
                try:
                    if entry.is_symlink():
                        continue
                    if entry.is_dir(follow_symlinks=False):
                        total += cls._directory_size(Path(entry.path))
                    elif entry.is_file(follow_symlinks=False):
                        total += entry.stat(follow_symlinks=False).st_size
                except FileNotFoundError:
                    continue
        return total

    @staticmethod
    def _write_json(path: Path, payload: dict[str, object]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=path.parent,
                prefix=f".{path.name}.",
                suffix=".tmp",
                delete=False,
            ) as temporary_file:
                temporary_path = Path(temporary_file.name)
                json.dump(payload, temporary_file, ensure_ascii=False, indent=2)
                temporary_file.write("\n")
                temporary_file.flush()
                os.fsync(temporary_file.fileno())
            os.replace(temporary_path, path)
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()
