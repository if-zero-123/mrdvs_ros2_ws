import json
import os
import tempfile
import threading
from pathlib import Path
from typing import Any

from .models import AppConfig


class ConfigStore:
    """Loads and atomically persists non-secret collector settings."""

    def __init__(self, path: Path):
        self.path = path.expanduser()
        self._lock = threading.RLock()

    def load(self) -> AppConfig:
        with self._lock:
            if not self.path.exists():
                config = AppConfig()
                self._write(config)
                return config
            with self.path.open("r", encoding="utf-8") as config_file:
                return AppConfig.model_validate(json.load(config_file))

    def update(self, patch: dict[str, Any]) -> AppConfig:
        with self._lock:
            current = self.load()
            merged = current.model_dump(mode="json")
            merged.update(patch)
            updated = AppConfig.model_validate(merged)
            self._write(updated)
            return updated

    def _write(self, config: AppConfig) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=self.path.parent,
                prefix=f".{self.path.name}.",
                suffix=".tmp",
                delete=False,
            ) as temporary_file:
                temporary_path = Path(temporary_file.name)
                json.dump(
                    config.model_dump(mode="json"),
                    temporary_file,
                    ensure_ascii=False,
                    indent=2,
                    sort_keys=True,
                )
                temporary_file.write("\n")
                temporary_file.flush()
                os.fsync(temporary_file.fileno())
            os.chmod(temporary_path, 0o600)
            os.replace(temporary_path, self.path)
            directory_fd = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()
