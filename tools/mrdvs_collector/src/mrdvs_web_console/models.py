from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, IPvAnyAddress, field_validator


class AppConfig(BaseModel):
    """Persistent, non-secret collector settings."""

    model_config = ConfigDict(
        extra="forbid", validate_assignment=True, validate_default=True
    )

    deploy_root: Path = Path("/home/cat/mrdvs_collector")
    bag_root: Path = Path("/home/cat/mrdvs_collector/bags")
    state_root: Path = Path("/home/cat/mrdvs_collector/state")
    radar_ip: IPvAnyAddress = "192.168.100.82"
    imu_range_level: int = Field(default=2, ge=0, le=4)
    min_free_bytes: int = Field(default=5 * 1024**3, gt=0)
    pointcloud_max_points: int = Field(default=50_000, ge=1, le=200_000)
    pointcloud_max_hz: float = Field(default=5.0, gt=0, le=20.0)
    imu_max_hz: float = Field(default=20.0, gt=0, le=200.0)
    imu_window_seconds: float = Field(default=10.0, gt=0, le=120.0)
    allowed_origins: list[str] = Field(
        default_factory=lambda: ["http://10.42.0.1", "http://localhost"]
    )

    @field_validator("deploy_root", "bag_root", "state_root")
    @classmethod
    def require_absolute_path(cls, value: Path) -> Path:
        expanded = value.expanduser()
        if not expanded.is_absolute():
            raise ValueError("运行目录必须使用绝对路径")
        return expanded
