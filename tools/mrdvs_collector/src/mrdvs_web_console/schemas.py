from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, IPvAnyAddress, model_validator

from .controller import RgbRecordingMode, TopicRecordingMode


class StrictRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")


class DriverStartRequest(StrictRequest):
    record: bool = False
    bag_name: str | None = None
    rgb_mode: RgbRecordingMode = RgbRecordingMode.RAW
    topic_mode: TopicRecordingMode = TopicRecordingMode.ALL
    selected_topics: list[str] | None = None

    @model_validator(mode="after")
    def require_name_for_recording(self):
        if self.record and not self.bag_name:
            raise ValueError("同时录制时必须填写数据包名称")
        return self


class RecordingStartRequest(StrictRequest):
    bag_name: str = Field(min_length=1, max_length=80)
    rgb_mode: RgbRecordingMode = RgbRecordingMode.RAW
    topic_mode: TopicRecordingMode = TopicRecordingMode.ALL
    selected_topics: list[str] | None = None


class DeleteBagRequest(StrictRequest):
    confirmation: str = Field(min_length=1, max_length=80)


class SettingsPatch(StrictRequest):
    bag_root: Path | None = None
    radar_ip: IPvAnyAddress | None = None
    imu_range_level: int | None = Field(default=None, ge=0, le=4)
    min_free_bytes: int | None = Field(default=None, gt=0)
    pointcloud_max_points: int | None = Field(default=None, ge=1, le=200_000)
    pointcloud_max_hz: float | None = Field(default=None, gt=0, le=20)
    imu_max_hz: float | None = Field(default=None, gt=0, le=200)
    imu_window_seconds: float | None = Field(default=None, gt=0, le=120)


class HotspotSettingsRequest(StrictRequest):
    ssid: str = Field(min_length=1, max_length=32)
    password: str = Field(min_length=8, max_length=63)

    @model_validator(mode="after")
    def validate_wifi_credentials(self):
        if len(self.ssid.encode("utf-8")) > 32:
            raise ValueError("热点名称的 UTF-8 长度不能超过 32 字节")
        if any(character.isspace() for character in self.password):
            raise ValueError("热点密码不能包含空白字符")
        return self


class AutostartRequest(StrictRequest):
    enabled: bool
