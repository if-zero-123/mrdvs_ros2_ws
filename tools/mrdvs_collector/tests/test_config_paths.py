from pathlib import Path

import pytest

from mrdvs_web_console.config import ConfigStore
from mrdvs_web_console.paths import BagNameError, resolve_bag_path, validate_bag_name


def test_unicode_bag_name_is_preserved():
    assert validate_bag_name("  走廊采集_01  ") == "走廊采集_01"


@pytest.mark.parametrize(
    "name",
    ["", "  ", "../escape", "a/b", "a\\b", ".", "..", "含 空格", "name.txt"],
)
def test_unsafe_bag_names_are_rejected(name: str):
    with pytest.raises(BagNameError):
        validate_bag_name(name)


def test_name_longer_than_80_characters_is_rejected():
    with pytest.raises(BagNameError):
        validate_bag_name("a" * 81)


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

    assert str(updated.radar_ip) == "192.168.100.83"
    assert store.load().imu_range_level == 3
    assert not list(tmp_path.glob("*.tmp"))


def test_invalid_config_update_keeps_previous_file(tmp_path: Path):
    store = ConfigStore(tmp_path / "config.json")
    original = store.load()

    with pytest.raises(ValueError):
        store.update({"imu_range_level": 9})

    assert store.load() == original
