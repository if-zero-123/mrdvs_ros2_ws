import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from mrdvs_web_console.bags import BagConflict, BagManager, BagState


def make_manager(tmp_path: Path, min_free_bytes: int = 1) -> BagManager:
    return BagManager(tmp_path / "bags", tmp_path / "state", min_free_bytes)


def test_prepare_returns_confined_path_without_creating_bag(tmp_path: Path):
    manager = make_manager(tmp_path)

    path = manager.prepare("走廊_01")

    assert path == tmp_path / "bags" / "走廊_01"
    assert path.parent.is_dir()
    assert not path.exists()


def test_prepare_refuses_existing_directory(tmp_path: Path):
    manager = make_manager(tmp_path)
    existing = tmp_path / "bags" / "room1"
    existing.mkdir(parents=True)

    with pytest.raises(BagConflict):
        manager.prepare("room1")


def test_tar_command_has_no_shell_and_is_confined(tmp_path: Path):
    manager = make_manager(tmp_path)
    (tmp_path / "bags" / "room1").mkdir(parents=True)

    command = manager.tar_command("room1")

    assert command == (
        "tar",
        "--format=posix",
        "-C",
        str((tmp_path / "bags").resolve()),
        "-cf",
        "-",
        "room1",
    )


def test_tar_command_rejects_symlink(tmp_path: Path):
    manager = make_manager(tmp_path)
    outside = tmp_path / "outside"
    outside.mkdir()
    manager.bag_root.mkdir(parents=True)
    (manager.bag_root / "link").symlink_to(outside, target_is_directory=True)

    with pytest.raises(FileNotFoundError):
        manager.tar_command("link")


def test_delete_requires_exact_confirmation(tmp_path: Path):
    manager = make_manager(tmp_path)
    bag = tmp_path / "bags" / "走廊_01"
    bag.mkdir(parents=True)
    (bag / "data.mcap").write_bytes(b"bag")

    with pytest.raises(ValueError):
        manager.delete("走廊_01", confirmation="走廊_02")

    manager.delete("走廊_01", confirmation="走廊_01")

    assert not bag.exists()


def test_mark_and_list_bags_include_size_and_state(tmp_path: Path):
    manager = make_manager(tmp_path)
    bag = tmp_path / "bags" / "room1"
    bag.mkdir(parents=True)
    (bag / "data.mcap").write_bytes(b"12345")
    (bag / "metadata.yaml").write_text("duration: 10\n", encoding="utf-8")

    manager.mark_status("room1", BagState.COMPLETE, "录制完成")
    summaries = manager.list_bags()

    assert len(summaries) == 1
    assert summaries[0].name == "room1"
    assert summaries[0].size_bytes >= 5
    assert summaries[0].state is BagState.COMPLETE
    state_file = manager.status_path("room1")
    assert json.loads(state_file.read_text(encoding="utf-8"))["detail"] == "录制完成"
    assert not list(state_file.parent.glob("*.tmp"))


def test_list_bags_ignores_files_and_symlink_directories(tmp_path: Path):
    manager = make_manager(tmp_path)
    manager.bag_root.mkdir(parents=True)
    (manager.bag_root / "not-a-bag.txt").write_text("x", encoding="utf-8")
    outside = tmp_path / "outside"
    outside.mkdir()
    (manager.bag_root / "linked").symlink_to(outside, target_is_directory=True)

    assert manager.list_bags() == []


def test_disk_guard_reports_stop_when_below_threshold(tmp_path: Path, monkeypatch):
    manager = make_manager(tmp_path, min_free_bytes=5_000)
    monkeypatch.setattr(
        "mrdvs_web_console.bags.shutil.disk_usage",
        lambda _: SimpleNamespace(total=10_000, used=6_000, free=4_000),
    )

    status = manager.disk_status()

    assert status.free_bytes == 4_000
    assert status.must_stop is True
