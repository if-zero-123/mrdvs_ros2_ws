import importlib.util
import subprocess
from pathlib import Path

import pytest


HELPER_PATH = Path("tools/mrdvs_collector/deploy/mrdvs_system_helper.py")
SPEC = importlib.util.spec_from_file_location("mrdvs_system_helper", HELPER_PATH)
assert SPEC is not None and SPEC.loader is not None
HELPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HELPER)


def test_hotspot_defaults_are_valid():
    assert HELPER.validate_ssid("MRDVS-Collector") == "MRDVS-Collector"
    assert HELPER.validate_password("12345678") == "12345678"


@pytest.mark.parametrize("ssid", ["", "\n", "x" * 33, "热点" * 11])
def test_invalid_ssid_is_rejected(ssid: str):
    with pytest.raises(ValueError):
        HELPER.validate_ssid(ssid)


@pytest.mark.parametrize("password", ["1234567", "contains space", "x" * 64, "含中文密码"])
def test_invalid_wpa2_password_is_rejected(password: str):
    with pytest.raises(ValueError):
        HELPER.validate_password(password)


def test_configure_hotspot_uses_fixed_nmcli_arguments_and_hides_password(monkeypatch):
    calls: list[tuple[list[str], dict]] = []

    def fake_run(arguments, **kwargs):
        calls.append((arguments, kwargs))
        return subprocess.CompletedProcess(arguments, 0, stdout="mrdvs-hotspot\n", stderr="")

    monkeypatch.setattr(HELPER.subprocess, "run", fake_run)
    result = HELPER.configure_hotspot("MRDVS-Collector", "12345678")

    assert result == {
        "status": "configured",
        "ssid": "MRDVS-Collector",
        "applies": "next_hotspot_start",
    }
    assert calls
    assert all(isinstance(arguments, list) for arguments, _ in calls)
    assert all(kwargs["shell"] is False for _, kwargs in calls)
    modify = next(arguments for arguments, _ in calls if "modify" in arguments)
    assert ["connection.autoconnect", "no"] == modify[-2:]
    assert "ipv4.method" in modify and "shared" in modify
    assert "ipv4.addresses" in modify and "10.42.0.1/24" in modify
    assert "12345678" in modify
    assert "12345678" not in repr(result)


def test_disabling_autostart_only_changes_next_boot(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(arguments, **kwargs):
        calls.append(arguments)
        return subprocess.CompletedProcess(arguments, 0, stdout="", stderr="")

    monkeypatch.setattr(HELPER.subprocess, "run", fake_run)
    assert HELPER.set_autostart(False) == {"enabled": False, "applies": "next_boot"}
    assert calls == [["systemctl", "disable", "mrdvs-collector.target"]]
    assert "--now" not in calls[0]


def test_enabling_autostart_uses_only_the_allow_listed_target(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(arguments, **kwargs):
        calls.append(arguments)
        return subprocess.CompletedProcess(arguments, 0, stdout="", stderr="")

    monkeypatch.setattr(HELPER.subprocess, "run", fake_run)
    assert HELPER.set_autostart(True) == {"enabled": True, "applies": "next_boot"}
    assert calls == [["systemctl", "enable", "mrdvs-collector.target"]]

