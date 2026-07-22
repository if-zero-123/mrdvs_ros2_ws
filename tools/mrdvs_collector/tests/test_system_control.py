import pytest

from mrdvs_web_console.system_control import SystemControl, SystemControlError


class FakeSubprocess:
    def __init__(self, returncode: int, stdout: bytes, stderr: bytes):
        self.returncode = returncode
        self._stdout = stdout
        self._stderr = stderr

    async def communicate(self):
        return self._stdout, self._stderr


@pytest.mark.asyncio
async def test_get_autostart_uses_fixed_helper_command(monkeypatch):
    captured = {}

    async def fake_exec(*arguments, **options):
        captured["arguments"] = arguments
        captured["options"] = options
        return FakeSubprocess(0, b'{"enabled": true}', b"")

    monkeypatch.setattr("asyncio.create_subprocess_exec", fake_exec)

    assert await SystemControl().get_autostart() is True
    assert captured["arguments"] == (
        "sudo",
        "-n",
        "/usr/local/libexec/mrdvs-system-helper",
        "get-autostart",
    )
    assert "shell" not in captured["options"]


@pytest.mark.asyncio
async def test_hotspot_failure_redacts_password(monkeypatch):
    async def fake_exec(*arguments, **options):
        return FakeSubprocess(1, b"", b"invalid password 12345678")

    monkeypatch.setattr("asyncio.create_subprocess_exec", fake_exec)

    with pytest.raises(SystemControlError) as error:
        await SystemControl().configure_hotspot("MRDVS-Collector", "12345678")

    assert "12345678" not in str(error.value)
    assert "********" in str(error.value)
