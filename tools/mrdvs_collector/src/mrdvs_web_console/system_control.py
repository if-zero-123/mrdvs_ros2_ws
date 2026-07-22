import asyncio
import json


class SystemControlError(RuntimeError):
    """Raised when the privileged allow-listed helper rejects an operation."""


class SystemControl:
    HELPER = "/usr/local/libexec/mrdvs-system-helper"

    async def _run(self, *arguments: str, secrets: tuple[str, ...] = ()) -> dict:
        process = await asyncio.create_subprocess_exec(
            "sudo",
            "-n",
            self.HELPER,
            *arguments,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await process.communicate()
        if process.returncode != 0:
            detail = stderr.decode("utf-8", errors="replace").strip()
            for secret in secrets:
                detail = detail.replace(secret, "********")
            raise SystemControlError(detail or "系统辅助程序执行失败")
        try:
            return json.loads(stdout.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise SystemControlError("系统辅助程序返回了无效数据") from error

    async def get_autostart(self) -> bool:
        result = await self._run("get-autostart")
        return bool(result["enabled"])

    async def set_autostart(self, enabled: bool) -> dict:
        return await self._run("set-autostart", "true" if enabled else "false")

    async def configure_hotspot(self, ssid: str, password: str) -> dict:
        return await self._run(
            "configure-hotspot",
            "--ssid",
            ssid,
            "--password",
            password,
            secrets=(password,),
        )
