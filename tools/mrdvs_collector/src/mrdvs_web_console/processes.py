import asyncio
import os
import signal
from collections import deque
from collections.abc import Mapping


class ManagedProcess:
    """A subprocess isolated in its own process group with bounded output."""

    def __init__(self, name: str, process: asyncio.subprocess.Process):
        self.name = name
        self._process = process
        self._logs: deque[str] = deque(maxlen=500)
        self._reader_task = asyncio.create_task(self._read_output())

    @property
    def alive(self) -> bool:
        return self._process.returncode is None

    @property
    def returncode(self) -> int | None:
        return self._process.returncode

    @property
    def logs(self) -> tuple[str, ...]:
        return tuple(self._logs)

    async def wait(self) -> int:
        return await self._process.wait()

    async def stop(self, timeout: float = 10.0) -> int:
        if self.alive:
            self._signal_group(signal.SIGINT)
            if not await self._wait_for_exit(timeout):
                self._signal_group(signal.SIGTERM)
                if not await self._wait_for_exit(min(timeout, 3.0)):
                    self._signal_group(signal.SIGKILL)
                    await self._process.wait()
        await asyncio.gather(self._reader_task, return_exceptions=True)
        return int(self._process.returncode or 0)

    async def _read_output(self) -> None:
        if self._process.stdout is None:
            return
        while True:
            line = await self._process.stdout.readline()
            if not line:
                return
            self._logs.append(line.decode("utf-8", errors="replace").rstrip())

    async def _wait_for_exit(self, timeout: float) -> bool:
        try:
            await asyncio.wait_for(asyncio.shield(self._process.wait()), timeout=timeout)
            return True
        except TimeoutError:
            return False

    def _signal_group(self, requested_signal: signal.Signals) -> None:
        try:
            os.killpg(self._process.pid, requested_signal)
        except ProcessLookupError:
            return


class SubprocessRunner:
    """Starts only argument-array commands and never invokes a shell."""

    def __init__(self, environment: Mapping[str, str] | None = None):
        self._environment = dict(environment) if environment is not None else None

    async def start(self, name: str, command: list[str]) -> ManagedProcess:
        process = await asyncio.create_subprocess_exec(
            *command,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            start_new_session=True,
            env=self._environment,
        )
        return ManagedProcess(name, process)
