import asyncio
import sys

import pytest

from mrdvs_web_console.processes import SubprocessRunner


async def wait_for_log(process, expected: str) -> None:
    for _ in range(100):
        if expected in process.logs:
            return
        await asyncio.sleep(0.01)
    raise AssertionError(f"没有收到预期日志：{expected}")


@pytest.mark.asyncio
async def test_runner_passes_arguments_without_shell_interpretation():
    runner = SubprocessRunner()
    literal = "; echo injected; $(uname)"

    process = await runner.start(
        "literal",
        [sys.executable, "-u", "-c", "import sys; print(sys.argv[1])", literal],
    )
    returncode = await process.wait()
    await wait_for_log(process, literal)

    assert returncode == 0
    assert process.logs == (literal,)


@pytest.mark.asyncio
async def test_stop_terminates_process_group_with_bounded_wait():
    runner = SubprocessRunner()
    process = await runner.start(
        "sleeper",
        [
            sys.executable,
            "-u",
            "-c",
            "import time; print('ready', flush=True); time.sleep(60)",
        ],
    )
    await wait_for_log(process, "ready")

    returncode = await asyncio.wait_for(process.stop(timeout=0.5), timeout=2.0)

    assert returncode != 0
    assert process.alive is False


@pytest.mark.asyncio
async def test_process_logs_keep_only_newest_500_lines():
    runner = SubprocessRunner()
    process = await runner.start(
        "logger",
        [
            sys.executable,
            "-u",
            "-c",
            "[print(f'line-{index}') for index in range(510)]",
        ],
    )

    assert await process.wait() == 0
    for _ in range(100):
        if len(process.logs) == 500:
            break
        await asyncio.sleep(0.01)

    assert len(process.logs) == 500
    assert process.logs[0] == "line-10"
    assert process.logs[-1] == "line-509"
