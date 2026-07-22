#!/usr/bin/env python3
"""Allow-listed privileged operations for the MRDVS collector appliance."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import unicodedata


HOTSPOT_CONNECTION = "mrdvs-hotspot"
HOTSPOT_INTERFACE = "wlan0"
COLLECTOR_TARGET = "mrdvs-collector.target"


class HelperCommandError(RuntimeError):
    """A fixed system command failed without exposing its arguments."""


def validate_ssid(value: str) -> str:
    ssid = value.strip()
    if not ssid or not 1 <= len(ssid.encode("utf-8")) <= 32:
        raise ValueError("热点名称必须为 1 到 32 字节")
    if any(unicodedata.category(character).startswith("C") for character in ssid):
        raise ValueError("热点名称不能包含控制字符")
    return ssid


def validate_password(value: str) -> str:
    if not re.fullmatch(r"[!-~]{8,63}", value):
        raise ValueError("热点密码必须为 8 到 63 个无空格 ASCII 可打印字符")
    return value


def _run_command(arguments: list[str]) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            arguments,
            check=True,
            text=True,
            capture_output=True,
            shell=False,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        raise HelperCommandError("系统命令执行失败") from error


def _connection_exists() -> bool:
    try:
        subprocess.run(
            ["nmcli", "connection", "show", HOTSPOT_CONNECTION],
            check=True,
            text=True,
            capture_output=True,
            shell=False,
        )
    except subprocess.CalledProcessError:
        return False
    except OSError as error:
        raise HelperCommandError("无法执行 NetworkManager 命令") from error
    return True


def configure_hotspot(ssid: str, password: str) -> dict[str, object]:
    safe_ssid = validate_ssid(ssid)
    safe_password = validate_password(password)
    if not _connection_exists():
        _run_command(
            [
                "nmcli",
                "connection",
                "add",
                "type",
                "wifi",
                "ifname",
                HOTSPOT_INTERFACE,
                "con-name",
                HOTSPOT_CONNECTION,
                "ssid",
                safe_ssid,
            ]
        )
    _run_command(
        [
            "nmcli",
            "connection",
            "modify",
            HOTSPOT_CONNECTION,
            "connection.interface-name",
            HOTSPOT_INTERFACE,
            "802-11-wireless.mode",
            "ap",
            "802-11-wireless.ssid",
            safe_ssid,
            "802-11-wireless-security.key-mgmt",
            "wpa-psk",
            "802-11-wireless-security.psk",
            safe_password,
            "ipv4.method",
            "shared",
            "ipv4.addresses",
            "10.42.0.1/24",
            "ipv6.method",
            "disabled",
            "connection.autoconnect", "no",
        ]
    )
    return {
        "status": "configured",
        "ssid": safe_ssid,
        "applies": "next_hotspot_start",
    }


def hotspot_up() -> dict[str, object]:
    _run_command(
        [
            "nmcli",
            "connection",
            "up",
            HOTSPOT_CONNECTION,
            "ifname",
            HOTSPOT_INTERFACE,
        ]
    )
    return {"active": True, "address": "10.42.0.1"}


def hotspot_down() -> dict[str, object]:
    try:
        _run_command(["nmcli", "connection", "down", HOTSPOT_CONNECTION])
    except HelperCommandError:
        # Stopping an already inactive one-shot service must remain idempotent.
        pass
    return {"active": False}


def set_autostart(enabled: bool) -> dict[str, object]:
    action = "enable" if enabled else "disable"
    _run_command(["systemctl", action, COLLECTOR_TARGET])
    return {"enabled": enabled, "applies": "next_boot"}


def get_autostart() -> dict[str, object]:
    try:
        result = subprocess.run(
            ["systemctl", "is-enabled", COLLECTOR_TARGET],
            check=True,
            text=True,
            capture_output=True,
            shell=False,
        )
        enabled = result.stdout.strip() == "enabled"
    except subprocess.CalledProcessError as error:
        if error.returncode not in {1, 3, 4}:
            raise HelperCommandError("无法读取采集服务自启状态") from error
        enabled = False
    except OSError as error:
        raise HelperCommandError("无法执行 systemd 命令") from error
    return {"enabled": enabled}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    configure = subparsers.add_parser("configure-hotspot")
    configure.add_argument("--ssid", required=True)
    configure.add_argument("--password", required=True)

    subparsers.add_parser("hotspot-up")
    subparsers.add_parser("hotspot-down")

    autostart = subparsers.add_parser("set-autostart")
    autostart.add_argument("enabled", choices=("true", "false"))
    subparsers.add_parser("get-autostart")
    return parser


def main() -> int:
    if os.geteuid() != 0:
        print(json.dumps({"error": "辅助程序必须以 root 身份运行"}, ensure_ascii=False), file=sys.stderr)
        return 1
    arguments = _parser().parse_args()
    try:
        if arguments.command == "configure-hotspot":
            result = configure_hotspot(arguments.ssid, arguments.password)
        elif arguments.command == "hotspot-up":
            result = hotspot_up()
        elif arguments.command == "hotspot-down":
            result = hotspot_down()
        elif arguments.command == "set-autostart":
            result = set_autostart(arguments.enabled == "true")
        else:
            result = get_autostart()
    except (HelperCommandError, ValueError) as error:
        print(json.dumps({"error": str(error)}, ensure_ascii=False), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
