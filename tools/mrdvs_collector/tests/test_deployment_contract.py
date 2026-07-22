from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEPLOY = PROJECT_ROOT / "deploy"


def read(name: str) -> str:
    return (DEPLOY / name).read_text(encoding="utf-8")


def test_web_service_is_unprivileged_and_kills_its_control_group():
    unit = read("mrdvs-web-console.service")
    assert "User=cat" in unit
    assert "KillMode=control-group" in unit
    assert "AmbientCapabilities=CAP_NET_BIND_SERVICE" in unit
    assert "Requires=mrdvs-hotspot.service" in unit
    assert "After=mrdvs-hotspot.service" in unit
    assert "PartOf=mrdvs-collector.target" in unit
    assert "NoNewPrivileges=yes" not in unit


def test_hotspot_service_is_tied_to_collector_target():
    unit = read("mrdvs-hotspot.service")
    assert "Type=oneshot" in unit
    assert "RemainAfterExit=yes" in unit
    assert "PartOf=mrdvs-collector.target" in unit
    assert "After=NetworkManager.service" in unit
    assert "hotspot-up" in unit
    assert "hotspot-down" in unit


def test_target_can_be_enabled_independently_for_next_boot():
    unit = read("mrdvs-collector.target")
    assert "Wants=mrdvs-hotspot.service mrdvs-web-console.service" in unit
    assert "After=NetworkManager.service" in unit
    assert "WantedBy=multi-user.target" in unit


def test_hotspot_profile_is_never_autoconnected_directly():
    helper = read("mrdvs_system_helper.py")
    assert '"connection.autoconnect", "no"' in helper
    assert "shell=True" not in helper


def test_sudoers_allows_only_the_fixed_helper():
    assert read("mrdvs-system-helper.sudoers").strip() == (
        "cat ALL=(root) NOPASSWD: /usr/local/libexec/mrdvs-system-helper"
    )


def test_wrapper_sources_ros_and_uses_the_standalone_root():
    wrapper = read("run_web_console.sh")
    assert "set -eo pipefail" in wrapper
    assert "source /opt/ros/jazzy/setup.bash" in wrapper
    assert "source /home/cat/mrdvs_ros2_ws/install/setup.bash" in wrapper
    assert wrapper.index("source /home/cat/mrdvs_ros2_ws/install/setup.bash") < wrapper.index(
        "set -u"
    )
    assert "MRDVS_COLLECTOR_ROOT=/home/cat/mrdvs_collector" in wrapper
    assert "exec /home/cat/mrdvs_collector/.venv/bin/mrdvs-web-console" in wrapper


def test_installer_is_arm64_only_idempotent_and_does_not_start_services():
    installer = read("install_lubancat.sh")
    assert '"$(uname -m)" == "aarch64"' in installer
    assert "python3 -m venv --system-site-packages" in installer
    assert "visudo -cf" in installer
    assert "configure-hotspot" in installer
    assert "systemctl daemon-reload" in installer
    assert "systemctl enable mrdvs-collector.target" in installer
    assert "enable --now mrdvs-collector.target" not in installer
    assert "systemctl start mrdvs-collector.target" not in installer
    assert "Python 依赖通过当前网络环境安装失败，改用直连重试" in installer
    assert "env -u ALL_PROXY -u all_proxy" in installer
    for directory in ["app", ".venv", "config", "state", "bags"]:
        assert directory in installer
