#!/usr/bin/env bash
set -euo pipefail

readonly collector_root=/home/cat/mrdvs_collector
readonly app_dir="$collector_root/app"
readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

fail() {
  printf '安装失败：%s\n' "$1" >&2
  exit 1
}

[[ "$(uname -m)" == "aarch64" ]] || fail "只允许在 ARM64 鲁班猫上安装"
[[ -f /opt/ros/jazzy/setup.bash ]] || fail "缺少 ROS 2 Jazzy"
[[ -f /home/cat/mrdvs_ros2_ws/install/setup.bash ]] || fail "缺少 MRDVS ROS 2 underlay"
[[ -f "$app_dir/pyproject.toml" ]] || fail "请先将应用同步到 $app_dir"
[[ "$script_dir" == "$app_dir/deploy" ]] || fail "安装器必须从独立应用目录运行"

for command in nmcli systemctl python3 sudo visudo; do
  command -v "$command" >/dev/null 2>&1 || fail "缺少命令 $command"
done

sudo install -d -o cat -g cat -m 0755 \
  "$collector_root" "$app_dir" "$collector_root/config" "$collector_root/state" "$collector_root/bags"
sudo chown -R cat:cat \
  "$app_dir" "$collector_root/config" "$collector_root/state" "$collector_root/bags"

python3 -m venv --system-site-packages "$collector_root/.venv"
"$collector_root/.venv/bin/python" -m pip install "$app_dir[test]"

sudo install -d -o root -g root -m 0755 /usr/local/libexec
sudo install -o root -g root -m 0755 \
  "$script_dir/mrdvs_system_helper.py" /usr/local/libexec/mrdvs-system-helper
sudo install -o root -g root -m 0440 \
  "$script_dir/mrdvs-system-helper.sudoers" /etc/sudoers.d/mrdvs-system-helper
sudo visudo -cf /etc/sudoers.d/mrdvs-system-helper

for unit in mrdvs-collector.target mrdvs-hotspot.service mrdvs-web-console.service; do
  sudo install -o root -g root -m 0644 "$script_dir/$unit" "/etc/systemd/system/$unit"
done

sudo /usr/local/libexec/mrdvs-system-helper configure-hotspot \
  --ssid MRDVS-Collector --password 12345678
sudo systemctl daemon-reload
sudo systemctl enable mrdvs-collector.target

printf '安装完成。热点和网页服务已设为下次开机启动，本次安装不会立即切换网络。\n'
