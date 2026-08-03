#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  echo "用法: $0 <数据包名称>"
  echo "录制 MRDVS LiDAR 点云、未去畸变 RGB 图像和 IMU 数据到 /home/zero/MRDVS_bags/<数据包名称>。"
}

if [[ "$#" -ne 1 ]]; then
  usage
  exit 1
fi

bag_name="$1"
if [[ ! "$bag_name" =~ ^[[:alnum:]_-]+$ ]]; then
  echo "错误：数据包名称只能包含字母、数字、下划线和短横线。" >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
setup_file="$script_dir/install/setup.bash"
if [[ ! -f "$setup_file" ]]; then
  echo "错误：未找到 $setup_file，请先在工作空间根目录执行 colcon build。" >&2
  exit 1
fi

# shellcheck source=/dev/null
set +u
source "$setup_file"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误：未找到 ros2 命令，请确认 ROS 2 环境已安装。" >&2
  exit 1
fi

bag_root="${MRDVS_BAG_ROOT:-/home/zero/MRDVS_bags}"
bag_path="$bag_root/$bag_name"
if [[ -e "$bag_path" ]]; then
  echo "错误：数据包目录已存在，拒绝覆盖：$bag_path" >&2
  exit 1
fi
mkdir -p "$bag_root"

bag_pid=""
driver_pid=""

cleanup() {
  local exit_code="$?"
  trap - EXIT INT TERM

  if [[ -n "$driver_pid" ]] && kill -0 "$driver_pid" 2>/dev/null; then
    echo "正在停止 MRDVS LiDAR 驱动..."
    kill -INT "$driver_pid" 2>/dev/null || true
    wait "$driver_pid" || true
  fi

  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    echo "正在停止 rosbag 并写入 MCAP 元数据..."
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" || true
  fi

  exit "$exit_code"
}

handle_interrupt() {
  exit 0
}

trap cleanup EXIT
trap handle_interrupt INT TERM

echo "rosbag 已准备录制到：$bag_path"
ros2 bag record \
  --storage mcap \
  --output "$bag_path" \
  /lx_camera_node/LxCamera_Cloud \
  /lx_camera_node/LxCamera_Rgb \
  /lx_camera_node/LxCamera_Imu &
bag_pid="$!"

sleep 1
if ! kill -0 "$bag_pid" 2>/dev/null; then
  echo "错误：rosbag 未能启动。" >&2
  wait "$bag_pid" || true
  exit 1
fi

echo "正在以默认 IP 192.168.100.82 启动 MRDVS LiDAR 驱动..."
ros2 launch lx_camera_ros lx_lidar_ros.launch.py \
  ip:=192.168.100.82 \
  enable_rviz:=false &
driver_pid="$!"

echo "录制中。按 Ctrl+C 停止驱动并安全写入数据包。"
wait "$driver_pid"
