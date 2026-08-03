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

stop_matching_processes() {
  local label="$1"
  local pattern="$2"
  local -a pids=()
  local pid
  local remaining

  process_is_alive() {
    [[ -r "/proc/$1/status" ]] && ! grep -q '^State:[[:space:]]*Z' "/proc/$1/status"
  }

  mapfile -t pids < <(pgrep -f -- "$pattern" || true)
  if ((${#pids[@]} == 0)); then
    return 0
  fi

  echo "发现旧的${label}进程，正在请求优雅停止..."
  for pid in "${pids[@]}"; do
    if [[ "$pid" != "$$" ]] && process_is_alive "$pid"; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done

  for _ in {1..20}; do
    remaining=0
    for pid in "${pids[@]}"; do
      if process_is_alive "$pid"; then
        remaining=1
        break
      fi
    done
    if ((remaining == 0)); then
      return 0
    fi
    sleep 0.25
  done

  echo "${label}进程未在 5 秒内退出，发送 SIGTERM..." >&2
  for pid in "${pids[@]}"; do
    if process_is_alive "$pid"; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done

  for _ in {1..20}; do
    remaining=0
    for pid in "${pids[@]}"; do
      if process_is_alive "$pid"; then
        remaining=1
        break
      fi
    done
    if ((remaining == 0)); then
      return 0
    fi
    sleep 0.25
  done

  echo "${label}进程仍未退出，发送 SIGKILL 以释放设备控制权..." >&2
  for pid in "${pids[@]}"; do
    if process_is_alive "$pid"; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done

  for _ in {1..20}; do
    remaining=0
    for pid in "${pids[@]}"; do
      if process_is_alive "$pid"; then
        remaining=1
        break
      fi
    done
    if ((remaining == 0)); then
      return 0
    fi
    sleep 0.25
  done

  echo "错误：${label}进程仍未退出，请手动清理后再录制。" >&2
  return 1
}

cleanup_stale_sessions() {
  stop_matching_processes \
    "旧 rosbag" \
    "ros2 bag record --storage mcap --output ${bag_root}"
  stop_matching_processes \
    "旧录制脚本 wrapper" \
    "record_mrdvs_sensor_bag.sh"
}

run_in_new_session() {
  exec setsid bash -c 'trap - INT TERM; exec "$@"' _ "$@"
}

bag_root="${MRDVS_BAG_ROOT:-/home/zero/MRDVS_bags}"
bag_path="$bag_root/$bag_name"
if [[ -e "$bag_path" ]]; then
  echo "错误：数据包目录已存在，拒绝覆盖：$bag_path" >&2
  exit 1
fi
mkdir -p "$bag_root"

bag_pid=""
cleanup_stale_sessions

cleanup() {
  local exit_code="$?"
  trap - EXIT INT TERM

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
run_in_new_session ros2 bag record \
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

echo "录制中（驱动请在另一个终端单独启动）。按 Ctrl+C 停止录包并安全写入数据包。"
wait "$bag_pid"
