#!/bin/bash
set -euo pipefail

usage() {
  echo "Usage: $0 <bag_name>"
  echo "Record all ROS2 topics to ~/bag/<bag_name>."
}

if [ "$#" -ne 1 ]; then
  usage
  exit 1
fi

bag_name="$1"

if [[ ! "$bag_name" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "Error: bag_name can only contain letters, numbers, '.', '_' and '-'."
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
setup_file="$script_dir/install/setup.bash"

if [ -f "$setup_file" ]; then
  # shellcheck source=/dev/null
  set +u
  source "$setup_file"
  set -u
fi

bag_root="$HOME/bag"
bag_path="$bag_root/$bag_name"

mkdir -p "$bag_root"

if [ -e "$bag_path" ]; then
  echo "Error: bag path already exists: $bag_path"
  echo "Use a different bag_name to avoid overwriting data."
  exit 1
fi

echo "Recording all ROS2 topics to: $bag_path"
echo "Press Ctrl+C to stop recording."

exec ros2 bag record -a -o "$bag_path"
