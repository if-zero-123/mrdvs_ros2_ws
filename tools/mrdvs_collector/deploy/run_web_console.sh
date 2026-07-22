#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /home/cat/mrdvs_ros2_ws/install/setup.bash
export LD_LIBRARY_PATH="/opt/MRDVS/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MRDVS_COLLECTOR_ROOT=/home/cat/mrdvs_collector
exec /home/cat/mrdvs_collector/.venv/bin/mrdvs-web-console
