#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "$script_dir/.." && pwd)"
vendor_dir="$project_dir/src/mrdvs_web_console/static/vendor"

three_source="$project_dir/node_modules/three/build/three.module.min.js"
three_core_source="$project_dir/node_modules/three/build/three.core.min.js"
chart_source="$project_dir/node_modules/chart.js/dist/chart.umd.js"

test -f "$three_source"
test -f "$three_core_source"
test -f "$chart_source"
mkdir -p "$vendor_dir"
install -m 0644 "$three_source" "$vendor_dir/three.module.min.js"
install -m 0644 "$three_core_source" "$vendor_dir/three.core.min.js"
install -m 0644 "$chart_source" "$vendor_dir/chart.umd.js"
