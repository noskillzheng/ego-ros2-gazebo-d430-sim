#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
export GIT_TERMINAL_PROMPT=0
mkdir -p src
clone() {
  local name=$1 url=$2 ref=$3
  if [[ ! -d "src/$name/.git" ]]; then
    git init "src/$name"
    git -C "src/$name" remote add origin "$url"
    git -C "src/$name" fetch --depth 1 origin "$ref"
    git -C "src/$name" checkout --detach FETCH_HEAD
  fi
  test "$(git -C "src/$name" rev-parse HEAD)" = "$ref" || { echo "Unexpected revision in src/$name; preserving checkout" >&2; exit 1; }
  git -C "src/$name" log -1 --format="$name %H"
}
clone ego-planner-swarm https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git 23a8d5a191711dd65633df689bd00f55d4dea8f9
clone ros_gz https://github.com/gazebosim/ros_gz.git 95e6fdea72fb57ef55a4136292dfa62370ab3fb9
clone realsense-ros https://github.com/IntelRealSense/realsense-ros.git 7a87097c499e3b0b69518d54a8e7a9ac1924e87c
clone px4_msgs https://github.com/PX4/px4_msgs.git 2e892300d8c20d3c29e198e324a10854b40ab8d3
# Extract small binary development dependencies locally; do not apt-install into /opt.
mkdir -p .deps/debs
cd .deps/debs
apt download ros-jazzy-gz-msgs-vendor ros-jazzy-gz-transport-vendor \
  ros-jazzy-gz-tools-vendor ros-jazzy-actuator-msgs ros-jazzy-gps-msgs \
  ros-jazzy-marine-acoustic-msgs ros-jazzy-vision-msgs \
  ros-jazzy-diagnostic-updater ros-jazzy-xacro
for deb in *.deb; do dpkg-deb -x "$deb" "$ROOT/.deps"; done
cd "$ROOT"
bash scripts/prepare_px4.sh
/usr/bin/python3 scripts/patch_dependencies.py
