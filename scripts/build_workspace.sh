#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
source "$ROOT/scripts/env.sh"
export CMAKE_PREFIX_PATH="$ROOT/.deps/opt/ros/jazzy/opt/gz_msgs_vendor/extra_cmake:$ROOT/.deps/opt/ros/jazzy/opt/gz_transport_vendor/extra_cmake:$CMAKE_PREFIX_PATH"
/usr/bin/python3 "$ROOT/scripts/patch_dependencies.py"
export ROS_LOG_DIR="$ROOT/log/ros"
export GZ_VERSION=harmonic
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-4}
colcon --log-base "$ROOT/log/build" build --base-paths "$ROOT/src" \
  --build-base "$ROOT/build/colcon" --install-base "$ROOT/install" \
  --executor sequential --packages-up-to ego_gazebo realsense2_camera realsense2_description ros_gz_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPython3_EXECUTABLE=/usr/bin/python3
