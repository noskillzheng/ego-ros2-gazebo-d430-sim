#!/usr/bin/env bash
# Source this file in every ROS terminal.
EGO_SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export EGO_SIM_ROOT
source /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH="$EGO_SIM_ROOT/.deps/opt/ros/jazzy:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$EGO_SIM_ROOT/.deps/opt/ros/jazzy:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$EGO_SIM_ROOT/.deps/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$EGO_SIM_ROOT/.deps/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH:-}"
export ROS_LOG_DIR="$EGO_SIM_ROOT/log/ros"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-27}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=${GZ_PARTITION:-ego_rmuc_2027}
export GZ_VERSION=harmonic
if [[ -f "$EGO_SIM_ROOT/install/setup.bash" ]]; then source "$EGO_SIM_ROOT/install/setup.bash"; fi
