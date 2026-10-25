#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
exec gz sim -r --physics-engine gz-physics-bullet-plugin "$@" worlds/rmuc_2027.sdf
