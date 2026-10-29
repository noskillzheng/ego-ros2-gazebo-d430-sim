#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# DART by default: the gz-physics7 Bullet plugin frees the btTriangleMesh before the
# btGImpactMeshShape that points to it, so it segfaults on exit with mesh collisions.
# Override with RM_PHYSICS_ENGINE=bullet.
engine="${RM_PHYSICS_ENGINE:-dartsim}"

# Render on the NVIDIA dGPU when present (the GUI crashed inside Mesa on the Intel iGPU).
# Set RM_USE_DGPU=0 to keep the default GPU.
if [ "${RM_USE_DGPU:-1}" = 1 ] && command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
fi

exec gz sim -r --physics-engine "gz-physics-${engine}-plugin" "$@" worlds/rmuc_2027.sdf
