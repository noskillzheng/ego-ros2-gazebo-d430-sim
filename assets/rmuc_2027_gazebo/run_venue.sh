#!/usr/bin/env bash
# Same as run.sh, but opens worlds/rmuc_2027_venue.sdf (lighting fitted to the RMUC 2026
# venue recordings, see docs/venue_lighting.md). Builds the lit models on first use.
set -e
cd "$(dirname "$0")"
if [ ! -f models/rmuc_2027_lit/meshes/rmuc_2027_visual_lit.glb ] \
   || [ models/rmuc_2027/meshes/rmuc_2027_visual.glb -nt models/rmuc_2027_lit/meshes/rmuc_2027_visual_lit.glb ]; then
  python3 tools/build_venue_lighting.py
fi
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
engine="${RM_PHYSICS_ENGINE:-dartsim}"
if [ "${RM_USE_DGPU:-1}" = 1 ] && command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
exec gz sim -r --physics-engine "gz-physics-${engine}-plugin" "$@" worlds/rmuc_2027_venue.sdf
