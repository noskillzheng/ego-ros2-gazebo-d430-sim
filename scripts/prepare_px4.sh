#!/usr/bin/env bash
# Copy runtime inputs only. Never write to the existing PX4 checkout.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PX4_DIR=${PX4_DIR:-/home/true1/Projects/PX4-Autopilot}
test -x "$PX4_DIR/build/px4_sitl_default/bin/px4"
mkdir -p "$ROOT/.runtime/px4" "$ROOT/assets/px4_models"
if [[ ! -d "$ROOT/.runtime/px4/etc" ]]; then
  cp -aL "$PX4_DIR/build/px4_sitl_default/rootfs/etc" "$ROOT/.runtime/px4/"
fi
for model in x500 x500_base; do
  if [[ ! -d "$ROOT/assets/px4_models/$model" ]]; then
    cp -a "$PX4_DIR/Tools/simulation/gz/models/$model" "$ROOT/assets/px4_models/"
  fi
done
# This optional plugin is not required for stock flight and is not installed everywhere.
sed -i '/<plugin filename="MotorFailurePlugin"/,+2d' "$ROOT/assets/px4_models/x500/model.sdf"
# PX4's current versioned uORB definitions, not an independently moving main branch.
mkdir -p "$ROOT/src/px4_msgs/upstream-msg-backup"
if [[ ! -f "$ROOT/src/px4_msgs/PX4_SOURCE_REVISION" ]]; then
  mv "$ROOT"/src/px4_msgs/msg/*.msg "$ROOT/src/px4_msgs/upstream-msg-backup/"
  cp "$PX4_DIR"/msg/*.msg "$PX4_DIR"/msg/versioned/*.msg "$ROOT/src/px4_msgs/msg/"
  git -C "$PX4_DIR" rev-parse HEAD > "$ROOT/src/px4_msgs/PX4_SOURCE_REVISION"
fi
if [[ "$(cat "$ROOT/src/px4_msgs/PX4_SOURCE_REVISION")" != "$(git -C "$PX4_DIR" rev-parse HEAD)" ]]; then
  echo 'PX4 revision changed: refresh px4_msgs definitions and rebuild before launching.' >&2
  exit 1
fi
# SITL parameter overrides are applied by run_px4.sh on every fresh run.
