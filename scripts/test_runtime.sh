#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$ROOT/scripts/env.sh"
mkdir -p "$ROOT/log/validation"
TEST_NAME=${1:-sitl}
TARGET='[5.0, 2.0, 1.8]'
DETOUR=false
if [[ "$TEST_NAME" == obstacle ]]; then TARGET='[16.0, 8.0, 1.8]'; DETOUR=true; fi
setsid ros2 launch ego_gazebo sim.launch.py headless:=true rviz:=false > "$ROOT/log/validation/$TEST_NAME.log" 2>&1 &
SIM_PID=$!
cleanup() {
  # Only the process group created by this test is signalled.
  kill -TERM -- "-$SIM_PID" 2>/dev/null || true
  for attempt in {1..20}; do
    if ! kill -0 "$SIM_PID" 2>/dev/null; then break; fi
    sleep 0.25
  done
  kill -KILL -- "-$SIM_PID" 2>/dev/null || true
  wait "$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT
ros2 run ego_gazebo verify_sim --ros-args -p use_sim_time:=true -p run_flight:=true -p timeout:=90.0 \
  -p "target:=$TARGET" -p "require_detour:=$DETOUR" | tee "$ROOT/log/validation/$TEST_NAME-result.json"
