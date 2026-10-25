#!/usr/bin/env bash
set -euo pipefail
: "${EGO_SIM_ROOT:?Set EGO_SIM_ROOT to the workspace}"
: "${PX4_DIR:?Set PX4_DIR to the PX4 checkout}"
cd "$EGO_SIM_ROOT/.runtime/px4"
export PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD=rmuc_2027 PX4_GZ_MODEL_NAME=x500_d430_0 PX4_GZ_NO_FOLLOW=1
export PX4_PARAM_COM_RC_IN_MODE=4 PX4_PARAM_COM_RCL_EXCEPT=4
export PX4_PARAM_NAV_DLL_ACT=0 PX4_PARAM_COM_DISARM_PRFLT=-1
export PX4_PARAM_COM_OF_LOSS_T=1 PX4_PARAM_COM_OBL_RC_ACT=4
export PX4_PARAM_UXRCE_DDS_DOM_ID=${ROS_DOMAIN_ID:-27}
# PX4 lockstep and ROS /clock share simulation time; Agent wall-clock conversion
# would make incoming setpoints appear stale by approximately one Unix epoch.
export PX4_PARAM_UXRCE_DDS_SYNCT=0
# Synthetic vision uses Gazebo truth for pose, height, velocity and yaw.
export PX4_PARAM_EKF2_EV_CTRL=15 PX4_PARAM_EKF2_EV_DELAY=0
export PX4_PARAM_EKF2_HGT_REF=3 PX4_PARAM_EKF2_GPS_CTRL=0
export PX4_PARAM_EKF2_BARO_CTRL=0 PX4_PARAM_EKF2_MAG_TYPE=5
PX4_RUN_DIR=$(mktemp -d "$EGO_SIM_ROOT/.runtime/px4/run.XXXXXX")
ln -s "$PWD/etc" "$PX4_RUN_DIR/etc"
cd "$PX4_RUN_DIR"
exec "$PX4_DIR/build/px4_sitl_default/bin/px4" -i 0 -d "$PWD" -s etc/init.d-posix/rcS
