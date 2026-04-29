#!/usr/bin/env bash
set -euo pipefail

MUR_NS="${MUR_NS:-mur620b}"
ARM="${ARM:-right}"
BAG_DIR="${BAG_DIR:-$HOME/rosbags/whole_body_print}"
PREFIX="${PREFIX:-whole_body_print_${MUR_NS}}"
SPLIT_SIZE_MB="${SPLIT_SIZE_MB:-2048}"

if [[ "${ARM}" == "right" ]]; then
  ARM_SUFFIX="r"
  ARM_NS="UR10_r"
elif [[ "${ARM}" == "left" ]]; then
  ARM_SUFFIX="l"
  ARM_NS="UR10_l"
else
  echo "Unsupported ARM='${ARM}'. Use ARM=right or ARM=left." >&2
  exit 2
fi

mkdir -p "${BAG_DIR}"

TOPICS=(
  "/tf"
  "/tf_static"

  "/${MUR_NS}/joint_states"
  "/${MUR_NS}/${ARM_NS}/joint_states"
  "/${MUR_NS}/cmd_vel"
  "/${MUR_NS}/mobile_base_controller/odom"
  "/${MUR_NS}/mir_pose_simple"
  "/${MUR_NS}/mir_pose_stamped_simple"
  "/${MUR_NS}/robot_pose"

  "/${MUR_NS}/mir/f_scan"
  "/${MUR_NS}/mir/b_scan"
  "/${MUR_NS}/mir/scan"

  "/${MUR_NS}/whole_body_print_controller/debug"
  "/${MUR_NS}/whole_body_print_controller/input_target_state"
  "/${MUR_NS}/whole_body_print_controller/path_marker"
  "/${MUR_NS}/whole_body_print_controller/current_marker"

  "/${MUR_NS}/tcp_path_trajectory_manager/progress"
  "/${MUR_NS}/tcp_path_trajectory_manager/speed_scale"
  "/${MUR_NS}/tcp_path_trajectory_manager/tracking_error"
  "/${MUR_NS}/tcp_path_trajectory_manager/tracking_guard_active"
  "/${MUR_NS}/tcp_path_trajectory_manager/tracking_guard_state"
  "/${MUR_NS}/tcp_path_trajectory_manager/path_marker"
  "/${MUR_NS}/tcp_path_trajectory_manager/current_marker"

  "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/target_pose"
  "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/target_state"
  "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/end_effector_state"
  "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/pipeline_debug"
  "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/joint_velocity_feedback"
)

echo "Recording whole-body print bag"
echo "  namespace: /${MUR_NS}"
echo "  arm:       ${ARM}"
echo "  output:    ${BAG_DIR}/${PREFIX}_*.bag"
echo "  split:     ${SPLIT_SIZE_MB} MB"
echo
echo "Stop recording with Ctrl-C."

exec rosbag record \
  --output-prefix="${BAG_DIR}/${PREFIX}" \
  --split \
  --size="${SPLIT_SIZE_MB}" \
  "${TOPICS[@]}"
