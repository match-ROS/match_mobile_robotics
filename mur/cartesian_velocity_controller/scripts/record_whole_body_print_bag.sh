#!/usr/bin/env bash
set -euo pipefail

MUR_NS="${MUR_NS:-mur620b}"
ARM="${ARM:-right}"
BAG_DIR="${BAG_DIR:-$HOME/rosbags/whole_body_print}"
PREFIX="${PREFIX:-whole_body_print_${MUR_NS}}"
SPLIT_SIZE_MB="${SPLIT_SIZE_MB:-2048}"
COMPRESSION="${COMPRESSION:-lz4}"
RECORD_SCANS="${RECORD_SCANS:-false}"
RECORD_ARM_MARKERS="${RECORD_ARM_MARKERS:-false}"
RECORD_LEGACY_MANAGER="${RECORD_LEGACY_MANAGER:-false}"
SNAPSHOT_PARAMS="${SNAPSHOT_PARAMS:-true}"

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
  "/rosout"
  "/tf"
  "/tf_static"

  "/${MUR_NS}/joint_states"
  "/${MUR_NS}/${ARM_NS}/joint_states"
  "/${MUR_NS}/cmd_vel"
  "/${MUR_NS}/${ARM_NS}/joint_group_vel_controller/command"
  "/${MUR_NS}/mobile_base_controller/odom"
  "/${MUR_NS}/mir_pose_simple"
  "/${MUR_NS}/mir_pose_stamped_simple"
  "/${MUR_NS}/robot_pose"

  "/${MUR_NS}/whole_body_print_controller/debug"
  "/${MUR_NS}/whole_body_print_controller/path_marker"
  "/${MUR_NS}/whole_body_print_controller/current_marker"
  "/${MUR_NS}/whole_body_print_controller/preferred_tcp_marker"
  "/${MUR_NS}/whole_body_print_controller/tcp_tracking_zone_markers"
  "/${MUR_NS}/whole_body_print_controller/reaction_perimeter_markers"
  "/${MUR_NS}/whole_body_print_controller/simulated_obstacle_markers"

  "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/target_pose"
  "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/target_state"
  "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/end_effector_state"
  "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/pipeline_debug"
  "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/joint_velocity_feedback"
)

if [[ "${RECORD_SCANS}" == "true" ]]; then
  TOPICS+=(
    "/${MUR_NS}/mir/f_scan"
    "/${MUR_NS}/mir/b_scan"
    "/${MUR_NS}/mir/scan"
  )
fi

if [[ "${RECORD_ARM_MARKERS}" == "true" ]]; then
  TOPICS+=(
    "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/velocity_markers"
    "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/target_markers"
    "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/command_markers"
    "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/repulsion_markers"
    "/${MUR_NS}/cartesian_velocity_controller_${ARM_SUFFIX}/tcp_fk_markers"
  )
fi

if [[ "${RECORD_LEGACY_MANAGER}" == "true" ]]; then
  TOPICS+=(
    "/${MUR_NS}/whole_body_print_controller/input_target_state"
    "/${MUR_NS}/tcp_path_trajectory_manager/progress"
    "/${MUR_NS}/tcp_path_trajectory_manager/speed_scale"
    "/${MUR_NS}/tcp_path_trajectory_manager/tracking_error"
    "/${MUR_NS}/tcp_path_trajectory_manager/tracking_guard_active"
    "/${MUR_NS}/tcp_path_trajectory_manager/tracking_guard_state"
    "/${MUR_NS}/tcp_path_trajectory_manager/path_marker"
    "/${MUR_NS}/tcp_path_trajectory_manager/current_marker"
  )
fi

if [[ "${SNAPSHOT_PARAMS}" == "true" ]]; then
  SNAPSHOT_FILE="${BAG_DIR}/${PREFIX}_params_$(date +%Y%m%d_%H%M%S).yaml"
  rosparam dump "${SNAPSHOT_FILE}" "/${MUR_NS}" || {
    echo "Warning: failed to dump /${MUR_NS} params to ${SNAPSHOT_FILE}" >&2
  }
fi

echo "Recording whole-body print bag"
echo "  namespace: /${MUR_NS}"
echo "  arm:       ${ARM}"
echo "  output:    ${BAG_DIR}/${PREFIX}_*.bag"
echo "  split:     ${SPLIT_SIZE_MB} MB"
echo "  compress:  ${COMPRESSION}"
echo "  scans:     ${RECORD_SCANS}"
echo "  arm marks: ${RECORD_ARM_MARKERS}"
echo
echo "Stop recording with Ctrl-C."

ROSbag_ARGS=(
  record
  --output-prefix="${BAG_DIR}/${PREFIX}" \
  --split \
  --size="${SPLIT_SIZE_MB}"
)

if [[ "${COMPRESSION}" == "lz4" ]]; then
  ROSbag_ARGS+=(--lz4)
elif [[ "${COMPRESSION}" == "bz2" ]]; then
  ROSbag_ARGS+=(--bz2)
elif [[ "${COMPRESSION}" != "none" ]]; then
  echo "Unsupported COMPRESSION='${COMPRESSION}'. Use lz4, bz2, or none." >&2
  exit 2
fi

exec rosbag "${ROSbag_ARGS[@]}" "${TOPICS[@]}"
