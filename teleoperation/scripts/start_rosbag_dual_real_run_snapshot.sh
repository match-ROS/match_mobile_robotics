#!/usr/bin/env bash

# Uso:
# ./start_rosbag_dual_real_run_snapshot.sh prova_contatto_01

set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/pantanetti/catkin_ws}"
BAG_DIR="${BAG_DIR:-${HOME}/rosbags}"
LABEL_RAW="${1:-dual_real}"
LABEL="$(printf '%s' "${LABEL_RAW}" | tr -c '[:alnum:]_.-' '_')"
TIMESTAMP="$(date +%F_%H-%M-%S)"
RUN_NAME="${LABEL}_${TIMESTAMP}"
RUN_DIR="${BAG_DIR}/${RUN_NAME}"
BAG_PREFIX="${RUN_DIR}/${RUN_NAME}"

TELEOP_DIR="${WORKSPACE}/src/match_mobile_robotics/teleoperation"
LAUNCH_FILE="${TELEOP_DIR}/launch/master_slave_mur620b_mur620d_dual_real.launch"
MASTER_CONFIG="${TELEOP_DIR}/config/master_real_mur620b_ur10_l.yaml"
SLAVE_CONFIG="${TELEOP_DIR}/config/slave_real_mur620b_ur10_r.yaml"

if [[ ! -f "${WORKSPACE}/devel/setup.bash" ]]; then
  echo "Workspace setup non trovato: ${WORKSPACE}/devel/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${LAUNCH_FILE}" ]]; then
  echo "Launch file non trovato: ${LAUNCH_FILE}" >&2
  exit 1
fi

source "${WORKSPACE}/devel/setup.bash"

mkdir -p "${RUN_DIR}/config_snapshot"

if ! rosnode list >/dev/null 2>&1; then
  echo "ROS master non raggiungibile. Avvia prima roscore/roslaunch e riprova." >&2
  exit 1
fi

TOPICS=(
  /teleop/mur620b_to_mur620d/left/target_pose
  /teleop/mur620b_to_mur620d/right/target_pose
  /teleop/mur620b_to_mur620d/left/feedforward_twist
  /teleop/mur620b_to_mur620d/right/feedforward_twist
  /teleop/mur620b_to_mur620d/left/slave_actual_pose
  /teleop/mur620b_to_mur620d/right/slave_actual_pose
  /teleop/mur620b_to_mur620d/left/home_return/status
  /teleop/mur620b_to_mur620d/right/home_return/status
  /mur620b/UR10_l/wrench
  /mur620b/UR10_r/wrench
  /mur620d/UR10_l/wrench
  /mur620d/UR10_r/wrench
  /mur620b/UR10_l/twist_controller/command_collision_free
  /mur620b/UR10_r/twist_controller/command_collision_free
  /mur620d/UR10_l/twist_controller/command_collision_free
  /mur620d/UR10_r/twist_controller/command_collision_free
  /mur620b/UR10_l/global_tcp_pose
  /mur620b/UR10_r/global_tcp_pose
  /mur620d/UR10_l/global_tcp_pose
  /mur620d/UR10_r/global_tcp_pose
  /mur620b/UR10_l/joint_states
  /mur620b/UR10_r/joint_states
  /mur620d/UR10_l/joint_states
  /mur620d/UR10_r/joint_states
  /mur620b/mir_pose_stamped_simple
  /mur620d/mir_pose_stamped_simple
  /teleop_debug/mur620b_UR10_l/master_wrench_filtered
  /teleop_debug/mur620b_UR10_l/slave_wrench_filtered
  /teleop_debug/mur620b_UR10_r/master_wrench_filtered
  /teleop_debug/mur620b_UR10_r/slave_wrench_filtered
  /teleop_master_haptic_controller_left/debug/admittance_stats
  /teleop_master_haptic_controller_left/debug/admittance_dynamics
  /teleop_master_haptic_controller_left/debug/dt_stats
  /teleop_master_haptic_controller_left/debug/v_cmd_pre
  /teleop_master_haptic_controller_left/debug/v_cmd_post
  /teleop_master_haptic_controller_right/debug/admittance_stats
  /teleop_master_haptic_controller_right/debug/admittance_dynamics
  /teleop_master_haptic_controller_right/debug/dt_stats
  /teleop_master_haptic_controller_right/debug/v_cmd_pre
  /teleop_master_haptic_controller_right/debug/v_cmd_post
  /tf_static
  /tf
)

dump_metadata() {
  local phase="$1"

  rosparam dump "${RUN_DIR}/rosparams_${phase}.yaml" || true
  rosparam list > "${RUN_DIR}/rosparam_list_${phase}.txt" || true
  rosnode list > "${RUN_DIR}/rosnode_list_${phase}.txt" || true
  rostopic list -v > "${RUN_DIR}/rostopic_list_${phase}.txt" || true
  rosservice list > "${RUN_DIR}/rosservice_list_${phase}.txt" || true
}

{
  echo "run_name=${RUN_NAME}"
  echo "timestamp=${TIMESTAMP}"
  echo "label=${LABEL_RAW}"
  echo "workspace=${WORKSPACE}"
  echo "bag_prefix=${BAG_PREFIX}"
  echo "launch_file=${LAUNCH_FILE}"
  echo "master_config=${MASTER_CONFIG}"
  echo "slave_config=${SLAVE_CONFIG}"
  echo "ros_master_uri=${ROS_MASTER_URI:-}"
  echo "ros_ip=${ROS_IP:-}"
  echo "ros_hostname=${ROS_HOSTNAME:-}"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -a)"
} > "${RUN_DIR}/run_metadata.env"

printf '%s\n' "${TOPICS[@]}" > "${RUN_DIR}/recorded_topics.txt"

cp "${LAUNCH_FILE}" "${RUN_DIR}/config_snapshot/" 2>/dev/null || true
cp "${MASTER_CONFIG}" "${RUN_DIR}/config_snapshot/" 2>/dev/null || true
cp "${SLAVE_CONFIG}" "${RUN_DIR}/config_snapshot/" 2>/dev/null || true

echo "Salvataggio run in: ${RUN_DIR}"
echo "Bag prefix: ${BAG_PREFIX}"
echo "Topic registrati: ${#TOPICS[@]}"
echo "Snapshot parametri iniziale..."
dump_metadata "before"
echo "Premi Ctrl-C per fermare la registrazione in modo pulito."

rosbag record \
  --lz4 \
  --split --size=4096 \
  -O "${BAG_PREFIX}.bag" \
  "${TOPICS[@]}" &

ROSBAG_PID=$!

stop_recording() {
  echo
  echo "Arresto rosbag..."
  kill -INT "${ROSBAG_PID}" 2>/dev/null || true
  wait "${ROSBAG_PID}" 2>/dev/null || true
  echo "Snapshot parametri finale..."
  dump_metadata "after"
  echo "Run salvata in: ${RUN_DIR}"
}

trap stop_recording INT TERM
wait "${ROSBAG_PID}"
trap - INT TERM
echo "Snapshot parametri finale..."
dump_metadata "after"
echo "Run salvata in: ${RUN_DIR}"
