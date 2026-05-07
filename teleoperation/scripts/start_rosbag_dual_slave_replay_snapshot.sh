#!/usr/bin/env bash

# Usage:
# ./start_rosbag_dual_slave_replay_snapshot.sh replay_test_01

set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/pantanetti/catkin_ws}"
BAG_DIR="${BAG_DIR:-${HOME}/rosbags}"
LABEL_RAW="${1:-dual_slave_replay}"
LABEL="$(printf '%s' "${LABEL_RAW}" | tr -c '[:alnum:]_.-' '_')"
TIMESTAMP="$(date +%F_%H-%M-%S)"
RUN_NAME="${LABEL}_${TIMESTAMP}"
RUN_DIR="${BAG_DIR}/${RUN_NAME}"
BAG_PREFIX="${RUN_DIR}/${RUN_NAME}"

if [[ ! -f "${WORKSPACE}/devel/setup.bash" ]]; then
  echo "Workspace setup non trovato: ${WORKSPACE}/devel/setup.bash" >&2
  exit 1
fi

source "${WORKSPACE}/devel/setup.bash"

mkdir -p "${RUN_DIR}"

if ! rosnode list >/dev/null 2>&1; then
  echo "ROS master non raggiungibile. Avvia prima roscore/roslaunch e riprova." >&2
  exit 1
fi

TOPICS=(
  /teleop/dual_slave_replay/status
  /mur620d/UR10_l/wrench
  /mur620d/UR10_r/wrench
  /teleop_debug/mur620d_UR10_l/slave_wrench_filtered
  /teleop_debug/mur620d_UR10_r/slave_wrench_filtered
  /mur620d/UR10_l/twist_controller/command_collision_free
  /mur620d/UR10_r/twist_controller/command_collision_free
  /mur620d/UR10_l/global_tcp_pose
  /mur620d/UR10_r/global_tcp_pose
  /mur620d/UR10_l/joint_states
  /mur620d/UR10_r/joint_states
  /mur620d/mir_pose_stamped_simple
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
  echo "ros_master_uri=${ROS_MASTER_URI:-}"
  echo "ros_ip=${ROS_IP:-}"
  echo "ros_hostname=${ROS_HOSTNAME:-}"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -a)"
} > "${RUN_DIR}/run_metadata.env"

printf '%s\n' "${TOPICS[@]}" > "${RUN_DIR}/recorded_topics.txt"

echo "Salvataggio replay in: ${RUN_DIR}"
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
  echo "Replay salvato in: ${RUN_DIR}"
}

trap stop_recording INT TERM
wait "${ROSBAG_PID}"
trap - INT TERM
echo "Snapshot parametri finale..."
dump_metadata "after"
echo "Replay salvato in: ${RUN_DIR}"
