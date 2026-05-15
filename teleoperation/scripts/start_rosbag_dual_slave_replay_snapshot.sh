#!/usr/bin/env bash

# Usage:
# ./start_rosbag_dual_slave_replay_snapshot.sh replay_test_01

set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/pantanetti/catkin_ws}"
BAG_DIR="${BAG_DIR:-${HOME}/rosbags}"
REPLAY_PARAM_NS="${REPLAY_PARAM_NS:-/replay_dual_slave_twist_from_bag}"
REPLAY_PARAM_WAIT_ATTEMPTS="${REPLAY_PARAM_WAIT_ATTEMPTS:-20}"
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

clean_rosparam_scalar() {
  local value="$1"
  case "${value}" in
    "''" | '""')
      value=""
      ;;
  esac
  printf '%s' "${value}"
}

get_replay_param() {
  local key="$1"
  rosparam get "${REPLAY_PARAM_NS}/${key}" 2>/dev/null || true
}

wait_for_replay_manifest_param() {
  local value=""
  local i=0
  for ((i = 0; i < REPLAY_PARAM_WAIT_ATTEMPTS; ++i)); do
    value="$(clean_rosparam_scalar "$(get_replay_param manifest)")"
    if [[ -n "${value}" ]]; then
      printf '%s' "${value}"
      return 0
    fi
    sleep 0.25
  done
  return 0
}

read_manifest_bag_path() {
  local manifest_path="$1"
  if [[ -z "${manifest_path}" || ! -f "${manifest_path}" ]]; then
    return 0
  fi
  python3 -c 'import sys, yaml
with open(sys.argv[1], "r") as f:
    data = yaml.safe_load(f) or {}
value = data.get("bag", "") if isinstance(data, dict) else ""
print(value or "")' "${manifest_path}" 2>/dev/null || true
}

SOURCE_MANIFEST="$(wait_for_replay_manifest_param)"
SOURCE_BAG_PARAM="$(clean_rosparam_scalar "$(get_replay_param bag)")"
SOURCE_BAG_FROM_MANIFEST="$(read_manifest_bag_path "${SOURCE_MANIFEST}")"
SOURCE_BAG="${SOURCE_BAG_PARAM:-${SOURCE_BAG_FROM_MANIFEST}}"
SOURCE_MANIFEST_COPY=""

if [[ -n "${SOURCE_MANIFEST}" && -f "${SOURCE_MANIFEST}" ]]; then
  SOURCE_MANIFEST_COPY="${RUN_DIR}/source_replay_manifest.yaml"
  cp -f "${SOURCE_MANIFEST}" "${SOURCE_MANIFEST_COPY}"
else
  echo "WARNING: manifest replay non trovato nei parametri (${REPLAY_PARAM_NS}/manifest)." >&2
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
  echo "replay_param_ns=${REPLAY_PARAM_NS}"
  echo "source_manifest=${SOURCE_MANIFEST}"
  echo "source_manifest_copy=${SOURCE_MANIFEST_COPY}"
  echo "source_bag_param=${SOURCE_BAG_PARAM}"
  echo "source_bag_manifest=${SOURCE_BAG_FROM_MANIFEST}"
  echo "source_bag=${SOURCE_BAG}"
  echo "ros_master_uri=${ROS_MASTER_URI:-}"
  echo "ros_ip=${ROS_IP:-}"
  echo "ros_hostname=${ROS_HOSTNAME:-}"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -a)"
} > "${RUN_DIR}/run_metadata.env"

{
  echo "replay_param_ns=${REPLAY_PARAM_NS}"
  echo "source_manifest=${SOURCE_MANIFEST}"
  echo "source_manifest_copy=${SOURCE_MANIFEST_COPY}"
  echo "source_bag_param=${SOURCE_BAG_PARAM}"
  echo "source_bag_manifest=${SOURCE_BAG_FROM_MANIFEST}"
  echo "source_bag=${SOURCE_BAG}"
} > "${RUN_DIR}/source_replay_metadata.env"

printf '%s\n' "${TOPICS[@]}" > "${RUN_DIR}/recorded_topics.txt"

echo "Salvataggio replay in: ${RUN_DIR}"
echo "Bag prefix: ${BAG_PREFIX}"
echo "Manifest sorgente: ${SOURCE_MANIFEST:-non trovato}"
echo "Bag sorgente: ${SOURCE_BAG:-non trovata}"
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
