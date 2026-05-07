#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/pantanetti/catkin_ws"
BAG_DIR="${HOME}/Rosbag"
TIMESTAMP="$(date +%F_%H-%M-%S)"
BAG_PATH="${BAG_DIR}/dual_real_${TIMESTAMP}.bag"

if [[ ! -f "${WORKSPACE}/devel/setup.bash" ]]; then
  echo "Workspace setup non trovato: ${WORKSPACE}/devel/setup.bash" >&2
  exit 1
fi

if [[ ! -d "${BAG_DIR}" ]]; then
  echo "Cartella di salvataggio non trovata: ${BAG_DIR}" >&2
  exit 1
fi

source "${WORKSPACE}/devel/setup.bash"

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
  #/mur620b/UR10_l/twist_controller/command_collision_free_stamped
  #/mur620b/UR10_r/twist_controller/command_collision_free_stamped
  #/mur620b/UR10_l/twist_controller/controller_input
  #/mur620b/UR10_r/twist_controller/controller_input
  #/mur620d/UR10_l/twist_controller/controller_input
  #/mur620d/UR10_r/twist_controller/controller_input
  /mur620b/UR10_l/global_tcp_pose
  /mur620b/UR10_r/global_tcp_pose
  /mur620d/UR10_l/global_tcp_pose
  /mur620d/UR10_r/global_tcp_pose
  #/mur620b/joint_states
  #/mur620d/joint_states
  /mur620b/UR10_l/joint_states
  /mur620b/UR10_r/joint_states
  /mur620d/UR10_l/joint_states
  /mur620d/UR10_r/joint_states
  #/mur620b/UR10_l/ewellix_tlt_node_l/joint_states_lift
  #/mur620b/UR10_r/ewellix_tlt_node_r/joint_states_lift
  #/mur620d/UR10_l/ewellix_tlt_node_l/joint_states_lift
  #/mur620d/UR10_r/ewellix_tlt_node_r/joint_states_lift
  /mur620b/mir_pose_stamped_simple
  /mur620d/mir_pose_stamped_simple
  #/qualisys_map/mur620b/pose
  #/qualisys_map/mur620d/pose
  #/mur620b/UR10_r/global_tcp_pose_mocap
  #/mur620d/UR10_r/global_tcp_pose_mocap
  /teleop_debug/mur620b_UR10_l/master_wrench_filtered
  /teleop_debug/mur620b_UR10_l/slave_wrench_filtered
  /teleop_debug/mur620b_UR10_r/master_wrench_filtered
  /teleop_debug/mur620b_UR10_r/slave_wrench_filtered
  #/teleop_debug/mur620d_UR10_l/slave_wrench_filtered
  #/teleop_debug/mur620d_UR10_r/slave_wrench_filtered
  /teleop_master_haptic_controller_left/debug/admittance_stats
  /teleop_master_haptic_controller_left/debug/admittance_dynamics
  /teleop_master_haptic_controller_left/debug/dt_stats
  /teleop_master_haptic_controller_left/debug/v_cmd_pre
  /teleop_master_haptic_controller_left/debug/v_cmd_post
  #/teleop_master_haptic_controller_left/debug/passivity_stats
  /teleop_master_haptic_controller_right/debug/admittance_stats
  /teleop_master_haptic_controller_right/debug/admittance_dynamics
  /teleop_master_haptic_controller_right/debug/dt_stats
  /teleop_master_haptic_controller_right/debug/v_cmd_pre
  /teleop_master_haptic_controller_right/debug/v_cmd_post
  #/teleop_master_haptic_controller_right/debug/passivity_stats
  #/mur620b/UR10_l/ur_hardware_interface/robot_mode
  #/mur620b/UR10_l/ur_hardware_interface/safety_mode
  #/mur620b/UR10_l/ur_hardware_interface/robot_program_running
  #/mur620b/UR10_r/ur_hardware_interface/robot_mode
  #/mur620b/UR10_r/ur_hardware_interface/safety_mode
  #/mur620b/UR10_r/ur_hardware_interface/robot_program_running
  #/mur620d/UR10_l/ur_hardware_interface/robot_mode
  #/mur620d/UR10_l/ur_hardware_interface/safety_mode
  #/mur620d/UR10_l/ur_hardware_interface/robot_program_running
  #/mur620d/UR10_r/ur_hardware_interface/robot_mode
  #/mur620d/UR10_r/ur_hardware_interface/safety_mode
  #/mur620d/UR10_r/ur_hardware_interface/robot_program_running
  /tf_static
  /tf
)

echo "Salvataggio rosbag in: ${BAG_PATH}"
echo "Topic registrati: ${#TOPICS[@]}"
echo "Premi Ctrl-C per fermare la registrazione in modo pulito."

exec rosbag record \
  --lz4 \
  --split --size=4096 \
  -O "${BAG_PATH}" \
  "${TOPICS[@]}"
