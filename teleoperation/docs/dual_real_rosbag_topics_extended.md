# Dual-Real Teleoperation: rosbag estesa consigliata

Questo file contiene la versione estesa che consiglierei di registrare per il setup:

- `teleoperation/launch/master_slave_mur620b_mur620d_dual_real.launch`

La selezione e stata costruita leggendo il `launch`, i nodi:

- `teleop_master_haptic_controller_node.cpp`
- `teleop_slave_twist_outer_loop_node.cpp`

e confrontando i topic attivi a runtime con `rostopic list`.

L'obiettivo e avere una bag abbastanza ricca per:

- analisi sperimentale da articolo scientifico
- ricostruzione offline del comportamento del controller
- costruzione di un dataset per addestrare una rete neurale

senza arrivare a registrare tutti i topic del sistema.

## Lista finale da salvare

### 1. Loop teleoperation principale

- `/teleop/mur620b_to_mur620d/left/target_pose`
- `/teleop/mur620b_to_mur620d/right/target_pose`
- `/teleop/mur620b_to_mur620d/left/feedforward_twist`
- `/teleop/mur620b_to_mur620d/right/feedforward_twist`
- `/teleop/mur620b_to_mur620d/left/slave_actual_pose`
- `/teleop/mur620b_to_mur620d/right/slave_actual_pose`

Questi sono i topic piu importanti per ricostruire il mapping:

- intenzione master -> riferimento inviato allo slave
- stato reale slave -> feedback rimandato al master

### 2. Wrench raw dei 4 bracci

- `/mur620b/UR10_l/wrench`
- `/mur620b/UR10_r/wrench`
- `/mur620d/UR10_l/wrench`
- `/mur620d/UR10_r/wrench`

Servono per:

- contatto con l'ambiente
- interazione con l'oggetto
- apprendimento di policy force-aware
- confronto raw vs filtrato

### 3. Comandi finali verso i controller

- `/mur620b/UR10_l/twist_controller/command_collision_free`
- `/mur620b/UR10_r/twist_controller/command_collision_free`
- `/mur620d/UR10_l/twist_controller/command_collision_free`
- `/mur620d/UR10_r/twist_controller/command_collision_free`
- `/mur620b/UR10_l/twist_controller/command_collision_free_stamped`
- `/mur620b/UR10_r/twist_controller/command_collision_free_stamped`

Questi topic descrivono l'azione effettivamente generata dal controllo bilaterale.

### 4. Input al controller a valle

- `/mur620b/UR10_l/twist_controller/controller_input`
- `/mur620b/UR10_r/twist_controller/controller_input`
- `/mur620d/UR10_l/twist_controller/controller_input`
- `/mur620d/UR10_r/twist_controller/controller_input`

Li includerei per capire se il controller/cartesian layer a valle modifica il comando ricevuto oppure no.

### 5. Stato cartesiano reale dei TCP

- `/mur620b/UR10_l/global_tcp_pose`
- `/mur620b/UR10_r/global_tcp_pose`
- `/mur620d/UR10_l/global_tcp_pose`
- `/mur620d/UR10_r/global_tcp_pose`

Sono molto utili per:

- tracking error
- confronto target vs posa reale
- label cinematiche per dataset

### 6. Stato articolare dei robot

- `/mur620b/joint_states`
- `/mur620d/joint_states`
- `/mur620b/UR10_l/joint_states`
- `/mur620b/UR10_r/joint_states`
- `/mur620d/UR10_l/joint_states`
- `/mur620d/UR10_r/joint_states`
- `/mur620b/UR10_l/ewellix_tlt_node_l/joint_states_lift`
- `/mur620b/UR10_r/ewellix_tlt_node_r/joint_states_lift`
- `/mur620d/UR10_l/ewellix_tlt_node_l/joint_states_lift`
- `/mur620d/UR10_r/ewellix_tlt_node_r/joint_states_lift`

Qui c'e un po' di ridondanza, ma nella versione estesa la terrei: aiuta a non perdere dettaglio su bracci e lift.

### 7. Stato delle basi mobili

- `/mur620b/mir_pose_stamped_simple`
- `/mur620d/mir_pose_stamped_simple`
- `/qualisys_map/mur620b/pose`
- `/qualisys_map/mur620d/pose`

Li salverei per avere:

- posa base in frame locale del robot
- posa base in frame esterno/mocap
- contesto sperimentale riproducibile nel paper

### 8. Pose TCP da mocap disponibili

- `/mur620b/UR10_r/global_tcp_pose_mocap`
- `/mur620d/UR10_r/global_tcp_pose_mocap`

Sono extra ad alto valore se vuoi confrontare encoder/kinematica interna contro misura esterna.

### 9. Wrench filtrati usati dal controllo

- `/teleop_debug/mur620b_UR10_l/master_wrench_filtered`
- `/teleop_debug/mur620b_UR10_l/slave_wrench_filtered`
- `/teleop_debug/mur620b_UR10_r/master_wrench_filtered`
- `/teleop_debug/mur620b_UR10_r/slave_wrench_filtered`
- `/teleop_debug/mur620d_UR10_l/slave_wrench_filtered`
- `/teleop_debug/mur620d_UR10_r/slave_wrench_filtered`

Questi topic sono molto utili nel paper, perche mostrano il segnale effettivamente usato dal controller e non solo il segnale raw del sensore.

### 10. Debug interni del master controller

- `/teleop_master_haptic_controller_left/debug/admittance_stats`
- `/teleop_master_haptic_controller_left/debug/admittance_dynamics`
- `/teleop_master_haptic_controller_left/debug/dt_stats`
- `/teleop_master_haptic_controller_left/debug/v_cmd_pre`
- `/teleop_master_haptic_controller_left/debug/v_cmd_post`
- `/teleop_master_haptic_controller_left/debug/passivity_stats`
- `/teleop_master_haptic_controller_right/debug/admittance_stats`
- `/teleop_master_haptic_controller_right/debug/admittance_dynamics`
- `/teleop_master_haptic_controller_right/debug/dt_stats`
- `/teleop_master_haptic_controller_right/debug/v_cmd_pre`
- `/teleop_master_haptic_controller_right/debug/v_cmd_post`
- `/teleop_master_haptic_controller_right/debug/passivity_stats`

Li considero fondamentali nella versione estesa, perche permettono di capire:

- quando il controller satura
- quanta dinamica virtuale viene aggiunta
- se il timing del loop e regolare
- differenza tra comando prima e dopo i limiter
- eventuale attivazione del layer di passivity

Nota: nel tuo assetto attuale la passivity e disabilitata, ma registrare `passivity_stats` costa poco e puo essere utile in run futuri o per verificare che resti inattiva.

### 11. Stato hardware e safety dei 4 UR

- `/mur620b/UR10_l/ur_hardware_interface/robot_mode`
- `/mur620b/UR10_l/ur_hardware_interface/safety_mode`
- `/mur620b/UR10_l/ur_hardware_interface/robot_program_running`
- `/mur620b/UR10_r/ur_hardware_interface/robot_mode`
- `/mur620b/UR10_r/ur_hardware_interface/safety_mode`
- `/mur620b/UR10_r/ur_hardware_interface/robot_program_running`
- `/mur620d/UR10_l/ur_hardware_interface/robot_mode`
- `/mur620d/UR10_l/ur_hardware_interface/safety_mode`
- `/mur620d/UR10_l/ur_hardware_interface/robot_program_running`
- `/mur620d/UR10_r/ur_hardware_interface/robot_mode`
- `/mur620d/UR10_r/ur_hardware_interface/safety_mode`
- `/mur620d/UR10_r/ur_hardware_interface/robot_program_running`

Questi topic servono soprattutto a posteriori, per etichettare:

- safety stop
- fault hardware
- programma robot fermo/non fermo

### 12. TF da tenere

- `/tf_static`

Nella versione estesa non includo `/tf` di default per non far crescere troppo la bag. Con i topic gia selezionati hai comunque abbastanza informazione per un'analisi seria.

## Blocco unico pronto per la rosbag

```text
/teleop/mur620b_to_mur620d/left/target_pose
/teleop/mur620b_to_mur620d/right/target_pose
/teleop/mur620b_to_mur620d/left/feedforward_twist
/teleop/mur620b_to_mur620d/right/feedforward_twist
/teleop/mur620b_to_mur620d/left/slave_actual_pose
/teleop/mur620b_to_mur620d/right/slave_actual_pose
/mur620b/UR10_l/wrench
/mur620b/UR10_r/wrench
/mur620d/UR10_l/wrench
/mur620d/UR10_r/wrench
/mur620b/UR10_l/twist_controller/command_collision_free
/mur620b/UR10_r/twist_controller/command_collision_free
/mur620d/UR10_l/twist_controller/command_collision_free
/mur620d/UR10_r/twist_controller/command_collision_free
/mur620b/UR10_l/twist_controller/command_collision_free_stamped
/mur620b/UR10_r/twist_controller/command_collision_free_stamped
/mur620b/UR10_l/twist_controller/controller_input
/mur620b/UR10_r/twist_controller/controller_input
/mur620d/UR10_l/twist_controller/controller_input
/mur620d/UR10_r/twist_controller/controller_input
/mur620b/UR10_l/global_tcp_pose
/mur620b/UR10_r/global_tcp_pose
/mur620d/UR10_l/global_tcp_pose
/mur620d/UR10_r/global_tcp_pose
/mur620b/joint_states
/mur620d/joint_states
/mur620b/UR10_l/joint_states
/mur620b/UR10_r/joint_states
/mur620d/UR10_l/joint_states
/mur620d/UR10_r/joint_states
/mur620b/UR10_l/ewellix_tlt_node_l/joint_states_lift
/mur620b/UR10_r/ewellix_tlt_node_r/joint_states_lift
/mur620d/UR10_l/ewellix_tlt_node_l/joint_states_lift
/mur620d/UR10_r/ewellix_tlt_node_r/joint_states_lift
/mur620b/mir_pose_stamped_simple
/mur620d/mir_pose_stamped_simple
/qualisys_map/mur620b/pose
/qualisys_map/mur620d/pose
/mur620b/UR10_r/global_tcp_pose_mocap
/mur620d/UR10_r/global_tcp_pose_mocap
/teleop_debug/mur620b_UR10_l/master_wrench_filtered
/teleop_debug/mur620b_UR10_l/slave_wrench_filtered
/teleop_debug/mur620b_UR10_r/master_wrench_filtered
/teleop_debug/mur620b_UR10_r/slave_wrench_filtered
/teleop_debug/mur620d_UR10_l/slave_wrench_filtered
/teleop_debug/mur620d_UR10_r/slave_wrench_filtered
/teleop_master_haptic_controller_left/debug/admittance_stats
/teleop_master_haptic_controller_left/debug/admittance_dynamics
/teleop_master_haptic_controller_left/debug/dt_stats
/teleop_master_haptic_controller_left/debug/v_cmd_pre
/teleop_master_haptic_controller_left/debug/v_cmd_post
/teleop_master_haptic_controller_left/debug/passivity_stats
/teleop_master_haptic_controller_right/debug/admittance_stats
/teleop_master_haptic_controller_right/debug/admittance_dynamics
/teleop_master_haptic_controller_right/debug/dt_stats
/teleop_master_haptic_controller_right/debug/v_cmd_pre
/teleop_master_haptic_controller_right/debug/v_cmd_post
/teleop_master_haptic_controller_right/debug/passivity_stats
/mur620b/UR10_l/ur_hardware_interface/robot_mode
/mur620b/UR10_l/ur_hardware_interface/safety_mode
/mur620b/UR10_l/ur_hardware_interface/robot_program_running
/mur620b/UR10_r/ur_hardware_interface/robot_mode
/mur620b/UR10_r/ur_hardware_interface/safety_mode
/mur620b/UR10_r/ur_hardware_interface/robot_program_running
/mur620d/UR10_l/ur_hardware_interface/robot_mode
/mur620d/UR10_l/ur_hardware_interface/safety_mode
/mur620d/UR10_l/ur_hardware_interface/robot_program_running
/mur620d/UR10_r/ur_hardware_interface/robot_mode
/mur620d/UR10_r/ur_hardware_interface/safety_mode
/mur620d/UR10_r/ur_hardware_interface/robot_program_running
/tf_static
```

## Topic che non includerei in questa bag

- tutti i topic `move_group`, `planning_scene`, `pickup`, `place`
- topic RViz e marker di visualizzazione
- `parameter_descriptions` e `parameter_updates`
- `/rosout`, `/rosout_agg`, `/tf_old`
- topic di joystick e GUI, se non fanno parte del protocollo sperimentale
- `/tf`, perche in questo setup aumenta molto la dimensione della bag
- topic di coupling `coupling_wrench_filtered`, perche nel launch corrente `coupling_wrench_topic` e vuoto

## Nota importante per dataset NN

Per addestrare una rete neurale a manipolare l'oggetto in autonomia, questa bag e gia buona per:

- osservazioni robot-centriche
- azioni teleoperate
- forze di contatto
- stato dei controller

Ma il vero salto di qualita arriva se aggiungi anche un topic con la posa reale dell'oggetto manipolato.

Nel `rostopic list` corrente non ho visto un topic chiaramente dedicato alla pose dell'oggetto. Se in un secondo momento ne compare uno da vision o mocap, per me va aggiunto subito alla bag estesa.
