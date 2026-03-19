# Dual-Real Teleoperation: topic da salvare in rosbag

Questo documento riassume i topic utili da salvare durante i test con il launch:

- `teleoperation/launch/master_slave_mur620b_mur620d_dual_real.launch`

L'obiettivo e avere una lista unica, pronta per la rosbag, con:

- topic sicuramente pubblicati dal setup teleoperation
- topic extra consigliati per ricostruzione offline
- significato dei messaggi
- significato dettagliato dei campi nei topic con payload generico (`Float64MultiArray`)

## Modifiche applicate

Per rendere il logging piu pulito nel setup dual-arm sono state applicate queste modifiche:

- i debug del master `admittance_stats`, `dt_stats`, `v_cmd_pre`, `v_cmd_post` ora sono privati del nodo, quindi non si mischiano piu tra left e right
- nel launch dual-real e stato abilitato `publish_filtered_wrench_debug` sui due nodi slave, quindi i topic di wrench filtrato dello slave vengono effettivamente pubblicati

I topic debug del master diventano quindi:

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

## Lista topic consigliata per rosbag

### 1) Topic teleoperation da salvare sicuramente

#### Wrench raw sensori FT

- `/mur620b/UR10_l/wrench`
- `/mur620b/UR10_r/wrench`
- `/mur620d/UR10_l/wrench`
- `/mur620d/UR10_r/wrench`

#### Comandi twist pubblicati dai controller

- `/mur620b/UR10_l/twist_controller/command_collision_free`
- `/mur620b/UR10_r/twist_controller/command_collision_free`
- `/mur620d/UR10_l/twist_controller/command_collision_free`
- `/mur620d/UR10_r/twist_controller/command_collision_free`

#### Comandi twist con timestamp del master

- `/mur620b/UR10_l/twist_controller/command_collision_free_stamped`
- `/mur620b/UR10_r/twist_controller/command_collision_free_stamped`

#### Traiettoria master -> slave

- `/teleop/mur620b_to_mur620d/left/target_pose`
- `/teleop/mur620b_to_mur620d/right/target_pose`
- `/teleop/mur620b_to_mur620d/left/feedforward_twist`
- `/teleop/mur620b_to_mur620d/right/feedforward_twist`

#### Feedback slave -> master per molla virtuale

- `/teleop/mur620b_to_mur620d/left/slave_actual_pose`
- `/teleop/mur620b_to_mur620d/right/slave_actual_pose`

#### Wrench filtrati di debug

- `/teleop_debug/mur620b_UR10_l/master_wrench_filtered`
- `/teleop_debug/mur620b_UR10_l/slave_wrench_filtered`
- `/teleop_debug/mur620b_UR10_r/master_wrench_filtered`
- `/teleop_debug/mur620b_UR10_r/slave_wrench_filtered`
- `/teleop_debug/mur620d_UR10_l/slave_wrench_filtered`
- `/teleop_debug/mur620d_UR10_r/slave_wrench_filtered`

#### Debug master left/right

- `/teleop_master_haptic_controller_left/debug/admittance_stats`
- `/teleop_master_haptic_controller_left/debug/admittance_dynamics`
- `/teleop_master_haptic_controller_left/debug/dt_stats`
- `/teleop_master_haptic_controller_left/debug/v_cmd_pre`
- `/teleop_master_haptic_controller_left/debug/v_cmd_post`
- `/teleop_master_haptic_controller_right/debug/admittance_stats`
- `/teleop_master_haptic_controller_right/debug/admittance_dynamics`
- `/teleop_master_haptic_controller_right/debug/dt_stats`
- `/teleop_master_haptic_controller_right/debug/v_cmd_pre`
- `/teleop_master_haptic_controller_right/debug/v_cmd_post`

### 2) Topic opzionali ma molto consigliati

Questi non appartengono direttamente ai nodi teleoperation, ma sono molto utili per analisi offline:

- `/tf`
- `/tf_static`
- `/mur620b/joint_states`
- `/mur620d/joint_states`

Se il tuo bringup espone altri topic di stato controller, vale la pena salvarli anche loro.

### 3) Topic opzionali solo se usi passivity debug

Nel tuo YAML attuale `passivity/enabled` e `false`, quindi questi topic non sono prioritari per i test standard. Se vuoi comunque monitorare il layer:

- `/teleop_master_haptic_controller_left/debug/passivity_stats`
- `/teleop_master_haptic_controller_right/debug/passivity_stats`

## Esempio comando rosbag

Adatta il path di output come preferisci:

```bash
rosbag record -O dual_real_teleop_test.bag \
  /mur620b/UR10_l/wrench \
  /mur620b/UR10_r/wrench \
  /mur620d/UR10_l/wrench \
  /mur620d/UR10_r/wrench \
  /mur620b/UR10_l/twist_controller/command_collision_free \
  /mur620b/UR10_r/twist_controller/command_collision_free \
  /mur620d/UR10_l/twist_controller/command_collision_free \
  /mur620d/UR10_r/twist_controller/command_collision_free \
  /mur620b/UR10_l/twist_controller/command_collision_free_stamped \
  /mur620b/UR10_r/twist_controller/command_collision_free_stamped \
  /teleop/mur620b_to_mur620d/left/target_pose \
  /teleop/mur620b_to_mur620d/right/target_pose \
  /teleop/mur620b_to_mur620d/left/feedforward_twist \
  /teleop/mur620b_to_mur620d/right/feedforward_twist \
  /teleop/mur620b_to_mur620d/left/slave_actual_pose \
  /teleop/mur620b_to_mur620d/right/slave_actual_pose \
  /teleop_debug/mur620b_UR10_l/master_wrench_filtered \
  /teleop_debug/mur620b_UR10_l/slave_wrench_filtered \
  /teleop_debug/mur620b_UR10_r/master_wrench_filtered \
  /teleop_debug/mur620b_UR10_r/slave_wrench_filtered \
  /teleop_debug/mur620d_UR10_l/slave_wrench_filtered \
  /teleop_debug/mur620d_UR10_r/slave_wrench_filtered \
  /teleop_master_haptic_controller_left/debug/admittance_stats \
  /teleop_master_haptic_controller_left/debug/admittance_dynamics \
  /teleop_master_haptic_controller_left/debug/dt_stats \
  /teleop_master_haptic_controller_left/debug/v_cmd_pre \
  /teleop_master_haptic_controller_left/debug/v_cmd_post \
  /teleop_master_haptic_controller_right/debug/admittance_stats \
  /teleop_master_haptic_controller_right/debug/admittance_dynamics \
  /teleop_master_haptic_controller_right/debug/dt_stats \
  /teleop_master_haptic_controller_right/debug/v_cmd_pre \
  /teleop_master_haptic_controller_right/debug/v_cmd_post \
  /tf \
  /tf_static \
  /mur620b/joint_states \
  /mur620d/joint_states
```

## Cosa contiene ogni topic

## Topic `geometry_msgs/WrenchStamped`

Questi topic hanno tutti la stessa struttura:

- `/mur620b/UR10_l/wrench`
- `/mur620b/UR10_r/wrench`
- `/mur620d/UR10_l/wrench`
- `/mur620d/UR10_r/wrench`
- `/teleop_debug/mur620b_UR10_l/master_wrench_filtered`
- `/teleop_debug/mur620b_UR10_l/slave_wrench_filtered`
- `/teleop_debug/mur620b_UR10_r/master_wrench_filtered`
- `/teleop_debug/mur620b_UR10_r/slave_wrench_filtered`
- `/teleop_debug/mur620d_UR10_l/slave_wrench_filtered`
- `/teleop_debug/mur620d_UR10_r/slave_wrench_filtered`

### Campi

- `header.stamp`: timestamp ROS del campione
- `header.frame_id`: frame in cui il wrench e espresso
- `wrench.force.x`: forza asse x `[N]`
- `wrench.force.y`: forza asse y `[N]`
- `wrench.force.z`: forza asse z `[N]`
- `wrench.torque.x`: coppia asse x `[N*m]`
- `wrench.torque.y`: coppia asse y `[N*m]`
- `wrench.torque.z`: coppia asse z `[N*m]`

### Significato pratico

- i topic `.../wrench` sono i segnali raw del sensore FT
- i topic `...filtered...` sono i wrench gia filtrati e ruotati nel frame usato dal controller
- i filtered wrench del master sono molto utili per confrontare raw vs segnale realmente usato dall'anello di controllo

## Topic `geometry_msgs/PoseStamped`

Questi topic hanno tutti la stessa struttura:

- `/teleop/mur620b_to_mur620d/left/target_pose`
- `/teleop/mur620b_to_mur620d/right/target_pose`
- `/teleop/mur620b_to_mur620d/left/slave_actual_pose`
- `/teleop/mur620b_to_mur620d/right/slave_actual_pose`

### Campi

- `header.stamp`: timestamp ROS
- `header.frame_id`: frame in cui la posa e espressa
- `pose.position.x`: coordinata x `[m]`
- `pose.position.y`: coordinata y `[m]`
- `pose.position.z`: coordinata z `[m]`
- `pose.orientation.x`: quaternione x
- `pose.orientation.y`: quaternione y
- `pose.orientation.z`: quaternione z
- `pose.orientation.w`: quaternione w

### Significato pratico

- `target_pose`: posa target che il master manda allo slave
- `slave_actual_pose`: posa reale del TCP slave rimandata al master per la molla virtuale

## Topic `geometry_msgs/TwistStamped`

Questi topic hanno tutti la stessa struttura:

- `/teleop/mur620b_to_mur620d/left/feedforward_twist`
- `/teleop/mur620b_to_mur620d/right/feedforward_twist`
- `/mur620b/UR10_l/twist_controller/command_collision_free_stamped`
- `/mur620b/UR10_r/twist_controller/command_collision_free_stamped`
- `/teleop_master_haptic_controller_left/debug/v_cmd_pre`
- `/teleop_master_haptic_controller_left/debug/v_cmd_post`
- `/teleop_master_haptic_controller_right/debug/v_cmd_pre`
- `/teleop_master_haptic_controller_right/debug/v_cmd_post`

### Campi

- `header.stamp`: timestamp ROS
- `header.frame_id`: frame in cui la twist e espressa
- `twist.linear.x`: velocita lineare x `[m/s]`
- `twist.linear.y`: velocita lineare y `[m/s]`
- `twist.linear.z`: velocita lineare z `[m/s]`
- `twist.angular.x`: velocita angolare x `[rad/s]`
- `twist.angular.y`: velocita angolare y `[rad/s]`
- `twist.angular.z`: velocita angolare z `[rad/s]`

### Significato pratico

- `feedforward_twist`: twist che il master invia come feedforward allo slave
- `command_collision_free_stamped`: stessa idea del comando master, ma con header utile per analisi offline
- `v_cmd_pre`: twist prima dei clamp/limiti finali del master
- `v_cmd_post`: twist finale dopo limitazioni e saturazioni del master

## Topic `geometry_msgs/Twist`

Questi topic hanno tutti la stessa struttura:

- `/mur620b/UR10_l/twist_controller/command_collision_free`
- `/mur620b/UR10_r/twist_controller/command_collision_free`
- `/mur620d/UR10_l/twist_controller/command_collision_free`
- `/mur620d/UR10_r/twist_controller/command_collision_free`

### Campi

- `linear.x`: velocita lineare x `[m/s]`
- `linear.y`: velocita lineare y `[m/s]`
- `linear.z`: velocita lineare z `[m/s]`
- `angular.x`: velocita angolare x `[rad/s]`
- `angular.y`: velocita angolare y `[rad/s]`
- `angular.z`: velocita angolare z `[rad/s]`

### Significato pratico

- sui topic master: comando finale generato dall'ammettenza/haptic controller
- sui topic slave: comando finale generato dallo slave outer-loop

Nota: i topic slave command non hanno `header`; per il timing offline userai il timestamp della bag.

## Topic `std_msgs/Float64MultiArray`

Questi sono i topic "generici" piu importanti, quindi sotto trovi il significato dettagliato dei campi.

### `/teleop_master_haptic_controller_left/debug/dt_stats`
### `/teleop_master_haptic_controller_right/debug/dt_stats`

Ordine dei campi:

1. `dt_raw` `[s]`: dt misurato grezzo tra due tick
2. `dt_used` `[s]`: dt effettivamente usato dopo il clamp
3. `dt_step` `[s]`: dt del singolo sub-step interno
4. `n_substeps` `[-]`: numero di sub-step usati nel tick
5. `dt_min_seen` `[s]`: minimo dt_raw visto finora
6. `dt_max_seen` `[s]`: massimo dt_raw visto finora
7. `dt_mean` `[s]`: media incrementale del dt_raw

Uso pratico:

- serve per capire jitter del loop
- utile se vedi oscillazioni o comportamento diverso tra run apparentemente uguali

### `/teleop_master_haptic_controller_left/debug/admittance_stats`
### `/teleop_master_haptic_controller_right/debug/admittance_stats`

Ordine dei campi:

1. `|F_hand|` `[N]`: norma della forza mano/master dopo elaborazione
2. `|F_feedback|` `[N]`: norma del contributo di feedback riflesso verso il master
3. `|a_lin_des|` `[m/s^2]`: accelerazione lineare desiderata prima dei limitatori
4. `|a_lin_cmd|` `[m/s^2]`: accelerazione lineare effettivamente applicata
5. `|v_lin_pre|` `[m/s]`: velocita lineare prima della saturazione finale
6. `|v_lin_cmd|` `[m/s]`: velocita lineare finale pubblicata dal master
7. `lin_saturated` `[0/1]`: flag saturazione lineare
8. `|Tau_hand|` `[N*m]`: norma della coppia mano/master dopo elaborazione
9. `|Tau_feedback|` `[N*m]`: norma della coppia riflessa
10. `|a_ang_des|` `[rad/s^2]`: accelerazione angolare desiderata prima dei limitatori
11. `|a_ang_cmd|` `[rad/s^2]`: accelerazione angolare effettivamente applicata
12. `|v_ang_pre|` `[rad/s]`: velocita angolare prima della saturazione finale
13. `|v_ang_cmd|` `[rad/s]`: velocita angolare finale pubblicata dal master
14. `ang_saturated` `[0/1]`: flag saturazione angolare

Uso pratico:

- permette di capire se il comportamento del master e guidato davvero dal wrench
- permette di vedere se i limiter stanno tagliando il moto
- e il topic migliore per leggere rapidamente saturazioni, intensita di feedback e velocita finali

### `/teleop_master_haptic_controller_left/debug/admittance_dynamics`
### `/teleop_master_haptic_controller_right/debug/admittance_dynamics`

Questo topic contiene 36 valori, organizzati in gruppi da 3 assi `x,y,z`.

Ordine dei campi:

1. `M_lin_base.x` `[kg]`
2. `M_lin_base.y` `[kg]`
3. `M_lin_base.z` `[kg]`
4. `M_lin_extra.x` `[kg]`
5. `M_lin_extra.y` `[kg]`
6. `M_lin_extra.z` `[kg]`
7. `M_lin_eff.x` `[kg]`
8. `M_lin_eff.y` `[kg]`
9. `M_lin_eff.z` `[kg]`
10. `D_lin_base.x` `[N*s/m]`
11. `D_lin_base.y` `[N*s/m]`
12. `D_lin_base.z` `[N*s/m]`
13. `D_lin_extra.x` `[N*s/m]`
14. `D_lin_extra.y` `[N*s/m]`
15. `D_lin_extra.z` `[N*s/m]`
16. `D_lin_eff.x` `[N*s/m]`
17. `D_lin_eff.y` `[N*s/m]`
18. `D_lin_eff.z` `[N*s/m]`
19. `M_ang_base.x` `[kg*m^2]`
20. `M_ang_base.y` `[kg*m^2]`
21. `M_ang_base.z` `[kg*m^2]`
22. `M_ang_extra.x` `[kg*m^2]`
23. `M_ang_extra.y` `[kg*m^2]`
24. `M_ang_extra.z` `[kg*m^2]`
25. `M_ang_eff.x` `[kg*m^2]`
26. `M_ang_eff.y` `[kg*m^2]`
27. `M_ang_eff.z` `[kg*m^2]`
28. `D_ang_base.x` `[N*m*s/rad]`
29. `D_ang_base.y` `[N*m*s/rad]`
30. `D_ang_base.z` `[N*m*s/rad]`
31. `D_ang_extra.x` `[N*m*s/rad]`
32. `D_ang_extra.y` `[N*m*s/rad]`
33. `D_ang_extra.z` `[N*m*s/rad]`
34. `D_ang_eff.x` `[N*m*s/rad]`
35. `D_ang_eff.y` `[N*m*s/rad]`
36. `D_ang_eff.z` `[N*m*s/rad]`

Uso pratico:

- serve per verificare se `dynamic_mass` e `dynamic_damping` stanno davvero aggiungendo contributo
- se tutti i campi `*_extra` sono circa zero, lo scheduling dinamico non sta intervenendo
- se `eff = base + extra`, il comportamento e coerente con il modello atteso

### `/teleop_master_haptic_controller_left/debug/passivity_stats`
### `/teleop_master_haptic_controller_right/debug/passivity_stats`

Ordine dei campi:

1. `energy_before` `[J]`
2. `energy_after` `[J]`
3. `gamma_raw` `[-]`
4. `gamma_applied` `[-]`
5. `power_out_requested` `[W]`
6. `power_out_applied` `[W]`
7. `power_diss` `[W]`
8. `|F_reflection_requested|` `[N]`
9. `|F_reflection_used|` `[N]`
10. `|Tau_reflection_requested|` `[N*m]`
11. `|Tau_reflection_used|` `[N*m]`

Uso pratico:

- `gamma_applied < 1` significa che il layer sta limitando il feedback
- `energy_after` vicino a `tank_energy_min` significa tank quasi scarico
- se `|F_reflection_used| << |F_reflection_requested|`, il layer sta tagliando in modo visibile

## Topic `sensor_msgs/JointState`

I topic consigliati:

- `/mur620b/joint_states`
- `/mur620d/joint_states`

Campi:

- `header.stamp`: timestamp ROS
- `name[]`: nomi dei joint
- `position[]`: posizioni joint `[rad]` o `[m]` per attuatori prismatici
- `velocity[]`: velocita joint `[rad/s]` o `[m/s]`
- `effort[]`: sforzo/coppia/forza joint

Uso pratico:

- permettono di ricostruire la configurazione reale dei robot offline
- aiutano a confrontare comando cartesiano e risposta meccanica reale

## Topic `tf2_msgs/TFMessage`

I topic consigliati:

- `/tf`
- `/tf_static`

Ogni messaggio contiene un array `transforms[]` di `geometry_msgs/TransformStamped`.

Per ogni trasformazione:

- `header.stamp`: timestamp ROS
- `header.frame_id`: frame padre
- `child_frame_id`: frame figlio
- `transform.translation.{x,y,z}`: traslazione `[m]`
- `transform.rotation.{x,y,z,w}`: rotazione in quaternione

Uso pratico:

- permette di ricostruire pose e frame reali nel tempo
- e fondamentale se vuoi confrontare i wrench o i target in frame diversi

## Note finali

- i topic master debug sono ora separati correttamente tra `left` e `right`
- i wrench filtrati slave sono ora disponibili anche lato `mur620d`
- i topic `command_collision_free` dello slave restano non-stamped: per analisi temporale usa il timestamp della bag oppure aggiungi in futuro un publisher `TwistStamped` anche lato slave
