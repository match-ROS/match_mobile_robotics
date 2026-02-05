# PipelineDebug Message - Documentazione Completa

Questo documento descrive in dettaglio ogni campo del messaggio `PipelineDebug.msg`, utilizzato per il debug e l'analisi della pipeline di controllo.

Il messaggio viene pubblicato dalla funzione `publishPipelineDebug()` in `feedback_publisher.cpp` sul topic `pipeline_debug`.

---

## Struttura della Pipeline

```
Global Planner → Local Planner → Motion Generator → PID + IK → Safety Limiter
     (A)             (B)              (C)              (D)           (E)
```

---

## Header

| Campo | Tipo | Descrizione |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Header standard ROS con timestamp e frame di riferimento |

---

## LEVEL A: Global Planner

Il Global Planner gestisce la navigazione attraverso una sequenza di waypoint. Determina quale waypoint è attivo e calcola la distanza dal TCP corrente.

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `distance_waypoint_to_current` | `float64` | m | Distanza euclidea tra il waypoint attivo e la posizione TCP corrente. Usata per determinare quando passare al waypoint successivo. |
| `active_waypoint_index` | `int32` | - | Indice del waypoint attualmente in esecuzione (0-based). |
| `total_waypoints` | `int32` | - | Numero totale di waypoint nella traiettoria. |
| `active_waypoint` | `geometry_msgs/Pose` | m, quaternion | Posa del waypoint attivo (posizione + orientamento). È l'obiettivo corrente della navigazione. |
| `current_pose` | `geometry_msgs/Pose` | m, quaternion | Posa attuale del TCP (Tool Center Point) del robot. |

### Interpretazione
- Quando `distance_waypoint_to_current` scende sotto una soglia configurata, il planner passa al waypoint successivo
- `active_waypoint_index == total_waypoints - 1` indica che siamo sull'ultimo waypoint

---

## LEVEL B: Local Planner (Virtual Point con Campi Artificiali)

Il Local Planner implementa un sistema di navigazione reattiva basato su campi potenziali artificiali:
- **Campo attrattivo**: attrae il robot verso il waypoint
- **Campo repulsivo TCP**: respinge il TCP dagli ostacoli
- **Campo repulsivo link**: respinge i link del robot dagli ostacoli

### Target Raw

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `distance_target_raw_to_waypoint` | `float64` | m | Distanza tra P_target_raw e il waypoint attivo. |
| `distance_target_raw_to_current` | `float64` | m | Distanza tra P_target_raw e la posa corrente del robot. |
| `target_raw` | `geometry_msgs/Pose` | m, quaternion | Posa target "grezza" calcolata dal local planner prima del filtraggio. Rappresenta dove il robot vorrebbe andare considerando attrazione e repulsione. |

### Composizione della Velocità Desiderata (V_desired)

La velocità desiderata è composta da tre contributi:

```
V_desired = k_att * V_goal + k_rep_tcp * V_obs + k_rep_links * V_link
```

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `v_goal_linear` | `geometry_msgs/Vector3` | m/s | **Velocità attrattiva lineare (V_goal)**: velocità che spinge il TCP verso il waypoint. |
| `v_goal_angular` | `geometry_msgs/Vector3` | rad/s | **Velocità attrattiva angolare (V_goal)**: velocità angolare per allineare l'orientamento al waypoint. |
| `v_obs_linear` | `geometry_msgs/Vector3` | m/s | **Velocità repulsiva TCP (V_obs)**: velocità che allontana il TCP dagli ostacoli vicini. |
| `v_link_linear` | `geometry_msgs/Vector3` | m/s | **Velocità repulsiva link (V_link)**: velocità derivata dalla repulsione sui link del robot, convertita in velocità TCP. |
| `v_desired_linear` | `geometry_msgs/Vector3` | m/s | **Velocità desiderata lineare risultante**: combinazione pesata di tutte le componenti. |
| `v_desired_angular` | `geometry_msgs/Vector3` | rad/s | **Velocità desiderata angolare risultante**. |

### Guadagni dei Campi Potenziali

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `k_attractive` | `float64` | - | Guadagno del campo attrattivo. Valori più alti → maggiore attrazione verso il goal. |
| `k_repulsive_tcp` | `float64` | - | Guadagno del campo repulsivo per il TCP. Valori più alti → maggiore repulsione dagli ostacoli. |
| `k_repulsive_links` | `float64` | - | Guadagno del campo repulsivo per i link. Valori più alti → maggiore protezione dei link. |

### Informazioni Ostacoli

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `closest_obstacle_distance` | `float64` | m | Distanza dall'ostacolo più vicino al robot. |
| `closest_obstacle_id` | `string` | - | Identificativo dell'ostacolo più vicino (es. nome dell'oggetto nella scena). |
| `closest_link_name` | `string` | - | Nome del link del robot più vicino all'ostacolo. |

### Interpretazione
- Se `v_obs_linear` è significativo, il robot sta evitando un ostacolo
- `closest_obstacle_distance` < soglia di sicurezza → il robot rallenta o devia
- `v_desired` = 0 può indicare un minimo locale (il robot è "bloccato")

---

## LEVEL C: Motion Generator (Filtro del Secondo Ordine)

Il Motion Generator applica un filtro del secondo ordine alla velocità desiderata per garantire continuità in velocità, accelerazione e jerk. Questo produce movimenti smooth e rispettosi dei limiti cinematici.

### Target Filtrato

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `distance_target_filtered_to_current` | `float64` | m | Distanza tra il target filtrato e la posa corrente. |
| `target_filtered` | `geometry_msgs/Pose` | m, quaternion | Posa target dopo il filtraggio. Questa è la posa che il PID cercherà di raggiungere. |

### Output del Filtro

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `v_filtered_linear` | `geometry_msgs/Vector3` | m/s | Velocità lineare filtrata (output del filtro). |
| `v_filtered_angular` | `geometry_msgs/Vector3` | rad/s | Velocità angolare filtrata. |
| `acceleration_filtered_linear` | `geometry_msgs/Vector3` | m/s² | Accelerazione lineare filtrata. |
| `acceleration_filtered_angular` | `geometry_msgs/Vector3` | rad/s² | Accelerazione angolare filtrata. |
| `jerk_filtered_linear` | `geometry_msgs/Vector3` | m/s³ | Jerk (derivata dell'accelerazione) lineare filtrato. |
| `jerk_filtered_angular` | `geometry_msgs/Vector3` | rad/s³ | Jerk angolare filtrato. |

### Parametri del Filtro

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `filter_tau` | `float64` | s | Costante di tempo del filtro (τ). Valori più alti → risposta più lenta ma più smooth. |

### Interpretazione
- `v_filtered` segue `v_desired` con un ritardo proporzionale a `τ`
- Accelerazione e jerk limitati garantiscono movimenti "morbidi"
- Se `v_filtered` << `v_desired`, il filtro sta limitando il movimento

---

## LEVEL D: PID Controller

Il controller PID calcola la correzione necessaria per inseguire il target filtrato. Opera in spazio cartesiano con termini proporzionale, integrale e derivativo separati per posizione e orientamento.

### Errore di Posizione

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `pid_position_error` | `geometry_msgs/Vector3` | m | Errore di posizione: `P_target_filtered - P_current` |
| `pid_orientation_error` | `geometry_msgs/Vector3` | rad | Errore di orientamento (rappresentato come vettore axis-angle o RPY). |
| `pid_position_error_norm` | `float64` | m | Norma euclidea dell'errore di posizione. |
| `pid_orientation_error_norm` | `float64` | rad | Norma dell'errore di orientamento. |

### Guadagni PID

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `pid_kp_position` | `float64` | 1/s | Guadagno proporzionale per la posizione. |
| `pid_ki_position` | `float64` | 1/s² | Guadagno integrale per la posizione. |
| `pid_kd_position` | `float64` | - | Guadagno derivativo per la posizione. |
| `pid_kp_orientation` | `float64` | 1/s | Guadagno proporzionale per l'orientamento. |
| `pid_ki_orientation` | `float64` | 1/s² | Guadagno integrale per l'orientamento. |
| `pid_kd_orientation` | `float64` | - | Guadagno derivativo per l'orientamento. |

### Componenti PID - Lineari

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `pid_p_term_linear` | `geometry_msgs/Vector3` | m/s | Termine proporzionale: `Kp * error` |
| `pid_i_term_linear` | `geometry_msgs/Vector3` | m/s | Termine integrale: `Ki * ∫error dt` |
| `pid_d_term_linear` | `geometry_msgs/Vector3` | m/s | Termine derivativo: `Kd * d(error)/dt` |
| `pid_integral_linear` | `geometry_msgs/Vector3` | m·s | Valore accumulato dell'integrale (utile per debug anti-windup). |

### Componenti PID - Angolari

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `pid_p_term_angular` | `geometry_msgs/Vector3` | rad/s | Termine proporzionale angolare. |
| `pid_i_term_angular` | `geometry_msgs/Vector3` | rad/s | Termine integrale angolare. |
| `pid_d_term_angular` | `geometry_msgs/Vector3` | rad/s | Termine derivativo angolare. |
| `pid_integral_angular` | `geometry_msgs/Vector3` | rad·s | Valore accumulato dell'integrale angolare. |

### Output del PID

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `pid_feedforward_linear` | `geometry_msgs/Vector3` | m/s | Feed-forward lineare: `V_filtered` (velocità filtrata usata come FF). |
| `pid_feedforward_angular` | `geometry_msgs/Vector3` | rad/s | Feed-forward angolare. |
| `pid_output_linear` | `geometry_msgs/Vector3` | m/s | Output del solo PID: `P + I + D` |
| `pid_output_angular` | `geometry_msgs/Vector3` | rad/s | Output PID angolare. |
| `cartesian_cmd_linear` | `geometry_msgs/Vector3` | m/s | **Comando cartesiano finale**: `feedforward + pid_output` |
| `cartesian_cmd_angular` | `geometry_msgs/Vector3` | rad/s | Comando cartesiano angolare finale. |

### Interpretazione
- `pid_position_error_norm` grande → il robot è indietro rispetto al target
- `pid_integral_linear` che cresce → possibile errore sistematico o saturazione
- `cartesian_cmd` è il comando finale in spazio cartesiano prima della conversione in spazio giunto

---

## LEVEL D: Jacobian Inverse Kinematics

Questa sezione converte i comandi cartesiani in velocità giunti usando la pseudo-inversa della Jacobiana.

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `jacobian_damping_factor` | `float64` | - | Fattore di damping per la pseudo-inversa (Damped Least Squares). Previene instabilità vicino a singolarità. |
| `jacobian_min_singular_value` | `float64` | - | Valore singolare minimo della Jacobiana. Valori vicini a 0 indicano prossimità a una singolarità. |
| `joint_velocity_from_ik` | `float64[]` | rad/s | Velocità giunti calcolate dalla pseudo-inversa (prima del safety limiter). |

### Interpretazione
- `jacobian_min_singular_value` < 0.01 → prossimità a singolarità, damping aumentato
- `jacobian_damping_factor` alto → movimento degradato ma stabile
- `joint_velocity_from_ik` sono le velocità "ideali" prima dei limiti di sicurezza

---

## Safety Limiter

Il Safety Limiter applica limiti di sicurezza alle velocità giunti, scalando uniformemente se necessario per rispettare i vincoli.

| Campo | Tipo | Unità | Descrizione |
|-------|------|-------|-------------|
| `joint_velocity_after_limiter` | `float64[]` | rad/s | Velocità giunti dopo l'applicazione dei limiti di sicurezza. **Questi sono i comandi effettivamente inviati al robot.** |
| `safety_scaling_factor` | `float64` | - | Fattore di scala applicato (0.0 - 1.0). `1.0` = nessuna limitazione, `< 1.0` = velocità ridotte. |
| `safety_limiting_reason` | `string` | - | Motivo della limitazione (es. "velocity_limit", "acceleration_limit", "collision_proximity"). Vuoto se nessuna limitazione attiva. |
| `joint_names` | `string[]` | - | Nomi dei giunti (per riferimento e correlazione con gli array di velocità). |

### Interpretazione
- `safety_scaling_factor` = 1.0 → il robot si muove alla velocità calcolata
- `safety_scaling_factor` < 1.0 → il movimento è stato rallentato per sicurezza
- `safety_limiting_reason` indica quale limite è stato raggiunto

---

## Esempio di Utilizzo

### Sottoscrivere al topic in Python

```python
import rospy
from cartesian_velocity_controller.msg import PipelineDebug

def callback(msg):
    # Global Planner
    print(f"Waypoint {msg.active_waypoint_index}/{msg.total_waypoints}")
    print(f"Distance to waypoint: {msg.distance_waypoint_to_current:.3f} m")
    
    # Local Planner - Obstacle avoidance
    if msg.closest_obstacle_distance < 0.1:
        print(f"WARNING: Obstacle '{msg.closest_obstacle_id}' at {msg.closest_obstacle_distance:.3f} m")
    
    # PID tracking
    print(f"Position error: {msg.pid_position_error_norm:.4f} m")
    
    # Safety
    if msg.safety_scaling_factor < 1.0:
        print(f"Safety limiting: {msg.safety_limiting_reason} (scale: {msg.safety_scaling_factor:.2f})")

rospy.Subscriber("pipeline_debug", PipelineDebug, callback)
```

### Sottoscrivere al topic in C++

```cpp
#include <cartesian_velocity_controller/PipelineDebug.h>

void callback(const cartesian_velocity_controller::PipelineDebug::ConstPtr& msg)
{
    ROS_INFO("Position error: %.4f m", msg->pid_position_error_norm);
    
    if (msg->safety_scaling_factor < 1.0)
    {
        ROS_WARN("Safety limiting: %s", msg->safety_limiting_reason.c_str());
    }
}
```

---

## Diagramma di Flusso dei Dati

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                     GLOBAL PLANNER (A)                       │
                    │  Waypoints → active_waypoint, distance_waypoint_to_current  │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                                              ▼
                    ┌─────────────────────────────────────────────────────────────┐
                    │                     LOCAL PLANNER (B)                        │
                    │  V_goal + V_obs + V_link → V_desired, target_raw            │
                    │  Obstacle info: closest_obstacle_distance, closest_link     │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                                              ▼
                    ┌─────────────────────────────────────────────────────────────┐
                    │                   MOTION GENERATOR (C)                       │
                    │  V_desired → Filtro 2° ordine → V_filtered, target_filtered │
                    │  + acceleration_filtered, jerk_filtered                     │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                                              ▼
                    ┌─────────────────────────────────────────────────────────────┐
                    │                    PID CONTROLLER (D)                        │
                    │  error = target_filtered - current                          │
                    │  cartesian_cmd = feedforward + P + I + D                    │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                                              ▼
                    ┌─────────────────────────────────────────────────────────────┐
                    │                  JACOBIAN IK (D)                             │
                    │  cartesian_cmd → J⁺ → joint_velocity_from_ik               │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                                              ▼
                    ┌─────────────────────────────────────────────────────────────┐
                    │                   SAFETY LIMITER (E)                         │
                    │  joint_velocity_from_ik → limits → joint_velocity_after    │
                    │  safety_scaling_factor, safety_limiting_reason              │
                    └─────────────────────────┴───────────────────────────────────┘
                                              │
                                              ▼
                                        [Robot HW]
```

---

## Note Importanti

1. **Frequenza di pubblicazione**: Il messaggio viene pubblicato solo se ci sono subscriber attivi (ottimizzazione delle risorse).

2. **Frame di riferimento**: Tutti i dati sono espressi nel frame globale specificato nell'header (`global_frame_`).

3. **Debugging consigliato**:
   - Se il robot non si muove: controllare `v_desired` e `safety_scaling_factor`
   - Se il robot oscilla: controllare i guadagni PID e `filter_tau`
   - Se il robot devia dalla traiettoria: controllare `v_obs` e `closest_obstacle_distance`
   - Se il robot è lento: controllare `safety_limiting_reason`

4. **Visualizzazione**: Usare `rqt_plot` o PlotJuggler per visualizzare i dati nel tempo.

