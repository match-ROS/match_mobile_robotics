# scene_builder

`scene_builder` centralizes planning scene management and distance monitoring utilities for MoveIt-based applications.

## Features

- Distance monitoring node publishes nearest contact information and RViz markers.
- Topic-based interface to add, move, and remove collision objects.
- Support for waypoint queueing and velocity-based commands using `geometry_msgs/Twist`.
- Configurable default objects and motion sequences loaded from ROS parameters/YAML files.
- Centralized configuration file for all parameters including update frequencies.

## Configuration

### File di configurazione principale

Il file `config/scene_builder_params.yaml` contiene tutti i parametri configurabili del pacchetto, organizzati in sezioni logiche:

```yaml
# Frequenze di aggiornamento
update_rate: 60.0              # Hz - aggiornamento animazioni oggetti
computation_rate: 50.0          # Hz - calcolo distanze robot-ostacoli
state_update_frequency: 100.0   # Hz - aggiornamento stato robot

# Timeouts
state_wait_timeout: 2.0         # secondi - attesa stato iniziale robot
default_velocity_timeout: 1.0   # secondi - timeout comandi velocità

# Limiti
max_velocity_norm: 2.0          # m/s o rad/s - velocità max oggetti

# Animazioni
default_command_duration: 0.5   # secondi - durata default waypoints
autostart_loops: false          # avvio automatico sequenze

# Visualizzazione (RViz)
arrow_length: 0.15              # metri - lunghezza frecce distanza
arrow_shaft_diameter: 0.01      # metri
arrow_head_diameter: 0.02       # metri
arrow_head_length: 0.03         # metri
```

### File oggetti e sequenze

Il file `config/scene_config.yaml` contiene la definizione degli oggetti e delle sequenze di movimento:

```yaml
objects:
  sphere_1:
    primitive:
      type: sphere
      dimensions: [0.15]
    pose:
      position: [0.5, 0.0, 0.5]
      orientation: [0.0, 0.0, 0.0, 1.0]

motion_sequences:
  sphere_1:
    loop: true
    waypoints:
      - position: [0.5, 0.0, 0.5]
        orientation: [0.0, 0.0, 0.0, 1.0]
        duration: 2.0
```

## Nodes

### `scene_distance_monitor`

Publishes timestamped distance data between robot links and world objects.

#### Parameters

| Parametro | Tipo | Default | Descrizione |
|-----------|------|---------|-------------|
| `computation_rate` | double | 15.0 | Frequenza calcolo distanze (Hz) |
| `state_update_frequency` | double | 100.0 | Frequenza aggiornamento stato robot (Hz) |
| `state_wait_timeout` | double | 2.0 | Timeout attesa stato iniziale (s) |
| `joint_state_topic` | string | "/joint_states" | Topic joint states |
| `monitor_planning_scene_topic` | string | "planning_scene" | Topic planning scene |
| `default_objects_param` | string | "objects" | Parametro oggetti default |
| `object_loader_move_group` | string | "manipulator" | MoveGroup per caricamento |
| `arrow_length` | double | 0.15 | Lunghezza frecce marker (m) |
| `arrow_shaft_diameter` | double | 0.01 | Diametro asta freccia (m) |
| `arrow_head_diameter` | double | 0.02 | Diametro punta freccia (m) |
| `arrow_head_length` | double | 0.03 | Lunghezza punta freccia (m) |

#### Topics

- Publishes `planning_scene`, `nearest_points_markers`, and `distance_info`.

### `object_command_node`

Consumes topic commands for managing collision objects.

#### Parameters

| Parametro | Tipo | Default | Descrizione |
|-----------|------|---------|-------------|
| `update_rate` | double | 60.0 | Frequenza aggiornamento animazioni (Hz) |
| `move_group` | string | "arm" | Nome del MoveGroup MoveIt |
| `default_command_duration` | double | 0.5 | Durata default waypoints (s) |
| `default_velocity_timeout` | double | 1.0 | Timeout comandi velocità (s) |
| `max_velocity_norm` | double | 1.0 | Velocità massima oggetti (m/s, rad/s) |
| `default_objects_param` | string | "default_objects" | Parametro oggetti default |
| `motion_sequences_param` | string | "" | Parametro sequenze movimento |
| `autostart_loops` | bool | false | Avvio automatico loop |

#### Subscriptions

- `add_object` (`moveit_msgs/CollisionObject`)
- `object_command` (`scene_builder/ObjectCommand`)
- `object_velocity_command` (`scene_builder/ObjectVelocityCommand`)
- `object_animation` (`geometry_msgs/PoseArray`)

#### Services

- `~set_motion_sequence` - Imposta una sequenza di waypoints
- `~get_motion_sequence` - Ottiene lo stato della sequenza corrente
- `~clear_motion_sequence` - Cancella la sequenza attiva
- `~list_objects` - Lista tutti gli oggetti nella scena

## Messages

- `DistanceInfo`
  - Header `stamp` in the planning frame (`header.frame_id`).
  - `contacts`: per link–object proximity records.
- `DistanceContact`
  - `link_name`, `object_id`: identifiers of the interacting bodies.
  - `distance`: scalar minimum distance.
  - `robot_point`, `object_point`: closest points in world coordinates.
  - `distance_vector`: world-frame vector from obstacle point to robot point.
  - `joint_model_group`: joint group used to compute Jacobian (empty if skipped).
  - `joint_names`: ordered list for Jacobian columns.
  - `jacobian_rows`, `jacobian_cols`: matrix dimensions.
  - `jacobian`: row-major flattened matrix; empty if Jacobian not computed.

## Launch

Use provided launch files:

```bash
# Distance monitor con configurazione default
roslaunch scene_builder distance_monitor.launch

# Object command con configurazione default
roslaunch scene_builder object_command.launch

# Con autostart dei loop di movimento
roslaunch scene_builder object_command.launch autostart_loops:=true

# Con file di configurazione custom
roslaunch scene_builder distance_monitor.launch config_file:=/path/to/custom_params.yaml
```

### Launch Arguments

#### distance_monitor.launch

| Argomento | Default | Descrizione |
|-----------|---------|-------------|
| `start_delay` | 0.0 | Ritardo prima dell'avvio (secondi) |
| `config_file` | `scene_builder_params.yaml` | File parametri |
| `scene_config_file` | `scene_config.yaml` | File oggetti/sequenze |

#### object_command.launch

| Argomento | Default | Descrizione |
|-----------|---------|-------------|
| `autostart_loops` | false | Avvia automaticamente i loop |
| `config_file` | `scene_builder_params.yaml` | File parametri |
| `scene_config_file` | `scene_config.yaml` | File oggetti/sequenze |

## Performance Tuning

### Frequenze consigliate

**Per applicazioni standard:**
- `update_rate`: 60 Hz
- `computation_rate`: 30 Hz
- `state_update_frequency`: 100 Hz

**Per obstacle avoidance in tempo reale:**
- `update_rate`: 100 Hz
- `computation_rate`: 50 Hz
- `state_update_frequency`: 200 Hz

**Per basso consumo CPU:**
- `update_rate`: 30 Hz
- `computation_rate`: 10 Hz
- `state_update_frequency`: 50 Hz
