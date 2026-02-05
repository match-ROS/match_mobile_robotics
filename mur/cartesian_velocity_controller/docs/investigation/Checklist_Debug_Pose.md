# Checklist Debug Bug Pose Target

> **Guida rapida per investigare bug relativi alle pose target**  
> **Ultimo aggiornamento:** 2026-02-03  
> **Documenti correlati:**
> - [Gestione_Pose_Target.md](./Gestione_Pose_Target.md)
> - [Pipeline_Jacobiano_Velocita.md](./Pipeline_Jacobiano_Velocita.md)

---

## Quick Reference: Flusso Dati Pose

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           SCRIPT PYTHON                                      │
│ ┌────────────────┐   ┌────────────────┐   ┌─────────────────────────────┐   │
│ │ saved_poses.   │──▶│ PoseManager    │──▶│ ROSInterface.               │   │
│ │ yaml           │   │ .get_pose()    │   │ publish_target_pose()       │   │
│ │ [x,y,z,qx,qy,  │   │ Dict→          │   │ frame_id = _global_frame    │   │
│ │  qz,qw]        │   │                │   │ (può essere "base_link"!)   │   │
│ └────────────────┘   └────────────────┘   └──────────────┬──────────────┘   │
└──────────────────────────────────────────────────────────┼──────────────────┘
                                                           │ topic: /target_pose
                                                           ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                           CONTROLLER C++                                      │
│ ┌───────────────────┐   ┌────────────────────┐   ┌─────────────────────┐    │
│ │ targetPoseCallback│──▶│ setTargetPose()    │──▶│ GlobalPlanner.      │    │
│ │ (PoseStamped)     │   │ ⚠️ NO transform TF!│   │ addWaypoint()       │    │
│ │                   │   │ global_frame="world"   │                     │    │
│ └───────────────────┘   └────────────────────┘   └─────────────────────┘    │
│                                                           │                   │
│                                                           ▼                   │
│ ┌─────────────────────────────────────────────────────────────────────────┐ │
│ │                        executePipeline()                                 │ │
│ │  LocalPlanner → VelocityFilter → PID → Jacobian⁺ → SafetyLimiter        │ │
│ └─────────────────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 🔍 Checklist di Investigazione

### 1. Verifica Configurazione Frame

#### 1.1 Frame nel Controller

```bash
# Controlla il frame configurato
rosparam get /cartesian_velocity_controller/global_frame
```

**Atteso:** `world` (o lo stesso usato dallo script)

#### 1.2 Frame nel File YAML (Script)

Verifica in `config.py`:

```python
DEFAULT_GLOBAL_FRAME: str = "base_link"  # ⚠️ Deve matchare!
```

#### 1.3 Frame Attivo nello Script

Durante l'esecuzione:

```python
# In robot_interactive_control.py, stampa:
print(f"Using frame: {controller.ros.global_frame}")
```

### 2. Verifica TF Tree

```bash
# Visualizza TF tree
rosrun tf2_tools view_frames.py

# Controlla trasformazione specifica
rosrun tf tf_echo world base_link
```

**Domande:**
- [ ] Esiste il frame `world` nel TF tree?
- [ ] Esiste connessione `world` → `base_link`?
- [ ] Per robot mobile: `world` e `base_link` sono uguali o diversi?

### 3. Verifica Posa Salvata

Apri `config/saved_poses.yaml` e controlla una posa problematica:

```yaml
posa_problematica:
  position: [x, y, z]
  orientation: [qx, qy, qz, qw]  # ← Ordine corretto?
```

**Verifiche:**
- [ ] L'ordine è `[qx, qy, qz, qw]` (non `[qw, qx, qy, qz]`)?
- [ ] Il quaternione è normalizzato? `sqrt(qx²+qy²+qz²+qw²) ≈ 1`
- [ ] La posizione è ragionevole (nel workspace del robot)?

### 4. Confronto Pose Lette vs Pubblicate

Aggiungi log temporaneo in `ros_interface.py`:

```python
def publish_target_pose(self, pose_data):
    rospy.logwarn(f"Publishing pose in frame '{self._global_frame}':")
    rospy.logwarn(f"  Position: {pose_data['position']}")
    rospy.logwarn(f"  Orientation: {pose_data['orientation']}")
```

### 5. Verifica Ricezione nel Controller

```bash
# Monitora topic
rostopic echo /cartesian_velocity_controller/target_pose
```

Confronta:
- [ ] `frame_id` ricevuto == `global_frame` del controller?
- [ ] Posizione/orientamento matchano ciò che hai inviato?

### 6. Verifica IK Reachability

```bash
# Controlla log per reject
rqt_console  # Filtra per "cartesian_velocity_controller"
```

Cerca messaggi tipo:
- `Target pose REJECTED (not reachable via IK)`
- `Frame mismatch`

---

## 🐛 Bug Comuni e Soluzioni

### Bug 1: Posa in Frame Sbagliato

**Sintomo:** Robot si muove verso posizione completamente errata.

**Causa:** Posa salvata in `base_link`, interpretata come `world`.

**Soluzione:**
```python
# In config.py, cambia:
DEFAULT_GLOBAL_FRAME = "world"  # Stesso del controller
```

### Bug 2: Quaternione Ordine Errato

**Sintomo:** Orientamento finale ruotato di ~90°-180°.

**Causa:** Quaternione in ordine `[qw, qx, qy, qz]` invece di `[qx, qy, qz, qw]`.

**Soluzione:** Converti l'ordine nel file YAML.

### Bug 3: Quaternione Non Normalizzato

**Sintomo:** Orientamento instabile, rotazioni erratiche.

**Verifica:**
```python
import math
quat = [qx, qy, qz, qw]
norm = math.sqrt(sum(q**2 for q in quat))
print(f"Quaternion norm: {norm}")  # Deve essere ~1.0
```

### Bug 4: Frame TF Non Esiste

**Sintomo:** `TF exception: Lookup would require extrapolation`

**Causa:** Il frame `global_frame` non esiste nel TF tree.

**Verifica:**
```bash
rosrun tf2_tools view_frames.py
evince frames.pdf
```

### Bug 5: Singolarità Jacobiano

**Sintomo:** Velocità zero o molto piccole nonostante grande errore posizione.

**Verifica:**
```cpp
// Aggiungi log temporaneo
ROS_INFO("Min singular value: %f", jacobian_solver_->getLastMinSingularValue());
```

Se `σ_min < 0.01`, il robot è vicino a singolarità.

---

## 📊 Comandi di Diagnostica

### Stato Pipeline

```bash
# Debug completo del pipeline
rostopic echo /cartesian_velocity_controller/pipeline_debug

# Posizione TCP corrente
rosrun tf tf_echo world tool0

# Comando velocità inviato
rostopic echo /joint_group_vel_controller/command
```

### Confronto Pose

```bash
# Posa target
rostopic echo /cartesian_velocity_controller/target_pose -n1

# Posa corrente (via marker)
rostopic echo /cartesian_velocity_controller/visualization_marker_array -n1 | grep -A10 "current"
```

### Stato Controller

```bash
# Controller attivo
rosservice call /controller_manager/list_controllers

# Switch controller
rosrun controller_manager controller_manager list
```

---

## 🧪 Test Suggeriti

### Test 1: Round-Trip Pose

1. Leggi posa corrente
2. Salvala
3. Muovi il robot altrove
4. Invia la posa salvata
5. Verifica che ritorni esattamente alla stessa posizione

```bash
# Posa corrente
rosrun tf tf_echo base_link tool0

# Dopo invio, verifica
rosrun tf tf_echo base_link tool0
```

### Test 2: Confronto MoveIt vs Velocity

1. Invia stessa posa via MoveIt
2. Invia stessa posa via velocity controller
3. Confronta posizione finale

**Se diverse:** Problema di frame/trasformazione.

### Test 3: Pose Semplice

Crea posa di test con valori semplici:

```yaml
test_identity:
  position: [0.5, 0.0, 0.5]
  orientation: [0.0, 0.707, 0.0, 0.707]  # 90° intorno a Y
```

Verifica che il robot raggiunga esattamente questa configurazione.

---

## 📝 Template Report Bug

```markdown
## Descrizione Bug
[Descrivi il comportamento errato]

## Comportamento Atteso
[Cosa dovrebbe succedere]

## Comportamento Osservato
[Cosa succede realmente]

## Configurazione
- global_frame (controller): [valore]
- global_frame (script): [valore]
- Posa inviata: [x, y, z, qx, qy, qz, qw]
- Frame posa: [frame_id]

## Diagnostica
- [ ] Frame TF esistono
- [ ] Quaternione normalizzato
- [ ] IK accettata
- [ ] Singolarità avoidata (σ_min > 0.05)

## Log Rilevanti
[Incolla log da rqt_console]
```

---

## 🔧 Fix Rapidi

### Fix 1: Forza Frame Corretto nello Script

```python
# In ros_interface.py, forza il frame:
self._global_frame = "world"  # Hardcode temporaneo
```

### Fix 2: Aggiungi Trasformazione TF nel Controller

Modifica `cartesian_velocity_controller.cpp`:

```cpp
bool CartesianVelocityController::setTargetPose(
    const geometry_msgs::PoseStamped& pose_stamped)
{
  if (pose_stamped.header.frame_id != global_frame_)
  {
    // Trasforma prima di usare
    geometry_msgs::PoseStamped transformed;
    try {
      tf_buffer_.transform(pose_stamped, transformed, global_frame_);
      return setTargetPose(transformed.pose);
    } catch (...) {
      ROS_ERROR("Transform failed");
      return false;
    }
  }
  return setTargetPose(pose_stamped.pose);
}
```

### Fix 3: Normalizza Quaternione

```python
# In pose_manager.py, aggiungi normalizzazione:
def normalize_quat(q):
    norm = math.sqrt(sum(x**2 for x in q))
    return [x/norm for x in q]

orientation = normalize_quat(orientation)
```
