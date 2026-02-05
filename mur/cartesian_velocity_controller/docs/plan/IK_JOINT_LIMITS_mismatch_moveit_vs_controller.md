# Specifica implementativa: limiti giunti “controller-only” + reachability IK + guardrail runtime + elbow injection

Questo documento sostituisce la parte “analisi” con una **specifica pronta per l’implementazione** nel pacchetto `cartesian_velocity_controller`.

## Decisione (TL;DR)

- **MoveIt resta “libero”**: non imponiamo limiti custom in `universal_robot/.../joint_limits.yaml` (né in URDF).
- Aggiungiamo limiti **solo per il controller** tramite **file YAML dedicato** in `cartesian_velocity_controller`.
- Quei limiti vengono applicati in **due punti**:
  - **Reachability**: il check IK rifiuta target/waypoints se la soluzione IK viola i limiti del controller.
  - **Runtime guardrail**: se durante l’esecuzione un giunto si avvicina al limite, il comando \(\dot q\) viene “frenato” in modo continuo; vicino al limite si può applicare anche una piccola **velocità di rientro impostabile**.
- Aggiungiamo una **elbow velocity injection** (anti-singolarità gomito esteso) **filtrata jerk/acc-safe** e **gated** usando la soluzione IK del target:
  - se il target “richiede” gomito vicino all’estensione → injection OFF/attenuata
  - se non c’è target → injection ON (spingi via dalla singolarità)
  - se il target cambia → si ricalcola il gate dalla nuova soluzione IK

## Contesto e problema

Sintomo originale: MoveIt può essere configurato con limiti in `joint_limits.yaml`, ma il controller fa reachability con IK senza necessariamente applicare gli stessi limiti, quindi accetta pose “raggiungibili” per l’IK ma “non valide” per MoveIt.

Con la nuova scelta progettuale, il problema viene eliminato alla radice: **la reachability del controller non dipende più dai limiti di MoveIt** ma da un set di limiti esplicitamente definito e versionato nel controller.

## Scope

- **Obiettivi**
  - Definire limiti posizione giunti *controller-only* (rad) e applicarli a reachability + runtime.
  - Garantire comportamento **continuo** (no “velocity jump”) tramite `JointVelocityFilter` (jerk/acc-safe).
  - Implementare “elbow injection” con gate basato sul target.

- **Non-obiettivi**
  - Non cambiare URDF o MoveIt config.
  - Non delegare la reachability a `move_group` (niente service MoveIt).

## Dove toccare il codice (punti di inserimento)

- **Reachability (IK)**: `src/components/robot_state_manager.cpp`, `RobotStateManager::checkPoseReachability(...)`
  - oggi: `setFromIK(...)` e se `true` ritorna `true` senza check limiti controller-only
  - domani: `setFromIK(...)` con **validity callback** che rifiuta soluzioni che violano i limiti controller-only; inoltre salva la soluzione trovata in `joint_solution`.

- **Runtime guard + elbow injection**: `src/cartesian_velocity_controller.cpp`, in `executePipeline(dt)`
  - punto esatto: subito dopo
    - `Eigen::VectorXd joint_velocities = jacobian_pinv * command_twist;`
  - e prima di:
    - `joint_velocity_filter_->filter(...)`
    - `safety_limiter_->limit(...)`

## Configurazione

### File nuovo: `config/controller_joint_limits.yaml`

Questo file contiene **solo** limiti legati al controller, in **radianti**.

Schema proposto:

```yaml
controller_joint_limits:
  enabled: true

  # I limiti vengono applicati ai soli giunti elencati in "limits".
  # Nota: i nomi devono combaciare con i joint names del gruppo MoveIt (`RobotStateManager::getJointNames()`).
  limits:
    elbow_joint:
      enabled: true
      min: -6.283185307179586
      max:  1.5707963267948966

      # ========== Runtime guardrail ==========
      runtime_guard:
        enabled: true

        # Inizia a frenare quando la distanza dal limite è < soft_zone (rad)
        soft_zone: 0.30

        # Tolleranza numerica / hard margin vicino al limite (rad)
        # Se dentro questo margine e stai andando verso il limite: forza un piccolo rientro.
        margin: 0.01

        # Velocità di rientro (rad/s) quando dentro margin e stai andando verso il limite
        # (segno deciso automaticamente verso l’interno)
        reentry_velocity: 0.05

    # ... altri giunti se necessario ...
```

### Caricamento parametri

Aggiornare `launch/cartesian_velocity_controller.launch` aggiungendo una riga:

- `<rosparam file="$(find cartesian_velocity_controller)/config/controller_joint_limits.yaml" command="load"/>`

Nota: oggi il launch carica solo:
- `config/controller_params.yaml`
- `config/velocity_filter_params.yaml`

### Parametri runtime aggiuntivi (elbow injection)

I parametri dell’injection vivono in `controller_params.yaml` (perché sono “pipeline behavior”, non limiti statici):

```yaml
elbow_injection:
  enabled: true

  # Joint index del gomito in qdot (coerente con joint ordering del group).
  # Nota: se vuoi evitare errori di mapping, in implementazione è consigliato supportare anche:
  #   elbow_joint_name: "elbow_joint"
  # e derivare l'indice da `RobotStateManager::getJointIndexMap()`.
  elbow_index: 2

  # Zona critica vicino all'estensione (riferita al limite MAX del controller per quel giunto)
  critical_zone: 0.40         # rad

  # Gating dal target: se la soluzione IK del target richiede gomito vicino al limite
  # (distanza dal max < target_near_limit_threshold) -> gate=0
  target_near_limit_threshold: 0.25  # rad

  # Gain e saturazione della spinta (verso gomito più flesso, cioè lontano dal max)
  k: 1.0
  max_push_velocity: 0.20     # rad/s
```

## Reachability IK con limiti controller-only

### Comportamento richiesto

Quando `global_planner/reachability_check_enabled` è `true` (parametro già esistente in `cartesian_velocity_controller.cpp`):

- `setTargetPose(...)` e `setWaypoints(...)` devono rifiutare target/waypoints se:
  - l’IK non trova soluzione **oppure**
  - trova soluzione ma la soluzione viola i limiti definiti in `controller_joint_limits/limits`.

### Implementazione (scelta)

Usare l’overload di `moveit::core::RobotState::setFromIK(...)` con **validity callback**.

- **Motivo**: evita falsi negativi dovuti a “prima soluzione fuori limiti anche se ne esiste una valida”.
- **Callback**: riceve la soluzione candidate; verificare per ciascun giunto limitato:
  - \(q \in [min + margin,\; max - margin]\) con `margin` dedicato alla validazione (può riusare `runtime_guard.margin` o avere un `validation_margin` separato; per semplicità iniziale: riusare `margin`).

### Output e caching (necessario per gating)

Quando un target viene accettato:

- salvare una copia della soluzione in una cache nel controller (es. membro `last_target_ik_solution_` + flag `has_last_target_ik_solution_`).
- Per i waypoints:
  - scelta minima: cache sempre la soluzione dell’ultimo waypoint validato (ok per gating “immediato” quando il target corrente cambia).
  - scelta migliore (opzionale): cache per-waypoint (vector) nel `GlobalPlanner` o nel controller.

## Runtime guardrail di posizione (soft braking + hard margin)

### Obiettivo

Anche con target valido, durante l’esecuzione possono esistere termini che spingono i giunti verso i limiti (repulsioni/leash/rumore). Il guardrail deve garantire:

- niente superamento di \([min, max]\) (hard safety)
- rallentamento *morbido e continuo* vicino al limite (soft braking)
- opzionale piccola **velocità di rientro** quando si è molto vicini al limite (configurabile)

### Interazione con i limiter esistenti

- `JointVelocityFilter` (pre-safety) è il posto giusto per garantire jerk/acc-safe, quindi:
  - **requisito**: se `controller_joint_limits.enabled` e almeno un `runtime_guard.enabled` è `true`, allora `joint_velocity_filter/enabled` deve essere **true** (abilitazione forzata con warning, oppure errore fatale: scegliere una policy e documentarla nel codice).
- `JointSafetyLimiter` resta lo strato finale (hard vel/acc uniform scaling).

### Algoritmo (per singolo giunto)

Per ogni giunto limitato con `runtime_guard.enabled`:

- leggere posizione corrente \(q\) (da `RobotStateManager::getCurrentJointPositions`)
- leggere comando desiderato \(\dot q\) (dal Jacobian solve / dopo injection)
- definire distanza dal limite nella direzione del moto:
  - se \(\dot q > 0\): \(d = max - q\)
  - se \(\dot q < 0\): \(d = q - min\)

Regole:

1) **Hard margin + reentry**
- se \(d \le margin\) e stai andando verso il limite:
  - forza \(\dot q\) verso l’interno con modulo `reentry_velocity`

2) **Soft braking**
- se \(margin < d < soft_zone\) e stai andando verso il limite:
  - scala \(\dot q\) con una funzione liscia \(s(d)\in(0,1]\), ad esempio smoothstep:
    - \(x = \text{clamp}(d / soft\_zone, 0, 1)\)
    - \(s(x)=x^2(3-2x)\)
  - \(\dot q \leftarrow s(x)\dot q\)

3) **Se stai andando via dal limite**: non modificare.

## Elbow velocity injection (anti-singolarità gomito esteso)

### Obiettivo

Quando il gomito è vicino all’estensione massima (zona di singolarità), aggiungere una piccola velocità che riporti il gomito verso una configurazione più “flessa”, migliorando manovrabilità.

### Gating richiesto

Definire:
- \(q_{max}\): limite massimo controller-only per `elbow_joint` (da `controller_joint_limits`).
- zona critica: \([q_{max}-critical\_zone,\; q_{max}]\).

Calcolo gate:

- se **non c’è target**: `gate = 1` (spingi via dalla singolarità)
- se c’è target e abbiamo cache IK valida:
  - \(q^*_{elbow,target}\) = valore gomito dalla soluzione IK cached
  - se \(q_{max} - q^*_{elbow,target} < target\_near\_limit\_threshold\) → `gate = 0`
  - altrimenti → `gate = 1`

### Term di injection

- definire un profilo liscio che cresce avvicinandosi a \(q_{max}\), ad esempio con smoothstep:
  - \(d = q_{max} - q_{elbow}\)
  - \(x = \text{clamp}(d / critical\_zone, 0, 1)\)
  - \(w = 1 - s(x)\) dove \(s(x)=x^2(3-2x)\) (quindi \(w\) vale ~1 vicino al limite, ~0 lontano)
- \(\dot q_{bias} = -\text{sat}(k \cdot w,\; max\_push\_velocity)\) (segno negativo = “rientro” dal limite max)
- comando finale:
  - \(\dot q_{elbow} \leftarrow \dot q_{elbow} + gate \cdot \dot q_{bias}\)

Inserimento: subito dopo `joint_velocities = ...` e **prima** di `JointVelocityFilter`, così la spinta viene resa jerk/acc-safe dal filtro.

## Parametri esistenti (da usare correttamente)

- `global_planner/reachability_check_enabled` (già letto in `initializeComponents()`): abilita/disabilita il check IK su target/waypoints.
- `joint_velocity_filter/enabled` e relativi limiti jerk/acc/vel (già implementati): usare come stadio “smooth + jerk-safe”.

Nota: nel YAML attuale esiste `block_unreachable_targets: true` ma nel codice il parametro effettivo è `global_planner/reachability_check_enabled`. Durante l’implementazione decidere se:
- deprecare `block_unreachable_targets` (documentare), oppure
- mapparlo a `global_planner/reachability_check_enabled` per compatibilità.

## Checklist implementazione (file da modificare/aggiungere)

- **Modificare**
  - `launch/cartesian_velocity_controller.launch`: caricare `config/controller_joint_limits.yaml`
  - `src/components/robot_state_manager.cpp`: `setFromIK` + validity callback + check limiti + output solution
  - `src/cartesian_velocity_controller.cpp`:
    - cache soluzione IK in `setTargetPose` / `setWaypoints`
    - applicare `runtime_guard` e `elbow_injection` tra Jacobian solve e `joint_velocity_filter_`
    - policy “joint_velocity_filter forced on” quando guard/injection abilitati

- **Aggiungere**
  - `config/controller_joint_limits.yaml` (nuovo file, versionato nel pacchetto)
  - (opzionale ma consigliato) nuovo componente `components/joint_position_guard.[hpp|cpp]` per tenere pulita `executePipeline`

## Acceptance criteria

- **Reachability**
  - Un target che richiede \(q_{elbow} > max\) (controller-only) viene rifiutato dal controller (anche se l’IK “fisica” lo risolve).
  - Un target valido viene accettato e produce una soluzione cached per il gating.

- **Runtime safety**
  - Nessun giunto supera i limiti controller-only anche in presenza di repulsioni/leash.
  - Avvicinandosi al limite, la velocità verso il limite diminuisce in modo continuo (no step).
  - Dentro `margin`, il comando applica una piccola velocità di rientro configurabile.

- **Elbow injection**
  - Senza target: se gomito entra in zona critica, compare una spinta che lo allontana (filtrata jerk/acc-safe).
  - Con target che richiede gomito vicino a \(q_{max}\): injection disabilitata/attenuata (gate=0).

## Test plan (manuale, rapido)

- **Test 1: reject target fuori limiti**
  - Impostare `controller_joint_limits.limits.elbow_joint.max` più “stretto” del fisico.
  - Pubblicare un `target_pose` che richiede gomito oltre quel max.
  - Atteso: `setTargetPose` ritorna false e logga “REJECTED”.

- **Test 2: guardrail vicino al limite**
  - Con target valido ma con leash/repulsione attivi, spingere il sistema verso il limite.
  - Atteso: \(\dot q\) verso limite viene frenato; dentro `margin` compare `reentry_velocity`.

- **Test 3: gating injection**
  - Caso A: target in configurazione non estesa → injection ON vicino a limite.
  - Caso B: target che richiede estensione → injection OFF.

## Note

Questa è la versione “pronta da implementare”. La precedente analisi (race di startup su `robot_description_planning/joint_limits`, alternative A/B/C/...) è stata rimossa per evitare duplicazioni e ambiguità: se serve recuperarla, usare la history git del file.