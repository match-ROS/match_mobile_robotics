## Scopo del documento

Questo documento definisce un **piano implementativo** per creare un package `teleoperation` (in `src/match_mobile_robotics/teleoperation`) con:

- **codice nuovo e indipendente** (master e slave), su cui poter lavorare senza dipendenze “architetturali” dal vecchio controller;
- **core logic in C++** (in particolare per lo slave), con eventuali script Python di supporto;
- riuso “pragmatico” tramite **copia/adattamento** delle parti di codice già buone presenti in `mur/cartesian_velocity_controller`.

### Nuove specifiche (da seguire fedelmente)

- **Master**:
  - per ora puoi usare il codice che hai già per muoverlo (freedrive/hand-guiding/altro).
  - in `teleoperation` possiamo comunque aggiungere un **publisher di stato** (posa+velocità cartesiana) del master verso lo slave (script Python semplice o nodo C++).

- **Slave** (da implementare subito nel nuovo package):
  - riceve **una posa cartesiana 6D** (target) per calcolare l’errore;
  - riceve **un feed-forward di velocità cartesiana 6D** dal master;
  - il comando di velocità deve essere **quasi integralmente preso dal master** (feed-forward dominante), e la correzione su errore deve essere “piccola” e configurabile.

---

## Sintesi architetturale (MVP aggiornato)

### Componenti

- **`teleoperation_master_state_publisher`** (nuovo, semplice; può essere Python all’inizio):
  - legge la posa TCP del master (tipicamente da TF);
  - pubblica:
    - `geometry_msgs/PoseStamped` (target pose)
    - `geometry_msgs/TwistStamped` (feed-forward twist)

- **`teleoperation_slave_cartesian_vv_controller`** (nuovo, core C++):
  - legge:
    - target pose (6D)
    - feed-forward twist (6D)
    - `sensor_msgs/JointState` (per FK/Jacobiano)
  - calcola:
    - errore posa (posizione + orientamento)
    - twist comando = **twist_ff (dominante)** + correzione PID su errore
    - IK in velocità via Jacobiano smorzato → `qdot_cmd`
  - pubblica:
    - `std_msgs/Float64MultiArray` su un topic `velocity_command_topic` (verso un controller di velocità giunti ros_control)

### Legge di controllo (essenziale)

Definiamo:

- $^{W}T_{tcp}$: posa corrente del TCP (FK dallo slave)
- $^{W}T_{tcp}^*$: posa target ricevuta (dal master)
- $v_{ff}\in\mathbb{R}^6$: twist feed-forward ricevuto dal master (dominante)
- $e_p\in\mathbb{R}^3$: errore posizione
- $e_o\in\mathbb{R}^3$: errore orientamento (axis-angle)

Comando cartesiano:

$$
v_{cmd} =
\underbrace{k_{ff}\,v_{ff}}_{\text{dominante}} +
\underbrace{\begin{bmatrix}
PID_p(e_p)\\
PID_o(e_o)
\end{bmatrix}}_{\text{correzione piccola}}
$$

Conversione a giunti:

$$
\dot{q}_{cmd} = J(q)^{+}\, v_{cmd}
$$

con $J^{+}$ pseudo-inversa **smorzata** (SDLS / DLS) per robustezza a singolarità.

---

## Cosa esiste già in `mur/cartesian_velocity_controller` (e perché è utile)

Questa sezione NON è per “dipendere” dal vecchio controller, ma per individuare **pezzi di codice copiabili** nel nuovo `teleoperation`.

### Parti “buone” da copiare/adattare (C++)

Dal package `src/match_mobile_robotics/mur/cartesian_velocity_controller` risultano molto riusabili (copiandole nel nuovo namespace/package):

- **Jacobian pseudo-inverse smorzata (SDLS)**:
  - origine: `include/cartesian_velocity_controller/components/jacobian_solver.hpp`
  - impl: `src/components/jacobian_solver.cpp`

- **Robot model + FK + Jacobiano via MoveIt RobotState**:
  - origine: `include/cartesian_velocity_controller/components/robot_state_manager.hpp`
  - impl: `src/components/robot_state_manager.cpp`

- **PID generico con feedforward, saturazione e anti-windup** (ottimo per fare la “correzione piccola”):
  - origine: `include/cartesian_velocity_controller/components/pid_controller.hpp`
  - impl: `src/components/pid_controller.cpp`

- **Limiter finale di sicurezza a scalatura uniforme** (velocità/accelerazione giunti):
  - origine: `include/cartesian_velocity_controller/components/joint_safety_limiter.hpp`
  - impl: `src/components/joint_safety_limiter.cpp`

- **(Opzionali) filtri di smoothness e dt robustness**:
  - filtro cartesiano jerk/acc/vel (`CartesianVelocityFilter`):
    - origine: `include/cartesian_velocity_controller/velocity_filter.hpp`
    - impl: `src/velocity_filter.cpp`
  - filtro joint-space jerk/acc/vel (`JointVelocityFilter`):
    - origine: `include/cartesian_velocity_controller/joint_velocity_filter.hpp`
    - impl: `src/joint_velocity_filter.cpp`

  **Risposta:** questa parte opzionale non mi serve

- **(Opzionale) guardrail su limiti di posizione giunti**:
  - origine: `include/cartesian_velocity_controller/components/joint_position_guard.hpp`
  - impl: `src/components/joint_position_guard.cpp`

  **Risposta:** questa parte opzionale non mi serve

- **Utility matematiche** (errore orientamento axis-angle, clamp, smoothstep, limitNorm):
  - origine: funzioni `inline` in `src/cartesian_velocity_controller.cpp` (es. `orientationErrorAxisAngle`)
  - da estrarre in un header nuovo (es. `teleoperation/math_utils.hpp`)

### Parti da NON portare nel controller slave (non necessarie per la specifica attuale)

Per mantenere semplice il codice nuovo dello slave, *non serve* copiare:

- planner/waypoints (`GlobalPlanner`, `LocalPlanner`)
- repulsione + map3d + POI
- marker RViz
- dynamic_reconfigure (a meno che tu non lo voglia)

---

## Package `teleoperation` (nuovo): struttura proposta

Obiettivo: avere un vero package catkin, con core C++ e componenti copiati dal vecchio controller ma **rinamespacizzati**.

Struttura (proposta):

- `src/match_mobile_robotics/teleoperation/`
  - `package.xml`
  - `CMakeLists.txt`
  - `include/teleoperation/`
    - `teleop_slave_controller.hpp`
    - `components/robot_state_manager.hpp` (copia/adattamento)
    - `components/jacobian_solver.hpp` (copia/adattamento)
    - `components/pid_controller.hpp` (copia/adattamento)
    - `components/joint_safety_limiter.hpp` (copia/adattamento)
    - `components/joint_velocity_filter.hpp` (opzionale)
    - `components/cartesian_velocity_filter.hpp` (opzionale)
    - `components/joint_position_guard.hpp` (opzionale)
    - `math_utils.hpp` (nuovo: estratto da vecchio controller)
    - `types.hpp` (nuovo: struct config minimali)
  - `src/`
    - `teleop_slave_controller.cpp`
    - `teleop_slave_controller_node.cpp`
    - `components/*.cpp` (copiati/adattati)
  - `scripts/`
    - `teleop_master_state_publisher.py` (opzionale iniziale)
  - `config/`
    - `slave_controller_params.yaml`
    - `master_publisher_params.yaml`
  - `launch/`
    - `teleop_slave_controller.launch`
    - `teleop_master_state_publisher.launch`

---

## Interfacce ROS (nuovo codice)

### Slave controller (C++)

Sottoscrizioni:

- **`target_pose`** (`geometry_msgs/PoseStamped`)
  - posa target 6D per errore
- **`feedforward_twist`** (`geometry_msgs/TwistStamped`)
  - twist 6D dal master (dominante)
- **`joint_states`** (`sensor_msgs/JointState`)
  - stato giunti dello slave per FK/Jacobiano

Pubblicazioni:

- **`velocity_command_topic`** (`std_msgs/Float64MultiArray`)
  - comandi $\dot{q}$ verso controller ros_control

Parametri minimi:

- `group_name`, `tcp_link`, `robot_description_param`
- `joint_state_topic`, `velocity_command_topic`
- `control_rate`
- `k_ff` (default consigliato: 1.0)
- PID pos/orient (Kp, Ki, Kd, limiti)

### Master state publisher (Python o C++)

Responsabilità minima:

- leggere `master_global_frame -> master_ee_frame` da TF;
- pubblicare `target_pose` e `feedforward_twist`.

Per la velocità feed-forward:

- opzione semplice: differenza finita della posa (con filtro leggero), oppure (se disponibile) leggere direttamente twist misurato.

---

## Inventario “safety controls” dal codice esistente + decisioni richieste

Qui sotto elenco i controlli/sistemi di sicurezza presenti nel vecchio `cartesian_velocity_controller` che possiamo copiare nel nuovo slave controller.

**Per ciascuno** ti chiedo esplicitamente di scegliere una delle tre opzioni:

- **MANTIENI** (sempre attivo)
- **DISATTIVABILE** (parametro runtime/rosparam)
- **ELIMINA**

Compila le checkbox nel modo che preferisci: è la “spec” che userò quando implemento il codice C++ dello slave.

| # | Safety / Robustness control | Cosa fa | Origine (vecchio codice) | Decisione (scegli) |
|---:|---|---|---|---|
| 1 | **SDLS/DLS damping su pseudo-inversa** | Evita esplosioni di $\dot{q}$ vicino a singolarità | `components/jacobian_solver.*` |  MANTIENI |
| 2 | **Limiti velocità giunti con scalatura uniforme** | Se un giunto supera $\dot{q}$ max, scala tutto il vettore preservando direzione | `components/joint_safety_limiter.*` |  MANTIENI  |
| 3 | **Limiti accelerazione giunti (uniform scaling)** | Limita $\ddot{q}$ usando $\dot{q}_{prev}$ e scala uniformemente | `components/joint_safety_limiter.*` | DISATTIVABILE |
| 4 | **JointVelocityFilter (jerk/acc/vel in joint space)** | Smoothing e limiti jerk/acc/vel prima del limiter finale | `joint_velocity_filter.*` | ELIMINA |
| 5 | **CartesianVelocityFilter (jerk/acc/vel in Cartesian)** | Smoothing del twist comando (utile se feedforward è rumoroso/jitter) | `velocity_filter.*` | ELIMINA |
| 6 | **dt clamp + fixed-dt substepping (cart/joint filter)** | Robustezza a jitter del loop; limita dt e substeps | `velocity_filter.hpp`, `joint_velocity_filter.hpp` | SENZA I FILTRI SOPRA NON CREDO QUESTO SERVA |
| 7 | **Pose deadband (pos/orient)** | Evita “hunting” per errori piccoli | `applyDeadband` + param deadband | DISATTIVABILE |
| 8 | **EMA filter su posa TCP misurata** | Attenua rumore su FK (TCP pose) | `pose_filter_alpha` + `filterTcpPose` | MANTIENI |
| 9 | **PID output saturation (norm clamp)** | Limita correzione PID (evita che la correzione domini il feedforward) | `PIDController::saturate()` | MANTIENI |
|10 | **Anti-windup dinamico** | Evita integrale che satura quando P domina | `PIDController::applyAntiWindup()` | MANTIENI |
|11 | **Derivata filtrata (low-pass)** | Riduce rumore sul D | `PIDConfig::derivative_filter_tau` | MANTIENI |
|12 | **Target reachability check via IK** | Può rifiutare target non raggiungibili | `RobotStateManager::checkPoseReachability()` | per la configurazione attuale non credo serva |
|13 | **Controller-only joint position limits** | Limiti min/max “controller-side” per gating/guardrail | `types/controller_joint_limits.*` | ELIMINA |
|14 | **JointPositionGuard (soft braking + hard margin)** | Evita che $\dot{q}$ spinga verso limiti posizione | `components/joint_position_guard.*` | ELIMINA |
|15 | **Stale-input timeout (pose/twist)** | Se non arrivano input recenti → $\dot{q}=0$ | (da implementare nel nuovo) | va bene |
|16 | **Publish zero velocity on fault** | Comportamento fail-safe su errori (FK/Jacobiano/input) | pattern `publishZeroVelocity()` | MANTIENI |
|17 | **Frame handling: reject_on_tf_failure / accept_empty_frame** | Evita target in frame errati o TF mancante | vecchio `setTargetPose(PoseStamped)` | in questo caso forse è meglio pubblicare una velocità nulla |
|18 | **Jacobian frame conversion (rotation-only)** | Allinea frame Jacobiano e comando twist | `jacobian_source_frame/target_frame` | MANTIENI |
|19 | **command_timeout_ (nota: nel vecchio codice è caricato ma non usato)** | Timeout “di comando” (da chiarire/decidere se implementare) | param `command_timeout` | ELIMINA |

---

## Piano implementativo (nuovo slave controller, C++)

### Step A — Skeleton package

- creare `package.xml` + `CMakeLists.txt` per `teleoperation` con dipendenze:
  - `roscpp`, `std_msgs`, `geometry_msgs`, `sensor_msgs`
  - `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_eigen`
  - `moveit_core`, `moveit_ros_planning`
  - `Eigen3`

### Step B — Copia “mirata” componenti buoni

Copie minime consigliate (rinamespacizzate):

- `RobotStateManager`
- `JacobianSolver`
- `PIDController`
- `JointSafetyLimiter`

Componenti opzionali in base alla tabella “Safety controls”:

- `JointVelocityFilter`, `CartesianVelocityFilter`, `JointPositionGuard`, (altro)

### Step C — Implementare `TeleopSlaveController` (core)

- caching dell’ultimo `target_pose` e `feedforward_twist` + timestamp
- loop timer a `control_rate`
- FK TCP + Jacobiano
- calcolo errore posa (pos + axis-angle)
- correzione PID (pos/orient separati) + somma a feedforward
- $J^{+}$ smorzata → $\dot{q}$
- applicazione dei safety scelti (guard/filter/limiter)
- publish `Float64MultiArray`

### Step D — Script master publisher (opzionale, Python)

Solo per fornire input coerenti allo slave:

- TF lookup master pose
- stima twist feedforward
- publish `PoseStamped` + `TwistStamped`

---

## Nota su “indipendenza” dal vecchio controller

Anche se copiamo file/componenti, il nuovo codice deve risultare:

- in **namespace `teleoperation`** (o similare), con nomi chiari e file propri;
- con config/parametri pensati per teleop (pose+ff), non per planning/waypoints;
- senza dipendenze runtime dal nodo `cartesian_velocity_controller` (si riusano solo idee/implementazioni copiate).

