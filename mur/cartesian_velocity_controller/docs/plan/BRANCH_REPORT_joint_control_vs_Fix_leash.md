### Report modifiche `cartesian_velocity_controller`: `joint_control` vs `Fix_leash`

**Workspace**: `/home/simone/Test_ws/src`  
**Package**: `cartesian_velocity_controller/`  
**Base comune**: `5878de1b` (branch `main_develop`)  
**Branch analizzati**:
- **`joint_control`**: `5878de1b` → `e4126ba9`
- **`Fix_leash`**: `5878de1b` → `788f4fd7`

> Nota: in questo repo, i due “stati attuali” dei branch coincidono con i commit indicati (`e4126ba9` e `788f4fd7`).

---

### Metodologia

Per ciascun branch ho considerato:
- **Commit inclusi nel range** (filtrati su `cartesian_velocity_controller/`)
- **Diff per file** (stat + lettura delle sezioni di codice rilevanti)
- Traduzione della diff in **funzionalità introdotte** (parametri, comportamento runtime, componenti nuove)

---

### Branch `joint_control` (da `5878de1b` a `e4126ba9`)

#### Commit inclusi (filtrati su package)
- `2d250fc` Update leash reset threshold and improve dynamic reconfiguration logic
- `5f7187b` Move docs file to "old" folder
- `093cb8b` Aggiunti piani per implementazione controllo tramite giunti
- `08ee998` Add IK timeout parameter for reachability checks and improve documentation
- `4030bdd` Aggiunti file pianificazione filtro giunti
- `a2b190f` Implement fixed dt policy for CartesianVelocityFilter to enhance determinism and robustness *(nel range: aggiornamento doc)*
- `46485f5` Add JointVelocityFilter for joint-space smoothing in CartesianVelocityController
- `a899e63` Introdotto piano di modifica per joint control
- `769f8d9` Aggiornato piano di implementazione
- `6d0adfc` Add hybrid joint/cartesian control features to CartesianVelocityController
- `e4126ba` Enhance CartesianVelocityController with joint servo PID tuning and elbow joint limits

#### Funzionalità introdotte (sintesi)

- **Controllo ibrido joint/cartesian (opzionale)**:
  - **Cos’è**: oltre al classico comando in joint-space derivato da \(J^+\cdot V\), viene introdotto un ramo “joint-servo” (basato su IK + PID in spazio giunti) e un meccanismo di **blending** fra i due comandi.
  - **Quando si attiva**: blending weight \(W\) calcolato da:
    - **singolarità** (tramite \(\sigma_{min}\) non pesata dal solver SVD) → `W_sing`
    - **near-target** (vicinanza all’ultimo waypoint, con gate sugli ostacoli) → `W_target`
  - **Come si comporta**:
    - \(qdot\_cart = J^+ V\)
    - \(qdot\_joint\) da **PID su errore di giunto** verso una soluzione IK cachata
    - mix: `qdot_mix = (1-W)*qdot_cart + W*qdot_joint`
    - rate-limit su \(W\) per evitare chattering (`hybrid/blending/w_rate_limit`)
  - **Parametri** (nuovi in `config/controller_params.yaml` e/o via dynamic reconfigure):
    - `hybrid/enabled`, `hybrid/singularity/*`, `hybrid/near_target/*`, `hybrid/joint_servo/*`, `hybrid/ik/*`,
      `hybrid/cartesian_check/*`, `hybrid/safety/*`, `hybrid/joint_weights/*`

- **Joint-space smoothing con limiti jerk/acc/vel (nuovo `JointVelocityFilter`)**:
  - **Cos’è**: filtro in spazio giunti inserito **tra** Jacobian solve (o output ibrido) e `JointSafetyLimiter`.
  - **Obiettivo**: rendere l’uscita più “morbida” e robusta, rispettando **limiti su jerk, accelerazione e velocità**.
  - **Parametri** (nuova sezione in `config/controller_params.yaml`):
    - `joint_velocity_filter/enabled`, `tau`, `uniform_scaling_enabled`
    - `max_joint_velocity`, `max_joint_acceleration`, `max_joint_jerk`
    - policy timing “fixed-step”: `dt_nominal`, `min_dt`, `max_dt`, `max_substeps`, `reset_dt_threshold`, `large_dt_policy`

- **Limiti sul gomito (elbow) con braking + hard guard**:
  - **Cos’è**: possibilità di imporre **hard joint position limits** su un giunto (tipicamente elbow) in gradi in YAML.
  - **Braking**: cap della velocità “in avvicinamento” al limite usando una distanza di arresto coerente con i limiti di accel/jerk.
  - **Hard guard**: protezione finale per evitare il superamento del limite in un singolo step.
  - **Parametri** (nuova sezione in `config/controller_params.yaml`):
    - `elbow_joint_limits/enabled`, `joint_name`, `limits_deg`, `margin_deg`,
      `braking_enabled`, `braking_safety_factor`, `hard_guard_enabled`

- **IK reachability con timeout configurabile**:
  - **Cosa cambia**: la reachability check via MoveIt (`setFromIK`) usa un timeout parametrico per evitare blocchi nel loop.
  - **Parametro**: `target_reachability/ik_timeout` (in `config/controller_params.yaml`)

- **Leash: aggiunta/uso del `leash_reset_threshold` + miglioramenti dynamic_reconfigure**:
  - `local_planner/leash_reset_threshold` viene letto e propagato al `LocalPlanner`.
  - In callback dynamic_reconfigure, `leash_stop` viene clampato \(\ge leash\_start + \epsilon\), mentre `leash_reset` viene solo clampato \(\ge 0.01\) (non vincolato a `leash_stop`).
  - **Nota comportamento**: in questo branch, il leash **scala** (quando attivo) sia `v_combined_linear` sia `v_combined_angular` e quindi può “bloccare” anche componente tangenziale/rotazionale.

- **Debug/telemetria: estensione `PipelineDebug.msg`**:
  - aggiunti campi per **virtual target scaling factor** e per lo stato del **blending ibrido** (W, validità IK, vettori qdot, scaling cartesian-check, ecc.).

#### File/documentazione rilevanti
- **Codice**:
  - `src/cartesian_velocity_controller.cpp` (introduzione ibrido + filtri + elbow limits)
  - `src/joint_velocity_filter.cpp`, `include/.../joint_velocity_filter.hpp` (nuovo filtro)
  - `src/components/local_planner.cpp` (leash “blocking”)
  - `cfg/ControllerTuning.cfg` (nuovi parametri dynamic_reconfigure per hybrid)
  - `msg/PipelineDebug.msg` (debug esteso)
- **Config**:
  - `config/controller_params.yaml` (nuove sezioni: `joint_velocity_filter`, `target_reachability/ik_timeout`, `elbow_joint_limits`, `hybrid`)
- **Docs**:
  - aggiunte: `docs/HYBRID_*`, `docs/JOINT_*`
  - riordino: vari md spostati in `docs/old/`

---

### Branch `Fix_leash` (da `5878de1b` a `788f4fd7`)

#### Commit inclusi (filtrati su package)
- `b46fa9f` Add detailed implementation plan for robust pose-tracking pipeline with non-blocking Virtual Target Leash
- `788f4fd` Add pose tracking parameters and implementation for Cartesian Velocity Controller

#### Funzionalità introdotte (sintesi)

- **Pose tracking (target_raw → target_filtered) per generare la twist di ingresso al filtro (Level C)**:
  - **Cos’è**: invece di usare direttamente l’uscita del `LocalPlanner` come `desired_twist`, viene calcolata una velocità desiderata a partire dall’errore tra:
    - `target_raw` (virtual target integrato nel LocalPlanner)
    - `target_filtered_prev` (stato del `CartesianVelocityFilter`)
  - **Formula operativa (concetto)**:
    - $v\_{des} \propto \frac{k}{\tau}\cdot (target\_raw - target\_filtered\_prev)$
    - con cap sulle velocità (mai oltre i limiti del local planner).
  - **Parametri nuovi (ROS params)**:
    - `pose_tracking/enabled`
    - `pose_tracking/tau_linear`, `pose_tracking/tau_angular`
    - `pose_tracking/k_linear`, `pose_tracking/k_angular`
    - `pose_tracking/max_linear_velocity`, `pose_tracking/max_angular_velocity`
  - **Nota config**: questi parametri sono letti da param server nel codice, ma **non risultano aggiunti** a `config/controller_params.yaml` in questo branch.

- **Leash “non‑blocking” (non scala la componente tangenziale)**:
  - **Problema affrontato**: con il leash “classico” (scaling uniforme) il virtual target può essere rallentato anche nella direzione tangenziale, creando comportamenti poco desiderati quando il robot viene respinto da ostacoli/POI.
  - **Soluzione**:
    - quando `scaling_factor < 1`, viene scalata **solo** la componente di velocità “radiale uscente” (che aumenta la distanza robot→target_raw),
    - la componente tangenziale viene lasciata invariata,
    - per scelta di design, **non viene scalata** la velocità angolare.

- **Validazione più “coerente” dei parametri leash in runtime**:
  - in dynamic reconfigure viene imposto:
    - `leash_stop >= leash_start + 0.01`
    - `leash_reset >= leash_stop + 0.01`

#### File/documentazione rilevanti
- **Codice**:
  - `src/cartesian_velocity_controller.cpp` (pose tracking params + generazione twist via pose error)
  - `src/components/local_planner.cpp` (leash tangenziale non bloccante)
  - `include/.../cartesian_velocity_controller.hpp` (struct/parametri per pose tracking)
- **Docs**:
  - `docs/VIRTUAL_TARGET_LEASH_TANGENTIAL_BLOCK.md`
  - `docs/IMPLEMENTATION_PLAN_POSE_TRACKING_PIPELINE.md`

---

### Differenze tra gli stati attuali: `joint_control` (`e4126ba9`) vs `Fix_leash` (`788f4fd7`)

#### Differenze funzionali principali

- **Architettura motion generation (Level C)**:
  - **`joint_control`**: `desired_twist = local_output.combined_*` → `CartesianVelocityFilter`
  - **`Fix_leash`**: `desired_twist` generata da **pose tracking** (errore `target_raw` vs `target_filtered_prev`) → `CartesianVelocityFilter`

- **Gestione leash**
  - **`joint_control`**: scaling uniforme su `v_combined_linear` **e** `v_combined_angular` (effetto “blocking” anche tangenziale/rotazionale)
  - **`Fix_leash`**: scaling “non‑blocking” (scala solo la componente **radiale uscente**), non scala l’angolare
  - **Validazione parametri**:
    - `joint_control`: `leash_reset_threshold` clampato \(\ge 0.01\) (può risultare < `leash_stop`)
    - `Fix_leash`: `leash_reset_threshold` clampato \(\ge leash_stop + 0.01\)

- **Controllo ibrido e filtri in joint space**
  - **presenti solo in `joint_control`**:
    - blending `qdot_cart` / `qdot_joint` con IK caching + joint PID
    - `JointVelocityFilter` (jerk/acc/vel) + policy fixed-step
    - limiti gomito con braking + hard guard
    - check post-mix sulle velocità cartesiane

- **Configurazione e debug**
  - **`joint_control`** aggiunge/estende:
    - `config/controller_params.yaml` (sezioni `joint_velocity_filter`, `hybrid`, `elbow_joint_limits`, `target_reachability/ik_timeout`)
    - `cfg/ControllerTuning.cfg` (gruppo dynamic_reconfigure per hybrid)
    - `msg/PipelineDebug.msg` (campi leash/hybrid)
  - **`Fix_leash`** aggiunge:
    - parametri `pose_tracking/*` letti da ROS param server (non in YAML del package)
    - docs specifiche su leash tangenziale e pipeline pose-tracking

#### Impatto su un eventuale merge/rebase

I conflitti “grossi” attesi sono su:
- `src/cartesian_velocity_controller.cpp` (pipeline Level C/D e gestione parametri)
- `src/components/local_planner.cpp` (logica leash)
- `include/.../cartesian_velocity_controller.hpp`

Suggerimento pratico: se l’obiettivo è avere **ibrido + leash non‑blocking + pose tracking**, conviene scegliere un branch “base” e portare l’altra feature in modo mirato:
- portare **leash non‑blocking + clamp reset** da `Fix_leash` dentro `joint_control`
- valutare se il **pose tracking** resta necessario in presenza del nuovo ibrido (dipende da come vuoi che il filtro Level C si comporti: tracking del virtual target vs smoothing della twist del local planner)

