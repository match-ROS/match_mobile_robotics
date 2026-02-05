## Joint velocity/acc/jerk in spazio giunti — piano d’azione aggiornato (2026) + punti aperti

> Obiettivo: avere un documento unico “operativo” (compatibilità + piano d’implementazione) in cui puoi rispondermi direttamente sui punti aperti.

---

### 0) TL;DR (come sta messo il codice oggi)

La pipeline attuale (semplificata) è:

- Level B: `LocalPlanner` (target_raw + v_desired)
- Level C: `CartesianVelocityFilter` (jerk→acc→vel + τ in cartesiano, con stato e integrazione)
- Level D: `PIDController` → `JacobianSolver` (twist → qdot)
- Final: `JointSafetyLimiter` (uniform scaling su **vel** + **acc**, hard safety)
- Publish: `final_joint_velocities`

Punti chiave dal codice attuale:

- **`dt` già clampato** nel timer callback: `dt = clamp(dt, 0.001, 0.1)`.
- **`previous_joint_velocity_` è la velocità effettivamente inviata al robot** (post-limiter) e viene aggiornato ogni ciclo.
- La diagnostica del safety limiter è già pubblicata su `PipelineDebug` (scaling + reason string).

Questo rende **fattibili** entrambe le strade:

- **Piano A**: aggiungere jerk come vincolo “hard” dentro `JointSafetyLimiter` (uniform scaling, opzionale).
- **Piano B**: introdurre un nuovo `JointVelocityFilter` (jerk→acc→vel + τ) tra IK e safety limiter (comfort/smoothness, opzionale).

---

### 1) Scopo funzionale (definizione precisa)

Dato un comando candidato in giunti \( \dot q_{cmd} \) (output IK), vogliamo produrre un comando finale \( \dot q_{out} \) che:

- rispetti limiti di **velocità**, **accelerazione** e (opzionale) **jerk** in giunti
- sia configurabile (enabled/disabled)
- sia diagnosticabile (scaling, joint limiting, motivo)

Due modalità desiderabili (da decidere):

- **Uniform scaling**: preserva direzione di \( \dot q \) (un giunto limitante rallenta tutti).
- **Per-giunto indipendente**: ogni giunto viene limitato indipendentemente (cambia direzione del vettore qdot; spesso più “pratico”).

---

### 2) Piano A — “Joint jerk limiter” dentro `JointSafetyLimiter` (hard safety)

#### 2.1 Che cosa fa

Estende il safety limiter per aggiungere un ulteriore vincolo:

- vel: \( |\dot q_{out,i}| \le \dot q_{max,i} \)
- acc: \( |\ddot q_{out,i}| \le \ddot q_{max,i} \)
- jerk (nuovo): \( |\dddot q_{out,i}| \le \dddot q_{max,i} \)

con output definito come:

\[
\dot q_{out} = s \dot q_{cmd}, \quad s \in [0,1]
\]

#### 2.2 Stato necessario

Per limitare jerk serve conoscere l’accelerazione precedente:

- **`previous_joint_acceleration_`** (nuovo stato, size = num joints)

Nota importante: nel controller oggi abbiamo già `previous_joint_velocity_` = comando effettivo precedente.

#### 2.3 Algoritmo consigliato (intersezione intervalli su s)

Per ogni giunto si traduce ogni vincolo in un intervallo ammissibile per \( s \), si intersecta globalmente \( S=[0,1] \), e si sceglie \( s = \max(S) \) (minimo intervento).

**Policy su intersezione vuota**: va decisa (vedi “punti aperti”).

#### 2.4 File impattati (stima)

- `include/cartesian_velocity_controller/components/joint_safety_limiter.hpp`
- `src/components/joint_safety_limiter.cpp`
- `include/cartesian_velocity_controller/types/pipeline_types.hpp` (enum `LimitType`)
- `src/cartesian_velocity_controller.cpp` (mappatura `safety_limiting_reason`)
- `config/controller_params.yaml` (parametri)
- `cfg/ControllerTuning.cfg` (dynamic reconfigure)

#### 2.5 Pro / contro operativi

- **Pro**: minimo impatto sulla pipeline; vero “guardrail hard”; compatibile col concetto “final safety”.
- **Contro**: non hai una manopola τ; jerk dipende da campioni successivi → può risultare “nervoso” se `dt` varia.

---

### 3) Piano B — `JointVelocityFilter` (jerk→acc→vel + τ) tra IK e safety limiter (comfort)

#### 3.1 Dove si inserisce (minima invasività)

Oggi:

- IK → `qdot_cmd` → `JointSafetyLimiter` → publish

Proposto:

- IK → `qdot_cmd` → **NEW `JointVelocityFilter`** → `qdot_filtered` → `JointSafetyLimiter` → publish

#### 3.2 Algoritmo (analogo al filtro cartesiano, ma in spazio giunti)

Per ogni ciclo:

1) \( \ddot q_{des} = (\dot q_{des} - \dot q)/\tau \)
2) \( \dddot q_{cmd} = (\ddot q_{des} - \ddot q)/dt \)
3) saturazione jerk
4) integrazione acc + saturazione acc
5) integrazione vel + saturazione vel

**Nota**: in B puoi scegliere *uniform scaling* o *per-giunto*.

#### 3.3 Stato interno richiesto

- \( \dot q \) corrente (vettore)
- \( \ddot q \) corrente (vettore)
- (opzionale) \( \dddot q \) per debug
- (opzionale) accumulator per `dt_nominal` se scegli substeps

#### 3.4 Impatto sul controllo (criticità principale)

Filtrare **dopo** PID/IK aggiunge dinamica sull’attuatore: il PID potrebbe “spingere di più” (windup) per inseguire il target filtrato.

Mitigazioni possibili:

- ridurre/ritarare `Ki` (o migliorare anti-windup)
- usare `JointVelocityFilter` come “soft layer” con limiti più larghi del safety limiter
- tenere `JointSafetyLimiter` come hard guardrail finale (consigliato)

#### 3.5 File impattati (stima)

- `include/cartesian_velocity_controller/joint_velocity_filter.hpp` (nuovo)
- `src/joint_velocity_filter.cpp` (nuovo)
- `src/cartesian_velocity_controller.cpp` (istanziazione + chiamata nel loop + param loading)
- `CMakeLists.txt` (nuova lib + link nel nodo)
- `config/controller_params.yaml` (parametri)
- `cfg/ControllerTuning.cfg` (dynamic reconfigure)
- (opzionale) diagnostica: `PipelineDebug.msg` o msg/topic dedicato

---

### 4) Strategia consigliata di implementazione (rollout a basso rischio)

#### Step 1 — “No behavior change” (foundation)

Obiettivo: aggiungere struttura + parametri con default “off” così non cambia nulla.

- Piano A:
  - aggiungere campi/parametri jerk al limiter ma `jerk_limiting_enabled=false` di default
- Piano B:
  - aggiungere nuovo `JointVelocityFilter` ma `enabled=false` di default (pass-through)

#### Step 2 — Abilitare la feature (behavior change controllato)

- implementare logica core (A o B)
- definire policy sui corner case (dt piccolo, inversioni, intersezione vuota)
- aggiungere log throttled/diagnostica

#### Step 3 — Parametrizzazione completa + tuning

- supporto limiti scalari vs “3+3” vs array size=6
- dynamic_reconfigure per i parametri “pratici”

#### Step 4 — Diagnostica (necessaria per tuning serio)

Decidere se:

- estendere `PipelineDebug.msg` (richiede rebuild downstream)
- oppure creare un topic/msg separato del joint filter (più isolato)

---

### 5) Piano d’azione “file-per-file” (checklist)

> Questo è pensato per essere eseguibile senza ambiguità una volta chiusi i punti aperti.

#### 5.1 Se scegli Piano A (jerk nel safety limiter)

- **`JointSafetyLimiter`**
  - aggiungere `max_joint_jerks_`, `jerk_limiting_enabled_`, `previous_joint_acceleration_`
  - aggiungere `computeJerkScalingFactor(...)`
  - estendere `reset()` per azzerare `previous_joint_acceleration_`
  - aggiornare `limit(...)`:
    - calcolare jerk scaling e combinarlo con vel/acc (min factor)
    - aggiornare `previous_joint_acceleration_` usando la velocità *post-scaling* e `previous_velocity`
- **Tipi / debug**
  - aggiungere `JERK` a `SafetyLimiterOutput::LimitType` in `pipeline_types.hpp`
  - aggiornare `safety_limiting_reason` in `cartesian_velocity_controller.cpp`
- **Parametri**
  - YAML:
    - `joint_safety_limiter/jerk_limiting_enabled`
    - `joint_safety_limiter/max_joint_jerk` (scalare) e/o `max_joint_jerk_array` (array)
    - eventuale `min_dt_for_jerk` (se serve)
  - dynamic_reconfigure:
    - `max_joint_jerk` (double)
    - `jerk_limiting_enabled` (bool)

#### 5.2 Se scegli Piano B (`JointVelocityFilter`)

- **Nuovi file**
  - `include/cartesian_velocity_controller/joint_velocity_filter.hpp`
  - `src/joint_velocity_filter.cpp`
- **CMake**
  - aggiungere `add_library(joint_velocity_filter ...)`
  - linkare la lib nel `cartesian_velocity_node`
- **Controller**
  - in `initializeComponents()`:
    - leggere parametri `joint_velocity_filter/*`
    - creare istanza filtro e settare config
  - in `executePipeline()`:
    - `qdot_cmd = jacobian_pinv * command_twist;`
    - `qdot_filtered = joint_velocity_filter_->filter(qdot_cmd, dt);`
    - passare `qdot_filtered` al `JointSafetyLimiter`
  - in `start()/reset`:
    - `joint_velocity_filter_->reset()` (policy da decidere)
- **Parametri**
  - YAML base:
    - `joint_velocity_filter/enabled` (default false)
    - `joint_velocity_filter/tau`
    - limiti vel/acc/jerk: scalari o array 6
    - `dt_nominal/min_dt/max_dt/max_substeps/reset_dt_threshold/large_dt_policy` (solo se decidiamo “dt fisso”)
  - dynamic_reconfigure (senza array):
    - `joint_filter_tau`
    - `joint_filter_max_acceleration`
    - `joint_filter_max_jerk`
    - `joint_filter_enabled` (opzionale)
- **Diagnostica**
  - minimo: pubblicare scaling/limit reason in un topic dedicato o estendere `PipelineDebug`.

---

### 6) Criticità tecniche (risk register)

- **R1 — dt variabile e jerk**
  - jerk usa \(1/dt\) e spesso \(1/dt^2\) in formulazioni “hard”.
  - oggi dt è clampato → aiuta, ma non elimina jitter.
  - mitigazione “robusta”: `dt_nominal` + substeps (decidere).

- **R2 — Direction reversal**
  - con uniform scaling e vincoli jerk/acc, l’insieme ammissibile può diventare vuoto.
  - serve policy esplicita (stop vs degrade).

- **R3 — Interazione con PID (soprattutto Piano B)**
  - aggiunta di dinamica in giunti post-PID può generare windup/oscillazioni.
  - serve strategia di tuning/anti-windup.

- **R4 — Dynamic reconfigure vs limiti per-giunto**
  - dynreconf non supporta array → o si usano 2 scalari “3+3” oppure si rinuncia all’editing runtime dei limiti per-giunto.

- **R5 — Diagnostica e compatibilità msg**
  - cambiare `PipelineDebug.msg` richiede rebuild di chiunque lo usi.
  - alternativa: nuovo msg/topic dedicato.

---

### 7) Punti aperti — rispondi qui

> Compila le risposte direttamente sotto ogni domanda (puoi cancellare le opzioni non scelte).

#### P1 — Quale strada implementiamo per prima?

- [ ] **A**: jerk “hard” dentro `JointSafetyLimiter` (uniform scaling)
- [ ] **B**: nuovo `JointVelocityFilter` (τ + jerk→acc→vel)
- [ ] **B + safety limiter** (consiglio): filtro per comfort + limiter hard finale
- [ ] Altro:

**Risposta:**

---

#### P2 — Modalità di limitazione in giunti (direzione del vettore qdot)

- [ ] **Uniform scaling** (preserva direzione)
- [ ] **Per-giunto indipendente** (non preserva direzione)
- [ ] Voglio entrambe, selezionabili con parametro (default: ________)

**Risposta:**

---

#### P3 — Formato limiti (vel/acc/jerk)

Scegli un formato “source of truth”:

- [ ] solo scalari uniformi (come oggi nel safety limiter)
- [ ] array size=6 in YAML (es: `[a,a,a,b,b,b]`)
- [ ] “3+3” con 2 scalari (arm/wrist) in YAML + dynreconf
- [ ] altro:

**Dettaglio valori iniziali che vuoi (indicativi):**

- vel arm: ___, vel wrist: ___
- acc arm: ___, acc wrist: ___
- jerk arm: ___, jerk wrist: ___

---

#### P4 — Gestione dt (semplice vs robusta)

- [ ] usare `dt` del loop (già clampato 0.001..0.1) per tutto (più semplice)
- [ ] introdurre `dt_nominal` + accumulator + substeps (più robusto; un po’ più CPU)

Se scegli `dt_nominal`:

- `dt_nominal` = ________ s (es. 0.002 per 500Hz, 0.01 per 100Hz)
- `max_substeps` = ________
- policy per dt grande (`large_dt_policy`): `hold_last` / `reset_to_zero` / `reset_to_desired`
- `reset_dt_threshold` = ________ s

**Risposta:**

---

#### P5 — Reset / continuità quando cambia target

Oggi esiste `reset_filter_on_target_change` e il reset “re-seeda” il filtro cartesiano allo stato attuale (per evitare jump).

Per il filtro in giunti (se Piano B):

- [ ] **non resettare** e mantenere stato (come da tua nota)
- [ ] resettare a zero
- [ ] reseed allo stato “misurato” (qdot attuale da joint_states)
- [ ] reseed a `qdot_cmd` (quello che arriva da IK) con acc=0

**Risposta:**

---

#### P6 — Policy corner case: intersezione vuota / inversione segno (Piano A) o saturazioni aggressive (Piano B)

Scegli una policy “safe”:

- [ ] stop per 1 ciclo (`s=0`)
- [ ] degrade: ignora jerk per 1 ciclo e applica solo acc+vel (log throttled)
- [ ] altro:

**Risposta:**

---

#### P7 — Diagnostica: dove la pubblichiamo?

- [ ] estendiamo `PipelineDebug.msg` (aggiungendo campi del joint filter/jerk)
- [ ] nuovo topic/msg dedicato (isolato, meno rischio compatibilità)
- [ ] nessuna diagnostica extra oltre a scaling/reason (sconsigliato)

Se vuoi estendere `PipelineDebug.msg`, quali campi ti servono?

- [ ] `joint_velocity_after_joint_filter`
- [ ] `joint_acceleration_after_joint_filter`
- [ ] `joint_jerk_after_joint_filter`
- [ ] `joint_filter_scaling_factor`
- [ ] `joint_filter_limit_reason`
- [ ] altro:

**Risposta:**

---

#### P8 — Interazione con PID (solo se Piano B)

Preferenze:

- [ ] accetto che il tracking peggiori (comfort > tracking)
- [ ] voglio minimizzare impatto: filtro leggero + limiti più larghi del safety limiter
- [ ] sono disposto a ritunare PID (soprattutto Ki) dopo l’introduzione del filtro

**Risposta:**

---

### 8) Note finali (per quando iniziamo a codare)

- Se scegliamo **Piano B** e **per-giunto indipendente**, il safety limiter (uniform scaling) resta comunque utile come “ultima rete”, ma potrà intervenire più spesso se i limiti del filtro sono più permissivi.
- Se scegliamo di estendere `PipelineDebug.msg`, va pianificato un rebuild di workspace e di eventuali nodi/tooling che lo consumano.

