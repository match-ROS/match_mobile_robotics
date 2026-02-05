## Proposta di implementazione: limitazione jerk→acc→vel in spazio giunti (scaling uniforme, opzionale)

### Contesto (stato attuale nel pacchetto)

Nel pacchetto `cartesian_velocity_controller`:

- **Filtro cartesiano (Level C)**: `CartesianVelocityFilter` implementa una cascata **jerk→accelerazione→velocità** con stato interno (vel, acc, jerk) e integrazione di `target_filtered`:
  - implementazione: `include/cartesian_velocity_controller/velocity_filter.hpp` + `src/velocity_filter.cpp`
  - logica: \(a_\text{des}=(v_\text{des}-v)/\tau\), \(j=(a_\text{des}-a)/dt\), poi saturazioni su \(\|j\|\), \(\|a\|\), \(\|v\|\).

- **Limitatore in giunti (final stage)**: `JointSafetyLimiter` applica **scaling uniforme** dell’intero vettore \(\dot q\) per rispettare:
  - **velocità** per-giunto (max per giunto, oggi tipicamente uniformi in config)
  - **accelerazione** per-giunto (basata su `previous_velocity` e `dt`)
  - implementazione: `include/cartesian_velocity_controller/components/joint_safety_limiter.hpp` + `src/components/joint_safety_limiter.cpp`
  - non esiste ancora un controllo esplicito del **jerk** in giunti.

Obiettivo di questa proposta: aggiungere anche la **limitazione del jerk in giunti**, mantenendo:

- **scaling uniforme** (direzione in spazio giunti preservata: \(\dot q_\text{out} \parallel \dot q_\text{cmd}\))
- **abilitazione/disabilitazione via parametro** (default “off” per non cambiare comportamento)
- **integrazione minima** nel codebase esistente (basso rischio di regressione).

---

### Requisito funzionale (definizione precisa)

Dato un comando in giunti candidato \(\dot q_\text{cmd}\) (dopo IK, e prima dell’invio al controller di giunti), vogliamo produrre:

\[
\dot q_\text{out} = s \, \dot q_\text{cmd}, \qquad s \in [0,1]
\]

Tale che, per ogni giunto \(i\):

- **Velocità**: \(|\dot q_{\text{out},i}| \le \dot q_{\max,i}\)
- **Accelerazione**: \(|\ddot q_{\text{out},i}| \le \ddot q_{\max,i}\)
- **Jerk** (opzionale): \(|\dddot q_{\text{out},i}| \le \dddot q_{\max,i}\)

con \(dt\) noto ad ogni ciclo.

Nota: la scelta “scaling uniforme” implica che **non** si clampa per giunto; si trova un unico \(s\) che renda **tutti** i giunti conformi simultaneamente.

---

### Opzioni architetturali (scelta consigliata)

- **Opzione A (consigliata)**: estendere `JointSafetyLimiter` aggiungendo jerk come *ulteriore vincolo* nel calcolo di \(s\).
  - **Pro**: mantiene l’idea “final safety layer”; minima invasività; output sempre parallelo al comando; semplice da abilitare/disabilitare.
  - **Contro**: è più un *limiter* che un *filtro dinamico con \(\tau\)* (ma vincola jerk comunque, perché jerk dipende da comandi successivi).

- **Opzione B**: introdurre un nuovo componente tipo `JointVelocityFilter` analogo al filtro cartesiano (stato interno \(\dot q,\ddot q,\dddot q\) e “servo” su velocità con \(\tau\)).
  - **Pro**: concettualmente simile al `CartesianVelocityFilter`.
  - **Contro**: tende a rompere lo “scaling uniforme” (o lo rende meno naturale), cambia di più la dinamica e richiede tuning/validazione più ampia.

Questa proposta dettaglia **Opzione A**.

---

### Proposta implementativa (Opzione A): jerk come vincolo su \(s\)

#### Stato necessario

Per limitare il jerk serve conoscere l’accelerazione “precedente”. Propongo di aggiungere stato interno a `JointSafetyLimiter`:

- \(\ddot q_\text{prev}\) (vettore) = accelerazione effettiva al ciclo precedente
- (opzionale) un `min_dt` per evitare instabilità numerica quando \(dt\) è troppo piccolo

Lo stato viene aggiornato **dopo** aver calcolato \(\dot q_\text{out}\). In reset: \(\ddot q_\text{prev}=0\).

#### Definizioni

Dato:

- \(\dot q_\text{cmd}\): comando candidato (input della funzione `limit(...)`)
- \(\dot q_\text{prev}\): comando precedente (già passato oggi a `limit(...)`)
- \(\ddot q_\text{prev}\): stato interno (nuovo)
- \(dt\): tempo del ciclo

Con output:

\[
\dot q_\text{out} = s\dot q_\text{cmd}
\]

Allora:

\[
\ddot q_\text{out} = \frac{\dot q_\text{out}-\dot q_\text{prev}}{dt}
= \frac{s\dot q_\text{cmd}-\dot q_\text{prev}}{dt}
\]

\[
\dddot q_\text{out} = \frac{\ddot q_\text{out}-\ddot q_\text{prev}}{dt}
= \frac{s\dot q_\text{cmd}-\dot q_\text{prev}-\ddot q_\text{prev}dt}{dt^2}
\]

#### Vincoli come intervalli su \(s\)

Per ogni giunto \(i\), ciascun vincolo induce un insieme ammissibile per \(s\).

- **Vincolo velocità**:

\[
|s\dot q_{\text{cmd},i}| \le \dot q_{\max,i}
\Rightarrow
s \le \frac{\dot q_{\max,i}}{|\dot q_{\text{cmd},i}|}
\quad (\text{se }|\dot q_{\text{cmd},i}|>0)
\]

- **Vincolo accelerazione**:

\[
\left|\frac{s\dot q_{\text{cmd},i}-\dot q_{\text{prev},i}}{dt}\right|
\le \ddot q_{\max,i}
\Rightarrow
|s\dot q_{\text{cmd},i}-\dot q_{\text{prev},i}| \le \ddot q_{\max,i}dt
\]

che definisce un intervallo (lineare) su \(s\).

- **Vincolo jerk** (nuovo, opzionale):

\[
\left|\frac{s\dot q_{\text{cmd},i}-\dot q_{\text{prev},i}-\ddot q_{\text{prev},i}dt}{dt^2}\right|
\le \dddot q_{\max,i}
\Rightarrow
|s\dot q_{\text{cmd},i}-A_i| \le \dddot q_{\max,i}dt^2
\]

dove \(A_i = \dot q_{\text{prev},i} + \ddot q_{\text{prev},i}dt\).

Quindi anche qui si ottiene un intervallo su \(s\).

#### Composizione “uniform scaling” (intersezione intervalli)

Algoritmo consigliato:

- inizializza intervallo globale \(S=[0,1]\)
- per ogni giunto, calcola il range ammissibile di \(s\) per:
  - velocità (se abilitata)
  - accelerazione (se abilitata)
  - jerk (se abilitato)
- aggiorna \(S \leftarrow S \cap S_i\)
- scegli \(s\) come **massimo valore ammissibile** (quello più vicino a 1, quindi “minimo intervento”):

\[
s = \max S
\]

Se l’intersezione è vuota (caso raro ma possibile, tipicamente con inversioni di verso aggressive e limiti molto stretti), fallback conservativo:

- **fallback 1 (semplice/safe)**: \(s=0\) (ferma il comando per un ciclo)
- **fallback 2 (più “liscio”)**: ignorare jerk e applicare solo acc+vel (o solo vel) per quel ciclo, loggando diagnostica.

Questa scelta è un punto aperto (vedi sezione dedicata).

---

### Modifiche al codice (lista concreta)

#### 1) `JointSafetyLimiter`: nuovi parametri e stato

File:

- `include/cartesian_velocity_controller/components/joint_safety_limiter.hpp`
- `src/components/joint_safety_limiter.cpp`

Modifiche proposte:

- **Nuovi limiti**:
  - `Eigen::VectorXd max_joint_jerks_;`
  - setter/getter analoghi a vel/acc:
    - `setJointJerkLimits(...)`
    - `setJointJerkLimit(i, ...)`
    - `getJointJerkLimits()`
    - estensione `setUniformLimits(...)` o nuova `setUniformLimits(max_vel, max_acc, max_jerk)`
- **Nuovo enable flag**:
  - `bool jerk_limiting_enabled_{false};`
  - `setJerkLimitingEnabled(bool)` + `isJerkLimitingEnabled()`
- **Nuovo stato**:
  - `Eigen::VectorXd previous_joint_acceleration_;` (inizializzato a zero)
- **Nuovo calcolo scaling**:
  - `computeJerkScalingFactor(commanded_velocity, previous_velocity, dt, limiting_joint)`
  - idealmente calcolato con la logica “intervallo su s” (vedi sopra)
- **Diagnostica**:
  - estendere `SafetyLimiterOutput::LimitType` con `JERK`
  - salvare `last_limit_type_` coerentemente

#### 2) Tipi pipeline: aggiunta di `JERK`

File:

- `include/cartesian_velocity_controller/types/pipeline_types.hpp`

Modifica proposta:

- aggiungere `JERK` a `SafetyLimiterOutput::LimitType`.

#### 3) Parametri ROS (YAML) + dynamic_reconfigure

File:

- `config/controller_params.yaml`
- `cfg/ControllerTuning.cfg`
- `src/cartesian_velocity_controller.cpp` (caricamento parametri)

Parametri proposti (coerenti con quelli esistenti):

- `joint_safety_limiter/jerk_limiting_enabled` (bool, default `false`)
- `joint_safety_limiter/max_joint_jerk` (double, default ragionevole es. `50.0` rad/s³ per UR; da validare)

Aggiornamenti al controller:

- caricare `max_joint_jerk` come scalare e costruire `Eigen::VectorXd::Constant(num_joints, max_joint_jerk)`
- impostare i limiti nel `JointSafetyLimiter`
- impostare `setJerkLimitingEnabled(...)`

Per `dynamic_reconfigure`:

- aggiungere nel gruppo `safety`:
  - `max_joint_jerk` (double)
  - (opzionale ma consigliato) `jerk_limiting_enabled` (bool)

---

### Pseudocodice (ciclo realtime, lato limiter)

```cpp
// inputs: qdot_cmd, qdot_prev, dt
// state (new): qddot_prev
// output: qdot_out = s * qdot_cmd

S = [0, 1]

for each joint i:
  if vel_enabled:
    intersect S with: |s*qdot_cmd[i]| <= vmax[i]
  if acc_enabled:
    intersect S with: |s*qdot_cmd[i] - qdot_prev[i]| <= amax[i]*dt
  if jerk_enabled:
    A = qdot_prev[i] + qddot_prev[i]*dt
    intersect S with: |s*qdot_cmd[i] - A| <= jmax[i]*dt*dt

if S empty:
  s = 0.0 (or fallback policy)
else:
  s = max(S)

qdot_out = s * qdot_cmd
qddot_out = (qdot_out - qdot_prev) / dt
qddot_prev = qddot_out
```

---

### Criticità / punti aperti (da decidere insieme prima di codare)

- **(1) “Filtro” vs “limiter”**  
  Questa proposta aggiunge jerk come vincolo sul comando (limiter). Se invece vuoi un filtro dinamico “alla `CartesianVelocityFilter`” anche in giunti (con \(\tau\) e integrazione interna), conviene discutere l’Opzione B.

- **(2) Gestione \(dt\) variabile**  
  Nel controller `dt` è clampato (tipicamente 0.001…0.1). Il jerk usa \(dt^2\): piccole variazioni possono rendere il vincolo molto “nervoso”. Opzioni:
  - clamp già esistente può bastare
  - usare un `min_dt` dedicato nel limiter (es. 1e-3)
  - (più complesso) usare una stima \(dt_\text{eff}\) filtrata per la parte jerk.

- **(3) Definizione di \(\ddot q_\text{prev}\)**  
  Va aggiornato usando le **velocità realmente inviate** (post-scaling), non quelle pre-limiter. Nel codebase attuale `previous_joint_velocity_` viene mantenuto lato controller; serve coerenza tra “prev_velocity” passato e lo stato interno \(\ddot q_\text{prev}\).

- **(4) Caso “inversione di segno” / intersezione vuota**  
  Con limiti stretti e comandi che invertono verso, l’intervallo su \(s\) può diventare vuoto se imponiamo \(s\in[0,1]\). Serve scegliere una policy:
  - fermare (s=0)
  - degradare (ignorare jerk per 1 ciclo, o ignorare acc+jerk)
  - ammettere \(s<0\) (sconsigliato perché cambia direzione rispetto a \(\dot q_\text{cmd}\))

- **(5) Retrofit dell’attuale limitazione accelerazione**  
  Oggi `computeAccelerationScalingFactor()` usa una logica “approssimata” con casi speciali. Se adottiamo l’approccio “intervalli su s”, varrebbe la pena **uniformare** anche l’accelerazione a quella logica (più chiara e spesso più robusta). Questo però è un cambiamento potenzialmente behavior-changing: da valutare se farlo subito o in step separato.

- **(6) Parametri: uniformi vs per-giunto**  
  La config attuale usa scalari uniformi (`max_joint_velocity`, `max_joint_acceleration`). Per jerk propongo lo stesso (scalare uniforme) per coerenza. Se vuoi, possiamo prevedere in futuro un array `max_joint_jerk: [..]` mantenendo retrocompatibilità (se array presente, sovrascrive lo scalare).

- **(7) Diagnostica / rosbag**  
  Utile esporre:
  - `scaling_factor`
  - `limit_type` (incluso JERK)
  - `limiting_joint`
  - (opzionale) `qddot_out` e/o `qdddot_out` per debug  
  Questo richiede decidere se estendere `PipelineDebug.msg` o pubblicare su topic separato.

---

### Proposta di rollout (incrementale, basso rischio)

- **Step 1 (no behavior change)**: aggiungere parametri e struttura dati (limiti jerk + enable) ma con `jerk_limiting_enabled=false` di default.
- **Step 2 (behavior change controllato)**: implementare jerk limiting con policy “fallback s=0” e log throttled.
- **Step 3**: aggiungere diagnostica e tuning dei default (UR10e).
- **Step 4 (opzionale)**: refactor della sola parte accelerazione su “intervalli su s” se vogliamo maggiore coerenza e meno corner-case.


