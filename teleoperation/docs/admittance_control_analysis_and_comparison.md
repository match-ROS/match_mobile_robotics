# Analisi approfondita del controllo in ammettenza (Python) e confronto con `teleop_master_haptic_controller_node.cpp` (C++)

Questo documento analizza in dettaglio l’implementazione del controllo in ammettenza in:

- **Tua implementazione (C++)**: `src/match_mobile_robotics/teleoperation/src/teleop_master_haptic_controller_node.cpp`
- **Seconda implementazione (Python)**: `src/match_mobile_robotics/mur/mur_control/scripts/admittance_control.py`

e mette in evidenza **differenze strutturali**, **scelte di modellazione**, **problemi numerici** e **punti che impattano la fluidità**.

---

## 1) Cosa intendo qui per “ammettenza”

In robotica, un controllo in ammettenza tipicamente realizza una dinamica del tipo:

$$
M \dot{v} + D v + K x = F_\text{ext}
$$

con:
- $F_\text{ext}$: forza/coppia misurata (wrench)
- $v$: velocità cartesiana (twist)
- $x$: posizione/orientamento (opzionale in ammettenza “mass–damper–spring”)
- $M, D, K$: parametri “virtuali”

Esistono due famiglie molto comuni:
- **Ammettenza in velocità** (mass–damper): $M\dot{v}+Dv = F$ e poi si integra $v$ in un generatore di pose (o si invia direttamente una velocità).
- **Ammettenza in posizione/compliance**: $x = A \, F$ (compliance statica), spesso implementata come offset di posa + anello P sulla posa.

Queste due famiglie **si comportano in modo molto diverso** (smorzamento, risposta alle alte frequenze, stabilità/discretizzazione).

---

## 2) La tua implementazione C++: struttura reale del controllo

### 2.1 Pipeline del segnale (cosa entra e cosa esce)

- **Ingresso**: `geometry_msgs/WrenchStamped` dal master (es. loadcell al TCP) e opzionalmente altri wrench:
  - master: `master_wrench_topic`
  - slave: `slave_wrench_topic` (force reflection)
  - coupling: `coupling_wrench_topic` (accoppiamento bimanuale / vincoli virtuali)
- **Uscita**: `geometry_msgs/Twist` su `command_topic`

Quindi il nodo C++ è un **generatore di velocità cartesiane** guidato da wrench.

### 2.2 Gestione frame e TF (elemento spesso trascurato)

Prima di usare le forze, il nodo **ruota** forza e coppia dal frame sorgente al frame target:

- sorgente: `msg.header.frame_id` oppure `wrench_source_frame_override`
- target: `wrench_target_frame`
- timestamp TF: `ros::Time(0)` se `use_latest_tf_for_wrench = true`, altrimenti `msg.header.stamp`

Questa parte è critica perché:
- **forze/orientamenti misurati in frame diversi** portano a comandi incoerenti (sensazione “non fluida”/“strana”).
- l’opzione “latest TF” riduce errori se lo stamp non è coerente, ma può introdurre piccoli mismatch temporali (comunque spesso meglio che TF fail).

### 2.3 Robustezza temporale

Nel `tick()`:
- controlla che i wrench non siano **stale** (timeout `wrench_timeout_s`)
- calcola $dt$ da `last_time_`
- se $dt$ non è valido (troppo piccolo/non finito) pubblica zero
- in caso di stale può **resettare lo stato** (velocità e filtri) se `reset_on_stale=true`

Questo evita transitori “impazziti” quando mancano messaggi o quando $dt$ si corrompe.

### 2.4 Filtro + deadband + saturazione sul wrench

Il wrench viene preprocessato da `filterClampDeadbandWrench()`:

- **EMA** (exponential moving average) per ogni asse:
  - `ema3(prev, curr, alpha)` con $\alpha \in [0,1]$
- **deadband assoluta** per asse (`applyDeadbandAbs3`)
- **clamp sulla norma** (`clampNorm3`) per limitare la magnitudine complessiva
- opzionale: **torque disabilitato** mettendo tau a zero

Questo significa che prima della dinamica di ammettenza stai facendo:
- **attenuazione rumore alta frequenza**
- **rimozione micro-forze** (deadband) che altrimenti generano micro-comandi → “grattare”/non fluidità
- **limite duro** su forze/coppie

Nota: nel tuo YAML attuale `wrench_filter_alpha: 0.5` (EMA abbastanza aggressiva verso il valore corrente).

### 2.5 Dinamica di ammettenza implementata (mass–damper in velocità)

La parte chiave è:

- forza mano: $F_\text{hand} = F_\text{master,filt}$
- feedback: $F_\text{feedback} = k_f \, F_\text{slave,filt} + F_\text{coupling,filt}$

e la dinamica:

$$
a = \frac{F_\text{hand} - F_\text{feedback} - d\, v}{m}
\qquad
v \leftarrow v + a\, dt
$$

con:
- $m = \text{mass\_linear}$
- $d = \text{damping\_linear}$

e poi:
- clamp sulla norma della velocità (`max_linear_speed`)

Per l’angolare (se `use_torques=true`) è analogo su $\tau$ e $v_\omega$ con `mass_angular`, `damping_angular`, `max_angular_speed`.

Questa è una **ammettenza dinamica** con:
- **smorzamento esplicito** su $v$ (termine $-d v$)
- integrazione discreta esplicita (Euler forward)
- saturazione su $v$

È una struttura classica per “fluidità” perché filtra naturalmente alte frequenze (insieme ai filtri sul wrench) e limita i transitori via $m,d$.

---

## 3) Implementazione Python `admittance_control.py`: com’è costruita davvero

### 3.1 Architettura generale

Il nodo Python, a livello di pipeline, fa:

1. legge:
   - wrench attuale (sensore): `actual_wrench_topic`
   - wrench target: `target_wrench_topic`
   - pose target: `target_pose_topic`
   - pose attuale: `actual_pose_topic`
   - twist feed-forward cartesiano: `cartesian_ff_velocity_topic`
   - joint states: `joint_states_topic`
2. calcola un **offset di compliance** in 6D (3 forze + 3 coppie) in funzione dell’errore di wrench
3. aggiorna la pose target integrando la feed-forward twist
4. calcola errore di posa (pose target − pose attuale) **più** l’offset di compliance
5. genera una twist target: $v = K_p \cdot e_\text{pose} + v_\text{ff}$
6. converte twist → joint velocities con Jacobiano e invio a `joint_group_vel_controller`

Quindi lo script Python non realizza una dinamica $M\dot{v}+Dv=F$: realizza piuttosto una catena:

- **wrench error → “compliance” statica (offset di posa)**
- **offset di posa + tracking P → velocità**
- **velocità cartesiana → velocità giunti**

### 3.2 “Admittance” nel Python: è una compliance statica (non mass–damper)

La funzione `desired_compliance()` calcola:

```text
desired_compliance[i] = (target_wrench[i] - actual_wrench[i]) * admittance_gain_i
```

Questa è una **mappa algebrica** $x_\text{comp} = A \, (F_\text{ref}-F)$.

Da sola, questa parte:
- non introduce uno stato dinamico smorzato (come $v$ nel C++)
- non definisce una frequenza naturale/rapporto di smorzamento esplicito
- è molto sensibile a rumore e offset se non filtrata

### 3.3 Aggiornamento target pose con feed-forward

`update_target_pose()` integra:
- posizione: $p \leftarrow p + v_\text{ff} \, dt$
- orientamento: converte quaternion → euler (roll,pitch,yaw), integra $\omega_\text{ff}\,dt$, poi euler → quaternion

Criticità tipiche:
- integrazione in euler introduce **singolarità/gimbal lock** e discontinuità (wrap di yaw ecc.)
- non c’è normalizzazione esplicita del quaternion (dipende dal tool `quaternion_from_euler`, di solito ok)

### 3.4 Errore di posa: bug logico (la compliance viene annullata)

Dentro `pose_error()` c’è:

```python
def pose_error(self, actual_pose, target_pose, desired_compliance):
    desired_compliance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pose_error = ...
    pose_error[i] = desired_compliance[i] + (target - actual)
```

Questa riga **resetta `desired_compliance` a zero**, quindi di fatto:
- l’“ammettenza” (compliance da wrench) **non entra mai** nel controllo di posa
- il controllo diventa un puro tracking della pose target (feed-forward integrata + P)

Se questa è davvero la versione che stai usando, il comportamento non è “admittance”: è più simile a un **servo di posa** con feed-forward.

### 3.5 Generazione velocità: puro proporzionale su errore di posa

`calculate_target_tcp_velocity()` fa:

```text
v = Kp * pose_error + v_ff
```

Non ci sono:
- termini derivativi (oltre al feed-forward)
- filtri
- saturazioni su $v$
- anti-windup (non c’è integratore)

Questa architettura tende a risultare meno “morbida” quando:
- la pose target cambia rapidamente
- la stima della pose attuale è rumorosa
- le conversioni euler creano scatti

### 3.6 Conversione twist → joint velocity: inversione Jacobiano fragile

`tcp_velocity_to_joint_velocity()`:

- usa `jacobian = move_group.get_jacobian_matrix(self.q)`
- poi fa `jacobian_pinv = np.linalg.inv(jacobian)` (non è una pseudo-inversa; è l’inversa vera)

Problemi pratici:
- vicino a singolarità l’inversa esplode → comandi joint velocity molto grandi → non fluidità
- anche lontano da singolarità, la matrice può essere mal condizionata → jitter
- tipicamente si usa **damped least squares**:
  $$
  J^\# = J^T (J J^T + \lambda^2 I)^{-1}
  $$
  o una `pinv` con soglia sui singolari

Inoltre:
- viene calcolato un “sorting” dell’ordine dei giunti, ma la funzione **ritorna `target_joint_velocity_unsorted`**:
  - quindi lo “sorting” preparato non viene usato (altro bug logico).

### 3.7 Frame: conversione base→tcp calcolata ma poi ignorata

Dentro `tcp_velocity_to_joint_velocity()`:

```python
target_tcp_velocity = self.base_to_tcp_velocity(target_tcp_velocity_base)
target_tcp_velocity = target_tcp_velocity_base
```

La seconda riga **sovrascrive** la conversione. Quindi la trasformazione base→tcp non viene usata.

Questo è importante perché:
- se il Jacobiano di MoveIt è espresso in una convenzione/frame specifico, usare la twist nel frame sbagliato produce comportamenti “strani” (anche se spesso MoveIt usa il base frame; dipende dalla funzione e dal setup).

### 3.8 Switching controller dentro `update()`

All’inizio di `update()` lo script fa sempre switch:
- stop: `arm_controller`
- start: `joint_group_vel_controller_l/unsafe`

Farlo dentro lo stesso nodo che controlla può introdurre:
- ritardi e transitori di switching
- difficoltà nel riprodurre comportamento stabile

---

## 4) Confronto diretto: differenze chiave con impatto su “taratura” e “fluidità”

### 4.1 “Vera” ammettenza dinamica vs compliance statica + P

**C++ (tuo)**:
- dinamica: $M\dot{v} + D v = F$ (più feedback)
- stato interno: $v$ (integrato)
- smorzamento esplicito con $D$

**Python**:
- (intenzione) compliance statica: $x = A(F_\text{ref}-F)$, poi $v = K_p e_x$
- nella versione attuale, per bug: **la compliance è zero**, quindi $v$ non dipende dal wrench

Implicazione:
- il C++ è naturalmente “morbido” e filtrante (se $D$ e i filtri sono ben scelti)
- il Python, anche se corretto, può risultare più “nervoso” perché l’anello P su posa non smorza da solo le componenti ad alta frequenza come una dinamica mass–damper ben tarata

### 4.2 Trattamento del rumore e micro-forze

**C++**:
- EMA sul wrench
- deadband per asse
- clamp sulla norma del wrench
- clamp sulla norma della velocità

**Python**:
- nessun filtro su wrench/pose/twist
- nessuna deadband
- nessuna saturazione esplicita su twist o joint velocity

Implicazione:
- il Python è molto più esposto a jitter del sensore e a piccole oscillazioni
- nel C++ puoi ottenere molta “fluidità percepita” già solo con alpha+deadband corretti

### 4.3 Robustezza temporale (dt, timeouts)

**C++**:
- dt validato
- timeout wrench + reset state

**Python**:
- dt calcolato solo in `update_target_pose()`; non validato
- nessun timeout su sensori/pose

Implicazione:
- se un topic rallenta o si blocca, il C++ va a zero in modo safe; il Python può continuare con stati “vecchi”.

### 4.4 Frame/TF

**C++**:
- ruota forza/coppia in un frame target definito (TF)

**Python**:
- non gestisce TF per wrench
- calcola una trasformazione base→tcp per twist ma poi la annulla

Implicazione:
- mismatch di frame è una causa frequente di comandi non intuitivi e “non fluidi”.

### 4.5 Stabilità numerica nell’inversione del Jacobiano

**C++**:
- non fa nessuna inversione di Jacobiano (pubblica twist, demandando a un controller a valle)

**Python**:
- inverte $J$ direttamente con `np.linalg.inv`
- nessuna regolarizzazione, nessun controllo condizionamento, nessun limite su $\dot{q}$

Implicazione:
- il Python può diventare instabile vicino a singolarità e produrre scatti anche se la twist è liscia.

### 4.6 Orientamento: quaternion vs euler

**C++**:
- nel tuo nodo specifico l’orientamento non viene integrato come pose; lavora direttamente su torque→vel angolare (opzionale) con modello mass–damper.

**Python**:
- integra orientamento in euler (roll/pitch/yaw), più soggetto a wrap/discontinuità
- errore orientamento calcolato come differenza di euler, che non è una metrica robusta globalmente

Implicazione:
- scatti e non linearità sull’orientamento sono più probabili nel Python.

---

## 5) In cosa il Python “somiglia” e in cosa “non somiglia” al tuo C++

### 5.1 Somiglianze

- entrambi generano **velocità** finali (twist nel C++, joint velocities nel Python)
- entrambi hanno un concetto di “guida” attraverso segnali esterni (wrench/pose/ff)

### 5.2 Differenze fondamentali

- il tuo C++ è un **ammettenza mass–damper in velocità** con filtri e saturazioni ben definite
- il Python è (o vorrebbe essere) un **compliance+servo di posa** e, così com’è scritto, contiene bug che di fatto eliminano la parte di ammettenza
- il Python include una catena twist→joint (con inversione Jacobiano) che introduce complessità e instabilità potenziale che nel C++ non esistono

---

## 6) Checklist rapida di “red flags” nel Python (probabili cause di non-fluidità / non funzionamento)

- **Compliance annullata** in `pose_error()` (bug).
- **Conversione base→tcp annullata** in `tcp_velocity_to_joint_velocity()` (bug).
- **Inversa Jacobiano** con `inv()` invece di pseudo-inversa / damped least squares.
- **“Sorting” joint velocity calcolato ma ignorato** (ritorno dell’array sbagliato).
- **Nessun filtro/deadband/saturazione**.
- **Orientamento in euler** (wrap e discontinuità).
- **Switch controller** eseguito nello stesso nodo di controllo (transitori).

---

## 7) Nota su “taratura” nel tuo C++ (perché può risultare poco fluido se non tarato)

Anche se strutturalmente il tuo C++ è adatto alla fluidità, la “sensazione” dipende molto da:

- **`wrench_filter_alpha`**:
  - alto → segue molto il rumore (meno smooth)
  - basso → più filtrato ma più ritardato
- **deadband (`force_deadband`)**:
  - troppo bassa → micro-comandi continui
  - troppo alta → zona morta fastidiosa
- **rapporto $d/m$**:
  - a parità di dt, un $d$ basso dà risposta più oscillante; $d$ alto più “viscosa”
- **clamp su velocità**:
  - se troppo basso può dare sensazione “impastata”
  - se troppo alto può rendere pericolosi i transitori se il wrench è rumoroso
- **frame wrench target** (`wrench_target_frame`): scegliere un frame inerziale coerente ai comandi aiuta molto la prevedibilità.

---

## 8) Conclusione (differenze in una frase)

- Il tuo nodo C++ implementa una **ammettenza dinamica mass–damper in velocità con filtro/deadband/clamp e gestione TF**, tipicamente più adatta a un comportamento “fluido”.
- Lo script Python è (per struttura) una **compliance statica + servo di posa + inversione Jacobiano**, e nella versione attuale contiene bug che **rimuovono di fatto la parte di ammettenza** e introducono ulteriori fonti di instabilità/jitter.

