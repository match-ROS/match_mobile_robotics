# Piano d’azione (dettagliato) — Pipeline pose-tracking robusta con Virtual Target Leash “non bloccante”

> Nota: questo piano è costruito **solo su ciò che oggi fa il codice** (non sui file docs), perché i docs possono essere non aggiornati.

## Obiettivo
Eliminare i due problemi segnalati:

1. **Blocco tangenziale** quando `target_raw` è alla distanza massima dal robot (leash scala a zero tutta la velocità).
2. **Robot che non recupera radialmente** quando è indietro rispetto al target (leash riduce anche il feed-forward e di fatto “strozza” la catena che muove il robot).

Garantendo inoltre che:
- quando le repulsive sono nulle, il robot **converga** alla posa definita dal Global Planner.

---

## Stato attuale (da codice)

### Dove nasce il problema
- `LocalPlanner::compute()` calcola `v_combined` (attrattiva + repulsiva) e applica il **Virtual Target Leash** scalando **tutto** `v_combined` con `scaling_factor`. Questo:
  - ferma anche la tangenziale,
  - e riduce la velocità che passa a valle.
- `CartesianVelocityController::controlLoopCallback()` fa:
  - `desired_twist := local_output.combined_*`
  - `filtered_twist := velocity_filter_->filter(desired_twist, dt)` (Level C)
  - `feedforward := filtered_twist`
  - `PID.compute(error(target_filtered - current), feedforward)`
  - dove `target_filtered := velocity_filter_->getFilteredPosition()`.

### Punti di aggancio principali
- Local planner (Level B): `cartesian_velocity_controller/src/components/local_planner.cpp`
- Controller loop: `cartesian_velocity_controller/src/cartesian_velocity_controller.cpp`
- Filtro (Level C): `cartesian_velocity_controller/src/velocity_filter.cpp` + header `include/.../velocity_filter.hpp`

---

## Architettura proposta (nuova)
Separiamo chiaramente:

1) **Generazione del setpoint** (`target_raw`) con vincoli (leash come vincolo, non come “kill switch” della velocità).  
2) **Motion generation**: un filtro jerk/acc che produce `filtered_twist` e aggiorna `target_filtered` inseguendo `target_raw` (pose-tracking).  
3) **Servo** (PID + IK) invariato: insegue `target_filtered` e usa `filtered_twist` come feed-forward.

La chiave: **il leash non deve strozzare il feed-forward**; deve vincolare la geometria del setpoint o, al massimo, solo la componente radiale uscente.

---

## Piano di implementazione — step by step

### Step 0 — “Safety net”: misure e segnali che useremo per validare
Non cambiamo codice, ma definiamo cosa guardare nei log/debug:

- `virtual_target_scaling_factor` (già pubblicato nel debug pipeline)
- `distance_target_raw_to_current` (già in debug)
- `pid_feedforward_linear`, `pid_output_linear`
- `safety_scaling_factor` (JointSafetyLimiter): serve per distinguere “strozzatura leash” vs “limite giunti”.

**Criteri di successo**:
- Caso tangenziale al limite: il robot si muove tangenzialmente, `virtual_target_scaling_factor` non forza `feedforward` a zero.
- Caso recupero radiale: il robot recupera distanza con dinamica governata da limiti di accel/jerk e (se presente) safety limiter.
- A repulsione nulla: `target_raw → waypoint`, `target_filtered → target_raw`, errore PID → 0.

---

### Step 1 — Convergenza strutturale: cambiare l’attrattiva perché guidi `target_raw → waypoint`
**Perché**: oggi l’attrattiva è calcolata come errore *robot→waypoint*. Questo rende `target_raw` un integratore “guidato indirettamente” e apre la porta a overshoot e leash aggressivi.

**Modifica proposta** (minima, ma concettualmente importante):
- in `LocalPlanner::compute()` calcolare la velocità attrattiva lineare/angolare usando come “current” **`target_raw_`** invece di `current_pose`.

In pratica:
- `v_attractive_linear := waypoint.translation() - target_raw_.translation()`
- `v_attractive_angular := orientation_error(target_raw_.rot, waypoint.rot)`
- poi `* k_att` come oggi.

**File**:
- `cartesian_velocity_controller/src/components/local_planner.cpp`

**Punti delicati**:
- thread-safety: `target_raw_` è protetto da `state_mutex_`. Va letto in modo coerente.
- inizializzazione: quando `!has_target_` già viene inizializzato a `current_pose` (ok).

**Output atteso**:
Quando repulsione = 0, `target_raw` converge al waypoint (dinamica di primo ordine con guadagno `k_att` + saturazione `max_linear_velocity`).

---

### Step 2 — Leash “non bloccante”: trasformare il leash da scaling isotropo a vincolo geometrico
Qui scegliamo una delle due implementazioni (A o B). Entrambe eliminano il blocco tangenziale; B è più “anti-windup” sullo stato, A è più dolce e continua.

#### Opzione 2A (consigliata come prima implementazione): leash anisotropo sulla componente radiale uscente
**Idea**: lascia intatta la tangenziale, limita solo la radiale uscente quando sei oltre `leash_start_distance`.

Schema:
- `r = target_raw - current_pose`
- `n = r / |r|`
- decomponi `v_combined_linear`:
  - `v_out = max(0, v·n) * n` (solo parte uscente)
  - `v_rest = v - v_out` (tangenziale + radiale entrante)
- applica scaling solo su `v_out`:
  - `v' = v_rest + scaling_factor * v_out`

Nota: `scaling_factor` può rimanere la funzione attuale (lineare tra start e stop), oppure essere cambiata (sigmoide).

#### Opzione 2B: clamp sullo stato dopo integrazione
**Idea**: integra con la velocità, poi proietta `target_raw` sul bordo se oltre `leash_stop_distance`.

Schema:
- `target_raw += v*dt`
- se `|target_raw - robot| > R`: proietta `target_raw` su sfera di raggio `R`.

**Tradeoff**:
- A è più smooth (continua sulle velocità), B può introdurre piccoli “salti” nello stato.

**File**:
- `cartesian_velocity_controller/src/components/local_planner.cpp`

**Nota importante**:
Nel nuovo assetto (Step 3) **`v_combined` non deve più essere l’unico driver del feed-forward**; quindi anche se il leash agisce su `target_raw`, non deve “strozzare” la pipeline a valle.

---

### Step 3 — Motion generation pose-tracking: far inseguire al filtro `target_raw` (non integrare una velocità leashed)
Questo è il cuore della robustezza.

#### Scelta di implementazione (minima invasività)
Non è necessario stravolgere `CartesianVelocityFilter`. Possiamo:

1. calcolare un `desired_twist` **da errore pose** tra `target_raw` e `target_filtered`,
2. passarlo a `velocity_filter_->filter(desired_twist, dt)` (che già applica jerk/acc/vel limits e integra `filtered_position_`),
3. usare `filtered_twist` come feed-forward e `target_filtered` come setpoint per il PID.

Quindi: il filtro rimane “velocity-based”, ma noi generiamo la velocità desiderata come *funzione del pose error*.

#### Dove implementarlo
In `CartesianVelocityController::controlLoopCallback()`:

- dopo `local_output = local_planner_->compute(...)` (per ottenere `target_raw`)
- prima di chiamare `velocity_filter_->filter(...)`

**Nuova logica**:
1. leggere la posa `target_raw` dal `LocalPlannerOutput`.
2. leggere la posa `target_filtered_prev = velocity_filter_->getFilteredPosition()` (setpoint filtrato del ciclo precedente).
3. calcolare un errore di posa \(e\) tra `target_raw` e `target_filtered_prev`:
   - `pos_err = target_raw.translation() - target_filtered_prev.translation()`
   - `ori_err` in asse-angolo come già fatto per PID (quaternion diff + AngleAxis).
4. costruire un `desired_twist_pose_track`:
   - `v_des = pos_err / tau_pose_track` (con saturazione su `max_linear_velocity`)
   - `w_des = ori_err / tau_pose_track_ori` (saturazione su `max_angular_velocity`)
   - (opzionale) guadagni dedicati `k_pose_track_lin`, `k_pose_track_ang`
5. passare quel twist al filtro:
   - `filtered_twist = velocity_filter_->filter(desired_twist_pose_track, dt)`
6. usare:
   - `target_filtered = velocity_filter_->getFilteredPosition()` (aggiornato dal filtro)
   - `feedforward = filtered_twist`
   - e poi fare PID come oggi.

#### Parametri nuovi (consigliati)
Servono parametri separati per il “pose tracking” del filtro, per non dipendere dal tuning del PID:

- `pose_tracking/tau_linear` (s)
- `pose_tracking/tau_angular` (s)
- opzionali `pose_tracking/k_lin`, `pose_tracking/k_ang`
- opzionale `pose_tracking/max_linear_velocity` e `pose_tracking/max_angular_velocity` (se vuoi cappare diversamente dal local planner)

**Nota**: anche senza nuovi parametri, puoi iniziare con:
- `tau_linear := velocity_filter_->getTimeConstant()`
- `tau_angular := velocity_filter_->getTimeConstant()`
e cappare con i limiti già esistenti del local planner.

#### Perché questa scelta garantisce convergenza (quando repulsione = 0)
Se in Step 1 rendi `target_raw → waypoint`, allora:

- pose-tracking: `target_filtered → target_raw` (per costruzione: il filtro genera una velocità proporzionale all’errore di posa)
- PID: `robot → target_filtered`

Quindi la catena converge al waypoint senza richiedere che il feed-forward “rimanga alto” quando il leash è attivo.

---

### Step 4 — Decide cosa fare del vecchio `v_combined` a valle (deprecazione controllata)
Dopo Step 3, `local_output.combined_*` **non dovrebbe più essere il driver** del filtro (né del feed-forward).

Scelte:
- **4A (pulita)**: `local_output.combined_*` resta solo diagnostica/visualizzazione per capire cosa avrebbe voluto fare il planner.
- **4B (ibrida)**: sommi una parte di `combined_*` al twist di pose-tracking (sconsigliata all’inizio: rischia di reintrodurre “strozzature” e incoerenze).

Raccomandazione: partire con **4A**.

---

### Step 5 — Aggiornare dynamic reconfigure / YAML (solo dopo che la logica è stabile)
Una volta validato il comportamento, rendiamo tunabili i parametri aggiunti.

**File tipici**:
- `cartesian_velocity_controller/config/controller_params.yaml`
- `cartesian_velocity_controller/cfg/ControllerTuning.cfg`
- `cartesian_velocity_controller/src/cartesian_velocity_controller.cpp` (load params + dynamic reconfigure callback)

---

### Step 6 — Test plan (ripetibili) per validare
1. **Convergenza senza repulsione**:
   - `repulsive_enabled = false`
   - comando un waypoint statico
   - atteso: `target_raw`, `target_filtered` e robot convergono al waypoint.
2. **Tangenziale al limite**:
   - porta `dist(target_raw, robot)` vicino a `leash_stop_distance`
   - applica una repulsiva tangenziale
   - atteso: robot si muove tangenzialmente (non più bloccato).
3. **Recupero radiale**:
   - forza target_raw a stare avanti (o crea disturbo che rallenta robot)
   - atteso: robot recupera fino al bordo imposto dal leash, senza collassare feed-forward.
4. **Controllo limiter**:
   - monitorare `safety_scaling_factor`: se < 1 spesso, il vero collo di bottiglia è joint safety limiter e serve ritarare limiti/accelerazioni.

---

## Domande aperte (rispondi qui sotto punto-per-punto)

### Q1 — Repulsione calcolata su quale punto?
Oggi la repulsione ostacoli usa `current_pose.translation()` come punto di riferimento.
Con la nuova architettura, preferisci che la repulsione (TCP/payload) sia calcolata:
- (A) sul robot (`current_pose`) — più “fisico”
- (B) sul target virtuale (`target_raw`) — più “planner-like”
- (C) ibrido (A per sicurezza immediata, B per guida del target)

**Risposta Q1**: A (consigliata per la prima implementazione, perché la repulsione deve riflettere la situazione “fisica” del robot; in seguito si può valutare una C aggiungendo una piccola componente “preview” sul target se serve anticipare ostacoli)

### Q2 — Leash: opzione 2A o 2B?
Preferisci:
- (A) anisotropo su velocità (smooth, continuo)
- (B) clamp sullo stato (anti-windup diretto, ma con proiezione)

**Risposta Q2**: A

### Q3 — Orientamento: leash anche su angolare o no?
Attualmente il leash scala anche `v_combined_angular`.
Nel nuovo schema vuoi:
- (A) leash solo traslazionale
- (B) leash anche angolare (con parametri separati)

**Risposta Q3**: A

### Q4 — Banda di lavoro del leash
Confermi che `leash_start_distance` / `leash_stop_distance` / `leash_reset_threshold` restano concetti validi?
Se sì, quali valori target vuoi come default?

**Risposta Q4**: I concetti restano validi, puoi lasciare i valori di default come sono

### Q5 — Parametri pose-tracking
Vuoi introdurre parametri dedicati (`pose_tracking/tau_*`, `pose_tracking/k_*`) o preferisci riusare `cartesian_filter_tau` e i limiti del local planner?

**Risposta Q5**: Si, introduci i parametri dedicati


