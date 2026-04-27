# Piano d'azione — Smoothing velocità repulsiva (implementazione)

Questo documento descrive un piano operativo per implementare:

- ✅ **3.1** filtro a valle sulla velocità repulsiva (EMA)
- ❌ **3.2** interpolazione temporale tra buffer mappa (rimandata)
- ✅ **3.3** rate-limiter sulla variazione di velocità (jerk/acc limit sul comando repulsivo)
- ✅ **3.4** profili **Smoothstep** e **Smootherstep** (entrambi disponibili, selezionabili a runtime)
- ⏸️ **3.5** mantenere `map3d/update_rate_hz = 15` (nessuna modifica)
- ✅ **3.6** filtro EMA sul gradiente (per POI / query)
- ✅ **3.7** predictive query **solo sui POI**, con approccio robusto/conservativo
- ✅ **extra** criterio **asimmetrico** (risposta rapida in “avvicinamento” e rilascio più lento)

> Nota: i riferimenti numerici (3.1, 3.3, …) corrispondono a `docs/Smoothing_Repulsive_Velocity.md`.

---

## Obiettivo e vincoli

- **Obiettivo**: ridurre jerk/jitter dovuti a swap mappa e rumore su gradiente/distanza, mantenendo reattività in caso di ostacolo vicino.
- **Vincolo**: evitare 3.2 per ora (complessità buffering/coerenza e ulteriore latenza).
- **Principio guida**: smoothing **a valle** + limiti di variazione + profilo C², con scelta di parametri conservativa e disattivabile.

---

## Strategia complessiva (ordine consigliato)

1. Implementare in `LocalPlanner`:
   - **3.4** profili Smoothstep/Smootherstep (puro calcolo scalare)
   - **3.1** filtro EMA su `v_rep` (e opzionalmente su `v_rep_links`)
   - **3.3** rate limiter (jerk/Δv max per ciclo) sul comando repulsivo
   - **extra** criterio **asimmetrico** integrato nel filtro (rise/fall tau) o nel limiter
2. Implementare in `RepulsionDataManager`:
   - **3.6** filtro EMA sul gradiente per POI (stabilizza direzione)
   - **3.7** predictive query sui POI, con clamp e conservatività
3. Aggiornare `config/controller_params.yaml` con i nuovi parametri (default sensati, feature disabilitabili).
4. Aggiungere debug/telemetria minima per verificare:
   - salti su `distance/gradient` e risposta del filtro/limer
   - differenza `v_rep_raw` vs `v_rep_out`
5. Test (anche manuale) su scenari: ostacolo fisso, robot in movimento, vicino boundary influence.

---

## Modifiche previste per componente

### A) `LocalPlanner` — 3.1, 3.3, 3.4, asimmetria

**File tipici** (da verificare nel repo):
- `cartesian_velocity_controller/src/components/local_planner.cpp`
- `cartesian_velocity_controller/include/cartesian_velocity_controller/components/local_planner.hpp`

#### A.1 — 3.4: profili Smoothstep e Smootherstep (entrambi)

1. Introdurre un enum/modalità selezionabile da param:
   - `LINEAR`, `QUADRATIC` (esistente), `SMOOTHSTEP`, `SMOOTHERSTEP`
2. Implementare due funzioni pure (p.es. in un helper interno o in `LocalPlanner`):
   - `double smoothstep(double r)` con \(3r^2 - 2r^3\) (C¹)
   - `double smootherstep(double r)` con \(6r^5 - 15r^4 + 10r^3\) (C²)
3. Applicazione: calcolare \(r\) come nel doc e poi \(v_{rep} = v_{max} * f(r)\).

**Criteri di accettazione**
- Per `r <= 0` ⇒ output 0
- Per `r >= 1` ⇒ output 1
- Continuità ai bordi: derivata ~0 a 0 e 1 per `SMOOTHERSTEP`.

#### A.2 — 3.1: filtro EMA su `v_rep` (con opzione asimmetrica)

Implementare uno stato filtro (persistente) e aggiornamento per ciclo:
- Stato: `Eigen::Vector3d v_rep_filtered_`
- Parametri:
  - `repulsive_filter_tau_rise` (piccolo, risposta rapida)
  - `repulsive_filter_tau_fall` (più grande, rilascio lento)
  - `repulsive_filter_enable` (o `tau_* = 0` per disabilitare)

**Criterio asimmetrico (consigliato)**
- Decidere rise/fall confrontando l’“intensità”:
  - `if (v_raw.norm() > v_filtered_.norm()) tau = tau_rise else tau = tau_fall`
- Calcolare \(\alpha = \frac{dt}{\tau + dt}\) (clamp in [0,1]) e aggiornare:
  - `v_filtered = alpha * v_raw + (1-alpha) * v_filtered`

**Criteri di accettazione**
- Se `tau_rise = tau_fall = 0` ⇒ output = raw (nessun filtro)
- Se `tau_rise < tau_fall` ⇒ salita più reattiva della discesa (riduce on/off)

#### A.3 — 3.3: rate limiter (Δv per ciclo) sul comando repulsivo

Implementare un limiter su `v_rep` (dopo profilo + eventuale filtro, o subito dopo raw; scelta consigliata: **dopo** filtro, come “paracadute”).

- Stato: `Eigen::Vector3d v_rep_prev_`
- Parametro: `repulsive_max_dv` (m/s per ciclo) o `repulsive_max_acc` (m/s²)
  - se `max_acc`: `max_dv = max_acc * dt`
- Limitare:
  - `dv = v_target - v_prev; if (dv.norm() > max_dv) dv = dv.normalized() * max_dv;`
  - `v_out = v_prev + dv; v_prev = v_out;`

**Nota su sicurezza**
- Se si teme ritardo in avvicinamento ostacolo: rendere anche il limiter **asimmetrico**:
  - consentire Δv più grande quando la repulsione aumenta (`max_dv_rise > max_dv_fall`).
  
**Aggiunta utente**: Fai anche il limiter asimmetrico

**Criteri di accettazione**
- `max_acc <= 0` ⇒ limiter disabilitato
- Non deve introdurre NaN quando `dv.norm() ~ 0`.

---

### B) `RepulsionDataManager` — 3.6 e 3.7 (solo POI)

**File già individuato:**
- `cartesian_velocity_controller/src/components/repulsion_data_manager.cpp`

#### B.1 — 3.6: filtro EMA sul gradiente

Il gradiente in `VoxelGrid3D::getGradientInterpolated()` è già “rumoroso” per natura (differenze finite su distanza interpolata). L’EMA sul gradiente stabilizza la **direzione**.

Implementazione:
- Mappa per POI: `std::unordered_map<std::string, Eigen::Vector3d> g_filtered_by_poi_` (o `std::map`)
- Parametri:
  - `gradient_filter_alpha` (0..1) oppure `gradient_filter_tau` (più coerente con dt)
  - `gradient_filter_enable`
- Aggiornamento:
  - normalizzare il gradiente raw se valido
  - fare EMA sul vettore (non sulla norma), poi rinormalizzare
  - gestire casi degeneri (norma ~ 0) con fallback al precedente o verso centro come già fa la mappa out-of-bounds

**Criticità nota**
- Il gradiente è un’unit vector: EMA + rinormalizzazione può “smussare” ma anche introdurre bias; con alpha troppo basso può diventare lento a cambiare direzione vicino agli ostacoli.

#### B.2 — 3.7: predictive query sui POI (robusta + conservativa)

Obiettivo: compensare latenza mappa rispetto al movimento del robot (POI si muove perché il robot si muove).

**Componenti**
- Per ogni POI, mantenere:
  - `prev_position_world` e `prev_stamp` (o usare `dt` del loop controllo)
  - `v_poi` stimata e filtrata (EMA) + clamp

**Predizione**
- `p_pred = p_now + v_poi * T_pred`
- `T_pred` clamped (es. 0.02–0.08s), idealmente legato a `1/update_rate_hz` ma limitato
- `v_poi` clamped (es. 0.5–2.0 m/s in base a robot)

**Conservatività (raccomandata)**
- Fare **due query**: `q_now = queryWorld(p_now)`, `q_pred = queryWorld(p_pred)`
- Combinare in modo “safe”:
  - distanza: `d = min(d_now, d_pred)`
  - gradiente: scegliere quello associato alla distanza minore, oppure una fusione ma mantenendo coerenza con la distanza scelta
  - repulsione risultante deve essere >= di quella che avresti usando solo `q_now` (evita predizione che “spegne”)

**Costi**
- 2 query/POI ⇒ ~2× del costo di query mappa per POI (accettabile per N POI moderato).

**Nota importante**
- Implementare 3.7 **solo sui POI**, non sugli ostacoli: i POI sono “controllabili” e la loro velocità è più affidabile della velocità ostacoli (se non tracciata).

---

## Parametri da aggiungere (proposta)

Aggiornare `cartesian_velocity_controller/config/controller_params.yaml` con default che non stravolgano il comportamento e che siano facilmente disabilitabili.

Esempio (nomi da adattare allo stile già presente nel repo):

```yaml
local_planner:
  repulsive_velocity_profile: "QUADRATIC"   # QUADRATIC, SMOOTHSTEP, SMOOTHERSTEP

  # 3.1 + asimmetria
  repulsive_filter_tau_rise: 0.02           # [s] 0 = disable
  repulsive_filter_tau_fall: 0.08           # [s] 0 = disable (se entrambi 0 -> no filter)

  # 3.3
  repulsive_max_acc_rise: 20.0              # [m/s^2] 0 = disable
  repulsive_max_acc_fall: 8.0               # [m/s^2] 0 = disable

repulsion:
  # 3.6
  gradient_filter_alpha: 0.3                # [0..1] 0=off, 1=raw

  # 3.7 (POI only)
  poi_predict_enable: true
  poi_predict_horizon: 0.06                 # [s] clamped internally
  poi_velocity_filter_tau: 0.05             # [s]
  poi_velocity_max: 1.5                     # [m/s]
  poi_predict_conservative_min_distance: true
```

---

## Debug/strumentazione minima consigliata

Senza cambiare architettura, è utile loggare o pubblicare (anche solo in debug msg):
- `v_rep_raw`, `v_rep_filtered`, `v_rep_limited`
- `d_now`, `d_pred`, `d_used`
- `|grad_raw|`, `|grad_filtered|` (e dot(grad_raw, grad_filtered) per vedere quanto “ruota”)
- `T_pred` effettivo e `|v_poi|` clampato

---

## Test plan (pragmatico)

1. **Scenario statico** (ostacolo fermo, robot quasi fermo):
   - verificare che non compaiano drift/oscillazioni e che il comando sia stabile.
2. **Scenario swap mappa** (robot in movimento continuo vicino influence zone):
   - confrontare jerk prima/dopo (visivamente e con logging di Δv).
3. **Scenario “avvicinamento rapido”**:
   - verificare che con asimmetria (rise rapido) la repulsione salga in modo sufficientemente reattivo.
4. **Predictive POI**:
   - controllare che `d_used` non sia maggiore di `d_now` quando `min_distance` è attivo.

---

## Punti aperti / criticità (da risolvere mentre si implementa)

1. **Sorgente di `dt`**
   - Serve un `dt` affidabile nel loop controllo per EMA/limiter e per stimare velocità POI. Se `dt` può variare, preferire `tau` e formule robuste (clamp alpha).

2. **Numero e identità dei POI**
   - Per filtri “per POI” (gradiente e velocità) serve una chiave stabile (nome/id POI). Se i POI possono comparire/scomparire, gestire cleanup per evitare crescita mappe.

3. **Coerenza gradiente-distanza nel caso predittivo**
   - Se si fa `d = min(d_now, d_pred)` bisogna anche scegliere un gradiente coerente (tipicamente quello della query che ha dato la distanza minore).

4. **Rischio di eccessiva latenza**
   - EMA + limiter + profilo più morbido possono sommare ritardo. Mitigazione: asimmetria rise/fall e default `tau_rise` piccolo.

5. **Prestazioni query (gradiente)**
   - Il gradiente oggi costa ~6× distanza interpolata (differenze finite). Con 3.7 (2 query) il costo aumenta. Se diventasse un problema, una futura ottimizzazione è precomputare gradient grid o calcolare gradiente con accesso diretto alla griglia (senza 6 trilineari), ma non è parte di questo piano.

6. **Interazione con “fallback last_valid_gradients”**
   - Se c’è già un fallback a gradiente precedente quando `distance < clamp_distance`, l’EMA sul gradiente va integrato con attenzione per non “incollare” la direzione troppo a lungo.

---

## Deliverable

- Nuovi parametri in `config/controller_params.yaml`
- Implementazione in `LocalPlanner`: 3.1, 3.3, 3.4, asimmetria
- Implementazione in `RepulsionDataManager`: 3.6 e 3.7 (POI only)
- Logging/debug minimo per valutazione

