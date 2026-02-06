# Piano per completare la portabilità dual-arm (focus: repulsione / Map3D)

## Contesto e obiettivo

Il pacchetto `cartesian_velocity_controller` è stato portato a girare con **due istanze** in parallelo (una per braccio) su MUR620, usando:

- `launch/mur620_dual_cartesian_velocity_controller.launch`
- `config/controller_params_mur620_ur10_l.yaml`
- `config/controller_params_mur620_ur10_r.yaml`

Nel setup MVP attuale **Map3D e repulsione sono disabilitate**. Il prossimo step è rendere la **repulsione funzionante e robusta** con due nodi contemporanei.

Questo documento descrive:

- **stato attuale** (cosa è già “dual-instance safe”),
- **gap** da chiudere (soprattutto repulsione/Map3D),
- un **piano operativo** con deliverable e test plan.

---

## Stato attuale (evidenze nel codice)

### Architettura repulsione (attuale)

- **Fonte dati ostacoli**: `Map3DManager` costruisce un distance field (EDT) voxelizzando **collision objects SPHERE** dal PlanningScene di MoveIt (world objects).
  - Lettura da PlanningScene monitor o fallback via service `get_planning_scene`.
  - Implementazione principale: `src/map3d/map3d_manager.cpp`, `src/map3d/planning_scene_sphere_reader.cpp`.
- **Bridge verso il planner**: `RepulsionDataManager`
  - Calcola POI (link + offset) su `RobotStateManager`
  - Query Map3D (distance + gradient) e produce `ObstacleInfo` (TCP) e `LinkPOI` (altri POI)
  - File: `src/components/repulsion_data_manager.cpp`
- **Uso runtime**: il controller, nel loop, chiama `repulsion_manager_->getRepulsionData()` **solo se** `repulsive_enabled` è true.
  - File: `src/cartesian_velocity_controller.cpp`

### Multi-istanza: cosa è già “OK”

- **Topic/servizi per-istanza** (privati):
  - `target_pose` è sottoscritto su private namespace (`~target_pose`) ⇒ L/R separati.
  - `MarkerPublisher` e `FeedbackPublisher` sono creati con `pnh_` ⇒ marker/debug per-istanza.
  - `Map3DManager` espone `~map3d/query` ⇒ niente collisioni tra istanze.
- **Parametri utili alla portabilità** già introdotti:
  - `controller_manager_ns`
  - `robot_description_param`
  - feature flag `map3d/enabled` (se false, Map3D/RepulsionDataManager non vengono creati)

### Config MUR620: stato attuale

Nei file `controller_params_mur620_ur10_{l,r}.yaml`:

- `map3d/enabled: false`
- `local_planner/repulsive_enabled: false`
- POI e parametri repulsione (sezione `repulsion:`) sono presenti ma con `points/*/enabled: false`.

---

## Gap principali da risolvere (repulsione / Map3D)

### G1) Bug/semantica: `map3d/update_rate_hz == 0` causa busy-loop

In `Map3DManager::updateLoop()` se `update_rate_hz <= 0` **non viene eseguita alcuna sleep**: la thread chiama `updateOnce()` in loop continuo consumando CPU.

Implicazione pratica:

- appena si abilita `map3d/enabled: true` ma si lascia `update_rate_hz: 0.0` (come nei YAML MVP), si rischia **100% CPU** su uno o più core per ogni istanza.

**Azione**: definire e implementare una semantica “pausa” per `update_rate_hz <= 0` (vedi piano).

### G2) Data source: la mappa esiste solo se il PlanningScene contiene SFERE

Map3D legge **solo primitive SPHERE** dai collision objects del PlanningScene (world geometry). Se:

- gli ostacoli sono mesh/box/cylinder,
- oppure non vengono pubblicati come collision objects nel PlanningScene,

allora `spheres` risulta vuoto ⇒ la mappa è “vuota” ⇒ repulsione di fatto nulla.

**Azione**: verificare/garantire che il pipeline sensori/scene-builder pubblichi ostacoli come sfere (o estendere Map3D ad altri tipi).

### G3) Parametri “assoluti”/risoluzione namespace da rendere deterministica

Per dual-arm robusto servono convenzioni chiare (e default corretti) su:

- `robot_description_param` (controller) e `map3d/robot_description_param` (Map3D)
- `map3d/get_planning_scene_service`
- topic `joint_states` e `planning_scene_topic`

**Azione**: nel launch MUR620 impostare questi parametri in modo esplicito per evitare che dipendano dal namespace corrente o da parametri “copiati” altrove.

### G4) Tooling: lo script interattivo gestisce la repulsione con API legacy non più presenti

`scripts/interactive_control/controllers/repulsive_manager.py` prova a usare:

- servizi `.../repulsive/set_enabled`, `.../repulsive/get_config`
- topic `.../repulsive/set_target_link`, ecc.

Nel C++ attuale la repulsione è controllata via:

- dynamic reconfigure (`cfg/ControllerTuning.cfg`)
- e parametri (es. `~repulsive_enabled`, `~repulsive_*`, POI config)

Quindi oggi la UI “repulsione” è a rischio di essere **incoerente/non funzionante**.

**Azione**: aggiornare `RepulsiveManager` e il menu UI per usare dynamic reconfigure / param server coerenti con la nuova architettura POI+Map3D.

### G5) Performance: due istanze ⇒ due EDT (potenzialmente costoso)

Con due controller, abilitare Map3D in entrambi significa:

- 2 thread che fanno voxelize+EDT, e potenzialmente 2 chiamate `get_planning_scene` per ciclo mappa.

**Azione**: iniziare con parametri conservativi (bassa frequenza, griglia piccola), e pianificare un’opzione “Map3D condivisa” come miglioramento successivo.

---

## Piano operativo (step-by-step)

### Step 0 — Baseline: verifiche minime (senza cambiare codice)

Obiettivo: avere una checklist riproducibile per capire se Map3D può funzionare nel tuo runtime.

- [ ] Verificare che esista `robot_description` nel namespace robot (`/mur620/robot_description`).
- [ ] Verificare che `get_planning_scene` sia raggiungibile nel namespace corretto (es. `/mur620/get_planning_scene`).
- [ ] Verificare che il PlanningScene contenga collision objects con primitive SPHERE.

Deliverable:

- una pagina “comandi + output atteso” (vedi sezione Test plan).

### Step 1 — Mettere in sicurezza Map3D quando `update_rate_hz <= 0`

Obiettivo: rendere Map3D “safe by default” anche in configurazioni MVP.

Azioni consigliate (una delle due):

- **Opzione A (consigliata)**: se `update_rate_hz <= 0`, la thread dorme e **non aggiorna** (mappa congelata).
  - Esempio: `ros::Rate pause_rate(10.0); pause_rate.sleep(); continue;`
- **Opzione B**: interpretare `update_rate_hz <= 0` come “update on-demand”:
  - aggiungere un trigger (service/topic) per eseguire `updateOnce()` manualmente
  - più lavoro, ma utile per debug.

Deliverable:

- patch a `src/map3d/map3d_manager.cpp` + note di semantica in documentazione.

### Step 2 — Rendere espliciti i parametri “di integrazione” in launch/config MUR620

Obiettivo: eliminare ambiguità di risoluzione namespace e rendere la configurazione ripetibile.

Azioni:

- [ ] In `mur620_dual_cartesian_velocity_controller.launch`, impostare sempre (non condizionalmente):
  - `robot_description_param` (controller) a `"/$(arg mur_ns)/robot_description"`
  - `map3d/robot_description_param` a `"/$(arg mur_ns)/robot_description"`
  - `map3d/get_planning_scene_service` a `"/$(arg mur_ns)/get_planning_scene"` (o quello reale)
  - (se necessario) `map3d/joint_state_topic` a `"/$(arg mur_ns)/joint_states"`
  - (se necessario) `map3d/planning_scene_topic` al topic reale del PlanningScene nel ns robot
- [ ] Nei YAML MUR620 sostituire hard-code `map3d/frame_id: "mur620/base_link"` con `$(arg robot_ns)/base_link`.

Deliverable:

- launch + YAML aggiornati (L e R).

### Step 3 — Abilitare Map3D per un solo braccio (test isolato)

Obiettivo: validare Map3D senza “rumore” da due istanze.

Azioni:

- [ ] In `controller_params_mur620_ur10_l.yaml`:
  - `map3d/enabled: true`
  - `map3d/update_rate_hz: 5.0` (iniziare basso)
  - abilitare debug minimo:
    - `map3d/debug/publish_spheres_marker: true`
    - `map3d/debug/publish_bounds_marker: true`
    - opzionale `publish_slice` per vedere gradienti
- [ ] Lasciare repulsione **spenta** (`local_planner/repulsive_enabled: false`) in questo step.

Acceptance:

- [ ] Il servizio `/<ns>/cartesian_velocity_controller_l/map3d/query` risponde con `valid=true` (almeno dentro bounds).
- [ ] Il marker “spheres” mostra ostacoli attesi (se presenti nel PlanningScene).
- [ ] Nessun consumo CPU anomalo.

### Step 4 — Abilitare repulsione (un braccio) con tuning conservativo

Obiettivo: far “entrare” la repulsione nel loop senza instabilità.

Azioni:

- [ ] Abilitare repulsione a livello LocalPlanner:
  - `local_planner/repulsive_enabled: true`
  - `local_planner/repulsive_obstacle_gain` (iniziare piccolo, es. 0.2–0.5)
  - `influence_distance`, `min_safe_distance` coerenti con risoluzione mappa e scale ostacoli
- [ ] Abilitare POI minimi:
  - `repulsion/points/tcp/enabled: true`
  - lasciare gli altri POI off finché il TCP non è stabile
- [ ] Abilitare smoothing minimo se necessario:
  - `local_planner/repulsive_filter_tau_rise`, `repulsive_filter_tau_fall`
  - `repulsion/gradient_filter_alpha` (se gradient “rumoroso”)

Acceptance:

- [ ] Avvicinando il TCP a un ostacolo (sfera), il vettore repulsivo cresce e impedisce la penetrazione.
- [ ] Nessuna oscillazione evidente (oppure mitigabile con i parametri di smoothing/limiter).

### Step 5 — Abilitare Map3D + repulsione anche sul secondo braccio

Obiettivo: validare comportamento dual-instance.

Azioni:

- [ ] Abilitare Map3D e repulsione anche su `controller_params_mur620_ur10_r.yaml`.
- [ ] Verificare che:
  - i topic debug map3d siano distinti (namespace privato),
  - i due servizi `map3d/query` siano distinti,
  - il carico CPU sia accettabile.

Acceptance:

- [ ] Le due istanze rispondono indipendentemente a query e generano repulsione coerente.

### Step 6 — Porting tooling: aggiornare lo script interattivo per la nuova repulsione

Obiettivo: poter abilitare/tunare repulsione da UI senza API legacy.

Azioni:

- [ ] Aggiornare `RepulsiveManager` per usare **dynamic reconfigure** sul nodo controller (`/<controller>/set_parameters` via client `dynamic_reconfigure`).
  - Parametri da esporre in UI (minimo):
    - `repulsive_enabled`
    - `repulsive_obstacle_gain`, `repulsive_link_gain`
    - `poi_tcp_enabled`, `poi_tcp_radius` (+ altri POI se si desidera)
    - `gradient_filter_alpha`, `poi_predict_enable`, `poi_predict_*`
    - `map3d_slice_z` (debug)
- [ ] Rimuovere o nascondere le voci legacy (payload link / collision objects mode) se non mappabili sul design POI+Map3D attuale.

Deliverable:

- UI “repulsione” funzionante su entrambe le istanze (L/R) selezionando `--arm l|r`.

### Step 7 — Hardening + performance (iterativo)

Azioni possibili (in ordine consigliato):

- [ ] Ridurre costo EDT:
  - griglia più piccola
  - risoluzione più grossolana
  - `update_rate_hz` più basso
- [ ] (Opzionale) progettare una **Map3D condivisa**:
  - un nodo `map3d_server` sotto `/mur620`
  - i due controller chiamano un service `QueryMap3D` esterno (invece di dipendenza in-process)
  - tradeoff: latenza/serializzazione vs dimezzare CPU.

---

## Test plan (comandi e verifiche)

### T1) Query Map3D (per-istanza)

Esempio (left):

- Service: `/mur620/cartesian_velocity_controller_l/map3d/query`
- Request: un punto in `mur620/base_link` (sostituire con il frame reale)

Esempio comando (da adattare):

```bash
rosservice call /mur620/cartesian_velocity_controller_l/map3d/query \
"point: {x: 0.5, y: 0.0, z: 1.0}
frame_id: 'mur620/base_link'"
```

Atteso:

- `valid: true`
- `inside_bounds: true` (se dentro box)
- `distance` coerente (più piccola vicino alle sfere)

### T2) Verifica presenza sfere nel PlanningScene

Se `spheres` è sempre vuoto:

- [ ] verificare che la pipeline sensori inserisca collision objects come sfere
- [ ] in alternativa: creare un collision object SPHERE “manuale” (script/rosservice) e verificare che Map3D lo veda.

### T3) Repulsione TCP (solo un braccio)

- [ ] Mettere `poi_tcp_enabled=true`, `repulsive_enabled=true`, `repulsive_obstacle_gain>0`.
- [ ] Avvicinare target verso un ostacolo e osservare:
  - marker repulsione,
  - `pipeline_debug` (vettori `v_obs_linear`),
  - comportamento del TCP.

### T4) Dual-instance

- [ ] Ripetere T1–T3 per entrambi i controller contemporaneamente.
- [ ] Verificare assenza di collisioni di:
  - topic debug/marker,
  - service `map3d/query`,
  - dynamic reconfigure.

---

## Definition of Done (repulsione “portata”)

Consideriamo completata la portabilità “repulsione dual-arm” quando:

- [ ] Map3D può essere abilitata su L e R senza consumare CPU in modo anomalo.
- [ ] `map3d/query` funziona per entrambe le istanze e restituisce dati validi.
- [ ] Repulsione TCP funziona (almeno per ostacoli sferici) su entrambi i bracci.
- [ ] Lo script interattivo può abilitare/disabilitare e tunare repulsione via dynamic reconfigure su L e R.
- [ ] Esiste una checklist di test riproducibile con output atteso.

---

## Backlog (non necessario per chiudere la portabilità, ma utile)

- Repulsione rispetto a ostacoli non-sferici (box/mesh) o generazione automatica di sfere da mesh.
- Repulsione/inter-arm avoidance (modellare l’altro braccio come ostacolo dinamico in mappa o altro schema).
- Map3D condivisa tra istanze (server unico) per ridurre CPU.

