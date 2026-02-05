# Spec implementativa — Opzione B (topic/servizi privati per istanza)

> **Obiettivo**: applicare l’**Opzione B** descritta in `Piano_Azione_Frame_Robusto_DualArm.md`, rendendo il sistema dual-arm robusto **senza dipendere da remap fragili** per separare L/R.
>
> **Vincolo**: questa spec è pensata per la wiring attuale sotto `/<mur_ns>` (default `/mur620`) con due nodi:
> - `/mur620/cartesian_velocity_controller_l`
> - `/mur620/cartesian_velocity_controller_r`

---

## 1) Stato “as-is” (verificato)

### 1.1 Target pose
- Il controller C++ sottoscrive `target_pose` su `nh_` ⇒ senza remap sarebbe `/mur620/target_pose`.
- I launch attuali risolvono con remap a:
  - `/mur620/cartesian_velocity_controller_l/target_pose`
  - `/mur620/cartesian_velocity_controller_r/target_pose`
- Gli script Python (con `--arm l|r`) pubblicano già di default su `<controller_node_name>/target_pose`, quindi sono già allineati a una soluzione “per nodo”.

### 1.2 Debug/feedback
- `pipeline_debug`, `end_effector_state`, `joint_velocity_feedback` sono già pubblicati usando `pnh_` ⇒ già per-istanza.

### 1.3 Marker RViz
- Pubblicati con `nh_` su topic condivisi (es. `/mur620/target_markers`). Separazione interna via `Marker.ns_prefix` (config YAML) ma topic unico.

### 1.4 Map3D (se abilitato)
- Servizio `map3d/query` è esposto con `nh_` ⇒ collisione tra istanze.

---

## 2) Definizione Opzione B (target state desiderato)

### 2.1 Regola base
Tutto ciò che è “per-controller/per-arm” deve vivere nel **namespace del nodo** (cioè `~...`):
- Topic di input per target
- Topic marker/debug specifici del controller
- Servizi specifici della singola istanza (es. Map3D query)

### 2.2 ROS graph atteso (esempio)
Con `mur_ns=mur620`:

- **Left**
  - `/mur620/cartesian_velocity_controller_l/target_pose`
  - `/mur620/cartesian_velocity_controller_l/pipeline_debug` *(già così)*
  - `/mur620/cartesian_velocity_controller_l/target_markers` *(da rendere così)*
  - `/mur620/cartesian_velocity_controller_l/map3d/query` *(se Map3D abilitato)*

- **Right**
  - `/mur620/cartesian_velocity_controller_r/target_pose`
  - `/mur620/cartesian_velocity_controller_r/pipeline_debug`
  - `/mur620/cartesian_velocity_controller_r/target_markers`
  - `/mur620/cartesian_velocity_controller_r/map3d/query`

---

## 3) Modifiche previste (senza implementazione qui)

### 3.1 Controller C++ — target pose topic privato
**File**: `src/cartesian_velocity_controller.cpp`

**Punto attuale**: in `setupRosInterfaces()`:
- `target_pose_sub_ = nh_.subscribe("target_pose", ...)`

**Modifica Opzione B**:
- usare `pnh_` per sottoscrivere `target_pose` così diventa `~target_pose` (topic per nodo).

**Impatto launch**:
- i remap `from="target_pose" ...` diventano inutili e vanno rimossi (o convertiti in remap su `~target_pose` se si desidera comunque rimappare).

### 3.2 Controller C++ — marker topic per-istanza
**File**: `src/cartesian_velocity_controller.cpp` (creazione MarkerPublisher)

**Punto attuale**:
- `marker_publisher_ = std::make_unique<MarkerPublisher>(nh_, marker_config);`
- e in `MarkerPublisher` i topic sono `"velocity_markers"`, `"target_markers"`, ecc.

**Modifica Opzione B**:
- passare `pnh_` al `MarkerPublisher` oppure introdurre un `ros::NodeHandle marker_nh("~")` dedicato.
  - Risultato: topic marker diventano `~velocity_markers`, `~target_markers`, ...

**Nota RViz**:
- RViz dovrà puntare ai topic per braccio (o si può introdurre un alias/remap a topic “aggregati” se serve).

### 3.3 Map3D — service per-istanza (future-proof)
**File**: `src/map3d/map3d_manager.cpp`

**Punto attuale**:
- `query_srv_ = nh_.advertiseService("map3d/query", ...)`

**Modifica Opzione B**:
- usare `pnh_` per esporre il servizio nel namespace del nodo:
  - `~map3d/query` ⇒ `/<node_name>/map3d/query`

**Nota**:
- anche i debug topic map3d (se usati) vanno valutati: oggi sono configurabili via `cfg_.debug_*_topic` e pubblicati su `nh_`.

### 3.4 (Opzionale) Parametrizzazione nomi topic
Opzione B “pura” può funzionare solo con `~...` senza parametri.
Se però volete massimo controllo, introdurre parametri tipo:
- `~target_pose_topic` (default `"target_pose"` nel privato)
- `~marker_topics/*` o un prefisso
- `~map3d/query_service_name` (default `"map3d/query"` nel privato)

Questa parte è opzionale: il vantaggio principale dell’Opzione B è ridurre la superficie di configurazione.

---

## 4) Compatibilità con la wiring attuale

### 4.1 Launch dual controller
**File**: `launch/mur620_dual_cartesian_velocity_controller.launch`

Quando il controller sottoscrive `~target_pose`, i remap attuali:
- `<remap from="target_pose" to="cartesian_velocity_controller_l/target_pose" />`
non hanno più effetto sul subscriber privato.

Quindi a regime:
- **rimuovere i remap** di `target_pose` (il default già sarà `/<node>/target_pose`).

### 4.2 Interactive control
Gli script Python già pubblicano su `<controller_node_name>/target_pose` di default: combaciano perfettamente con Opzione B.

---

## 5) Test plan (post-implementazione)

### 5.1 Wiring / isolamento L-R
- Verificare che esistano:
  - `/mur620/cartesian_velocity_controller_l/target_pose`
  - `/mur620/cartesian_velocity_controller_r/target_pose`
- Pubblicare un target su uno e verificare che l’altro non reagisca.

### 5.2 Marker per-istanza
- Verificare che i marker siano pubblicati su:
  - `/mur620/cartesian_velocity_controller_l/target_markers`
  - `/mur620/cartesian_velocity_controller_r/target_markers`

### 5.3 Map3D (se abilitato in futuro)
- Verificare che entrambi i servizi esistano e rispondano:
  - `/mur620/cartesian_velocity_controller_l/map3d/query`
  - `/mur620/cartesian_velocity_controller_r/map3d/query`

---

## 6) Decisioni ancora da fissare prima di “scrivere codice”

1. **Marker**: volete davvero topic separati (Opzione B “pura”) oppure preferite topic condivisi + `ns_prefix` (comodo per RViz)? **Risposta:** Preferisco topic separati
2. **Map3D**: anche se ora è disabilitato, lo vogliamo già “Opzione B-compliant” per non ripagare debito dopo? **Risposta**: Si
3. **Parametri vs default**: preferite zero parametri (solo `~...`) o aggiungere parametri per sovrascrivere i nomi? **Risposta:** Va bene zero parametri

