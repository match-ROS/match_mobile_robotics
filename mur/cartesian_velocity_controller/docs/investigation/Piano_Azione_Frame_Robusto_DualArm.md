# Piano d’azione: gestione frame robusta (dual-arm, due nodi separati)

> **Scopo**: rendere la gestione di frame/pose **deterministica, parametrica e multi-istanza** per controllare **due bracci** in contemporanea lanciando **due nodi separati** del `cartesian_velocity_controller` (e relativi script/tooling).
>
> **Contesto (dai documenti in questa cartella)**:
> - oggi esiste il rischio di **mismatch di frame** tra Python (`DEFAULT_GLOBAL_FRAME` spesso `base_link`, e può cambiare runtime) e C++ (`global_frame` tipicamente `world`);
> - il controller C++ **non trasforma** `PoseStamped` se `frame_id != global_frame_` (fa solo warning), quindi può usare target in frame errato;
> - le pose salvate non includono `frame_id` → non sono auto-consistenti.
>
> **Nota (dai launch `mur620_dual_*`)**: nella configurazione dual-arm attuale si forza `global_frame` a `/<mur_ns>/base_link` (es. `/mur620/base_link`) per allinearsi a setup TF “prefissati” (`mur620/...`) e si separano i topic target per braccio.

---

## Obiettivi e non-obiettivi

### Obiettivi (Must)
- **Contratto chiaro sui frame**: ogni target pose ha un frame dichiarato e viene interpretato in modo univoco.
- **Trasformazioni TF robuste**: se arriva un target in frame diverso, viene trasformato (o rifiutato) in modo deterministico.
- **Dual-arm ready**: due istanze del controller (e dei publisher/marker/debug) possono convivere senza collisioni di topic/parametri.
- **Configurabile via parametri**: nessun comportamento critico deve dipendere da fallback “magici” o discovery non deterministico.
- **Diagnostica**: log e debug data sufficienti per capire *che frame è arrivato* e *che frame è stato usato*.

### Non-obiettivi (per ora)
- Rifare l’intera architettura del planner/pipeline (LocalPlanner/Filter/PID/Jacobian/Safety).
- Implementare “coordination” tra bracci (evitare self-collision tra bracci, task-level planning, ecc.). Qui ci limitiamo a robustezza frame + multi-istanza.

---

## Decisioni chiave (“Frame Contract”)

### Definizioni
- **`global_frame`**: frame in cui il controller **interpreta** i target e in cui esprime la logica interna del waypoint (es. `world` o `map`).
- **`tcp_link` / `ee_frame`**: frame/link dell’end-effector specifico del braccio (es. `tool0_l`, `tool0_r`).
- **`robot_base_frame`** *(opzionale ma consigliato)*: frame del robot mobile o base manipolatore (es. `base_link`, `arm_base_link_l`, `arm_base_link_r`). Utile per debug, marker e per future estensioni.

### Regole
1. Il controller **accetta** `PoseStamped` in qualunque frame **solo se** TF può trasformarlo in `global_frame` entro un timeout.
2. Se `frame_id` è vuoto: comportamento **esplicito** (parametrico) — o lo si considera `global_frame`, o si rifiuta.
3. Gli script **non cambiano** `global_frame` in runtime per “caching” del primo TF che funziona. Il frame deve essere una scelta di configurazione.

---

## Principali criticità da risolvere (e perché esplodono con due nodi)

- **Frame mismatch**: con robot mobile `world/map/odom/base_link` non sono identità → interpretare `base_link` come `world` porta a pose completamente sbagliate.
- **No TF transform nel controller**: oggi un target “sbagliato” entra comunque e viene eseguito.
- **Collisioni ROS name/topic**: due controller che pubblicano/consumano su topic assoluti o fissi si sovrascrivono.
- **Fallback non deterministici lato Python**: in dual-arm può cambiare diversamente per ogni braccio/istanza creando bug intermittenti.
- **Pose salvate senza metadati**: una posa “home_l” potrebbe essere stata registrata in un frame diverso da “home_r”, senza che sia visibile.

---

## Stato attuale (dai launch `mur620_dual_cartesian_velocity_controller.launch` e `mur620_dual_interactive_control.launch`)

Questa è la “wiring” attuale su MUR620 dual-arm (utile perché riduce alcune criticità, ma lascia comunque aperto il problema TF transform nel controller).

- **Namespace robot**: un unico namespace robot `/<mur_ns>` (default `mur620`), quindi tutti i nodi vivono sotto `/mur620/...`.
- **Due istanze controller**:
  - `/mur620/cartesian_velocity_controller_l`
  - `/mur620/cartesian_velocity_controller_r`
- **`robot_description`**: reso esplicito con `robot_description_param=/mur620/robot_description` (quindi MoveIt/RobotModel risolvono dentro il ns robot).
- **`global_frame`**: forzato per entrambe le istanze a `/<mur_ns>/base_link` (es. `/mur620/base_link`) per TF tree prefissato.
- **Topic target separati**:
  - il controller C++ sottoscrive `"target_pose"` su `nh_` (quindi, senza remap, sarebbe `/<mur_ns>/target_pose`);
  - il launch remappa `"target_pose"` per ottenere:
    - `/mur620/cartesian_velocity_controller_l/target_pose`
    - `/mur620/cartesian_velocity_controller_r/target_pose`
- **Interactive control**:
  - lancia due processi `robot_interactive_control.py` con `--mur-ns /mur620 --arm l|r`
  - per default pubblica su `<controller_node_name>/target_pose`, quindi combacia con i topic remappati sopra.

Implicazione: oggi la separazione L/R per il *solo* target pose è già gestita; va verificato/esteso lo stesso approccio anche per debug/marker/altre interfacce se non sono già namespaced.

---

## Verifica consistenza col codice (findings)

Questa sezione confronta “piano” vs implementazione corrente (C++/Python/launch/config) per evidenziare cosa è già coerente e cosa è un gap reale.

### A) Topic e multi-istanza (C++)

- **Target pose**
  - **Codice**: il controller C++ sottoscrive `target_pose` con `nh_.subscribe("target_pose", ...)`.
  - **Effetto**: in un namespace robot `/mur620`, senza remap sarebbe `/mur620/target_pose` (conflitto tra L/R).
  - **Stato attuale**: i launch risolvono con remap verso:
    - `/mur620/cartesian_velocity_controller_l/target_pose`
    - `/mur620/cartesian_velocity_controller_r/target_pose`
  - **Gap rispetto alla “Opzione B”**: per eliminare remap sarebbe meglio usare un topic **privato** (es. `~target_pose`) o parametrizzare il nome del topic.

- **Feedback/debug**
  - **Codice**: `FeedbackPublisher` viene creato con `pnh_` e pubblica `end_effector_state`, `pipeline_debug`, `joint_velocity_feedback` in **namespace privato** del nodo.
  - **Stato**: **già per-istanza**, quindi dual-arm safe.

- **Marker RViz**
  - **Codice**: `MarkerPublisher` pubblica su `velocity_markers`, `target_markers`, `command_markers`, `repulsion_markers` con `nh_` (topic “globali” rispetto al nodo).
  - **Stato attuale**: nei YAML L/R è impostato `marker_publisher/ns_prefix` (`UR10_l` / `UR10_r`) che separa i marker per `Marker.ns`, **ma non separa i topic**.
  - **Rischio**: se entrambe le istanze pubblicano sullo stesso topic (es. `/mur620/target_markers`), RViz vede un flusso unico (poi separabile per namespace interno del marker). Funziona, ma è fragile per tooling e debug automatico.
  - **Azione consigliata**: rendere anche i **topic per-istanza** (topic privati o parametri).

- **Map3D (quando abilitato)**
  - **Codice**: `Map3DManager` espone il servizio `map3d/query` con `nh_.advertiseService("map3d/query", ...)`.
  - **Rischio**: con due istanze c’è collisione di servizio (una sovrascrive l’altra). Nel setup MVP `map3d/enabled` è disabilitato, quindi oggi non esplode, ma è un rischio reale per “robustezza”.
  - **Azione consigliata**: servizio per-istanza (privato o parametrizzato).

### B) Frame/TF (C++)

- **Contratto sul frame del target pose**
  - **Codice**: `setTargetPose(const PoseStamped&)` contiene un `TODO` e, se `frame_id != global_frame_`, fa solo warning e **non trasforma**.
  - **Conclusione**: il punto “TF transform nel controller” del piano è confermato come **gap critico**.

### C) Script Python / Interactive control

- **Pubblicazione `target_pose` per istanza**
  - **Codice**: se non si passa `--target-pose-topic`, lo script pubblica su `<controller_node_name>/target_pose`.
  - **Stato attuale**: con `--arm l|r`, `controller_node_name` diventa `/mur620/cartesian_velocity_controller_l|r`, quindi pubblica sul topic corretto e separato (coerente con i launch).

- **Lettura `global_frame`**
  - **Codice**: lo script legge `/<controller_node>/global_frame` da parameter server e lo usa come `header.frame_id` quando pubblica target.
  - **Coerenza**: buona (allinea Python al controller).

- **Frame dinamico / fallback TF**
  - **Codice**: `get_current_ee_pose()` prova combinazioni (prefisso TF) e poi **cacha** la coppia funzionante aggiornando `self._global_frame` e `self.ee_frame`.
  - **Rischio**: rende il `global_frame` di fatto “mutabile” (anche se l’intento è solo gestire prefissi), e questo entra in conflitto con l’idea di frame contract “stabile”.
  - **Azione consigliata**: mantenere il fallback solo per risolvere **EE frame prefissato**, senza mutare il `global_frame` canonico (o renderlo modalità debug esplicita).

### D) Pose salvate (`saved_poses.yaml`)
- **Stato attuale**: per pose si salvano `position`, `orientation`, `description` (nessun `frame_id`).
- **Conclusione**: confermato gap rispetto al piano “pose auto-consistenti”.

---

## Strategia multi-istanza (due nodi separati)

### Opzioni consigliate (in ordine di “minima invasività”)

#### Opzione A (allineata ai launch attuali): un namespace robot + due nodi controller
- **Esempio**:
  - `/mur620/cartesian_velocity_controller_l`
  - `/mur620/cartesian_velocity_controller_r`
- **Separazione**: via remap dei topic (come già fatto per `target_pose`) e/o parametri di topic name.
- **Pro**: minimale, non stravolge MoveIt/robot_description.
- **Contro**: bisogna ricordarsi di remappare ogni topic “shared” che oggi sta in `/<mur_ns>/...`.

#### Opzione B (più “pulita” a regime): topic privati per istanza (riduce/remuove i remap)
- Nel controller, usare topic in namespace privato (es. `~target_pose` invece di `target_pose` su `nh_`), così ogni istanza parla su:
  - `/mur620/cartesian_velocity_controller_l/target_pose`
  - `/mur620/cartesian_velocity_controller_r/target_pose`
  automaticamente, senza remap.
- **Pro**: meno launch fragile; ogni istanza è naturalmente isolata.
- **Contro**: richiede modifica C++ (e verifiche su altri topic analoghi).

#### Opzione C (solo se serve davvero): due namespace separati per braccio
- **Esempio**:
  - `/mur620/arm_left/...`
  - `/mur620/arm_right/...`
- **Pro**: isolamento ancora più esplicito.
- **Contro**: più complesso per risolvere `robot_description`, servizi controller_manager, ecc. (da fare solo se l’Opzione B non basta).

### Regole pratiche
- **Topic relativi** nel controller (e negli script) dove possibile, così lo stesso binario funziona “per braccio” grazie al namespace.
- Dove non è possibile/si vuole esplicitare, introdurre **parametri di topic name** (`~target_pose_topic`, `~pipeline_debug_topic`, `~marker_topic`, …).
- Parametri sempre in private namespace (`~...`) per evitare collisioni globali.

---

## Proposte di parametri (API di configurazione)

### Controller C++ (`cartesian_velocity_controller`)
Parametri già esistenti (da consolidare e rendere “obbligatori” in dual-arm):
- `~global_frame` (string, **required** in dual-arm)
- `~tcp_link` (string, **required**)

Nuovi parametri proposti:
- **TF / pose input**
  - `~tf_timeout` (double, default es. `0.1`)
  - `~reject_on_tf_failure` (bool, default `true`)
  - `~accept_empty_frame_as_global` (bool, default `true` oppure `false`, da decidere)
  - `~target_pose_topic` (string, default `target_pose`)
- **Debug / marker (per evitare collisioni)**
  - `~pipeline_debug_topic` (string, default `pipeline_debug`)
  - `~marker_topic` (string, default `visualization_marker_array`)

Comportamento atteso:
- Se arriva `PoseStamped` con `frame_id != global_frame`: il controller prova TF transform → usa la posa trasformata.
- Se transform fallisce:
  - `reject_on_tf_failure=true` → rifiuta il target e mantiene stato precedente (o `has_target_=false`), pubblica warning + debug.
  - `reject_on_tf_failure=false` → fallback esplicito (sconsigliato in produzione).

### Script Python (publisher / tools)
Parametri proposti:
- `~controller_ns` o `~controller_node_name` (string) per puntare all’istanza corretta (es. `/arm_left/cartesian_velocity_controller`)
- `~global_frame` (string, **non** “dinamico”: deve essere coerente con quello del controller)
- `~ee_frame` (string, per leggere posa corrente via TF)
- `~target_pose_topic` (string, default `target_pose`, relativo al namespace del controller)

Comportamento atteso:
- `global_frame` viene letto dal parameter server dell’istanza corretta **e non cambia** durante l’esecuzione.
- `get_current_ee_pose()` usa *sempre* la coppia (`global_frame`, `ee_frame`) configurata (niente “candidate pairs” salvo modalità debug esplicita).

---

## Formato pose salvate: rendere `saved_poses.yaml` auto-consistente

### Nuovo schema (proposto)
Per ogni posa salvare almeno:
- `frame_id` (string)
- `position` (3)
- `orientation` (4, ordine esplicito **qx,qy,qz,qw**)
- opzionale: `ee_frame` (string), `stamp` (string), `source` (string)

Esempio:
```yaml
home_l:
  description: Home braccio sinistro
  frame_id: world
  ee_frame: tool0_l
  position: [0.1, 0.2, 0.9]
  orientation: [0.0, 0.7071068, 0.0, 0.7071068]   # qx,qy,qz,qw
```

### Compatibilità
- Supportare temporaneamente il vecchio schema (senza `frame_id`) con regola esplicita:
  - se manca `frame_id`, assumere `global_frame` configurato (con warning).

---

## Diagnostica e “guard rails”

### Validazioni input (minime)
- **Quaternione**: norma \( \|q\| \approx 1 \). Se fuori soglia:
  - opzione A: normalizzare e loggare warning;
  - opzione B: rifiutare (parametrico).
- **Frame**: se `frame_id` non esiste nel TF tree o non è connesso a `global_frame` → rifiutare.

### Debug data (molto utile in dual-arm)
Aggiungere a `pipeline_debug` (o ad un nuovo msg) campi tipo:
- `received_target_frame`
- `used_target_frame` (dopo trasformazione: deve essere `global_frame`)
- `tf_transform_ok` (bool), `tf_error` (string breve)
- `tf_latency_sec` / age della transform (se disponibile)

---

## Piano di implementazione (step-by-step)

### Step 0 — Inventario rapido (baseline)
- Identificare tutti i topic pubblicati/sottoscritti dal controller e dagli script, e segnare quali sono assoluti vs relativi.
- Identificare dove viene definito `global_frame` lato C++ e lato Python e dove viene “mutato” runtime.

**Deliverable**: tabella “topic/param → per istanza?” e lista dei punti di collisione.

### Step 1 — TF transform nel controller C++ (alta priorità)
- Implementare transform in `setTargetPose(const geometry_msgs::PoseStamped&)`.
- Aggiungere parametri `tf_timeout`, `reject_on_tf_failure`, `accept_empty_frame_as_global`.
- Log throttled chiari e debug info.

**Acceptance**:
- Target in `base_link` viene trasformato correttamente in `world/map` (se TF disponibile).
- Target con frame sconosciuto viene rifiutato (nessun movimento non voluto).

### Step 2 — Stabilizzare i frame lato Python
- Rimuovere/isolatre il caching “candidate frame pairs” come modalità **debug**.
- Rendere `global_frame` e `ee_frame` parametrici e coerenti con l’istanza controller.

**Acceptance**:
- La posa corrente letta via TF è sempre nello stesso frame dichiarato.
- Lo script non cambia frame senza log esplicito e senza modalità debug attiva.

### Step 3 — Upgrade `saved_poses.yaml` (metadati frame)
- Aggiornare `PoseManager` per salvare `frame_id` (+ opzionale `ee_frame`).
- In send_pose: se `pose.frame_id != controller.global_frame`, scegliere comportamento:
  - opzione consigliata: inviare con `pose.frame_id` e lasciare al controller la trasformazione;
  - alternativa: trasformare in Python prima di pubblicare (ma duplicazione logica).

**Acceptance**:
- Ogni posa stampata/loggata mostra frame e ordine quaternion.
- Il vecchio formato continua a funzionare con warning.

### Step 4 — Dual-arm: namespace + topic param
- Rendere i topic “shared” (almeno `target_pose`, marker, eventuali servizi map3d) **per-istanza**:
  - opzione consigliata: migrare a **topic privati** (`~target_pose`, `~velocity_markers`, …) così ogni nodo pubblica naturalmente nel proprio namespace;
  - alternativa: parametri di topic name + remap in launch (più fragile).
- Allineare i launch alla strategia scelta:
  - se si resta su **Opzione A**: assicurarsi che *tutti* i topic potenzialmente condivisi siano remappati per braccio (non solo `target_pose`);
  - se si passa a **Opzione B**: migrare progressivamente i topic a namespace privato (`~...`) così da eliminare i remap.

**Acceptance**:
- `rostopic list` mostra due topic target distinti (uno per braccio), ad esempio:
  - `/mur620/cartesian_velocity_controller_l/target_pose`
  - `/mur620/cartesian_velocity_controller_r/target_pose`
- Pubblicando su quello di sinistra si muove solo il braccio sinistro, e viceversa.

### Step 5 — Test riproducibili (con due istanze)
- **Test round-trip** per ciascun braccio: salva posa corrente → vai altrove → ritorna.
- **Test frame mismatch**: invia target con `frame_id=base_link` e `global_frame=world/map` (deve trasformare).
- **Test failure**: invia frame inesistente (deve rifiutare e non muovere).
- **Test simultaneo**: invia due target a distanza di pochi ms su due namespace (no cross-talk).

**Deliverable**: script/test manuale documentato con comandi ROS (e output atteso).

---

## Scelte aperte (da decidere insieme)

1. **Qual è il `global_frame` canonico?**
   - robot fisso: spesso `world`
   - robot mobile: spesso `map` (o `odom` per continuità locale)  
   Impatto: coerenza con MoveIt, saved poses e TF tree del MiR.

2. **Dove trasformare: Python o C++?**
   - raccomandato: **C++** (un solo punto di verità, più vicino all’esecuzione)
   - Python può comunque trasformare per preview/UX, ma non deve essere l’unico guardiano

3. **Gestione `frame_id` vuoto**: accettare come `global_frame` o rifiutare.

4. **Quaternione**: normalizzare sempre o rifiutare (parametro).

---

## Rischi e mitigazioni

- **TF intermittente / extrapolation**: mitigare con `tf_timeout` ragionevole e uso di `Time(0)` (latest) dove appropriato.
- **Incoerenza MoveIt vs controller**: MoveIt spesso gestisce trasformazioni “automatiche”; qui vogliamo esplicitare e allineare i frame.
- **Debug rumoroso con 2 istanze**: usare log throttled e topic debug namespaced.

---

## Done definition (quando consideriamo “robusto”)
- Con due nodi in parallelo, ogni nodo:
  - riceve target in frame arbitrario e lo trasforma in `global_frame` o lo rifiuta;
  - non condivide topic/parametri con l’altro nodo (salvo TF globale, joint_states se condivisi, ecc.);
  - salva e riusa pose con `frame_id` esplicito;
  - produce debug sufficiente per capire rapidamente eventuali mismatch.

