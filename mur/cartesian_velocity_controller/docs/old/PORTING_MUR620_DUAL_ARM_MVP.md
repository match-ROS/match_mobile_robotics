# Porting `cartesian_velocity_controller` su MUR620 (dual UR) — MVP “muovere i due bracci”

## Obiettivo (primo step di portabilità)

Portare il pacchetto `cartesian_velocity_controller` (nato per **un singolo braccio**) a funzionare sul robot lanciato da:

- `src/match_mobile_robotics/mur/mur_bringup/launch/single_mur620_gazebo.launch`

che avvia un MUR620 con **due bracci UR**.  
Per questo MVP l’obiettivo è **muovere entrambi i bracci in contemporanea ma indipendenti**, senza preoccuparsi ancora di:

- mappa 3D (`map3d`) e obstacle avoidance / repulsione (si può disabilitare o ridurre al minimo se causa blocchi)
- collision avoidance “intelligente” tra i due bracci (ne riparliamo dopo la stabilizzazione del controllo base)

## Stato attuale: cosa è “single-arm” nel pacchetto

### Parametri “single-arm” hard-coded / default
Nel file `config/controller_params.yaml` del pacchetto, e nei default del nodo C++, ci sono assunzioni tipiche di un UR singolo:

- `group_name: "manipulator"`
- `tcp_link: "tool0"`
- `joint_state_topic: "/joint_states"` (**assoluto** → ignora namespace)
- `velocity_command_topic: "/joint_group_vel_controller/command"` (**assoluto** → ignora namespace)
- controller switching:
  - `start_controller: "joint_group_vel_controller"`
  - `stop_controller: "vel_joint_traj_controller"`

Nel codice C++ (nodo `cartesian_velocity_controller`):

- Sottoscrive **relativamente** a `target_pose` (ok per namespace/remap).
- Ma usa servizi `controller_manager` in modo **assoluto**:
  - `"/controller_manager/switch_controller"`
  - `"/controller_manager/list_controllers"`
  → su MUR620 in Gazebo questi servizi esistono tipicamente sotto `/<mur_ns>/controller_manager/...` (es. `/mur620/controller_manager/...`).

### Map3D (thread) sempre attivo
Nel nodo C++ viene creato e avviato sempre:

- `map3d_manager_ = Map3DManager(nh_, pnh_);`
- `map3d_manager_->start();`

Con la configurazione attuale, in `config/controller_params.yaml` la sezione `map3d` contiene:

- `map3d/joint_state_topic: "/joint_states"` (**assoluto**)
- `map3d/get_planning_scene_service: "/get_planning_scene"` (**assoluto**)

Su un robot namespaced queste due cose sono tra le prime a “rompersi” e possono generare warning continui o mappa vuota.

### Script interattivo Python (single-arm + assoluti)
Gli script in `scripts/interactive_control/` hanno ulteriori assunzioni:

- Pubblicano su topic **assoluto** `'/target_pose'` (mentre il nodo C++ ascolta `target_pose` relativo).
- Usano servizi **assoluti**:
  - `'/controller_manager/switch_controller'`
  - `'/controller_manager/list_controllers'`
- Default MoveIt group: `"manipulator"` (su MUR620 i gruppi reali sono `UR_arm_l`, `UR_arm_r`).
- `DEFAULT_CONTROLLER_NODE = "/cartesian_velocity_controller"` (assoluto) usato per leggere parametri tipo `global_frame`.

## Stato attuale del robot MUR620 (da `mur_620.launch`)

Dal launch `src/match_mobile_robotics/mur/mur_launch_sim/launch/mur_620.launch` (incluso da `single_mur620_gazebo.launch`):

- Tutto gira sotto namespace `/<mur_ns>` (default: `/mur620`).
- I controller “velocità” per i due bracci vengono spawnati **stopped**:
  - `joint_group_vel_controller_l/unsafe`
  - `joint_group_vel_controller_r/unsafe`
- I controller MoveIt/FollowJointTrajectory sono:
  - `UR10_l/arm_controller`
  - `UR10_r/arm_controller`
- `robot_description` è pubblicato come parametro sotto `/<mur_ns>/robot_description` (es. `/mur620/robot_description`).
- Uno script (`copy_robot_description.py`) copia la URDF anche sotto:
  - `/mur620/UR10_l/robot_description`
  - `/mur620/UR10_r/robot_description`

I gruppi MoveIt (SRDF) per MUR620 sono:

- `UR_arm_l`
- `UR_arm_r`
- (anche `UR_arm_both`, ecc.)

Quindi una configurazione “dual-arm” naturale è: **due istanze del controller**, una per `UR_arm_l` e una per `UR_arm_r`.

## Strategia consigliata per l’MVP (minimo rischio / massimo riuso)

### Idea chiave
Invece di rifattorizzare il controller per gestire due bracci in un unico processo, avviare:

- **due istanze** del nodo `cartesian_velocity_node`
  - una con `group_name = UR_arm_l` e topic comando verso `joint_group_vel_controller_l/unsafe/command`
  - una con `group_name = UR_arm_r` e topic comando verso `joint_group_vel_controller_r/unsafe/command`

Questo permette di:

- riusare la pipeline così com’è
- isolare tuning e parametri per braccio
- muovere entrambi i bracci in parallelo (due loop indipendenti)

### Requisito fondamentale
Per funzionare in un robot namespaced (come `/mur620`) è necessario che:

- **topic e servizi non siano hard-coded assoluti** (`/...`)
- oppure siano sempre passati come parametri corretti (meglio: relativi)

## Piano d’azione (step-by-step)

### Step 0 — Definire il “layout” di namespace e topic (decisione architetturale)

Propongo questa convenzione per evitare collisioni:

- Namespace robot: `/mur620`
- Due controller:
  - `/mur620/cartesian_velocity_controller_l`
  - `/mur620/cartesian_velocity_controller_r`
- Due target topics distinti:
  - `/mur620/cartesian_velocity_controller_l/target_pose`
  - `/mur620/cartesian_velocity_controller_r/target_pose`

Il nodo C++ già sottoscrive a `target_pose` relativo, quindi è sufficiente:

- lanciare ogni istanza con un proprio namespace
- oppure remappare `target_pose` per ognuna

### Step 1 — Creare configurazioni YAML “per braccio”

Duplicare `config/controller_params.yaml` in due file (esempio):

- `config/controller_params_mur620_ur10_l.yaml`
- `config/controller_params_mur620_ur10_r.yaml`

e cambiare almeno:

- `group_name`:
  - left: `UR_arm_l`
  - right: `UR_arm_r`
- `tcp_link`:
  - **da verificare** (probabile `UR10_l/tool0` e `UR10_r/tool0`)
  - (alternativa: se nel modello i link non sono prefissati, può rimanere `tool0`)
- `joint_state_topic`:
  - evitare assoluti: usare `joint_states` (relativo), così sotto `/mur620` diventa `/mur620/joint_states`
- `velocity_command_topic`:
  - left: `joint_group_vel_controller_l/unsafe/command` (relativo)
  - right: `joint_group_vel_controller_r/unsafe/command` (relativo)
- controller switching (MVP):
  - **opzione A (più semplice)**: disattivare lo switching impostando:
    - `start_controller: ""`
    - `stop_controller: ""`
  - e avviare i controller `joint_group_vel_controller_*/unsafe` “a mano” (vedi Step 3)

Aggiornare anche la sezione `map3d` (anche se poi la disabilitiamo), almeno per non rompere tutto:

- `map3d/joint_state_topic: "joint_states"` (relativo)
- `map3d/get_planning_scene_service: "get_planning_scene"` (relativo)

### Step 2 — Creare un launch dual-arm dedicato al MUR620

Creare un launch nel pacchetto (esempio):

- `launch/mur620_dual_cartesian_velocity_controller.launch`

Che:

- assume che il robot sia già lanciato da `single_mur620_gazebo.launch`
- avvia due nodi con namespace sotto `/mur620`
- carica il YAML giusto per ciascun nodo

Esempio di design (concetto, non codice definitivo):

- gruppo `ns="/mur620"`
  - node `cartesian_velocity_node` name `cartesian_velocity_controller_l`
    - rosparam load: `controller_params_mur620_ur10_l.yaml`
  - node `cartesian_velocity_node` name `cartesian_velocity_controller_r`
    - rosparam load: `controller_params_mur620_ur10_r.yaml`

In più:

- opzionale: remap `target_pose` per renderlo esplicito (`target_pose` è già relativo)

### Step 3 — Garantire che i controller “unsafe” siano effettivamente running

Nel bringup, `joint_group_vel_controller_l/unsafe` e `_r/unsafe` sono spawnati **stopped**.

Per l’MVP ci sono 2 strade:

- **A) Procedura manuale / script esistente (consigliata per partire subito)**
  - Usare lo script già presente:
    - `src/match_mobile_robotics/mur/mur_examples/scripts/switch_URs_to_twist_control.py`
  - oppure chiamare `controller_manager/switch_controller` sotto `/mur620`.

- **B) Automatizzare nello stesso launch del controller**
  - Aggiungere un nodo “helper” che faccia lo switch (Python piccolo o call a servizio) dopo un `sleep`.
  - È più comodo ma aggiunge complessità “di orchestration”.

Criticità: MoveIt usa `UR10_l/arm_controller` e `UR10_r/arm_controller`. Se i due controller (traj e vel) vanno in conflitto, serve una policy chiara:

- quando uso velocity control → stop traj controller per quel braccio
- quando uso MoveIt → stop velocity controller per quel braccio

Per l’MVP puoi anche evitare lo switching automatico e gestire i controller “a mano” per capire il flusso.

**Risposta:** Per il momento lo farò manuale. Scrivimi da qualche parte tutti i comandi da terminale per effettuare lo switching dei controller

➡️ Vedi `docs/MUR620_CONTROLLER_SWITCHING_COMMANDS.md`.

### Step 4 — Sistemare (o mettere in pausa) Map3D/repulsione per non bloccare il porting

**Raccomandazione MVP**: disabilitare repulsione e rendere Map3D “innocua” finché non abbiamo il controllo base robusto.

Opzioni:

- **A) Solo parametri (zero modifiche C++)**
  - `local_planner/repulsive_enabled: false`
  - `map3d/update_rate_hz: 0.0` (o molto basso)
  - `map3d/debug/*: false` (tutto off)
  - assicurarsi che `map3d/joint_state_topic` e `get_planning_scene_service` siano corretti (relativi)
  - Nota: anche con update_rate=0, il thread esiste; va visto se `updateLoop` “dorme” correttamente o continua a ciclare.

- **B) Aggiungere una feature flag (minima modifica C++)**
  - introdurre `~map3d/enabled` (default true)
  - se false: non costruire/avviare `Map3DManager` e non creare `RepulsionDataManager` dipendente
  - È la soluzione “pulita” per portabilità e per performance quando avrai 2 istanze.

**Risposta:** Si, voglio la soluzione B che mi sembra più robusta.

### Step 5 — Aggiornare lo script interattivo (necessario per comandare 2 bracci)

Per controllare 2 bracci in parallelo, lo script deve smettere di usare assoluti “globali”.

Minimo indispensabile:

- pubblicare su topic target relativo o parametrizzato:
  - da `'/target_pose'` → `target_pose` (relativo) **oppure** `~target_pose_topic`
- usare servizi controller_manager namespaced:
  - da `'/controller_manager/...'` → `'controller_manager/...'` (relativo) o param `~controller_manager_ns`
- supportare selezione braccio:
  - scegliere `controller_node_name` (per leggere parametri come `global_frame`) per L o R
  - scegliere `ee_frame` corretto per TF (probabile `mur620/UR10_l/tool0` vs `mur620/UR10_r/tool0`)
- MoveItPoseController:
  - cambiare default group da `"manipulator"` a `UR_arm_l`/`UR_arm_r` (o renderlo selezionabile)

Per l’MVP puoi anche **saltare** lo script interattivo e inviare `target_pose` con un publisher “semplice” (anche temporaneo), ma appena vuoi usare l’interfaccia testuale, questi fix diventano obbligatori.

**Risposta:** Vorrei che la portabilità dello script venga fatta

### Step 6 — Test plan (verifica incrementale)

Test in ordine consigliato (per isolare problemi):

1) **Robot base**: lancia `single_mur620_gazebo.launch` e verifica che MoveIt sia su.
2) **Controller velocità running**:
   - verifica che `joint_group_vel_controller_l/unsafe` e `_r/unsafe` possano essere startati
3) **Una sola istanza del controller (solo braccio L)**:
   - controlla che riceva joint_states (niente warning di “RobotState not ready”)
   - controlla che pubblichi su `.../command`
4) **Aggiungi seconda istanza (braccio R)**:
   - verifica che non “rubino” le stesse risorse (topic, controller, target_pose)
5) **Comandi target_pose**:
   - invia target su L e R separatamente
6) **Solo dopo**: riattiva progressivamente repulsione/Map3D.

## Punti aperti / criticità (da risolvere o monitorare)

### 1) Nome esatto di `tcp_link` e frame TF
Molto probabilmente sul MUR620 i link/joint hanno prefisso `UR10_l/` e `UR10_r/`.  
Quindi `tcp_link` potrebbe dover essere:

- `UR10_l/tool0`
- `UR10_r/tool0`

Se `tcp_link` non matcha il modello MoveIt, il controller logga warning tipo “TCP link not found” e non si inizializza correttamente.

### 2) Parametro `robot_description` e RobotModelLoader
Il `RobotStateManager` carica sempre il modello da `"robot_description"`.  
Con il bringup attuale esistono:

- `/mur620/robot_description`
- `/mur620/UR10_l/robot_description`
- `/mur620/UR10_r/robot_description`

Quindi conviene lanciare i due controller in namespace coerenti (es. sotto `/mur620` o direttamente sotto `/mur620/UR10_l` e `/mur620/UR10_r`) e verificare dove il loader lo trova realmente.

Possibile miglioramento di portabilità:

- rendere il nome parametro (`robot_description_param`) configurabile nel controller.

### 3) Controller switching: path dei servizi
Nel C++ e negli script Python i servizi `controller_manager` sono usati come `"/controller_manager/..."`.  
Su MUR620 esistono sotto `"/mur620/controller_manager/..."`.  
Soluzioni:

- parametrizzare il namespace del controller_manager (consigliata) -> va bene, fai questo
- o disabilitare switching per MVP e farlo esternamente

### 4) Performance: due istanze + Map3D duplicato
Due controller ⇒ due loop a 100Hz + (potenzialmente) due Map3D a ~15Hz con voxelizzazione+EDT.  
Prima di pensare a avoidance serio, conviene:

- disabilitare Map3D per MVP
- poi decidere se Map3D deve essere **condivisa** (un solo nodo “map server”) o duplicata.

### 5) Collision avoidance tra bracci
Il bringup ha un launch `dual_arm_collision_avoidance.launch` (attivabile/disattivabile).  
Il tuo controller potrebbe in futuro generare comandi che portano ad auto-collisione (specialmente se muovi entrambi).  
Per MVP: accettabile, ma da mettere in backlog.

**Risposta:** Non so se quel launch file è affidabile, andrebbe verificato. Per il momento ignoralo

## Proposta di backlog dopo l’MVP (solo titolo)

- Unificazione “multi-arm” in un solo processo (architettura multi-controller manager interno).
- Nodo Map3D unico + query via service/topic da entrambi i controller.
- Obstacle avoidance: definire POI per entrambi i bracci e gestire inter-arm collision come “ostacoli dinamici”.
- Integrazione con costmap / navigazione base (quando servirà).

