## Obiettivo

Passare da una simulazione con **un singolo braccio UR** (setup attuale) a una simulazione del **robot completo** composto da:
- **base mobile**
- **due bracci UR**
- (presumibilmente: sensori, TF, controller, MoveIt, navigation/localization)

Vincoli richiesti:
- prima: **portare il robot nel workspace**, avviare la simulazione e verificare che stia in piedi
- poi: capire **come adattare** `cartesian_velocity_controller` al nuovo robot
- per ora: solo analisi e possibili soluzioni (no modifiche/operazioni)

Riferimenti forniti:
- Pacchetto sviluppato: `src/cartesian_velocity_controller`
- Launch “robot attuale” (UR singolo): `src/scene_builder/launch/ur10e_interactive_demo_without_distance_monitor.launch`
- Launch “esempio 4 robot” (MUR): `Match_ws/.../mur_examples/launch/multi_mur620_handling.launch` (non avvia Gazebo)

---

## Problema reale (cosa cambia passando a “robot intero”)

### 1) Architettura ROS più complessa
Nel robot completo entrano in gioco contemporaneamente:
- **Gazebo** + spawn di modello (URDF/xacro) + plugin gazebo
- **controller** (ros_control) per base e bracci, spesso in **namespace**
- **TF tree** più articolato (map/odom/base_link + catene per 2 bracci)
- **MoveIt** (planning scene, controllers, joint state, gruppi di planning) per *due* manipolatori
- **Navigation/localization** per la base (map/odom, laser/pointcloud, amcl/slam, costmap)
- gestione di **nomi** e **namespace** per evitare collisioni (soprattutto se l’esempio spawna 4 robot)

### 2) Riuso del tuo algoritmo
`cartesian_velocity_controller` probabilmente oggi assume:
- un solo `robot_description`
- un solo gruppo / catena cinematica
- topic/frames noti (es. `base_link`, `tool0`, ecc.)
Con due bracci e una base mobile:
- devi decidere *a quale braccio* si applica (o se a entrambi)
- devi gestire **namespace**, **TF**, e possibili differenze nei frame
- devi capire se “base mobile” entra nel controllo (es. inseguimento target con base + braccio)

---

## Quale workspace conviene usare? (strategie possibili)

### Opzione A — Lavorare direttamente in `Match_ws`
**Quando conviene**
- se il robot completo (MUR620 + base + 2 bracci) e navigation sono già “nativi” lì
- se i pacchetti MUR sono mantenuti e funzionanti in quel workspace

**Pro**
- riduci il lavoro di porting del robot
- usi l’infrastruttura già pronta (launch, config, navigation, controller)

**Contro**
- devi importare dentro `Match_ws` il tuo pacchetto `cartesian_velocity_controller` (e dipendenze)
- rischio di “inquinare” due ambienti se fai debug e fix rapidi

**Quando sceglierla**
- se il tuo obiettivo primario è “far partire presto la simulazione del robot intero”.

---

### Opzione B — Lavorare in `Test_ws` importando dentro i pacchetti del robot
**Quando conviene**
- se `Test_ws` è il tuo ambiente di sviluppo principale
- se vuoi mantenere tutto sotto controllo in un singolo workspace

**Pro**
- integrazione diretta col tuo pacchetto e con `scene_builder`
- pipeline di sviluppo più semplice per te

**Contro**
- costo iniziale più alto: portare robot + navigation + config
- maggiore rischio mismatch di dipendenze (versioni, distro ROS, plugin gazebo, moveit config)

**Quando sceglierla**
- se prevedi di lavorare molto sul codice e vuoi un solo workspace “definitivo”.

---

### Opzione C (consigliata in molti casi) — Creare un “workspace di simulazione” overlay
Crea un terzo workspace, ad es. `Sim_ws`, che contiene:
- i pacchetti del robot (da `Match_ws`)
- il tuo pacchetto `cartesian_velocity_controller` (da `Test_ws`)
- eventuali pacchetti ponte/launch personalizzati (minimi)

**Pro**
- separi sviluppo (Test) da stack robot (Match) e crei un ambiente riproducibile
- riduci conflitti e lavori “sporchi”
- ti permette di iterare senza rompere i workspace originali

**Contro**
- un workspace in più da mantenere

**Quando sceglierla**
- quasi sempre, se vuoi un percorso pulito: prima “simulazione funziona”, poi integrazione controller.

---

## Isolare la simulazione su un singolo robot (partendo dal launch “4 robot”)

Hai un launch di esempio che spawna 4 robot e non lancia Gazebo. Tipicamente per isolare 1 robot servono 3 interventi concettuali (senza ancora implementare):

### 1) Gazebo “unico” e spawn separato
- Gazebo va avviato una volta
- lo spawn del robot deve essere separato (o parametrizzato) per scegliere `N=1`

### 2) Namespace del robot
Per ogni robot (anche se uno solo) conviene usare namespace coerente, es:
- `/mur620_1/robot_description`
- `/mur620_1/joint_states`
- `/mur620_1/controller_manager`
- TF: valutare se i frame includono prefisso o se si usa `tf_prefix`/static transforms

Questo torna utilissimo quando più avanti vorrai passare a 2 robot/4 robot.

### 3) Parametri “robot_count” o inclusione condizionale
Molti launch multi-robot hanno:
- argomento `robot_count`
- oppure includono 4 blocchi quasi identici
Soluzioni:
- introdurre un argomento `robot_ids:=1` o `spawn_1:=true spawn_2:=false...`
- creare un launch “wrapper” `single_mur620.launch` che include quello multi ma abilita solo un robot

**Output atteso di questa fase**
- Gazebo parte
- un solo robot appare in simulazione
- TF e joint states pubblicati
- controller della base e dei bracci attivi (o almeno caricabili)

---

## Base mobile: navigazione e localizzazione (cosa serve “minimo”)

Qui dipende se siete ROS 1 o ROS 2; dai `.launch` sembra ROS 1, ma la struttura va verificata.

### Se ROS 1 (tipico)
**Localizzazione**
- `map_server` (mappa statica) + `amcl` (se hai laser)
oppure SLAM (es. `gmapping`/`hector`/`cartographer`)

**Navigazione**
- `move_base` con `global_costmap`/`local_costmap`
- `cmd_vel` collegato al controller della base (diff drive / omni)

**Catena TF minima**
- `map -> odom -> base_link`
- sensori: `base_link -> laser` (o depth, ecc.)

**Cose che spesso bloccano subito**
- frame id non coerenti tra costmap e TF
- `cmd_vel` non arriva al controller corretto (namespace/remap)
- odometria non pubblicata o pubblicata su topic diverso da quello atteso

---

### Se ROS 2 (Nav2)
**Localizzazione**
- `slam_toolbox` o `amcl` di Nav2
**Navigazione**
- stack `nav2_bringup` con lifecycle nodes
**Cose critiche**
- `tf` e `robot_state_publisher`
- `cmd_vel` / `odom` topic remap + QoS

---

## Bracci: “dovrei avere già tutto nell’altro pacchetto” (rischi tipici)

Anche se il braccio UR è già ok in singolo robot, nel robot completo cambiano spesso:
- **nome del gruppo** MoveIt (es. `manipulator_left`, `manipulator_right`)
- **frame base** del braccio (es. attaccato a `base_link` e non più fisso in world)
- **controller names** (due controller, uno per braccio)
- `joint_states` aggregati: o un unico topic con tutti i joint, o due separati

Serve quindi mappare chiaramente:
- quale braccio userà il tuo `cartesian_velocity_controller`
- quali topic e frame userà

---

## Percorso consigliato (a prova di problemi)

### Fase 0 — Decisione workspace (scelta pratica)
- Se l’obiettivo è partire veloce: **Opzione A o C**
- Se vuoi “un solo ws definitivo” e sei disposto a spendere tempo: **Opzione B**

### Fase 1 — Riprodurre la simulazione del robot completo (senza il tuo algoritmo)
- Avvia Gazebo
- Spawna **1 solo** MUR620 (derivando dal launch multi-robot)
- Verifica:
  - `/tf` e catena `map/odom/base_link`
  - `joint_states` presenti
  - i controller sono caricati e non in errore
  - puoi teleoperare la base (anche senza navigation)

### Fase 2 — Portare su navigation/localization
- Porta up minimal (localizzazione + move_base / nav2)
- Verifica goal semplice e che la base si muova correttamente
- Solo dopo: aggiungi sensori/ostacoli realistici (costmap tuning)

### Fase 3 — Integrare MoveIt e poi il tuo `cartesian_velocity_controller`
- Attiva MoveIt per **un braccio** (es. sinistro)
- Verifica planning + execution in simulazione
- Integra `cartesian_velocity_controller`:
  - parametri per scegliere braccio (left/right)
  - remap topic e frame
  - gestione namespace

### Fase 4 — Estensione a due bracci (se serve davvero)
- Decidi se:
  - due istanze del nodo (una per braccio), oppure
  - un nodo unico multi-arm con parametri separati
- Verifica collisioni, planning scene, sincronizzazione

---

## Soluzioni concrete per “single robot” (pattern di launch)

Senza entrare nel codice, i pattern più robusti sono:

- **Pattern 1: wrapper launch**
  - `gazebo.launch` (solo Gazebo)
  - `spawn_single_mur.launch` (solo spawn + robot_state_publisher)
  - `controllers.launch` (solo controller)
  - `moveit_left.launch` (solo MoveIt per braccio sinistro)
  - `nav_bringup.launch` (localizzazione + navigation)
  - `cartesian_velocity_controller.launch` (il tuo nodo)

- **Pattern 2: parametric multi-robot**
  - mantieni il launch multi-robot ma aggiungi arg `robot_count:=1`
  - pro: stesso file scala 1->4
  - contro: spesso è più difficile da mantenere se il file nasce “copia-incolla 4 volte”

---

## Domande chiave (da risolvere presto, perché guidano tutto)

1) ROS distro e versione (ROS1/ROS2)?
2) Il robot MUR620 usa MoveIt “per robot completo” o config separati per bracci?
3) `joint_states`: unico topic o per-namespace?
4) La base è diff-drive o omnidirezionale? (influenza controller + nav params)
5) Sensori per nav: laser 2D, depth, 3D lidar? (influenza localizzazione e costmap)

---

## Output atteso per “iniziare a lavorare” (definizione di Done della prima milestone)

La prima milestone è completata quando:
- Gazebo avvia
- un singolo MUR620 spawnato correttamente
- TF stabile, joint states ok
- base teleoperabile
- (opzionale) navigation porta il robot a un goal in mappa semplice

Solo dopo questa milestone conviene iniziare ad adattare `cartesian_velocity_controller`.

---