# Analisi pacchetto e punti aperti per obstacle avoidance / stampa 3D

Data: 2026-04-27

Questo documento e' una prima analisi del pacchetto `cartesian_velocity_controller` rispetto alle specifiche in `spec/2026_04_27_Specifiche.md`.

Fonte usata: codice, launch, config, messaggi e servizi del pacchetto. Non sono stati usati gli altri file markdown, perche' possono essere obsoleti.

## Obiettivo richiesto dalle specifiche

Il nuovo scenario e' diverso dallo stato attuale del pacchetto:

- robot reale `mur620d`;
- base mobile MiR600 a sterzo differenziale;
- un solo braccio UR10e;
- lifter come asse verticale aggiuntivo controllabile in posizione, non in velocita';
- tool fittizio sul polso, con ugello sempre verticale;
- TCP che segue una traiettoria planare a quota tendenzialmente costante, ma con `z` eventualmente variabile;
- velocita' TCP costante lungo il percorso;
- whole-body control: base, lifter e braccio devono cooperare;
- obstacle avoidance su base e struttura alta;
- laser scanner MiR per ostacoli bassi che minacciano la base;
- ostacoli simulati nella griglia per la parte alta del robot;
- compensazione della dinamica del lifter.

## Stato attuale del pacchetto

Il pacchetto implementa oggi un controller cartesiano a velocita' per un gruppo MoveIt. La pipeline principale e':

1. `GlobalPlanner`: gestione di waypoint/target.
2. `LocalPlanner`: campo attrattivo verso il waypoint, repulsione, integrazione del virtual target.
3. `CartesianVelocityFilter`: filtro cartesiano su velocita', accelerazione e jerk.
4. PID cartesiano con feed-forward.
5. inversa dello Jacobiano pesata e smorzata.
6. filtri/limiti in spazio giunti.
7. pubblicazione di un `std_msgs/Float64MultiArray` verso il controller di velocita' dei giunti.

File principali:

- `src/cartesian_velocity_controller.cpp`: orchestratore e loop di controllo.
- `src/components/robot_state_manager.cpp`: stato robot, FK, IK, Jacobiano via MoveIt.
- `src/components/local_planner.cpp`: attrazione, repulsione, virtual target leash.
- `src/components/global_planner.cpp`: avanzamento waypoint.
- `src/map3d/map3d_manager.cpp`: griglia 3D/EDT basata su PlanningScene.
- `src/components/repulsion_data_manager.cpp`: bridge tra Map3D e POI robot.
- `config/controller_params_mur620_ur10_l.yaml`: configurazione MUR simulazione/braccio sinistro.
- `config/controller_params_hw_mur620b_ur10_l.yaml`: configurazione hardware MUR620b/braccio sinistro.

## Analisi launch hardware mur620d

Il launch hardware del robot e' `mur/mur_launch_hardware/launch/mur620d.launch`.

Configurazione di default rilevante:

- `tf_prefix: mur620d`;
- `launch_mir: true`;
- `launch_ur_l: false`;
- `launch_ur_r: true`;
- `left_arm_group_name: UR10_l`;
- `right_arm_group_name: UR10_r`;
- `localization_type: robot_pose`;
- include `general_mur600.launch`;
- passa esplicitamente `use_lift: true`.

Quindi, con i default attuali di `mur620d.launch`, il sistema avvia la base MiR e il braccio destro. Il braccio sinistro non parte, ma il modello e il launch generale supportano anche il lifter sinistro.

### Identificazione lifter nel modello

Il robot description viene caricato in `general_mur600.launch` dentro il namespace `mur620d` tramite:

```text
robot_description = mur_description/urdf/mur_620.gazebo.xacro
```

con argomento:

```text
use_lift:=true
```

Nel file `mur/mur_description/urdf/mur_620.gazebo.xacro`, quando `use_lift` e' true, viene chiamato:

```xml
<xacro:top_lift/>
```

e i bracci vengono montati sopra i link mobili del lifter:

```text
left_lift_top  -> UR10_l/base_ideal
right_lift_top -> UR10_r/base_ideal
```

La macro effettiva del lifter e' in:

```text
mir/mir_description/urdf/mir_600/mir_600_top_lift.urdf.xacro
```

Joint identificati:

```text
left_lift_joint
right_lift_joint
```

Entrambi sono joint prismatici:

```text
type: prismatic
axis: [0, 0, 1]
lower: 0.0
upper: 0.5
velocity limit dichiarato in URDF: 1.0
parent/child sinistro: left_lift_bottom -> left_lift_top
parent/child destro:   right_lift_bottom -> right_lift_top
```

Per il caso default `mur620d.launch`, il lifter operativo associato al braccio avviato e':

```text
joint:        right_lift_joint
bottom frame: mur620d/right_lift_bottom
top frame:    mur620d/right_lift_top
asse:         z
range:        0.0 - 0.5 m
```

### Driver hardware lifter

Il lifter e' avviato in `general_mur600.launch` tramite nodi del pacchetto `ewellix_tlt`.

Per il lato sinistro:

```text
node:  /mur620d/UR10_l/ewellix_tlt_node_l
pkg:   ewellix_tlt
type:  ewellix_tlt_node
port:  /dev/ttyUSB0
joint: left_lift_joint
```

Per il lato destro:

```text
node:  /mur620d/UR10_r/ewellix_tlt_node_r
pkg:   ewellix_tlt
type:  ewellix_tlt_node
port:  /dev/ttyUSB1
joint: right_lift_joint
```

Nel workspace la directory `mir/mir_submodules/ewellix_tlt` risulta presente ma vuota/non ispezionabile. Quindi dal codice disponibile posso identificare nome joint, frame e topic di feedback, ma non posso ancora confermare l'interfaccia di comando effettiva del nodo `ewellix_tlt_node`.

### Feedback joint state del lifter

`general_mur600.launch` avvia `joint_state_aggregator.py` sotto namespace `mur620d`.

Topic di input configurati:

```text
/mur620d/joint_states_mir
/mur620d/UR10_l/joint_states
/mur620d/UR10_r/joint_states
/mur620d/UR10_l/ewellix_tlt_node_l/joint_states_lift
/mur620d/UR10_r/ewellix_tlt_node_r/joint_states_lift
```

Topic di output:

```text
/mur620d/joint_states
```

Questo significa che, per il controller futuro, la posizione del lifter dovrebbe essere leggibile dal joint state aggregato:

```text
/mur620d/joint_states
```

cercando il nome:

```text
right_lift_joint
```

nel caso default `mur620d`.

### Lifter in MoveIt/SRDF

Nel file `mur/mur_moveit_config/config/mur620.srdf.xacro` esistono gruppi MoveIt che includono il lifter:

```text
UR_arm_lift_l: left_lift_joint + UR10_l
UR_arm_lift_r: right_lift_joint + UR10_r
UR_arm_all:    left_lift_joint + UR10_l + right_lift_joint + UR10_r
```

I gruppi `UR_arm_l` e `UR_arm_r`, invece, non includono il lifter.

Nel file `mur/mur_moveit_config/config/controllers620.yaml` sono presenti anche controller MoveIt separati:

```text
UR10_l/lift_controller -> left_lift_joint
UR10_r/lift_controller -> right_lift_joint
```

Nota: queste configurazioni MoveIt dichiarano action `FollowJointTrajectory` per il lifter, ma questo non dimostra da solo che sul robot reale il comando passi davvero da questi controller. Nel launch hardware visto, il lifter reale e' avviato tramite `ewellix_tlt_node`.

### Implicazioni per il controller futuro

Per il `mur620d` default, la prima integrazione dovrebbe considerare:

```text
arm side:      right
arm group:     UR_arm_r oppure, se si vuole includere il lifter nel modello MoveIt, UR_arm_lift_r
arm namespace: /mur620d/UR10_r
lifter joint:  right_lift_joint
lifter state:  /mur620d/joint_states
lifter frames: mur620d/right_lift_bottom -> mur620d/right_lift_top
```

Punti ancora da verificare sul robot acceso:

1. topic/action/servizio reale per comandare il lifter `ewellix_tlt_node`;
2. se il comando e' posizione, velocita' o un comando proprietario;
3. se `right_lift_joint` compare sempre in `/mur620d/joint_states`;
4. frequenza e latenza del feedback lifter;
5. segno e scala della posizione: confermare che `+z` e' salita e che il range reale e' 0.0-0.5 m;
6. se il controller MoveIt `UR10_r/lift_controller` e' effettivamente disponibile o solo configurato semanticamente.

## Cosa funziona gia' o e' riutilizzabile

### Controllo cartesiano del braccio

Il controller puo' ricevere una posa target su `~target_pose`, trasformarla nel `global_frame`, fare controllo cartesiano e generare velocita' di giunto per il gruppo MoveIt configurato. La parte PID + Jacobiano + limitatori e' gia' abbastanza strutturata.

Questo e' utile come nucleo iniziale per muovere l'UR10e lungo target intermedi.

### TCP offset/tool fittizio

Il parametro `tcp_offset_position` e `tcp_offset_orientation_rpy` e' gia' presente. Il TCP effettivo viene calcolato come:

```text
T_world_tcp = T_world_tcp_link * tcp_offset
```

Quindi l'idea del tool fittizio puo' essere integrata senza stravolgere la FK/IK, purche' siano chiari frame e orientamento dell'ugello.

### Orientamento target

Il sistema controlla anche l'orientamento tramite errore asse-angolo. Per mantenere l'ugello verticale si puo' imporre una policy di orientamento nella generazione della traiettoria, senza necessariamente cambiare subito il PID.

### Waypoint multipli

`GlobalPlanner` supporta waypoint multipli in C++, con avanzamento automatico in base a soglie di distanza e orientamento. Questo e' una base minima per seguire un percorso discretizzato.

Criticita': l'interfaccia ROS esposta oggi e' soprattutto `PoseStamped` singolo su `~target_pose`; non c'e' ancora un action/server/topic esplicito per caricare ed eseguire una traiettoria completa con velocita' costante, pause, stato di avanzamento, abort, resume, ecc.

### Filtri e limitatori

Sono presenti:

- filtro cartesiano su velocita', accelerazione e jerk;
- filtro in spazio giunti;
- safety limiter velocita'/accelerazione giunti;
- guardrail opzionale sui limiti di posizione;
- dynamic reconfigure.

Questa parte e' utile per prove progressive sul robot reale.

### Rotazione dello Jacobiano

La rotazione opzionale dello Jacobiano da `jacobian_source_frame` a `jacobian_target_frame` e' implementata come trasformazione rotazionale 6x6 cacheata:

```text
J_target = X_rot * J_source
```

Le configurazioni MUR la usano per esprimere lo Jacobiano nel frame base robot. Se oggi e' funzionante, la lascerei come modulo esistente. Va solo tenuto presente che e' una rotazione pura: non applica la trasformazione completa di adjoint con termini di traslazione tra frame.

### Map3D e repulsione per parte alta

Esiste una Map3D locale con EDT, query di distanza e gradiente, debug RViz e servizio `~map3d/query`. La sorgente ostacoli e' MoveIt PlanningScene, letta come primitive sferiche.

`RepulsionDataManager` puo' interrogare la mappa per POI definiti sui link del robot e produrre:

- repulsione TCP come `ObstacleInfo`;
- repulsione su link/POI come `LinkPOI`.

Pero' nelle configurazioni MUR analizzate `map3d/enabled: false` e `local_planner/repulsive_enabled: false`, quindi questa parte e' presente ma non attiva nel setup MUR attuale.

## Cosa manca rispetto alle specifiche

### Whole-body control reale

Il controller attuale lavora su un singolo `JointModelGroup` MoveIt e pubblica una singola lista di velocita' di giunto.

Non c'e' ancora un modello di controllo con:

- DOF base differenziale;
- DOF lifter;
- DOF braccio;
- mappatura `qdot_whole_body -> twist_tcp`;
- separazione tra giunti comandabili in velocita' e asse lifter comandabile in posizione;
- pubblicazione simultanea verso base, lifter e braccio.

Questa e' la criticita' principale.

### Base mobile MiR600

Nel codice non ci sono subscriber/publisher specifici per:

- `cmd_vel`;
- odometria base;
- stato base;
- laser scanner;
- footprint/collision della base;
- vincolo non olonomo differenziale.

La base oggi entra solo indirettamente come frame `global_frame` o come eventuale parte dell'URDF/TF. Non e' un grado di liberta' controllato dal controller.

### Lifter

Il lifter e' citato nelle specifiche ma non nel codice del pacchetto. Non esiste:

- interfaccia di comando posizione lifter;
- stato lifter esplicito;
- modello dinamico/ritardo;
- logica per decidere quando muoverlo;
- compensazione che faccia inseguire al braccio/base il moto effettivo del lifter.

Il punto importante e' che il lifter non puo' essere trattato come un giunto qualsiasi in velocita'. Va modellato come sottosistema lento e dominante: si comanda posizione/target, si misura o stima la risposta, e il resto della catena si adatta.

### Traiettoria TCP a velocita' costante

Oggi il controller accetta target/waypoint, ma non genera una legge di moto lungo una curva con parametrizzazione ad arco.

Per la stampa serve invece un livello superiore:

- definizione percorso: polilinea, spline, G-code semplificato, CSV, YAML o altro formato;
- parametrizzazione per lunghezza d'arco;
- generazione setpoint a velocita' costante;
- gestione `z(s)` eventualmente variabile;
- orientamento utensile verticale lungo tutto il path;
- gestione inizio/fine, stop, resume, pause, feed override;
- stato di avanzamento lungo percorso.

Con i soli waypoint, la velocita' non e' garantita costante e puo' dipendere da errore, filtri, PID, soglie di switch e limiti.

### Obstacle avoidance integrato con task primario

La repulsione attuale modifica il virtual target o aggiunge contributi da POI. Questo e' un approccio da potenziale locale, utile per prototipare, ma per il requisito "il TCP deve seguire comunque la traiettoria" serve definire priorita' e vincoli:

- il task TCP deve restare primario o puo' deviare?
- la deviazione e' ammessa solo per corpo/base e non per TCP?
- se un ostacolo rende impossibile rispettare la traiettoria, il sistema rallenta, ferma, replanifica o devia?
- come si recupera il punto corretto sulla traiettoria dopo avoidance?

Senza questa policy, la repulsione puo' far perdere la fedelta' di stampa.

### Laser scanner MiR

Non esiste ancora ingestione `sensor_msgs/LaserScan` o `PointCloud2` per i laser della base. Map3D usa PlanningScene, non laser reali.

Serve decidere se i laser entrano:

- in una mappa 2D separata per base/footprint;
- nella Map3D come ostacoli bassi estrusi;
- in entrambe, con ruoli diversi.

Per la base, probabilmente conviene un avoidance 2D dedicato alla footprint e al vincolo differenziale, separato dalla Map3D per braccio/lifter.

### Mur620d

Nel pacchetto ci sono configurazioni e launch per MUR620/MUR620b, dual arm e single arm. Non ho trovato configurazione esplicita `mur620d`.

Andranno verificati:

- namespace reale;
- nomi frame TF;
- nome gruppo MoveIt whole-body o arm-only;
- topic comando base;
- topic comando lifter;
- topic velocita' braccio;
- topic laser;
- controller disponibili;
- URDF/SRDF effettivi.

## Criticita' tecniche individuate

### 1. Architettura ancora arm-centric

Il codice e' ben modulare, ma il centro e' ancora "un gruppo MoveIt -> velocita' giunti". Il whole-body control richiede un cambio di astrazione: non basta aggiungere qualche joint al vettore se base e lifter hanno interfacce e vincoli diversi.

Possibile direzione: introdurre uno strato `WholeBodyController` sopra o al posto dell'attuale conversione finale PID/Jacobiano, con sottocomponenti:

- `RobotKinematicsModel`: calcolo FK/Jacobiano esteso;
- `TaskManager`: task TCP, postura, avoidance, limiti;
- `WholeBodySolver`: QP o weighted least squares;
- `CommandDispatcher`: invio comandi a base, braccio, lifter.

### 2. Lifter non velocitario

Il lifter rompe l'assunzione "il solver calcola qdot e pubblico qdot". Se il lifter viene incluso nel modello cinematico come asse verticale, il comando non puo' essere una velocita' diretta come per l'UR.

Possibili strategie da discutere:

- trattare il lifter come asse quasi-statico comandato da un supervisore;
- usarlo per riallocazione lenta di postura, non per tracking veloce;
- stimare `z_lifter(t)` reale e sottrarlo/compensarlo nel task TCP;
- generare target posizione lifter con rate limit e usare base/braccio per inseguire la dinamica reale.

### 3. Base differenziale e vincolo non olonomo

Una base differenziale non puo' generare direttamente velocita' laterale. Il solver whole-body deve rispettare:

```text
v_y_base = 0 nel frame base
```

e comandare solo:

```text
v_x_base, omega_z_base
```

Questo cambia lo Jacobiano esteso e la strategia di tracking. Per una traiettoria TCP planare, la base dovra' orientarsi e avanzare in modo coordinato, mentre il braccio assorbe le componenti trasversali/locali.

### 4. Fedelta' del path di stampa contro avoidance

La specifica richiede avoidance ma anche task primario di stampa. Questi due obiettivi possono confliggere.

Serve stabilire una gerarchia:

- sicurezza sempre prioritaria;
- TCP sul path come task principale in condizioni nominali;
- avoidance base/struttura come vincolo o task secondario;
- rallentamento/stop quando non esiste soluzione sicura che mantenga il path.

Per stampa 3D simulata, una deviazione laterale del TCP puo' essere inaccettabile. In quel caso l'obstacle avoidance dovrebbe muovere soprattutto la ridondanza, non spostare il TCP dalla curva.

### 5. Repulsione attuale basata su potenziali locali

Il `LocalPlanner` somma attrazione e repulsione e integra un virtual target. Questo puo' funzionare per test semplici, ma ha rischi noti:

- minimi locali;
- oscillazioni vicino agli ostacoli;
- perdita della traiettoria nominale;
- dipendenza forte dai gain;
- comportamento poco prevedibile vicino a vincoli cinematici.

Se il requisito e' robusto su robot reale, andrebbe valutato un solver vincolato o gerarchico, almeno per il livello whole-body.

### 6. Map3D non integra ancora i laser

Map3D legge ostacoli da PlanningScene, soprattutto primitive sferiche. Non e' una mappa multisensore. Per i laser MiR serve progettare:

- trasformazione dei punti laser nel frame mappa;
- filtro altezza/rumore;
- persistenza temporale;
- clearing o decadimento;
- inflazione footprint base;
- sincronizzazione con movimento base.

### 7. Frame e radici MoveIt/TF

Il codice ha gia' logica per trasformare target e ruotare Jacobiano, piu' servizio `~get_frame_info`. Questo indica che i frame sono stati un punto delicato.

Per whole-body e base mobile il problema cresce:

- il root MoveIt potrebbe essere `base_link`, `world`, `map` o un frame prefissato;
- la base mobile cambia posa nel mondo;
- il task di stampa potrebbe essere espresso in `map`, `odom`, `base_link`, frame pezzo o frame tavolo;
- la Map3D ha un proprio `frame_id`.

Serve decidere un frame canonico per: path, controllo, mappa, laser, debug.

### 8. Raggiungibilita' IK su target statici

L'attuale reachability check usa IK MoveIt sul singolo gruppo. Con base e lifter mobili, un target non raggiungibile dal solo braccio puo' diventare raggiungibile col whole-body.

Quindi la logica di reject target/waypoint andra' ripensata:

- check arm-only per modalita' legacy;
- check whole-body per nuovo sistema;
- oppure non reject immediato, ma demandare al solver con gestione dello stato `blocked/unreachable`.

### 9. Interfaccia traiettoria non definita

La specifica dice "definire un percorso", ma non definisce formato e sorgente.

Punti da scegliere:

- YAML/CSV per primi test;
- G-code subset;
- ROS action per esecuzione path;
- topic streaming di setpoint;
- servizio per caricare path;
- RViz marker/interactive marker;
- generatore interno di primitive: linea, arco, layer.

### 10. Test e validazione

Non ho visto test automatici nel pacchetto. Per portare il sistema su robot reale, conviene aggiungere almeno test unitari/offline per:

- parametrizzazione traiettoria;
- vincoli velocita' costante;
- solver cinematico whole-body;
- saturazioni comando;
- trasformazioni frame;
- logica lifter;
- conversione laser -> ostacoli.

## Proposta di decomposizione per la discussione successiva

### Fase A - Chiarire modello robot reale

Da definire:

- nomi topic base MiR: comando, odom, stato;
- tipo comando base accettato;
- topic e tipo comando lifter;
- feedback lifter disponibile;
- nomi laser e frame laser;
- URDF/SRDF mur620d effettivo;
- gruppo MoveIt disponibile: solo arm o whole-body;
- frame in cui e' definito il path di stampa.

Output atteso: tabella interfacce reali e frame tree minimo.

### Fase B - Trajectory manager per TCP

Creare un modulo dedicato che produce a ogni ciclo:

```text
pose_des(s), twist_des(s), progress, stato esecuzione
```

Requisiti:

- velocita' TCP costante lungo ascissa curvilinea;
- supporto path 2D con `z` costante o variabile;
- orientamento ugello verticale;
- gestione stop/pause/resume;
- markers RViz;
- interfaccia ROS stabile.

Questa fase puo' essere fatta anche prima del whole-body, usando ancora il braccio per testare il tracking.

### Fase C - Whole-body kinematics

Costruire il modello cinematico esteso:

```text
q_wb = [base_x_like, base_yaw_like, lifter_z, arm_1..arm_6]
```

Per la base differenziale non si deve introdurre una velocita' laterale comandabile. Meglio lavorare direttamente con input:

```text
u = [v_base_x, omega_base_z, v_lifter_virtual, qdot_arm_1..6]
```

anche se `v_lifter_virtual` poi non viene comandato direttamente, ma convertito in target posizione filtrato.

Output atteso: Jacobiano esteso del TCP rispetto agli input comandabili/virtuali.

### Fase D - Solver di controllo

Possibili livelli:

1. weighted least squares semplice;
2. null-space gerarchico;
3. QP con vincoli di velocita', limiti, avoidance e non olonomia.

Per il robot reale e i conflitti avoidance/stampa, un QP e' probabilmente piu' pulito nel medio periodo. Per un MVP, un least-squares pesato con saturazioni e task secondari puo' bastare.

Task candidati:

- primario: tracking TCP su traiettoria;
- vincolo: limiti base/braccio/lifter;
- secondario: postura braccio comoda;
- secondario: centrare lifter/range;
- avoidance: vincoli di distanza o velocita' repulsiva proiettata.

### Fase E - Lifter supervisor

Separare il lifter dal solver veloce:

- il solver calcola una richiesta lenta o una quota desiderata;
- un supervisore filtra/rate-limita il target posizione;
- il loop usa la posizione lifter misurata per la FK effettiva;
- base e braccio compensano il transitorio reale.

Da discutere: se il lifter deve muoversi durante la "stampa" o solo tra segmenti/layer.

### Fase F - Obstacle avoidance basso/alto

Separare due mondi:

- ostacoli bassi/base: laser MiR, footprint 2D, vincolo su comando base;
- ostacoli alti/struttura: Map3D/PlanningScene/ostacoli simulati, POI braccio/lifter/tool.

Possibile architettura:

```text
LaserScan -> BaseObstacleLayer2D -> vincoli base / slowdown / stop
PlanningScene/simulati -> Map3D -> POI distances -> vincoli o task avoidance whole-body
```

### Fase G - Command dispatcher

Modulo finale che pubblica verso interfacce eterogenee:

- braccio: velocita' giunti;
- base: `cmd_vel` o topic specifico MiR;
- lifter: target posizione;
- eventuali zero/safe stop coordinati.

Deve gestire timeout e stop sicuro in modo unitario.

## Domande aperte per procedere

1. Qual e' il frame in cui vuoi definire il percorso di stampa: `map`, `odom`, `mur620d/base_link`, frame pezzo/tavolo o altro?
2. Il TCP deve rimanere esattamente sulla traiettoria anche durante avoidance, oppure sono ammesse deviazioni controllate?
3. Se un ostacolo rende impossibile procedere mantenendo il path, il comportamento desiderato e': stop, rallentamento, replan, deviazione, attesa?
4. Il lifter deve muoversi durante la stampa o puo' essere usato solo per preparazione/cambio quota/layer?
5. Che interfaccia ha il lifter reale: topic tipo `Float64`, `FollowJointTrajectory`, action proprietaria, servizio?
6. Il feedback posizione lifter e' affidabile e pubblicato in `/joint_states`?
7. Il comando base MiR disponibile e' un normale `geometry_msgs/Twist`? In quale frame viene interpretato?
8. I laser scanner MiR pubblicano `LaserScan`, `PointCloud2` o altro? Quali topic/frame?
9. Gli ostacoli simulati per la parte alta saranno inseriti in PlanningScene, topic custom o file/config?
10. Vuoi mantenere MoveIt come modello cinematico principale o preferisci introdurre un modello custom per whole-body?
11. Il percorso di stampa arrivera' da file, GUI/RViz, script Python, G-code o generatore interno?
12. Quale velocita' TCP e quali accelerazioni massime vuoi imporre nella simulazione di stampa?
13. Serve sincronizzare "estrusione simulata" con avanzamento TCP o per ora basta il moto?
14. Il processo usa sempre un solo braccio specifico o deve essere parametrico left/right?
15. Quanto e' importante mantenere il vecchio controller arm-only mentre sviluppiamo il nuovo?

## Decisioni preliminari suggerite

1. Non forzare retrocompatibilita' totale: creare un percorso nuovo `whole_body` e mantenere il controller attuale come baseline di test.
2. Lasciare la rotazione dello Jacobiano esistente finche' non ostacola il nuovo modello.
3. Non infilare base e lifter direttamente nel `Float64MultiArray` attuale: servira' un `CommandDispatcher`.
4. Creare prima il `TrajectoryManager`, perche' definisce il task primario e rende misurabile l'errore di tracking.
5. Tenere obstacle avoidance e tracking path come problemi separati nel design, poi combinarli nel solver.
6. Trattare il lifter come sottosistema lento a posizione, non come settimo asse velocitario normale.
7. Separare avoidance base 2D da avoidance struttura alta 3D.

## Possibile roadmap MVP

### MVP 1 - Tracking path arm-only

- Aggiungere caricamento path semplice.
- Generare setpoint a velocita' costante.
- Pubblicare target pose sequenziali o integrare direttamente il setpoint nel controller.
- Vincolare orientamento ugello verticale.
- Verificare tracking con UR10e solo.

Valore: valida tool, frame, traiettoria e velocita' TCP senza complessita' base/lifter.

### MVP 2 - Base comandata separatamente

- Aggiungere interfaccia base.
- Definire una strategia semplice di riallineamento base rispetto al TCP/path.
- Pubblicare `cmd_vel` coordinato con braccio, ma senza solver QP completo.

Valore: prime prove di sinergia base-braccio.

### MVP 3 - Lifter supervisor

- Leggere stato lifter.
- Comandare target posizione filtrato.
- Compensare nel tracking TCP usando la posa reale.

Valore: gestione del settimo asse lento senza rompere la sicurezza.

### MVP 4 - Avoidance base laser

- Integrare laser scanner.
- Calcolare distanze footprint/ostacoli bassi.
- Implementare slowdown/stop o vincoli su `cmd_vel`.

Valore: sicurezza base su robot reale.

### MVP 5 - Avoidance struttura alta

- Riattivare Map3D/POI con ostacoli simulati.
- Validare distanze e marker RViz.
- Portare repulsione dentro solver whole-body o gerarchia task.

Valore: avoidance del braccio/tool/lifter coerente col task.

## Nota finale

Il pacchetto attuale e' una buona base per:

- controllo cartesiano del braccio;
- filtri e debug;
- gestione target;
- primi esperimenti di repulsione 3D.

Non e' ancora un whole-body controller. La parte nuova piu' importante non e' una singola funzione, ma una nuova architettura attorno a traiettoria TCP, base, lifter, solver e dispatcher comandi. La cosa positiva e' che molte parti esistenti possono diventare sottosistemi riutilizzabili invece di essere riscritte subito.
