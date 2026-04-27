# Stato MVP demo stampa 3D

Data: 2026-04-27

## Decisioni operative

- Robot target: `mur620d`.
- Braccio usato per ora: sinistro.
- Lifter: escluso dal primo MVP, da aggiungere dopo.
- Frame del percorso: configurabile, tipicamente `map`, `odom` o un frame statico rispetto a `map`.
- Formato percorso: elenco semplice di coordinate 3D.
- Obstacle avoidance: per ora approccio demo con slowdown/stop futuro; niente deviazione automatica del TCP.
- Base mobile: comando su `cmd_vel` sotto namespace robot, quindi per `mur620d` il topic atteso e' `/mur620d/cmd_vel`.

## Cosa e' stato implementato

### Whole-body print controller C++

Nuovo nodo:

```text
src/whole_body_print_controller_node.cpp
```

Funzioni:

- genera setpoint TCP lungo una polilinea a velocita' costante;
- pubblica il target TCP verso il controller cartesiano arm-only esistente;
- calcola un contributo base + lifter con least-squares pesato e regolarizzato;
- non usa QP;
- gestisce la base differenziale con soli due comandi: `linear.x` e `angular.z`;
- non genera mai `linear.y` per la base;
- usa una allocazione per zone tra base e braccio;
- usa il braccio come compensatore fine quando il target e' nella zona utile;
- espone servizi `pause`, `resume`, `restart`, `stop`;
- pubblica marker RViz del path e del setpoint corrente;
- pubblica debug strutturato su `~debug`.

Il solver usa una matrice cinematica demo:

```text
u = [v_base_x, omega_base_z, v_lifter_virtual]
```

con colonne:

- `v_base_x`: asse x della base espresso nel frame path;
- `omega_base_z`: contributo planare `z x (tcp - base)`;
- `v_lifter_virtual`: asse z verticale.

La parte null-space non e' gerarchica pura, ma e' approssimata con regolarizzazione verso obiettivi secondari:

- tenere il TCP in una zona comoda davanti alla base;
- ridurre errore laterale ruotando la base;
- allineare lentamente la base alla direzione del path;
- muovere il lifter lentamente e solo se abilitato.

Allocazione base/braccio:

- la posizione desiderata del target rispetto alla base e' definita nel frame `base/base_frame`, solidale con il robot;
- i parametri `base/preferred_tcp_x` e `base/preferred_tcp_y` sono quindi espressi in `mur620d/base_link` nel setup default;
- `x` e' avanti/dietro rispetto alla base;
- `y` e' laterale sinistra/destra rispetto alla base;
- se il target e' lontano dalla zona preferita, la base fa il posizionamento grossolano e il target inviato al braccio viene limitato/tenuto vicino alla posa corrente;
- quando il target entra nella zona utile, il braccio insegue il target completo;
- se il target si muove, la base usa un target filtrato piu' lento mentre il braccio usa il target istantaneo quando e' in zona.

Debug whole-body:

```text
/mur620d/whole_body_print_controller/debug
```

Messaggio:

```text
cartesian_velocity_controller/WholeBodyPrintDebug
```

Contiene stato, progress path, target TCP, target inviato al braccio, TCP corrente, target espresso nel frame base, scala tracking braccio, comando base, saturazioni e stato lifter.

### Config whole-body

Nuovo file:

```text
config/whole_body_print_demo.yaml
```

Contiene path, tracking, solver, parametri base e parametri lifter.

### Launch whole-body

Nuovo launch:

```text
launch/mur620_whole_body_print_mvp.launch
```

Avvia:

- controller cartesiano single-arm esistente;
- nuovo nodo C++ `whole_body_print_controller`;
- default `arm:=left`;
- default `mur_ns:=mur620d`;
- default `base_enabled:=true`;
- default `lifter_enabled:=false`.

### Trajectory manager TCP

Nuovo nodo:

```text
scripts/tcp_path_trajectory_manager.py
```

Funzioni:

- legge un path YAML o CSV;
- parametrizza una polilinea 3D per lunghezza d'arco;
- pubblica setpoint `geometry_msgs/PoseStamped` a velocita' costante circa;
- mantiene orientamento TCP costante da `orientation_rpy` o quaternione;
- pubblica marker RViz del path e del setpoint corrente;
- pubblica avanzamento su `~progress`;
- espone servizi `pause`, `resume`, `restart`, `stop`.

### File demo path

Nuovo file:

```text
config/print_path_demo.yaml
```

Contiene un rettangolo semplice in quota costante, configurabile in `map`.

### Launch MVP

Nuovo launch:

```text
launch/mur620_print_path_mvp.launch
```

Avvia:

- controller cartesiano single-arm esistente;
- trajectory manager TCP;
- default `arm:=left`;
- default `mur_ns:=mur620d`;
- default `start_paused:=true`.

### C-light base mobile

Nel trajectory manager e' stato aggiunto un controllo base minimale opzionale.

Quando `base_control_enabled:=true`, il nodo:

- usa TF per trasformare il setpoint TCP dal frame path al frame base;
- cerca di mantenere il setpoint TCP in una posizione comoda davanti alla base;
- pubblica `geometry_msgs/Twist` su `cmd_vel` relativo al namespace;
- rispetta limiti bassi configurabili su velocita' lineare e angolare;
- pubblica zero su pausa, stop, fine path o errore TF.

Questo non e' un whole-body controller vero: e' solo una strategia demo base-braccio.

## Esempi uso

Solo braccio:

```bash
roslaunch cartesian_velocity_controller mur620_print_path_mvp.launch arm:=left mur_ns:=mur620d path_frame:=map
```

Braccio + base C-light:

```bash
roslaunch cartesian_velocity_controller mur620_print_path_mvp.launch arm:=left mur_ns:=mur620d path_frame:=map base_control_enabled:=true
```

Whole-body C++ braccio + base:

```bash
roslaunch cartesian_velocity_controller mur620_whole_body_print_mvp.launch arm:=left mur_ns:=mur620d path_frame:=map
```

Whole-body C++ con lifter abilitato:

```bash
roslaunch cartesian_velocity_controller mur620_whole_body_print_mvp.launch arm:=left mur_ns:=mur620d path_frame:=map lifter_enabled:=true lifter_command_topic:=<topic_reale_lifter>
```

Avvio traiettoria:

```bash
rosservice call /mur620d/whole_body_print_controller/resume "{}"
```

Pausa:

```bash
rosservice call /mur620d/whole_body_print_controller/pause "{}"
```

## Limiti noti

- La velocita' costante e' sul setpoint pubblicato, non garantita sul TCP reale.
- Il controller base non conosce limiti cinematici completi, ostacoli o footprint.
- Il TCP puo' avere errore di inseguimento se il braccio satura o la base si muove troppo lentamente.
- Non c'e' ancora gestione lifter.
- Non c'e' ancora obstacle avoidance laser integrata.
- Il lifter e' implementato come supervisor a posizione con topic `std_msgs/Float64`, ma il topic reale va verificato sul robot.
- Il solver whole-body e' cinematico/demo: niente QP, niente vincoli duri, saturazioni applicate dopo il least-squares.
