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

Avvio traiettoria:

```bash
rosservice call /mur620d/tcp_path_trajectory_manager/resume "{}"
```

Pausa:

```bash
rosservice call /mur620d/tcp_path_trajectory_manager/pause "{}"
```

## Limiti noti

- La velocita' costante e' sul setpoint pubblicato, non garantita sul TCP reale.
- Il controller base non conosce limiti cinematici completi, ostacoli o footprint.
- Il TCP puo' avere errore di inseguimento se il braccio satura o la base si muove troppo lentamente.
- Non c'e' ancora gestione lifter.
- Non c'e' ancora obstacle avoidance laser integrata.
- Non c'e' ancora solver whole-body cinematico completo.
