# Replay dual slave arms da rosbag

Esempio d'uso per preparare e rieseguire una sequenza registrata.

## 1. Analisi della bag

Genera il manifest di replay e il grafico PNG con inizio/fine rilevati automaticamente:

```bash
rosrun teleoperation analyze_dual_slave_replay_window.py /path/to/original.bag \
  --start-offset-s -0.2 \
  --end-offset-s 0.5
```

Controlla il PNG generato prima di comandare il robot.

## 2. Dry-run

Valida manifest, finestra temporale e scheduling senza comandare i bracci:

```bash
roslaunch teleoperation replay_dual_slave_twist_from_bag.launch \
  manifest:=/path/to/replay_manifest.yaml \
  dry_run:=true
```

## 3. Test home-only

Porta i due bracci slave alla posa iniziale registrata, verifica i controller e la posa TCP, ma non esegue il replay:

```bash
roslaunch teleoperation replay_dual_slave_twist_from_bag.launch \
  manifest:=/path/to/replay_manifest.yaml \
  home_only:=true
```

## 4. Replay time-scaled

Esegue la stessa traiettoria con durata scalata, preservando l'integrale dei twist.
`speed_scale:=2.0` dimezza la durata, `speed_scale:=0.5` la raddoppia.
Il nodo ricampiona i comandi a `replay_rate_hz` (default 500 Hz).

```bash
roslaunch teleoperation replay_dual_slave_twist_from_bag.launch \
  manifest:=/path/to/replay_manifest.yaml \
  speed_scale:=0.5
```

Quando il test prudente e' corretto, ripetere con `speed_scale:=1.0` o con il
fattore desiderato. Lasciare `time_scale:=1.0`: il parametro e' mantenuto solo
per compatibilita' del launch file.
