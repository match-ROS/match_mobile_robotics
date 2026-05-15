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

## 5. Replay adaptive time-scaled

La modalita' `adaptive` mantiene la stessa logica area-preserving, ma usa
`speed_scale` come limite massimo locale. Prima dell'analisi dinamica, per
default il profilo registrato viene fatto passare attraverso un modello del
`ur_twist_limiter`; in questo modo il retiming lavora sul profilo stimato a
valle del filtro, non sui comandi grezzi in ingresso a `command_collision_free`.
Il profilo rallenta nei tratti in cui i Twist riscalati supererebbero i limiti
configurati di velocita' o accelerazione lineare/angolare.

Con `adaptive_source_filter:=ur_twist_limiter`, la traiettoria preservata e'
quella stimata dopo il limiter. Usare `adaptive_source_filter:=none` solo se si
vuole tornare all'analisi dei Twist grezzi registrati.

L'accelerazione introdotta dal retiming viene considerata esplicitamente: il
codice limita la scala locale usando l'accelerazione del profilo filtrato e poi
passa i comandi finali in un limiter deterministico leggermente sotto i limiti
del `ur_twist_limiter`. Se questo limiter finale interviene, lo smoothing e'
nel profilo pubblicato dal replay, quindi non rimane nascosto nel controller a
valle.

```bash
roslaunch teleoperation replay_dual_slave_twist_from_bag.launch \
  manifest:=/path/to/replay_manifest.yaml \
  replay_speed_mode:=adaptive \
  speed_scale:=5.0 \
  adaptive_max_linear_speed_mps:=0.59 \
  adaptive_max_angular_speed_radps:=0.98 \
  adaptive_max_linear_accel_mps2:=1.95 \
  adaptive_max_angular_accel_radps2:=3.45 \
  adaptive_source_filter:=ur_twist_limiter \
  adaptive_local_smoothing_window_s:=0.05
```

Eseguire prima con `dry_run:=true`: lo status riporta durata finale e
`speed_scale` effettivo min/mean/max del profilo adattivo.
La finestra `adaptive_local_smoothing_window_s` agisce sul profilo di scala
temporale nelle zone in cui il profilo filtrato richiede un cap locale di
accelerazione. Il filtro sorgente replica i limiti del `ur_twist_limiter` prima
dell'analisi:

- `adaptive_filter_linear_speed_mps:=0.6`
- `adaptive_filter_angular_speed_radps:=1.0`
- `adaptive_filter_linear_accel_step:=0.004`
- `adaptive_filter_angular_accel_step:=0.007`
- `adaptive_filter_linear_jerk_step:=0.0008`
- `adaptive_filter_angular_jerk_step:=0.0017`

I parametri `*_step` sono per callback del limiter, non per secondo. Se i valori
ddynamic del limiter vengono cambiati a runtime, aggiornare anche questi
parametri del replay per mantenere coerente il modello.
