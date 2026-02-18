# Guida parametri: `config/master_real_mur620b_ur10_l.yaml`

Questo documento spiega **come usare** e **cosa significano fisicamente** i parametri in:

- `teleoperation/config/master_real_mur620b_ur10_l.yaml`

Il nodo a cui si riferiscono è `teleop_master_haptic_controller` e implementa una dinamica in **ammettenza** sul comando di velocità (twist).

---

## Modello fisico (cosa stai regolando)

La parte lineare (e opzionalmente quella angolare) segue il modello:

$$
M \dot v + D v = F_{\text{hand}} - F_{\text{feedback}}
$$

dove:
- **$v$**: velocità commandata (m/s per la parte lineare, rad/s per la parte angolare)
- **$F_{\text{hand}}$**: forza “mano/operatore” (qui: wrench dal master)
- **$F_{\text{feedback}}$**: forza di ritorno (slave * scalata + coupling). Nel tuo setup “master-only” è tipicamente 0.
- **$M$**: “massa virtuale” (inerzia percepita)
- **$D$**: “smorzamento virtuale” (attrito viscoso percepito)

### Intuizioni utili
- **Regime stazionario** (accelerazione ~0): $v \approx \frac{F}{D}$
  - Aumentare **$D$** riduce la velocità a parità di forza (più “pesante/controllato”).
- **Transitorio**: costante di tempo approssimata $ \tau \approx \frac{M}{D} $
  - Aumentare **$M$** rende la risposta più lenta e “morbida”.
- **Sicurezza**: limiti su forza/velocità/accelerazione/jerk servono per evitare transitori bruschi e saturazioni “a scatto”.

---

## 1) Wiring (topic, TF, frame)

### `master_wrench_topic`
- **Tipo**: string
- **Significato**: topic `geometry_msgs/WrenchStamped` del sensore/stream di wrench sul master.

### `slave_wrench_topic`, `coupling_wrench_topic`
- **Tipo**: string
- **Significato**: wrench di feedback (slave) e/o termine di coupling.
- **Nota**: se vuoti (`""`) il feedback è disabilitato (utile per master-only).

### `wrench_source_frame_override`
- **Tipo**: string (frame TF)
- **Significato**: se l’header del wrench non è affidabile/risolubile in TF, forza il frame sorgente usato per la rotazione.

### `use_latest_tf_for_wrench`
- **Tipo**: bool
- **Significato**:
  - `true`: usa sempre l’ultimo TF disponibile (`stamp=0`)
  - `false`: usa lo `stamp` del messaggio wrench
- **Quando usarlo**: `true` è utile se gli stamp non sono coerenti o se vuoi minimizzare errori TF; `false` è più “corretto” temporalmente se TF e wrench sono sincronizzati.

### `wrench_target_frame`
- **Tipo**: string
- **Significato**: frame in cui vengono espressi forza/torque prima del controllo (rotazione).

### `tf_timeout_s`
- **Tipo**: double (s)
- **Significato**: timeout per `lookupTransform`.

---

## 2) Output e rate di controllo

### `command_topic`
- **Tipo**: string
- **Significato**: topic `geometry_msgs/Twist` verso il tuo controller di velocità.
- **Nota pratica**: il nodo pubblica anche `TwistStamped` su `command_topic + "_stamped"` (utile per debug/log).

### `control_rate`
- **Tipo**: double (Hz)
- **Significato**: frequenza del timer di controllo.
- **Effetti**:
  - più alto → integrazione più fine e filtri/limiter più “fedeli”
  - più basso → maggiore discrezione e più rischio di percepire scatti

---

## 3) Parametri di ammettenza (massa e damping)

### Versione scalare (semplice)

#### `mass_linear`, `damping_linear`
- **Tipo**: double
- **Unità**:
  - $M$ lineare: $ \mathrm{N} / (\mathrm{m/s^2}) = \mathrm{kg} $ (equivalente “massa virtuale”)
  - $D$ lineare: $ \mathrm{N} / (\mathrm{m/s}) = \mathrm{N\,s/m} $

#### `mass_angular`, `damping_angular`
- **Tipo**: double
- **Unità**:
  - $M$ angolare: $ \mathrm{N\,m}/(\mathrm{rad/s^2}) $
  - $D$ angolare: $ \mathrm{N\,m}/(\mathrm{rad/s}) $

### Versione per-asse (consigliata per tuning fine)

Se presenti, **sovrascrivono** i valori scalari.

#### `mass_linear_xyz`, `damping_linear_xyz`
- **Tipo**: array di 3 numeri `[x, y, z]`
- **Significato**: $M=\mathrm{diag}(m_x,m_y,m_z)$, $D=\mathrm{diag}(d_x,d_y,d_z)$
- **Perché utile**: puoi rendere, ad esempio, **Z** più “pesante” (più damping/inerzia) rispetto a XY.

#### `mass_angular_xyz`, `damping_angular_xyz`
- analogo per la parte angolare.

---

## 4) Forza di feedback (riflessione)

### `force_reflection_scale`, `torque_reflection_scale`
- **Tipo**: double
- **Significato**: scala il feedback del wrench proveniente dallo slave.
- **Esempio**: `force_reflection_scale: 0.3` significa $F_{\text{feedback}} = 0.3\,F_{\text{slave}} + F_{\text{coupling}}$.

### `use_torques`
- **Tipo**: bool
- **Significato**: abilita la dinamica angolare usando anche i torque.
- **Nota**: se `false`, il comando angolare viene azzerato.

---

## 5) Pre-processing del wrench: filtro, deadzone, clamp

La pipeline è:

1) (opzionale) **low-pass** sul wrench  
2) **deadzone soft sulla norma** (isotropica) + (opzionale) isteresi  
3) **clamp** sulla norma (sicurezza)  

### Filtro

#### `wrench_filter_cutoff_hz`
- **Tipo**: double (Hz)
- **Significato**: frequenza di taglio del low-pass.
- **Nota**: se `> 0`, ha priorità su `wrench_filter_alpha`.

#### `wrench_filter_alpha`
- **Tipo**: double (0..1)
- **Significato**: coefficiente EMA. Più vicino a 1 → meno filtro; più vicino a 0 → più filtro.
- **Suggerimento**: preferisci `wrench_filter_cutoff_hz` perché resta più consistente se $dt$ cambia.

### Deadzone soft + isteresi

#### `force_deadband_enter`, `force_deadband_exit`
- **Tipo**: double (N)
- **Significato**:
  - `enter`: soglia per “attivare” il movimento
  - `exit`: soglia più bassa per “restare attivi” (riduce chatter)
- **Comportamento**: quando $\|F\|$ supera `enter`, l’output cresce **continuamente** da 0; quando scende sotto `exit`, torna a 0 e si “disarma”.

#### `torque_deadband_enter`, `torque_deadband_exit`
- **Tipo**: double (N·m)
- **Significato**: analogo per i torque (utile solo se `use_torques: true`).

### Clamp su norma (sicurezza)

#### `max_force`, `max_torque`
- **Tipo**: double
- **Unità**: N, N·m
- **Significato**: limite massimo sul wrench dopo filtro/deadzone.

#### Debug wrench filtrato
- `publish_filtered_wrench_debug`: abilita publisher di debug
- `filtered_master_wrench_topic`: topic debug del wrench filtrato del master

---

## 6) Limiti sul comando di velocità

### `max_linear_speed`, `max_angular_speed`
- **Tipo**: double
- **Unità**: m/s e rad/s
- **Significato**: saturazione sulla **norma** della velocità commandata (lineare/angolare).

---

## 7) $dt$ sanitizzato (robustezza al jitter del timer)

### `dt_min_factor`, `dt_max_factor`
- **Tipo**: double
- **Significato**: clamp del $dt$ misurato rispetto a $dt_{\text{nom}} = 1 / \text{control\_rate}$
  - $dt_{\min} = dt_{\text{min\_factor}} \, dt_{\text{nom}}$
  - $dt_{\max} = dt_{\text{max\_factor}} \, dt_{\text{nom}}$

### `dt_use_substepping`
- **Tipo**: bool
- **Significato**: se $dt$ è troppo grande, spezza l’integrazione in $N$ sub-step più piccoli.

### `dt_max_substeps`
- **Tipo**: int
- **Significato**: massimo numero di sub-step per tick (limita costo computazionale).

---

## 8) Smoothness: limitatore su accelerazione e jerk

Questi limiti agiscono su $\dot v$ (accelerazione) e su $\ddot v$ (jerk), migliorando la fluidità quando:
- il wrench è discontinuo (contatto, micro-impatti, rumore)
- sei vicino alla saturazione di velocità

### Scalare (semplice)

#### `max_linear_accel`, `max_angular_accel`
- **Tipo**: double
- **Unità**: m/s² e rad/s²
- **Effetto**: limita quanto velocemente può crescere/variare la velocità.

#### `max_linear_jerk`, `max_angular_jerk`
- **Tipo**: double
- **Unità**: m/s³ e rad/s³
- **Effetto**: limita quanto velocemente può cambiare l’accelerazione (transitori ancora più morbidi).

### Per-asse (se serve)

Se presenti, sovrascrivono i limiti scalari:
- `max_linear_accel_xyz`, `max_linear_jerk_xyz`
- `max_angular_accel_xyz`, `max_angular_jerk_xyz`

---

## 9) Timeout e reset

### `wrench_timeout_s`
- **Tipo**: double (s)
- **Significato**: se il wrench è “stale” oltre questo tempo, il nodo pubblica zero.

### `reset_on_stale`
- **Tipo**: bool
- **Significato**: se `true`, quando il wrench è stale resetta stati interni (velocità, filtri, limiter).

---

## 10) Diagnostica (fortemente consigliata)

### `publish_diagnostics`
- **Tipo**: bool
- **Significato**: abilita publisher su `~debug/*`.

### `diagnostics_rate`
- **Tipo**: double (Hz)
- **Significato**: rate massimo dei messaggi diagnostici.

### Topic utili
(tutti sotto il namespace del nodo)
- `debug/dt_stats` (`std_msgs/Float64MultiArray`)
  - contiene $dt$ raw/usato/step, substeps, min/max/mean
- `debug/admittance_stats` (`std_msgs/Float64MultiArray`)
  - norme di $F_{\text{hand}}$, $F_{\text{feedback}}$, accelerazioni (desiderata vs limitata), velocità pre/post, flag saturazione
- `debug/v_cmd_pre`, `debug/v_cmd_post` (`geometry_msgs/TwistStamped`)
  - velocità prima/dopo integrazione+clamp

---

## Mini-ricetta di tuning (pratica)

1) **Stabilizza il segnale**
   - usa `wrench_filter_cutoff_hz` (es. 8–15 Hz tipicamente ok)
2) **Imposta deadzone**
   - `force_deadband_enter` abbastanza alta da eliminare drift/offset
   - `force_deadband_exit` un po’ più bassa (60–80% della enter) per evitare chatter
3) **Regola “peso” e “prontezza”**
   - se è “troppo reattivo”: aumenta `mass_linear` e/o `damping_linear`
   - se è “lento”: riduci `mass_linear` o riduci `damping_linear`
4) **Aggiungi limiti accel/jerk**
   - se percepisci scatti: riduci jerk e accel (più conservativo)
5) **Verifica in diagnostica**
   - se vedi saturazione continua (`sat=1` spesso): aumenta damping, riduci max speed, o riduci scaling/forza

