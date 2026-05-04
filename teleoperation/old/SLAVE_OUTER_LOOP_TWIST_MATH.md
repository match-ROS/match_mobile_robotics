## Slave `teleop_slave_twist_outer_loop`: spiegazione matematica

Questo documento descrive con formule la legge di controllo implementata nel nodo
`teleop_slave_twist_outer_loop` (file: `src/match_mobile_robotics/teleoperation/src/teleop_slave_twist_outer_loop_node.cpp`).

Obiettivo del nodo: generare un comando di twist del robot slave (TCP) in base a:

- una posa target $T^b_{tgt}$ (da master) e una twist feedforward $v_{ff}$,
- il wrench esterno misurato $w=(F,\tau)$,
- la posa corrente del TCP $T^b_{tcp}$ (da TF).

Il comando finale è la somma di tre contributi:

1) tracking posizione/orientamento via PID (outer loop pose $\to$ twist),
2) feedforward di twist scalato da un fattore $\alpha(F)$ (riduce l’avanzamento quando la forza cresce),
3) termine di compliance “tipo ammettenza” in **velocità** proporzionale al wrench esterno.

---

## 1) Frame e segnali

Il controller lavora nel frame base dello slave $\{b\}$:

- `base_frame` $\to$ $\{b\}$
- `tcp_frame` $\to$ frame del tool/TCP

Ingressi:

- `target_pose_topic`: $T^b_{tgt}$ (pose target, trasformata in $\{b\}$)
- `feedforward_twist_topic`: $v^b_{ff} = (v_{ff},\ \omega_{ff})$ (ruotata in $\{b\}$)
- `wrench_topic`: $w^b=(F^b,\tau^b)$ (ruotato in $\{b\}$)

Misura:

- $T^b_{tcp}$ da TF tra `base_frame` e `tcp_frame`.

Uscita:

- `command_topic`: $v^b_{cmd}=(v_{cmd},\ \omega_{cmd})$ come `geometry_msgs/Twist`.

---

## 2) Trasformazioni TF usate nel codice

### 2.1 Pose target in base

Il target pose viene trasformato con una trasformazione completa (rotazione + traslazione) via TF:

$$
{}^{b}T_{tgt} = {}^{b}T_{s}\;{}^{s}T_{tgt}
$$

nel codice tramite `tf2::doTransform`.

### 2.2 Twist feedforward in base (solo rotazione)

La twist feedforward viene ruotata nel frame base usando solo la rotazione $R_{b\leftarrow s}$ del TF:

$$
v^b_{ff} = R_{b\leftarrow s}\, v^s_{ff}, \qquad \omega^b_{ff} = R_{b\leftarrow s}\, \omega^s_{ff}.
$$

### 2.3 Wrench in base (solo rotazione)

Analogamente, il wrench viene ruotato (solo $R$):

$$
F^b = R_{b\leftarrow s} F^s, \qquad \tau^b = R_{b\leftarrow s} \tau^s.
$$

---

## 3) Gestione tempo e validità ingressi

### 3.1 Timeout ingressi e reset

Se manca uno tra target/ff/wrench, oppure se uno è “stale” (più vecchio del relativo timeout):

- il nodo pubblica twist nullo,
- resetta gli integratori dei PID (posizione e orientamento).

Parametri: `target_pose_timeout`, `feedforward_timeout`, `wrench_timeout`.

### 3.2 Clamping di $\Delta t$

Il dt viene calcolato da wall-clock e clampato:

$$
\Delta t_{nom} = \frac{1}{\texttt{control\_rate}},\qquad
\Delta t_{\min} = \texttt{dt\_min\_factor}\,\Delta t_{nom},\qquad
\Delta t_{\max} = \texttt{dt\_max\_factor}\,\Delta t_{nom}.
$$

Se $\Delta t < \Delta t_{\min}$ il codice usa $\Delta t=\Delta t_{nom}$, se $\Delta t>\Delta t_{\max}$ usa $\Delta t=\Delta t_{\max}$.

---

## 4) Errore di posa (posizione + orientamento)

### 4.1 Errore di posizione

Siano $p_{curr},p_{tgt}\in\mathbb{R}^3$ le traslazioni di $T^b_{tcp}$ e $T^b_{tgt}$.

$$
e_p = p_{tgt} - p_{curr}.
$$

### 4.2 Errore di orientamento (axis-angle)

Siano $q_{curr},q_{tgt}$ quaternioni unitari.
Il codice costruisce:

$$
q_{err} = q_{tgt}\, q_{curr}^{-1}
$$

e ricava l’errore come vettore axis-angle:

$$
e_o = \theta\,\hat u \in\mathbb{R}^3
$$

dove $\theta$ e $\hat u$ sono angolo e asse dell’$\mathrm{AngleAxis}(q_{err})$.

(È la funzione `orientationErrorAxisAngle(q_curr,q_tgt)` in `math_utils.hpp`.)

---

## 5) PID outer-loop: da errore a twist correttiva

Il nodo usa due PID a 3 dimensioni:

- PID posizione: $corr_p \in \mathbb{R}^3$
- PID orientamento: $corr_o \in \mathbb{R}^3$

con `PIDController` che implementa:

$$
u = P + I + D
$$

con:

$$
P = k_p\,e
$$

$$
I = k_i \int e\,dt
$$

La derivata usa una derivata discreta e un filtro del primo ordine:

$$
\dot e_{raw} = \frac{e_k - e_{k-1}}{\Delta t}
$$

$$
\dot e_f = \beta\,\dot e_{raw} + (1-\beta)\,\dot e_f^{prev}, \qquad \beta = \frac{\Delta t}{\tau_d + \Delta t}
$$

$$
D = k_d\,\dot e_f.
$$

### 5.1 Saturazione in norma e anti-windup

L’uscita del PID è saturata sulla norma:

$$
u \leftarrow
\begin{cases}
u & \|u\|\le u_{\max} \\
u\,\dfrac{u_{\max}}{\|u\|} & \|u\|>u_{\max}
\end{cases}
$$

dove $u_{\max}$ è `pid/*/output_limit`.

Anti-windup: l’ampiezza del termine integrale viene limitata alla “headroom” residua dopo il termine proporzionale:

$$
\|I\| \le \max(0,\ u_{\max} - \|P\|).
$$

Quindi:

$$
corr_p = \mathrm{PID}_{pos}(e_p,\Delta t),\qquad corr_o = \mathrm{PID}_{ori}(e_o,\Delta t).
$$

---

## 6) Pre-processing del wrench esterno

Il wrench ruotato in base viene filtrato e limitato con `filterClampDeadbandWrench`:

### 6.1 Filtro EMA (opzionale)

Per forza e coppia:

$$
F_f = \alpha F + (1-\alpha)F_f^{prev},\qquad \tau_f = \alpha \tau + (1-\alpha)\tau_f^{prev}
$$

con:

$$
\alpha =
\begin{cases}
\dfrac{\Delta t}{\tau + \Delta t} & \text{se `wrench\_filter\_cutoff\_hz`}>0 \\
\texttt{wrench\_filter\_alpha} & \text{altrimenti}
\end{cases}
$$

### 6.2 Deadband **per componente** (non sulla norma)

La deadband è assoluta per asse:

$$
(\mathrm{DB}(x))_i =
\begin{cases}
0 & |x_i|<d \\
x_i & \text{altrimenti}
\end{cases}
$$

con $d=$ `force_deadband` per $F$ e `torque_deadband` per $\tau$.

### 6.3 Clamp in norma

Dopo deadband, clamp in norma:

$$
\|F\|\le F_{\max},\qquad \|\tau\|\le \tau_{\max}
$$

con `max_force`, `max_torque`.

---

## 7) Fattore $\alpha(F)$: riduzione del feedforward con forza

Il nodo calcola uno scalare $\alpha\in[0,1]$ in base alla forza esterna filtrata.

### 7.1 Metrica

- `alpha_mode = "norm"`:

$$
m = \|F\|
$$

- `alpha_mode = "parallel"`:

se $\|v_{ff}\|>0$, sia $d = v_{ff}/\|v_{ff}\|$, allora:

$$
m = |d^\top F|
$$

altrimenti ricade su $m=\|F\|$.

### 7.2 Profilo smoothstep tra `force_start` e `force_stop`

$$
\alpha =
\begin{cases}
1 & m \le F_{start} \\
0 & m \ge F_{stop} \\
1 - s(t) & \text{altrimenti}
\end{cases}
$$

con:

$$
t=\dfrac{m-F_{start}}{F_{stop}-F_{start}},\qquad
s(t)=t^2(3-2t),\qquad t\in[0,1].
$$

---

## 8) Legge di comando twist: somma di feedforward, PID e compliance

Siano:

- $v_{ff},\omega_{ff}$ la twist feedforward in base,
- $corr_p,corr_o$ le correzioni PID,
- $F,\tau$ il wrench filtrato in base.

### 8.1 Feedforward scalato

$$
v_{ff,scaled} = \alpha\,k_{ff}\,v_{ff},\qquad
\omega_{ff,scaled} = \alpha\,k_{ff}\,\omega_{ff}
$$

con `k_ff`.

### 8.2 Compliance (ammettenza “in velocità”)

Il termine di compliance è proporzionale al wrench e ha segno “opponente”:

$$
v_{comp} = -k_{adm,lin}\,F,\qquad
\omega_{comp} = -k_{adm,ang}\,\tau
$$

con `k_adm_linear`, `k_adm_angular`.

Questi termini sono limitati in norma:

$$
\|v_{comp}\|\le v_{comp,\max},\qquad \|\omega_{comp}\|\le \omega_{comp,\max}
$$

con `max_compliance_linear_speed`, `max_compliance_angular_speed`.

### 8.3 Comando preliminare

$$
v_{cmd} = v_{ff,scaled} + corr_p + v_{comp}
$$

$$
\omega_{cmd} = \omega_{ff,scaled} + corr_o + \omega_{comp}
$$

Se `use_torques=false` il termine angolare di compliance non viene aggiunto (ma $\omega_{cmd}$ può comunque essere non nullo per PID+FF).

### 8.4 Saturazione finale velocità

$$
\|v_{cmd}\|\le v_{\max},\qquad \|\omega_{cmd}\|\le \omega_{\max}
$$

con `max_linear_speed`, `max_angular_speed`.

---

## 9) Hard guard su forza: stop/retreat

Si misura $f=\|F\|$. Se $f \ge F_{hard}$ per almeno $T_{hard}$:

- reset dei PID (evita accumulo integrale durante guard),
- override del comando in base a `hard_guard_action`:

### 9.1 `hard_guard_action = "stop"`

$$
v_{cmd}=0,\qquad \omega_{cmd}=0.
$$

### 9.2 `hard_guard_action = "retreat"`

Si sceglie una direzione $dir$:

- se $\|v_{ff}\|>0$: $dir = -\dfrac{v_{ff}}{\|v_{ff}\|}$
- altrimenti se $\|F\|>0$: $dir = -\dfrac{F}{\|F\|}$
- altrimenti $dir=0$

e:

$$
v_{cmd} = v_{ret}\,dir,\qquad \omega_{cmd}=0
$$

con `retreat_speed = v_ret`.

Parametri: `hard_force_threshold`, `hard_force_duration`, `hard_guard_action`, `retreat_speed`.

---

## 10) Parametri chiave (mappa rapida)

- **Pose tracking**: `pid/position/*`, `pid/orientation/*`
- **Feedforward**: `k_ff`
- **Compliance**: `k_adm_linear`, `k_adm_angular`, `max_compliance_*_speed`
- **Riduzione con forza**: `alpha_mode`, `force_start`, `force_stop`
- **Wrench conditioning**: `wrench_filter_alpha|cutoff_hz`, `force_deadband`, `torque_deadband`, `max_force`, `max_torque`
- **Saturazioni twist**: `max_linear_speed`, `max_angular_speed`
- **Hard guard**: `hard_force_threshold`, `hard_force_duration`, `hard_guard_action`, `retreat_speed`
- **Tempo**: `control_rate`, `dt_min_factor`, `dt_max_factor`

