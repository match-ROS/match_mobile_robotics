## Controllo in ammettenza del master e force-feedback (formule)

Questo documento descrive con formule la legge di controllo implementata nel nodo **`teleop_master_haptic_controller`**
(file: `src/match_mobile_robotics/teleoperation/src/teleop_master_haptic_controller_node.cpp`).

L’idea di fondo è:

- **Master**: misura un wrench “di mano” $(F,\tau)$ e lo converte in una velocità di comando tramite una dinamica di **ammettenza** (massa–smorzatore).
- **Feedback di forza**: usa il wrench dello slave (misurato al tool dello slave) come termine di “force reflection” con uno scalare e una convenzione di segno.
- **Pre-processing**: rotazioni TF, filtro EMA (LP), deadzone soft sulla norma con isteresi, saturazioni.

---

## 1) Notazione e segnali

Tutti i vettori sono in $\mathbb{R}^3$.

- **Frame target** (per tutti i wrench): $\{T\}$ = `wrench_target_frame` (default `base_link`).
- **Wrench master** (misurato): $w_m=(F_m,\tau_m)$.
- **Wrench slave** (misurato): $w_s=(F_s,\tau_s)$.
- **Wrench di coupling** (opzionale): $w_c=(F_c,\tau_c)$.
- **Stato del controller** (uscita interna):
  - velocità lineare $v\in\mathbb{R}^3$ = `v_lin_cmd_`
  - velocità angolare $\omega\in\mathbb{R}^3$ = `v_ang_cmd_`

Il nodo pubblica:

- `command_topic` (default `twist_controller/command`) come `geometry_msgs/Twist` e anche `..._stamped`.
- opzionalmente target pose e twist verso lo slave (se `publish_slave_targets=true`).

---

## 2) Rotazione dei wrench nel frame target (TF)

Ogni messaggio `geometry_msgs/WrenchStamped` viene ruotato nel frame $\{T\}$:

$$
F^{T} = R_{T\leftarrow S}\,F^{S}, \qquad \tau^{T} = R_{T\leftarrow S}\,\tau^{S}
$$

dove $S$ è il frame sorgente del messaggio (o `wrench_source_frame_override` se impostato).

**Nota (importante):** viene applicata **solo** la rotazione, *non* la trasformazione completa del momento
$$
\tau' = R\tau + p \times (RF).
$$
Quindi l’effetto $p\times F$ non viene modellato.

---

## 3) Filtraggio e pre-processing dei wrench

Ogni wrench passa da:

- filtro passa-basso tipo EMA su $F$ e $\tau$
- deadzone “soft” sulla **norma**, con isteresi (enter/exit)
- saturazione sulla norma

### 3.1 Filtro EMA

Per un generico vettore $x_k$ (forza o coppia) e il valore filtrato $y_k$:

$$
y_k = \alpha\,x_k + (1-\alpha)\,y_{k-1}, \qquad \alpha\in[0,1]
$$

Il coefficiente $\alpha$ può essere passato direttamente come parametro oppure derivato da una frequenza di taglio $f_c$:

$$
\tau = \frac{1}{2\pi f_c}, \qquad \alpha = \frac{\Delta t}{\tau + \Delta t}
$$

dove $\Delta t$ è una stima di tempo per il filtro (nel codice: `dt_for_filter`).

Il nodo usa due $\alpha$ distinti:

- $\alpha_m$ per il **master** (`master_wrench_filter_*` o fallback a `wrench_filter_*`)
- $\alpha_{fb}$ per **slave** e **coupling** (`feedback_wrench_filter_*` o fallback a `wrench_filter_*`)

### 3.2 Deadzone soft sulla norma con isteresi

Il codice implementa una deadzone sulla norma con due soglie:

- **enter**: $d_{\text{in}}$ (`force_deadband_enter`, `torque_deadband_enter`)
- **exit**: $d_{\text{out}}$ (`force_deadband_exit`, `torque_deadband_exit`)

e una variabile booleana di stato `active` (isteresi).

Definiamo $n=\|v\|$. La logica è:

- se `active=false` e $n \le d_{\text{in}}$  $\Rightarrow$ uscita $0$
- se `active=false` e $n > d_{\text{in}}$  $\Rightarrow$ `active=true` e si applica deadzone soft
- se `active=true` e $n \le d_{\text{out}}$ $\Rightarrow$ uscita $0$ e `active=false`
- se `active=true` e $n > d_{\text{out}}$  $\Rightarrow$ si applica deadzone soft

La deadzone soft (sulla norma) è:

$$
\mathrm{softDZ}(v; d)=
\begin{cases}
0 & \text{se } \|v\|\le d \\
\left(1-\dfrac{d}{\|v\|}\right)v & \text{se } \|v\|>d
\end{cases}
$$

### 3.3 Saturazione sulla norma

Dopo deadzone, forza e coppia vengono saturate in norma:

$$
\mathrm{clampNorm}(v;v_{\max})=
\begin{cases}
0 & \text{se } v_{\max}\le 0 \\
v & \text{se } \|v\|\le v_{\max} \\
v\,\dfrac{v_{\max}}{\|v\|} & \text{se } \|v\|>v_{\max}
\end{cases}
$$

Nel codice le soglie usate sono:

- **master**: `max_force_hand`/`max_force`, `max_torque_hand`/`max_torque`
- **feedback** (slave/coupling): `max_force_feedback`/`max_force`, `max_torque_feedback`/`max_torque`

---

## 4) Come arriva (e si compone) il feedback di forza

Dopo filtraggio/deadzone/clamp, il nodo costruisce il wrench di feedback come:

$$
F_{\text{fb}} = -k_f\,F_s + F_c
$$

$$
\tau_{\text{fb}} = -k_\tau\,\tau_s + \tau_c
$$

dove:

- $k_f$ = `force_reflection_scale` (`kf_force_`)
- $k_\tau$ = `torque_reflection_scale` (`kf_torque_`)

**Segno meno:** si assume che la FT dello slave misuri il wrench applicato *dall’ambiente sul tool*; per riflettere un contributo
“opponente” al master (azione–reazione), si inverte il segno.

**Coupling wrench:** $w_c$ è un termine additivo opzionale (se `coupling_wrench_topic` è vuoto, vale $0$).

---

## 5) Dinamica di ammettenza (equazioni continue)

Il controller implementa una ammettenza massa–smorzatore (nessuna molla $K$).

### 5.1 Parte lineare

$$
M_v\,\dot v + D_v\,v = F_{\text{hand}} - F_{\text{fb}}
$$

dove $F_{\text{hand}}=F_m$ (dopo filtri/deadzone/clamp) e $M_v,D_v$ sono diagonali per-assi.

### 5.2 Parte angolare (opzionale)

Se `use_torques=true`:

$$
M_\omega\,\dot \omega + D_\omega\,\omega = \tau_{\text{hand}} - \tau_{\text{fb}}
$$

Altrimenti $\omega \equiv 0$.

---

## 6) Implementazione discreta (tick, $\Delta t$, sub-stepping, limitatori)

Il nodo gira a `control_rate` (nominale), ma usa un $\Delta t$ misurato e “sanitizzato”.

### 6.1 Gestione del tempo e sub-stepping

Sia $\Delta t_{\text{raw}}$ il dt misurato tra due tick.

- se $\Delta t_{\text{raw}} < \Delta t_{\min}$ con $\Delta t_{\min}=\texttt{dt\_min\_factor}\cdot(1/\texttt{control\_rate})$,
  il codice usa $\Delta t_{\text{used}} = 1/\texttt{control\_rate}$
- se $\Delta t_{\text{used}} > \Delta t_{\max}$ con $\Delta t_{\max}=\texttt{dt\_max\_factor}\cdot(1/\texttt{control\_rate})$:
  - con `dt_use_substepping=true`: $N=\left\lceil \Delta t_{\text{used}}/\Delta t_{\max}\right\rceil$ (clampato a `dt_max_substeps`)
  - altrimenti: $\Delta t_{\text{used}} \leftarrow \Delta t_{\max}$

Ogni sub-step usa:

$$
\Delta t = \Delta t_{\text{step}} = \frac{\Delta t_{\text{used}}}{N}.
$$

### 6.2 Accelerazione desiderata da ammettenza

Riscrivendo la parte lineare:

$$
M_v\,\dot v = (F_{\text{hand}}-F_{\text{fb}}) - D_v\,v
$$

quindi:

$$
a_{\text{des}}=\dot v_{\text{des}} = M_v^{-1}\Big[(F_{\text{hand}}-F_{\text{fb}}) - D_v\,v\Big].
$$

Nel codice è per componente:

$$
a_{\text{des},i}=\frac{(F_{\text{hand}}-F_{\text{fb}})_i - D_i\,v_i}{M_i}, \qquad i\in\{x,y,z\}.
$$

Analogamente (se `use_torques=true`):

$$
\alpha_{\text{des}}=\dot\omega_{\text{des}} = M_\omega^{-1}\Big[(\tau_{\text{hand}}-\tau_{\text{fb}}) - D_\omega\,\omega\Big].
$$

### 6.3 Limitatore jerk/accelerazione (JerkLimiter3)

L’accelerazione comandata $a$ non segue direttamente $a_{\text{des}}$: viene limitata in jerk e in modulo (per asse).

Per ogni asse $i$:

$$
\Delta a_i = \mathrm{clip}\big(a_{\text{des},i}-a_i,\ -j_i\Delta t,\ +j_i\Delta t\big)
$$

$$
a_i \leftarrow a_i + \Delta a_i
$$

$$
a_i \leftarrow \mathrm{clip}(a_i,\ -a_{\max,i}, +a_{\max,i})
$$

In vettoriale, questo produce $a_{\text{cmd}}$ (lineare) e $\alpha_{\text{cmd}}$ (angolare).

### 6.4 Anti-windup coerente con saturazione della velocità (rimozione componente radiale)

Quando $\|v\| \ge v_{\max}-\varepsilon$ (con $\varepsilon=$ `speed_saturation_eps`), il codice rimuove la componente di accelerazione che
aumenterebbe ulteriormente $\|v\|$:

$$
u = \frac{v}{\|v\|},\qquad a_{\text{rad}}=u^\top a,\qquad
\text{se } a_{\text{rad}}>0:\ a \leftarrow a - a_{\text{rad}}\,u.
$$

(Stesso identico meccanismo per $\omega$ se abilitato.)

### 6.5 Integrazione della velocità e saturazione

Ad ogni sub-step:

$$
v^{+} = v + a_{\text{cmd}}\Delta t,\qquad v \leftarrow \mathrm{clampNorm}(v^{+}; v_{\max})
$$

e (se abilitato):

$$
\omega^{+}=\omega+\alpha_{\text{cmd}}\Delta t,\qquad \omega \leftarrow \mathrm{clampNorm}(\omega^{+}; \omega_{\max}).
$$

con $v_{\max}=$ `max_linear_speed`, $\omega_{\max}=$ `max_angular_speed`.

---

## 7) Robustezza: timeout wrench e reset stato

Se un wrench (master/slave/coupling quando abilitati) è più vecchio di `wrench_timeout_s`, il nodo pubblica twist nullo.

Se `reset_on_stale=true`, fa anche `resetControllerState()`:

- $v\leftarrow 0$, $\omega\leftarrow 0$
- reset dei jerk limiter
- reset dei filtri e degli stati di isteresi della deadzone

---

## 8) Parametri che entrano direttamente nelle formule

- **Force reflection**: `force_reflection_scale` ($k_f$), `torque_reflection_scale` ($k_\tau$)
- **Ammettenza**:
  - `mass_linear` o `mass_linear_xyz` ($M_v$)
  - `damping_linear` o `damping_linear_xyz` ($D_v$)
  - `mass_angular` o `mass_angular_xyz` ($M_\omega$)
  - `damping_angular` o `damping_angular_xyz` ($D_\omega$)
- **Filtri**: `*_wrench_filter_alpha|cutoff_hz` (calcolo $\alpha$)
- **Deadzone**: `force_deadband_enter/exit`, `torque_deadband_enter/exit`
- **Clamp wrench**: `max_force*`, `max_torque*`
- **Limiti dinamici**: `max_*_accel(_xyz)`, `max_*_jerk(_xyz)`
- **Saturazione velocità**: `max_linear_speed`, `max_angular_speed`, `speed_saturation_eps`
- **Tempo**: `control_rate`, `dt_min_factor`, `dt_max_factor`, `dt_use_substepping`, `dt_max_substeps`

---

## 9) Interpretazione rapida (senza limitatori/saturazioni)

Ignorando jerk limiting e saturazioni, la parte lineare per asse è un primo ordine su $v$ con costante di tempo $T=M/D$:

$$
v(s) = \frac{1}{M_v s + D_v}\Big(F_{\text{hand}}(s) - F_{\text{fb}}(s)\Big),
\qquad T=\frac{M}{D}.
$$

Quindi:

- aumentare $D$ rende $v$ più “frenata” e meno sensibile al rumore
- aumentare $M$ rende la risposta più “inerziale” (più lenta)
- aumentare $k_f$ amplifica il loop di riflessione forza (più “presenza” del contatto, ma maggiore rischio di oscillazioni se la catena slave→master è rumorosa/ritardata)

