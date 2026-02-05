## Proposta: `JointVelocityFilter` (analogo a `CartesianVelocityFilter`) con jerk→acc→vel e $\tau$, mantenendo scaling uniforme

### Perché questa opzione è adatta al tuo workflow

Se il problema pratico è la **frenata** (es. stop troppo secco quando il comando scende a zero, o transizioni brusche quando cambia target), un filtro con **$\tau$** ti dà una “manopola” diretta per regolare **quanto rapidamente** vuoi inseguire il comando desiderato, mentre jerk/acc/vel garantiscono che la dinamica resti fisicamente realizzabile.

In altre parole:

- **$\tau$** controlla la “morbidezza” della risposta (più grande = più dolce, più piccolo = più reattivo)
- **jerk/acc/vel** impediscono scatti e saturazioni dinamiche
- **scaling uniforme** preserva la direzione del vettore $\dot q$ (coerente col tuo approccio attuale)
- `JointSafetyLimiter` resta come **hard guardrail** finale (safety), anche se il filtro fa già molto lavoro.

---

### Dove inserirlo nella pipeline (minima invasività)

Oggi (semplificando) fai:

1. `command_twist` (cartesiano)
2. `JacobianSolver` → $\dot q_\text{cmd}$
3. `JointSafetyLimiter` → $\dot q_\text{out}$

Propongo:

1. `JacobianSolver` → $\dot q_\text{cmd}$
2. **NEW** `JointVelocityFilter` → $\dot q_\text{filtered}$
3. `JointSafetyLimiter` (hard) → $\dot q_\text{out}$

Nota: questo mantiene l’architettura attuale e introduce solo un componente “comfort/smoothness” prima del limiter finale.

---

### API e struttura (spec simile a `CartesianVelocityFilter`)

Nuovo componente:

- `include/cartesian_velocity_controller/joint_velocity_filter.hpp`
- `src/joint_velocity_filter.cpp`

Interfaccia suggerita:

- `reset()`
- `resetToState(qdot, qddot = 0, qjerk = 0)` (opzionale)
- `setEnabled(bool)`
- `setTimeConstant(double tau)`
- `setLimits(max_vel, max_acc, max_jerk)` (scalari o `Eigen::VectorXd`)
- `Eigen::VectorXd filter(const Eigen::VectorXd& desired_qdot, double dt)`

Stato interno:

- $\dot q$ corrente (vettore)
- $\ddot q$ corrente (vettore)
- $\dddot q$ corrente (vettore, opzionale per debug)

---

### Algoritmo proposto (analogo al filtro cartesiano, ma in spazio giunti)

Vogliamo un algoritmo “tipo Level C” ma applicato a $\dot q$:

1) **Accelerazione desiderata**:

$$
\ddot q_\text{des} = \frac{\dot q_\text{des}-\dot q}{\tau}
$$

2) **Jerk comandato**:

$$
\dddot q_\text{cmd} = \frac{\ddot q_\text{des}-\ddot q}{dt}
$$

3) **Saturazione jerk con scaling uniforme**

Qui sta la parte chiave: invece di clamp per-giunto, calcolo un fattore unico $s_\text{jerk}\in[0,1]$ tale che:

$$
|\; s_\text{jerk}\,\dddot q_{\text{cmd},i}\;| \le \dddot q_{\max,i}\quad \forall i
$$

che si ottiene con:

$$
s_\text{jerk}=\min_i \frac{\dddot q_{\max,i}}{|\dddot q_{\text{cmd},i}|}
\quad (\text{solo sui giunti che sforano})
$$

poi:

$$
\dddot q \leftarrow s_\text{jerk}\,\dddot q_\text{cmd}
$$

4) **Aggiornamento accelerazione + saturazione acc con scaling uniforme**

$$
\ddot q_\text{new}=\ddot q + \dddot q\,dt
$$

Se $|\ddot q_{\text{new},i}|>\ddot q_{\max,i}$, applico:

$$
s_\text{acc}=\min_i \frac{\ddot q_{\max,i}}{|\ddot q_{\text{new},i}|}
$$

e:

$$
\ddot q_\text{new} \leftarrow s_\text{acc}\,\ddot q_\text{new}
$$

5) **Aggiornamento velocità + saturazione vel con scaling uniforme**

$$
\dot q_\text{new}=\dot q + \ddot q_\text{new}\,dt
$$

Se $|\dot q_{\text{new},i}|>\dot q_{\max,i}$, applico:

$$
s_\text{vel}=\min_i \frac{\dot q_{\max,i}}{|\dot q_{\text{new},i}|}
$$

e:

$$
\dot q_\text{new} \leftarrow s_\text{vel}\,\dot q_\text{new}
$$

6) **Update stato**

- $\dot q \leftarrow \dot q_\text{new}$
- $\ddot q \leftarrow \ddot q_\text{new}$
- $\dddot q \leftarrow \dddot q$ (se lo memorizzi)

**Nota importante**: questa logica è volutamente “in cascata” come nel filtro cartesiano (prima jerk, poi acc, poi vel), ma usa scaling uniforme calcolato su componenti per-giunto.

---

### Cosa ottieni sulla “frenata”

Se $\dot q_\text{des}$ va a zero (stop), allora:

$$
\ddot q_\text{des}\approx -\frac{\dot q}{\tau}
$$

Quindi $\tau$ definisce quanto forte vuoi decelerare (prima dei limiti jerk/acc):

- $\tau$ piccolo → freni più forte → più reattivo ma rischio “secco”
- $\tau$ grande → freni più dolce → più comfort ma più inerzia

E i limiti jerk/acc impediscono comunque che la decelerazione cambi troppo rapidamente.

---

### Parametri ROS proposti

In YAML (nuova sezione):

- `joint_velocity_filter/enabled` (bool, default `false` per non cambiare comportamento)
- `joint_velocity_filter/tau` (double, es. 0.05–0.20)
- `joint_velocity_filter/max_joint_velocity` (double, rad/s) oppure array (size=6)
- `joint_velocity_filter/max_joint_acceleration` (double, rad/s²) oppure array (size=6)
- `joint_velocity_filter/max_joint_jerk` (double, rad/s³) oppure array (size=6)
- `joint_velocity_filter/dt_nominal` (double, s): timestep interno del filtro (consigliato costante)
- `joint_velocity_filter/min_dt` (double, s): clamp minimo per `dt` (guardrail numerico)
- `joint_velocity_filter/max_dt` (double, s): clamp massimo per `dt` (evita “salti” enormi)
- `joint_velocity_filter/max_substeps` (int): massimo numero di sub-step per ciclo (limite CPU)
- `joint_velocity_filter/reset_dt_threshold` (double, s): se `dt` supera questa soglia → policy di reset/hold
- `joint_velocity_filter/large_dt_policy` (string: `hold_last` / `reset_to_zero` / `reset_to_desired`)

Nota pratica sui limiti (il tuo requisito “3+3”):
- se vuoi stessi limiti per i primi 3 giunti e altri limiti per gli ultimi 3, passa comunque un array size=6:
  - `[$v_{arm}, $v_{arm}, $v_{arm}, $v_{wrist}, $v_{wrist}, $v_{wrist}]` (e analogo per acc/jerk)
  - esempio YAML:

```yaml
joint_velocity_filter:
  max_joint_velocity:     [1.0, 1.0, 1.0, 2.0, 2.0, 2.0]
  max_joint_acceleration: [3.0, 3.0, 3.0, 6.0, 6.0, 6.0]
  max_joint_jerk:         [30.0, 30.0, 30.0, 60.0, 60.0, 60.0]
```

Consiglio: anche se esiste già `joint_safety_limiter/max_joint_velocity`, tenere parametri separati ti permette di:

- usare limiti più “morbidi” nel filtro (comfort)
- lasciare limiti più “hard” nel safety limiter (safety)

Dynamic reconfigure (nuovo group `joint_filter` o estensione di `safety`):

- `joint_filter_tau`
- `joint_filter_max_acceleration`
- `joint_filter_max_jerk`
- (opzionale) `joint_filter_enabled`

---

### Interazione con `JointSafetyLimiter` (doppio stadio)

Raccomandazione pratica:

- `JointVelocityFilter` = **smoothness / comfort**
- `JointSafetyLimiter` = **hard safety** (vel + acc) con scaling uniforme

Se vuoi evitare “doppie limitazioni”:

- puoi lasciare `JointSafetyLimiter` attivo (consigliato) ma aspettarti che scali raramente
- in alternativa, se ti fidi del filtro, puoi disabilitare `acceleration_limiting_enabled` nel limiter e lasciargli solo il cap di velocità come last resort (meno robusto).

---

### Criticità / punti aperti

- **(1) Scelta dei limiti: per-giunto vs uniformi**  
  Il filtro lavora meglio con limiti per-giunto, ma puoi partire con scalari uniformi come fai oggi. **Risposta:** Io voglio che i limiti siano uguali per i primi 3 giunti e abbiano dei propri valori, mentre ne voglio altri per gli ultimi 3 giunti.

- **(2) Stabilità con $dt$ variabile**  
  Qui $dt$ entra due volte (jerk), quindi spike/jitter su `dt` possono creare comandi “esplosivi” (sensibilità tipo $1/dt^2$). La soluzione più robusta è rendere il filtro **indipendente dal `dt` istantaneo** usando un timestep interno fisso `dt_nominal` con accumulatore:

  - **Idea**: accumulo tempo reale `dt` (clampato), poi eseguo 0..N sub-step del filtro sempre con `dt_nominal`.
  - **Vantaggi**: niente jerk enorme quando `dt` è piccolo; comportamento molto più deterministico.
  - **Protezione CPU**: imposto `max_substeps` per evitare di “recuperare” troppo in caso di `dt` grande.
  - **Outlier**: se `dt > reset_dt_threshold` applico una policy (hold/reset) invece di integrare con un salto enorme.

  Pseudocodice:

```cpp
// each control cycle:
dt_clamped = clamp(dt, min_dt, max_dt);

if (dt > reset_dt_threshold)
{
  if (large_dt_policy == HOLD_LAST)      return qdot_last;
  if (large_dt_policy == RESET_TO_ZERO)  resetToState(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n));
  if (large_dt_policy == RESET_TO_DESIRED) resetToState(desired_qdot, Eigen::VectorXd::Zero(n));
}

accumulator_ += dt_clamped;
int steps = 0;
while (accumulator_ >= dt_nominal && steps < max_substeps)
{
  step(desired_qdot, dt_nominal); // one internal jerk->acc->vel update
  accumulator_ -= dt_nominal;
  steps++;
}

return qdot_;
```

  **Estensione richiesta: stessa modalità di `dt` fisso anche per `CartesianVelocityFilter`**  
  Il filtro cartesiano (Level C) è anch’esso sensibile al `dt` (integrazione jerk→acc→vel in task space). Per coerenza e robustezza, propongo di applicare **la stessa identica policy** già descritta sopra:

  - aggiungere a `CartesianVelocityFilter` i parametri:
    - `cartesian_velocity_filter/dt_nominal` (double, s)
    - `cartesian_velocity_filter/min_dt`, `cartesian_velocity_filter/max_dt` (double, s)
    - `cartesian_velocity_filter/max_substeps` (int)
    - `cartesian_velocity_filter/reset_dt_threshold` + `large_dt_policy` (hold/reset)
  - implementare (o riusare) un accumulatore interno e fare 0..N sub-step con `dt_nominal` (esattamente come nello pseudocodice sopra)
  - mantenere `tau` espresso in secondi: con `dt_nominal` fisso il comportamento del filtro diventa molto più deterministico al variare del jitter del loop.

  **Come scegliere `dt_nominal` in modo coerente con la frequenza del loop (100–500 Hz, anche se non nota)**  
  Due opzioni pratiche (non mutualmente esclusive):

  - **Opzione A (raccomandata: esplicita e deterministica)**: esporre un singolo parametro “di sistema” (es. `control_loop_rate_hz` oppure direttamente `dt_nominal`) e usare:
    - $ dt_\text{nominal} = 1 / f_\text{loop} $
    - esempi: 100 Hz → 0.01 s, 500 Hz → 0.002 s
    - vantaggio: cambi il rate del loop e aggiorni *un solo parametro*, valido sia per il filtro in giunti sia per il filtro cartesiano.

  - **Opzione B (auto‑calibrazione all’avvio: coerente col rate reale)**: se `dt_nominal <= 0` o `*_filter/auto_dt_nominal=true`,
    - raccogli una finestra di campioni `dt` (clampati) per N cicli (es. N=200),
    - stima un `dt_est` robusto (es. mediana),
    - imposta `dt_nominal = dt_est` (o, se vuoi quantizzare, `dt_nominal = 1/round(1/dt_est)`),
    - logga a console il valore stimato e quello “equivalente” in Hz (utile quando non sai a priori se il loop sta girando a 100 o 500 Hz).
    - nota: se prevedi che il rate possa cambiare a runtime, puoi ri‑armare la calibrazione quando `dt` resta lontano da `dt_nominal` per M cicli consecutivi (soglia tipo 20–30%).

  - **Risposta:** Va bene l'opzione A. Non implementare l'opzione B, non mi serve

- **(3) Reset / continuità**  
  Serve una policy chiara su `reset()`:
  - a start controller: `qdot=qddot=qjerk=0`
  - quando cambia target bruscamente: scegliere se resettare o mantenere stato (come fai per il filtro cartesiano con `reset_filter_on_target_change`)
  **Risposta:** Quando il target cambia voglio mantenere lo stato attuale del filtro

- **(4) Relazione con “scaling uniforme”**  
  Questo filtro preserva direzione del vettore $\dddot q$, $\ddot q$, $\dot q$ a ogni saturazione (scaling). È coerente col tuo stile, ma significa che un singolo giunto “limitante” rallenta tutti.
  **Risposta:** Voglio poter abilitare o disabilitare questa funzionalità. Probabilmente lavorerò con i giunti indipendentemente l'uno dall'altro.

---

### Confronto rapido con l’alternativa “jerk nel JointSafetyLimiter”

- **Solo jerk nel limiter**: ottimo per safety e semplice, ma non hai $\tau$ per “modellare” la frenata (solo vincoli hard tra campioni).
- **Filtro con $\tau$ + limiter** (questa proposta): migliore per il tuo obiettivo “frenata” + mantiene safety.


