# Risposte (puntuali) su rate, filtri e sottrazione nel force-feedback  
Setup di riferimento: `launch/master_slave_mur620b_real.launch` + YAML `config/master_real_mur620b_ur10_l.yaml` e `config/slave_real_mur620b_ur10_r.yaml`.

Contesto rapido (quello che stai usando adesso):
- **Master node**: `teleop_master_haptic_controller`  
  - loop interno di controllo: `control_rate: 250 Hz`  
  - publish comando twist su `command_topic` (es. `/mur620b/UR10_l/twist_controller/command_collision_free`)  
  - publish *target* verso slave (pose + feedforward twist) su timer separato a `slave_publish_rate: 250 Hz`
- **Slave node**: `teleop_slave_twist_outer_loop`  
  - loop interno di controllo: `control_rate: 250 Hz`  
  - publish comando twist su `command_topic` (es. `/mur620b/UR10_r/twist_controller/command_collision_free`)  
  - publish feedback pose verso master su timer separato a `master_feedback_publish_rate: 250 Hz`

In entrambi i nodi esiste **sub-stepping** sul tempo (`dt_use_substepping`, `dt_max_substeps`) ma serve per gestire jitter/late tick; non nasce per aumentare il rate di publish.

---

## 1) Se alzo la frequenza del controllo (master+slave), a che frequenza viene pubblicato il comando sul topic?

### Risposta diretta
- **Master**: il messaggio su `command_topic` viene pubblicato **una volta per tick** del timer di controllo, quindi (idealmente) a:
  - $\;f_{\text{cmd,master}} \approx \texttt{control\_rate}\;$
- **Slave**: analogamente:
  - $\;f_{\text{cmd,slave}} \approx \texttt{control\_rate}\;$

### Dettaglio importante: i sub-step NON moltiplicano la frequenza di publish
Nel master e nello slave il controllo fa:
- calcolo `dt` misurato
- eventuale `n_substeps > 1` se `dt` è grande (tick in ritardo)
- **integrazione interna** in un `for (k=0; k<n_substeps; ++k)`
- **publish del comando una sola volta** alla fine del tick

Quindi:
- **aumentare `dt_max_substeps` non aumenta la frequenza dei messaggi**
- il sub-stepping serve a “spezzare” un dt grande per evitare salti/instabilità numerica quando il thread non gira regolare

### Timer “secondari” (non confondere con il comando twist)
Nel tuo setup ci sono altri due rate indipendenti, su timer separati:
- **Master → Slave target** (pose + ff twist): $\;f_{\text{target}} \approx \texttt{slave\_publish\_rate}\;$
- **Slave → Master feedback pose**: $\;f_{\text{fbpose}} \approx \texttt{master\_feedback\_publish\_rate}\;$

Questi **non cambiano automaticamente** se aumenti `control_rate`.

### Implicazione pratica con il tuo vincolo “massimo 500 Hz”
Se la massima frequenza a cui puoi **inviare** comandi (o il downstream controller può consumarli “bene”) è **500 Hz**, allora:
- tieni **`control_rate <= 500`** su entrambi i nodi (master e slave)
- tieni **`slave_publish_rate <= 500`** e **`master_feedback_publish_rate <= 500`**
- se aumenti `control_rate` dello slave oltre `slave_publish_rate` del master, lo slave controllerà più spesso usando target/ff “vecchi” (finché non vanno in timeout): non è necessariamente utile e può peggiorare la sensazione di ritardo

Nota ROS: entrambe le pubblicazioni usano queue size 1; se pubblichi più veloce di quanto un subscriber/elaboratore riesca a processare, spesso **vedrai solo l’ultimo messaggio** (comportamento “latest-wins”), quindi “sparare” oltre il limite porta più carico che beneficio.

---

## 2) Filtri esponenziali con cutoff → posso disabilitarli? Come?

### 2.1 Filtro EMA sui wrench (quello che usi per calcolare `alpha` da `cutoff_hz`)
Sì: puoi disabilitare il **passa-basso EMA** senza toccare il codice.

La logica dei nodi è:
- se `*_wrench_filter_cutoff_hz > 0` ⇒ **usa cutoff** e calcola $\alpha$ da $f_c$
- altrimenti ⇒ usa `*_wrench_filter_alpha` (clampato in $[0,1]$)
- il filtro EMA è effettivamente applicato solo se $0 < \alpha < 1$

Per avere **nessun low-pass EMA** (cioè comportamento “pass-through”):
- imposta **cutoff <= 0** (così non override l’alpha)
- imposta **alpha = 1.0** (così $0<\alpha<1$ è falso e l’EMA è bypassata)

#### Master (`teleop_master_haptic_controller`)
Nel master ci sono **due pipeline distinte** (master locale vs feedback dallo slave):
- master locale: `master_wrench_filter_*` (fallback su `wrench_filter_*`)
- feedback: `feedback_wrench_filter_*` (fallback su `wrench_filter_*`)

Per disabilitare davvero l’EMA, una configurazione “sicura” (esplicita) è:
- `wrench_filter_cutoff_hz: 0.0`
- `wrench_filter_alpha: 1.0`
- `master_wrench_filter_cutoff_hz: 0.0`
- `master_wrench_filter_alpha: 1.0` *(opzionale, ma consigliato per chiarezza)*
- `feedback_wrench_filter_cutoff_hz: 0.0`
- `feedback_wrench_filter_alpha: 1.0` *(opzionale, ma consigliato per chiarezza)*

#### Slave (`teleop_slave_twist_outer_loop`)
- `wrench_filter_cutoff_hz: 0.0`
- `wrench_filter_alpha: 1.0`

### 2.2 Attenzione: “disabilitare i filtri” ≠ disabilitare tutta la catena di conditioning
Anche con EMA disabilitato, rimangono comunque:
- deadband (soft, con isteresi)
- saturazioni/clamp in norma (`max_force`, `max_torque`, …)

Questi non sono “filtri” in senso LTI ma sono non-linearità che cambiano molto la dinamica e possono influire sulla stabilità/feeling.

### 2.3 Altri low-pass presenti (non sul wrench) che puoi disabilitare
Nel master hai anche low-pass su parametri “schedulati”:
- `dynamic_damping/d_extra_lowpass_cutoff_hz`: **<= 0 disabilita**
- `dynamic_mass/m_extra_lowpass_cutoff_hz`: **<= 0 disabilita**

Nel tuo YAML c’è già la nota: “`(<=0 disables)`”.

---

## 3) Nella sottrazione dell’ammettenza master: meglio sottrarre prima e filtrare dopo?

### 3.1 Risposta matematica (caso ideale)
Se il filtro è **lineare** e identico per entrambi i segnali (stesso $\alpha$/stessa $f_c$) e non ci sono non-linearità in mezzo, allora:

$$
\mathrm{LP}\big(F_{\text{hand}} - F_{\text{fb}}\big)
=
\mathrm{LP}(F_{\text{hand}}) - \mathrm{LP}(F_{\text{fb}})
$$

Quindi, **in quel caso** “filtrare prima o dopo la sottrazione” è equivalente: non cambia nulla.

### 3.2 Perché nel tuo caso NON è equivalente (e può influire sulla stabilità)
Nel master attuale succede questo:
1) ruoti in frame target
2) applichi **EMA + deadband + clamp** separatamente a:
   - `master_raw` → `master_filt_`
   - `slave_raw`  → `slave_filt_`
3) componi il feedback:
   - $F_{\text{fb}} = F_{\text{spring}} + (-k_f\,F_{s,\text{filt}}) + F_{c,\text{filt}}$
4) entri in ammettenza con:
   - RHS $\propto (F_{\text{hand}} - F_{\text{fb}}) - Dv$

Non è equivalente a “sottraggo e poi filtro” per due motivi grossi:
- **(A) filtri diversi**: puoi avere `master_wrench_filter_cutoff_hz` diverso da `feedback_wrench_filter_cutoff_hz` (nel tuo YAML: 30 Hz vs 25 Hz).  
  Anche se entrambi sono “solo low-pass”, introducono **ritardi/phase-lag diversi** → la sottrazione può creare componenti spurie (specie in banda alta) che alimentano oscillazioni.
- **(B) non-linearità prima della sottrazione**: deadband con isteresi + clamp in norma rompono la linearità.  
  Quindi $\mathrm{deadband}(\cdot)$ e $\mathrm{clamp}(\cdot)$ applicati separatamente a master/slave **non commutano** con la sottrazione.

### 3.3 Quindi “meglio” farlo? Dipende dall’obiettivo, ma c’è una gerarchia di mosse sensate
Se il problema principale è **instabilità da ritardo**, la cosa più spesso efficace (a parità di architettura) è ridurre la “energia” della banda alta nel loop master↔slave (e minimizzare mismatch di fase), con mosse incrementali:

#### Opzione 0 (solo parametri, consigliata come primo test)
- **allinea i filtri**: imposta *uguali* `master_wrench_filter_cutoff_hz` e `feedback_wrench_filter_cutoff_hz` (o rendi il feedback più filtrato, mai meno)  
  Obiettivo: ridurre **differential delay** tra $F_{\text{hand}}$ e $F_{\text{fb}}$.
- se necessario, **abbassa** `feedback_wrench_filter_cutoff_hz` (es. 10–20 Hz) invece di alzare il rate: spesso stabilizza più dell’aumento frequenza.

#### Opzione 1 (ancora parametri): non “dare doppio feedback”
Nel tuo master hai sia:
- **molla virtuale** (position-based feedback) tramite `slave_actual_pose_topic` + `spring_stiffness_*`
sia
- **force reflection** tramite `force_reflection_scale` / `torque_reflection_scale` sul wrench dello slave

Con ritardi non trascurabili, la force reflection è quella più delicata. Se vedi instabilità:
- riduci `force_reflection_scale` / `torque_reflection_scale` (o prova a portarle a zero temporaneamente) e lascia la molla come principale “feeling” di contatto, perché è spesso più robusta ai ritardi.

#### Opzione 2 (modifica architettura): “sottraggo prima e filtro dopo”
Ha senso **solo se** il tuo obiettivo è “un solo filtro coerente sulla forza netta” per eliminare mismatch, ma va progettato bene:
- decidere dove mettere deadband/clamp (prima o dopo) cambia molto la dinamica e la sicurezza
- se filtri dopo la sottrazione, stai filtrando insieme anche mano+ambiente: feeling più “morbido” e potenzialmente più ritardo percepito

Se proprio vuoi avvicinarti al comportamento “sottraggo e filtro dopo” senza cambiare codice, la via più pulita è:
- usare **lo stesso filtro** su entrambe le pipeline (stesso cutoff/alpha)
Così, almeno per la parte lineare, ti avvicini al caso equivalente.

---

## Takeaway (in 3 righe)
- **Rate dei comandi twist**: ~`control_rate` (1 publish per tick), i sub-step **non** aumentano la frequenza di publish.  
- **Disabilitare EMA**: imposta `*_wrench_filter_cutoff_hz <= 0` e `*_wrench_filter_alpha = 1.0` (master e/o slave); restano deadband/clamp.  
- **Filtro prima/dopo sottrazione**: in teoria equivalente solo se filtro identico e niente non-linearità; nel tuo caso non lo è (cutoff diversi + deadband/clamp), quindi prima prova ad **allineare cutoffs** e ridurre/refinare il feedback FT prima di cambiare architettura.

