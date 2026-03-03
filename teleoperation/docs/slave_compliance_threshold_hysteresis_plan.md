# Piano: compliance slave con soglia, limite forza e isteresi

Contesto: pacchetto ROS `teleoperation` in `match_mobile_robotics`, setup reale MUR620b + UR10e.
Questa nota sintetizza:
- stato attuale dello **slave** (`teleop_slave_twist_outer_loop`)
- gap rispetto all’obiettivo “compliance che si attiva oltre soglia e limita forza con isteresi”
- piano d’azione implementativo (parametri + modifiche codice)
- punti aperti e criticità (anche di catena master↔slave che impattano la stabilità)

Riferimenti principali nel repo:
- `launch/master_slave_mur620b_real.launch`
- `config/slave_real_mur620b_ur10_r.yaml`
- `src/teleop_slave_twist_outer_loop_node.cpp`
- `include/teleoperation/core/wrench_utils.hpp`
- (contesto) `src/teleop_master_haptic_controller_node.cpp`

---

## 1) Stato attuale: cosa fa davvero lo slave oggi

### 1.1 Nodo e ingressi/uscite
Nodo: `teleop_slave_twist_outer_loop` (`src/teleop_slave_twist_outer_loop_node.cpp`)

Input:
- target pose (`PoseStamped`) dal master
- feedforward twist (`TwistStamped`) dal master
- wrench misurata sullo slave (`WrenchStamped`)
- TF `base_frame -> tcp_frame` per posa TCP corrente

Output:
- `geometry_msgs/Twist` verso `.../twist_controller/command_collision_free` (o equivalente)

### 1.2 Preprocessing della wrench: già robusto (filtro + deadband isteretica + clamp)
La wrench viene:
- ruotata in `base_frame` (rotazione pura dei vettori)
- filtrata (EMA) con `wrench_filter_cutoff_hz` o `wrench_filter_alpha`
- passata in **deadband sulla norma** con **isteresi** (`force_deadband_enter/exit`, idem torque)
- clampata su `max_force`, `max_torque`

Implementazione: `teleoperation::filterClampDeadbandWrenchNorm` in `include/teleoperation/core/wrench_utils.hpp`.

### 1.3 “Compliance” oggi: termine additivo in velocità proporzionale a F (senza soglia dedicata)
Nel codice, il termine chiamato “Compliance” è:
- `v_comp_lin = k_adm_linear * F_filt` (poi clamp su `max_compliance_linear_speed`)
- opzionale angolare con torque se `use_torques=true`

Nota importante: nel file `config/slave_real_mur620b_ur10_r.yaml` attuale:
- `k_adm_linear: 0.00` → la compliance è di fatto **spenta**

### 1.4 Scaling force-aware α(F): *non* è attivo nel tuo setup attuale
Il nodo ha una funzione `computeAlpha(F, v_ff)` con soglie `force_start/force_stop` e modalità `alpha_mode`,
ma viene applicata **solo nel ramo legacy PID**.

Con `use_spring_control: true` (tuo YAML), il codice *non* applica α(F) al feedforward e al tracking:
in spring mode si sommano:
- `k_ff * v_ff` + spring (P + I) + compliance (se attiva)

### 1.5 Meccanismi “contact-aware” già presenti, ma non equivalenti al requisito
Già presenti:
- freeze dell’integrale spring oltre `integral_force_freeze` (nel tuo YAML è 8 N)
- hard guard su forza oltre `hard_force_threshold` per `hard_force_duration` con azione `stop` o `retreat`

Però manca una logica “soft” che:
- si **attivi** oltre una soglia di contatto,
- impedisca “il più possibile” di superare una forza target più bassa del guard “duro”,
- abbia isteresi per evitare chattering.

---

## 2) Requisito desiderato (formulazione operativa)

Obiettivo: introdurre una compliance/limitazione “a stati” sullo slave:

- **NORMAL**: sotto soglia contatto → nessuna limitazione (a parte clamp velocità già esistenti).
- **COMPLIANT**: oltre soglia contatto → comportamento “morbido”/contact-aware.
- **LIMITING**: oltre soglia di forza massima → prevenire che la forza salga ulteriormente, con isteresi.

Vincoli desiderati:
- attivazione oltre soglia `F_on`
- disattivazione sotto soglia `F_off` con `F_off < F_on` (**isteresi**)
- “non superare” una forza `F_max_soft` (in pratica: rendere molto improbabile superarla e ridurre la spinta lungo la normale)
- isteresi anche sul passaggio in/out da limiting

---

## 3) Proposta tecnica: Force/Contact Limiter a stati (isteretico) sul comando twist

### 3.1 Scelta del punto di inserimento (robusta in entrambi i modi)
Inserire la logica **dopo** la composizione di `v_cmd_lin` (spring+ff+compliance) e **prima** del clamp finale su velocità:

- vale sia per `use_spring_control=true` che per `false`
- non dipende da applicare α(F) solo al feedforward
- permette di “tagliare” solo ciò che aumenta il contatto (componente normale)

### 3.2 Variabili geometriche chiave (normale di contatto)
Con `F = wrench_filt_.f`:
- $ f = \|F\| $
- $ \hat n = F / \|F\| $ (se $ f $ > eps)

Decomposizione della velocità lineare:
- componente normale: $ v_\parallel = \hat n (\hat n^\top v) $
- componente tangenziale: $ v_\perp = v - v_\parallel $

Intuizione:
- se $ \hat n^\top v > 0 $ il comando sta “spingendo nella direzione della forza misurata”
  (è un buon proxy per “sto aumentando la compressione/contatto”).
- in limiting, si agisce su $ v_\parallel $, lasciando più libera $ v_\perp $ (scorrimento).

### 3.3 Macchina a stati con isteresi (minima)
Stati:
- `normal`
- `compliant`
- `limiting`

Transizioni (esempio):
- `normal -> compliant` se $ f \ge F_{enable\_enter} $
- `compliant -> normal` se $ f \le F_{enable\_exit} $
- `compliant -> limiting` se $ f \ge F_{limit\_enter} $
- `limiting -> compliant` se $ f \le F_{limit\_exit} $

con:
- $ F_{enable\_exit} < F_{enable\_enter} $
- $ F_{limit\_exit} < F_{limit\_enter} $
- tipicamente $ F_{limit\_*} > F_{enable\_*} $

### 3.4 Azione di controllo per “non superare Fmax” (soft)
In `compliant`:
- ridurre gradualmente la componente **normale resistiva** (non tutta la velocità).

In `limiting`:
- azzerare o saturare fortemente $ v_\parallel $ quando $ \hat n^\top v > 0 $
- opzionale: aggiungere un termine di “retreat” controllato lungo $-\hat n$ se $ f $ supera `F_limit_enter`
  (più efficace per “mollare” forza quando il target resta dentro l’ostacolo).

Implementazione pratica consigliata:
- calcolare un fattore $ s \in [0,1] $ con smoothstep tra due soglie e applicarlo solo a $ v_\parallel^+ $:
  - $ v_\parallel^+ = \hat n \max(0, \hat n^\top v) $
  - $ v_\parallel^- = \hat n \min(0, \hat n^\top v) $ (uscita dal contatto, da NON limitare)

Esempio:
- in `compliant`: $ v \leftarrow v - (1 - s)\,v_\parallel^+ $ (riduci gradualmente la parte che spinge)
- in `limiting`: $ v \leftarrow v - v_\parallel^+ $ (taglio completo della parte “push”)
- se abiliti retreat: $ v \leftarrow v - k_{ret}(f)\,\hat n $ clampato a `retreat_speed_max`

Questa struttura crea “compliance strutturata” senza dover stravolgere spring/PID.

---

## 4) Parametri da introdurre nel YAML dello slave (proposta)

Creare un blocco nuovo, ad esempio:

```yaml
force_limiter:
  enabled: true
  metric: norm                 # norm | parallel_resistive (estendibile)
  enable_enter: 8.0            # N
  enable_exit: 6.0             # N
  limit_enter: 30.0            # N
  limit_exit: 24.0             # N
  mode: scale_parallel         # scale_parallel | clamp_parallel | retreat
  smooth_width: 5.0            # N (opzionale per rampa)
  retreat_speed_max: 0.05      # m/s (solo se mode include retreat)
```

Nota: i valori sopra sono segnaposto; vanno tarati sui tuoi obiettivi (es. “non superare 25 N”).

Allineamenti consigliati:
- `integral_force_freeze` ≈ `force_limiter.enable_enter` (o leggermente sotto)
- `hard_force_threshold` >> `force_limiter.limit_enter` (airbag)

---

## 5) Modifiche codice (dove e cosa)

File principale: `src/teleop_slave_twist_outer_loop_node.cpp`

### 5.1 Aggiunta parametri e stato
- Leggere parametri `~force_limiter/*`.
- Memorizzare:
  - `bool limiter_enabled_`
  - soglie enter/exit
  - `enum class ForceLimiterState { Normal, Compliant, Limiting }`
  - stato corrente `force_limiter_state_`

### 5.2 Applicazione del limiter
Nel metodo `tick()`:
- dopo il calcolo di `v_cmd_lin` (in entrambi i rami spring/PID) e prima del clamp finale:
  - calcolare $ f = \|F\| $ usando `wrench_filt_.f`
  - aggiornare la macchina a stati con isteresi
  - se `Compliant/Limiting`, modificare `v_cmd_lin` secondo decomposizione normale/tangenziale

### 5.3 Telemetria debug (consigliata per tuning)
Aggiungere topic `~debug/force_limiter_state` o `~debug/force_limiter_info` (es. `Float64MultiArray`):
- `f_norm`
- `state` (0/1/2)
- `v_parallel_push` (prima/dopo)

Non è obbligatorio ma accelera molto la taratura.

---

## 6) Test plan (minimo, pratico)

### 6.1 Test di base (in aria)
- `force_limiter.enabled=true`
- muovere il master: lo slave deve seguire come prima, senza “frenate” (f ≈ 0 → state NORMAL).

### 6.2 Contatto controllato (pushing nella normale)
Setup: target continua verso un ostacolo.
Atteso:
- oltre `enable_enter` → entra in COMPLIANT
- oltre `limit_enter` → entra in LIMITING
- in LIMITING la componente $v_\parallel^+$ viene soppressa (e se retreat è attivo, lo slave tende ad alleggerire).

### 6.3 Scorrimento tangenziale (contatto ma movimento laterale)
Atteso:
- componente tangenziale preservata il più possibile, evitando l’effetto “blocco totale”.

### 6.4 Isteresi (no chattering)
Atteso:
- quando la forza oscilla attorno alle soglie, lo stato non commuta rapidamente.

---

## 7) Punti aperti e criticità (da discutere)

### 7.1 (Critico) Convenzione/segno della wrench e della compliance
Nel codice slave, il termine “compliance” è `+ k_adm * F`.
In alcune note/commenti compare il segno opposto (`-k_adm*F`).

**Azione**: fare un test breve in contatto per validare che il segno scelto:
- non amplifichi la spinta verso l’ostacolo
- sia coerente con la convenzione del driver FT (forza ambiente→robot vs robot→ambiente).

**Risposta:** Teoricamente in termine di compliance è giusto come è stato implementato. la convenzione è forza ambiente→robot e quindi dovrebbe essere corretto con il +

### 7.2 In spring mode oggi α(F) non scala il comando
Il tuo YAML imposta `alpha_mode/force_start/force_stop` ma con `use_spring_control=true` quei parametri non agiscono.
Il limiter proposto risolve “a valle”, ma va deciso se:
- mantenere α(F) solo legacy, oppure
- estendere α(F) anche in spring mode (eventualmente su componenti specifiche).

**Risposta:** Mantenere α(F) solo legacy

### 7.3 “Non superare una forza” non è una garanzia assoluta (outer-loop twist)
Dipende da:
- rigidità ambiente
- latenza
- saturazioni/filtri a valle (`twist_controller/command_collision_free`)
- limiti interni UR e safety

Mitigazioni:
- limiter parallelo + (opzionale) retreat
- hard guard ben tarato come fallback

### 7.4 (Sistemico, impatta stabilità) Force reflection sul master: frame override unico per tutti i wrench
Nel master (`teleop_master_haptic_controller_node.cpp`) il parametro `wrench_source_frame_override` è unico e può essere applicato anche al wrench dello slave,
con rischio di rotazione incoerente e feedback destabilizzante quando la force reflection è attiva.

Questo non blocca l’implementazione del limiter slave, ma può rendere il tuning “impossibile” se il loop master↔slave è intrinsecamente eccitato.
**Risposta:** I frame sono stati già verificati, è tutto corretto nell'implementazione

### 7.5 Rotazione pura del wrench (niente termine $p \times f$)
La trasformazione della wrench oggi è “rotation-only”.
Finché `use_torques=false` e si resta vicino allo stesso origin frame, spesso è accettabile; ma è un limite da ricordare se si abilita torque.
**Risposta:** Anche la rotazione pura è corretta, è già stato verificato

### 7.6 Scelta metrica: abs(dot) vs “resistiva”
Se si usa una metrica tipo `parallel`, evitare il valore assoluto:
- meglio limitare solo quando la forza è **resistiva** rispetto alla velocità che stai comandando
  (altrimenti freni anche trazioni o forze non-opponenti).

---

## 8) Deliverable (cosa implementare, in ordine)

1) Implementare `force_limiter` nello slave (parametri + state machine + modifica `v_cmd_lin`).
2) Aggiungere debug minimo (stato + forza + componente normale) per tuning.
3) Tarare soglie:
   - `enable_enter/exit` vicino al punto in cui vuoi “attivare compliance”
   - `limit_enter/exit` al livello “non superare” (+ isteresi)
4) Allineare `integral_force_freeze` con `enable_enter`.
5) Verificare segno wrench/compliance in prova.
6) (Opzionale ma raccomandato) mitigare criticità force reflection frame sul master se si osservano oscillazioni.

