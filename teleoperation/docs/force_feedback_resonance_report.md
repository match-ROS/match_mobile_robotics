# Analisi risonanza feedback di forza (teleoperation)

Documento basato **solo** su:
- `launch/master_slave_mur620b_real.launch`
- `config/master_real_mur620b_ur10_l.yaml`
- `config/slave_real_mur620b_ur10_r.yaml`
- `src/teleop_master_haptic_controller_node.cpp`
- `src/teleop_slave_twist_outer_loop_node.cpp`

## Architettura effettiva (da `master_slave_mur620b_real.launch`)

- **MASTER** (`teleop_master_haptic_controller`)
  - Legge **wrench master** (`master_wrench_topic`)
  - Legge **wrench slave** (`slave_wrench_topic_for_master` → di default è uguale a `slave_wrench_topic`, quindi **abilitato**)
  - Esegue un’**admittance**: $ M \dot{v} + D v = (F_\text{hand} - F_\text{feedback}) $
  - Pubblica comandi twist al master e pubblica target (pose+twist) verso lo slave.
  - Implementa una **molla virtuale** basata sulla posa reale dello slave (topic `slave_actual_pose_topic`).

- **SLAVE** (`teleop_slave_twist_outer_loop`)
  - Consuma target pose + feedforward twist dal master.
  - Controllo principale: **spring control** (abilitato in YAML: `use_spring_control: true`) + eventuale compliance su forza.
  - Pubblica una `PoseStamped` del TCP reale sul topic `master_feedback_pose_topic` per la molla virtuale sul master.

## Cosa può generare risonanza / oscillazioni che si amplificano

### Premessa (come da tuo setup): override pose OK

Nel seguito **assumo** che gli override dei frame usati per pubblicare/consumare le pose (master→slave e slave→master) siano corretti nel tuo sistema perché:
- i due robot sono identici
- i frame scelti sono coerenti “numericamente” (stessa origine/assi nel contesto in cui li usi)

Con questa assunzione, la sorgente più probabile della risonanza diventa la catena di **force reflection da wrench** (rotazioni frame + guadagni + filtri), che nel codice ha un punto oggettivamente critico.

### 1) Problema “strutturale” molto probabile: un solo `wrench_source_frame_override` viene applicato sia a wrench master sia a wrench slave

Nel master, la conversione `wrenchMsgToWrench3()` usa **sempre** lo stesso `wrench_source_frame_override_` per QUALSIASI wrench (master/slave/coupling):

- se l’override è non vuoto, **ignora** `msg.header.frame_id` e usa `wrench_source_frame_override_`.

Nel tuo launch, per il master viene impostato:
- `wrench_source_frame_override = master_tool_frame`

Quindi il wrench del **braccio slave** (topic `/.../UR10_r/wrench`) viene ruotato verso `wrench_target_frame` assumendo come frame di origine **il tool del master**, non il tool reale dello slave.

Questo è sufficiente a creare un “feedback” che:
- ha direzione sbagliata
- non è dissipativo ma **eccita** certe direzioni → oscillazione che si amplifica.

**Nota**: questo punto è indipendente dall’identicità dei robot; anche se identici, i TF id e le pose del tool master/slave sono diversi nel TF tree e non sono intercambiabili a livello di rotazione del wrench.

### 2) Problema “strutturale” (secondario se i frame sono davvero coerenti): override dei frame = **rilabel** (nessuna trasformazione)

Nel launch vengono usati override dei frame id sia nei target master→slave sia nel feedback slave→master.

#### Master→Slave (target pose)
Nel master, la posa viene ottenuta via TF e poi pubblicata con `header.frame_id = slave_frame_id_override` **senza trasformare i numeri**:
- `TeleopMasterHapticController::slaveTargetTick()` pubblica la posa letta da `lookupTransform(slave_base_frame_, slave_tcp_frame_)` e poi fa:
  - `frame_out = slave_frame_id_override_ ? slave_frame_id_override_ : slave_base_frame_`
  - `pose_msg.header.frame_id = frame_out`
  - ma posizione/orientamento rimangono quelli di `T` (quindi espressi in `slave_base_frame_`)

Nel launch, però:
- `slave_base_frame` viene forzato a **base master** (`master_base_inertia_frame`)
- `slave_frame_id_override` viene forzato a **base slave** (`slave_base_inertia_frame`)

Risultato: lo slave riceve una pose numericamente in base master, ma etichettata come base slave e quindi **non la trasforma** (perché `transformPoseToBase()` nello slave, se `frame_id == base_frame`, prende i numeri “as is”).

#### Slave→Master (posa reale per molla virtuale)
Nel codice slave, `TeleopSlaveTwistOuterLoop::masterFeedbackTick()` costruisce `PoseStamped` con:
- numeri = posa TCP espressa in `base_frame_` (perché `getTcpPose()` fa `lookupTransform(base_frame_, tcp_frame_, ...)`)
- `header.frame_id = master_fb_frame_override_` se impostato (launch lo imposta a **master_base_inertia_frame**)
- anche qui: **rilabel, nessuna trasformazione** dei numeri.

Sul master, `slaveActualPoseCb()` salva solo numeri (pos+quat) e **ignora completamente `header.frame_id`**: usa quei numeri direttamente nella molla virtuale:
- `delta_p = p_master - p_slave`

Se `p_master` e `p_slave` non sono veramente nello **stesso frame fisico**, la molla virtuale genera forze spurie (spesso saturate a `max_spring_force`) che possono alimentare un’oscillazione.

**Segnale tipico in questo scenario**
- anche “a vuoto” o senza contatto, la molla virtuale produce una forza non nulla e spesso vicino al clamp `max_spring_force`.

### 3) Guadagni/parametri: configurazione attuale è intrinsecamente “aggressiva”

Dal YAML master:
- `force_reflection_scale: 0.8` (molto alto)
- con molla virtuale attiva (`spring_stiffness_linear: 200 N/m`, `spring_damping_linear: 20 N*s/m`)

Nota importante: nello stesso YAML c’è un’indicazione esplicita che con la molla attiva il force reflection dovrebbe essere **basso** (ordine 0.01–0.05), mentre qui è 0.8.

Inoltre la molla lineare:
- con massa virtuale `mass_linear: 3.0` (master)
- ha una damping `spring_damping_linear: 20`

Se si ragiona “da secondo ordine” (indicativo), uno smorzamento critico approssimato è:
$$
 B_c \approx 2\sqrt{K M} = 2\sqrt{200 \cdot 3} \approx 49 \; \text{N·s/m}
$$
Quindi 20 N·s/m è tendenzialmente **sotto-smorzato** → oscillazioni più probabili.

### 4) Filtri: cutoff alti sui wrench aumentano la probabilità di eccitare modi meccanici

Master:
- `wrench_filter_cutoff_hz: 30`
- `feedback_wrench_filter_cutoff_hz: 40`

Slave:
- `wrench_filter_cutoff_hz: 20`

Cutoff alti + guadagno alto su forza riflessa = più energia alle frequenze dove la struttura (braccio + sensore + controller twist) può avere risonanze.

### 5) Segni e convenzioni del wrench (possibile)

Nel master, la forza dello slave viene riflessa con segno invertito:
- `F_feedback = F_spring + (-kf_force * slave_filt_.f) + ...`

Se il topic `/.../wrench` pubblica una convenzione diversa da quella attesa (forza “sul tool” vs “del tool sull’ambiente”), il segno potrebbe risultare effettivamente “positivo” in catena e quindi destabilizzante.

Nel codice slave inoltre la compliance è:
- `v_comp_lin = k_adm_linear * wrench_filt_.f;` (segno +)

Non è “sbagliato” in assoluto: dipende dalla convenzione del sensore. Se però la convenzione è opposta, il termine compliance può spingere *dentro* al contatto invece che uscirne, alimentando oscillazioni.

## Soluzioni possibili (pratiche), ordinate per impatto e rischio

### A) Stabilizzazione rapida “senza codice” (solo parametri/launch)

1) **Disabilita temporaneamente il force reflection** (per isolare la causa)
- In `master_slave_mur620b_real.launch` imposta:
  - `slave_wrench_topic_for_master` a stringa vuota (così `slave_wrench_topic` del master diventa `""`)

Se l’oscillazione sparisce → la causa è nella catena `slave_wrench -> master feedback`.

2) **Riduci drasticamente `force_reflection_scale`** (se vuoi mantenerlo attivo)
- In `master_real_mur620b_ur10_l.yaml` porta:
  - `force_reflection_scale` da `0.8` a un valore molto più basso (tipicamente 0.01–0.05 come suggerito dallo stesso file)

2b) **Rimuovi l’override del frame del wrench sul master (test diagnostico molto forte)**

Dato come è scritto il master, se vuoi usare force reflection da `/.../UR10_r/wrench` devi evitare che il master “forzi” il frame sorgente a quello del tool master.

Senza cambiare codice hai due opzioni (mutuamente esclusive):
- **Opzione 1 (preferita)**: assicurati che *entrambi* i wrench (`UR10_l/wrench` e `UR10_r/wrench`) abbiano `header.frame_id` corretto e connesso in TF, poi nel launch imposta sul master:
  - `wrench_source_frame_override` = `""` (vuoto)
- **Opzione 2**: se *solo* il master ha `header.frame_id` “rotto” e ti serve l’override per lui, allora il force reflection del wrench slave è intrinsecamente rischioso con questo codice: in tal caso lascia `wrench_source_frame_override` al master tool e **disabilita** `slave_wrench_topic_for_master` (usa solo la molla virtuale per feedback).

3) **Aumenta smorzamento sul master**
- `damping_linear` (master) è `10.0` con `mass_linear: 3.0`: è “leggero”.
- Aumentarlo (es. 25–60) riduce la probabilità di risonanza quando arriva una forza riflessa o una molla virtuale.

4) **Rendi la molla virtuale più smorzata**
- A parità di `spring_stiffness_linear: 200`, aumenta `spring_damping_linear` verso valori più alti (indicativamente vicino a ~50 N·s/m è più “critico” rispetto a M=3).
- In alternativa: riduci `spring_stiffness_linear` (es. 50–150 N/m) e poi riporta su gradualmente.

5) **Abbassa la banda del feedback wrench**
- Master:
  - `feedback_wrench_filter_cutoff_hz`: riduci (es. 10–20 Hz)
- Slave:
  - `wrench_filter_cutoff_hz`: riduci se la compliance sullo slave produce jitter

6) **Aumenta deadband e/o limita `max_force_feedback`**
- Master:
  - aumenta `force_deadband_enter/exit` se hai rumore vicino a 1–2 N
  - imposta `max_force_feedback` (se vuoi clamp separato sul contributo di feedback) per evitare “kick” del riflesso.

### B) Correzione della causa più probabile: coerenza dei frame (senza toccare codice, per quanto possibile)

Obiettivo: fare in modo che **i numeri** siano davvero espressi nel frame dichiarato, evitando i rilabel.

#### Target master→slave (pose)
Evita `slave_frame_id_override` quando il frame non coincide realmente con `slave_base_frame`.
Approccio robusto (se TF è connesso):
- pubblica il target in un frame comune, ad esempio `common_base_frame` (`/mur620b/base_link`)
- lascia allo slave la trasformazione verso `base_frame` tramite TF (`transformPoseToBase()` la fa già)

Nel launch, questo significa concettualmente:
- `slave_base_frame` del master = `common_base_frame`
- `slave_tcp_frame` del master = `master_tool_frame`
- `slave_frame_id_override` = `""` (vuoto)

#### Feedback slave→master (posa reale)
Qui c’è una limitazione: il master **non usa** `header.frame_id` e quindi **non trasforma**.
Senza modifiche codice, l’unico modo è garantire che lo slave pubblichi già la posa numericamente nello stesso frame usato dal master per `p_master`.

Dato come è implementato, questo è affidabile solo se:
- `base_frame` (slave) coincide realmente con il frame con cui il master calcola `p_master` per la molla virtuale

Se non coincidono fisicamente (tipico per due bracci distinti), la soluzione corretta è la sezione C (modifica codice).

### C) Fix “definitivo” nel codice (raccomandato se i frame non coincidono fisicamente)

1) **Trasformare davvero la posa nel feedback slave→master**
In `TeleopSlaveTwistOuterLoop::masterFeedbackTick()`:
- se `frame_out != base_frame_`, bisogna trasformare `T_base_tcp` in `frame_out` (via TF) prima di pubblicare i numeri.

2) **Nel master, rispettare `header.frame_id` del feedback pose**
In `TeleopMasterHapticController::slaveActualPoseCb()`:
- oggi salva solo numeri e ignora frame/stamp.
- soluzione: memorizzare anche `frame_id` e poi trasformare quella posa nel frame usato per `p_master` (es. `slave_base_frame_` o un `common_base_frame`) prima di calcolare `delta_p`.

3) **Separare `wrench_source_frame_override` per master e per slave**
Oggi il parametro è unico e si applica a tutti i wrench.
Soluzione robusta:
- aggiungere parametri separati (es. `master_wrench_source_frame_override`, `slave_wrench_source_frame_override`)
- oppure non usare override e affidarsi ai `header.frame_id` corretti per ciascun wrench.

## “Checklist” di debug (sempre basata su ciò che il codice già pubblica)

- Verifica se la molla virtuale sta saturando:
  - master clampa `F_spring_lin` a `max_spring_force` (80 N nel YAML master).
  - se osservi che è quasi sempre a clamp anche senza contatto, è tipicamente **mismatch di frame** o offset enorme.

- Isola i contributi:
  - disabilita `slave_wrench_topic_for_master` → rimane solo molla virtuale (se `slave_actual_pose_topic` è attivo)
  - oppure imposta `spring_stiffness_linear=0` → rimane solo force reflection.

- Controlla la coerenza della rotazione del wrench:
  - se `wrench_source_frame_override` è valorizzato, il master ignora `header.frame_id` del wrench slave: è un punto critico.

