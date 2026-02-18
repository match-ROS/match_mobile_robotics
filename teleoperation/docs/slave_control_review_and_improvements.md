# Review controllo Slave (teleoperation)

Questo documento riassume **cosa è già implementato** nel pacchetto `teleoperation` per il controllo dello **slave** in teleoperazione bilaterale e **cosa migliorerei** per renderlo più robusto e “contact-aware”, in linea con l’obiettivo:

- **feedforward** della velocità twist del master sullo slave;
- inviare la **posizione/orientamento** del master allo slave e compensare l’errore con un **PID** (anti-windup ecc.);
- lo slave non deve essere rigido: deve **rilevare contatto** e **non spingere a forza massima** contro l’ostacolo;
- lo slave fornisce una **wrench** che viene riflessa sul master (force reflection).

Contesto: hai già tarato e verificato il controllo in ammettenza del master (standalone) e vuoi validare/raffinare la parte slave.

---

## 1) Architettura attuale (bilaterale) nel package

### Nodi coinvolti (coppia master↔slave)

La launch `launch/bilateral_pair.launch` istanzia (per una coppia):

- `src/teleop_master_measured_state_publisher_node.cpp` (nome nodo: `teleop_master_measured_state_publisher`)
  - pubblica **stato misurato** del master:
    - `target_pose` (`geometry_msgs/PoseStamped`)
    - `feedforward_twist` (`geometry_msgs/TwistStamped`)
  - twist stimata con **differenza finita su TF** e filtrata/clampata.

- `src/teleop_master_haptic_controller_node.cpp` (nome nodo: `teleop_master_haptic_controller`)
  - implementa il loop di **ammettenza aptica** sul master (già verificato da te).
  - legge `master_wrench_topic` e (opzionale) `slave_wrench_topic` per la **force reflection**.

- `src/teleop_slave_twist_outer_loop_node.cpp` (nome nodo: `teleop_slave_twist_outer_loop`)
  - è l’outer-loop sullo **slave**:
    - consumando `target_pose` + `feedforward_twist` dal master
    - e la `wrench` misurata sullo slave
    - pubblica un `geometry_msgs/Twist` verso la pipeline di velocità dello slave (tipicamente `twist_controller/...` oppure un limiter).

Esiste anche una implementazione più vecchia in `legacy/` (`legacy/src/teleop_slave_controller.cpp`) che fa tracking + feedforward ma comanda in **velocità di giunto** con Jacobiano e limiter joint-level. Al momento, le launch bilaterali “nuove” usano l’outer-loop twist (`teleop_slave_twist_outer_loop`), non il legacy.

---

## 2) Controllo slave “nuovo”: `teleop_slave_twist_outer_loop`

File: `src/teleop_slave_twist_outer_loop_node.cpp`  
Config: `config/bilateral_slave_outer_loop.yaml`

### 2.1 Interfacce ROS

**Input**

- `~target_pose_topic` (`geometry_msgs/PoseStamped`)
- `~feedforward_twist_topic` (`geometry_msgs/TwistStamped`)
- `~wrench_topic` (`geometry_msgs/WrenchStamped`)

**Output**

- `~command_topic` (`geometry_msgs/Twist`)

Nota: l’outer-loop pubblica **Twist non-stamped**, coerente con l’uso che avete già nel master haptic controller (che pubblica `Twist` e, in parallelo, anche `TwistStamped` su `command_topic + "_stamped"` per debug/log).

### 2.2 Assunzioni sui frame

Parametri principali:

- `~base_frame` (default: `base_link`)
- `~tcp_frame` (default: `tool0`)

Il nodo:

- trasforma `target_pose` nel `base_frame` dello **slave** usando TF (se necessario);
- ruota `feedforward_twist` nel `base_frame` dello **slave** usando TF (rotazione pura).
- ruota forza/torque del `wrench` nel `base_frame` dello **slave** usando TF (rotazione pura).

### 2.3 Legge di controllo implementata

Ad ogni tick (timer a `~control_rate`):

1) **Watchdog su freschezza input**  
Se mancano o sono “stale” (`~*_timeout`), pubblica twist zero e resetta PID/interni (per evitare salti al rientro).

2) **Misura posa corrente slave**  
Legge via TF la trasformazione `base_frame -> tcp_frame` dello slave per avere:

- posizione attuale $p_{curr}$
- orientamento attuale $q_{curr}$

3) **Errore di posa**  
Costruisce:

- errore posizione: $e_p = p_{tgt} - p_{curr}$
- errore orientamento: $e_o = \mathrm{axisAngleError}(q_{curr}, q_{tgt})$

4) **Correzione PID su errore**  
Due PID 3D (uno pos, uno ori), con:

- saturazione su norma del vettore output (`output_limit`)
- anti-windup (vedi §2.4)
- filtro sul termine derivativo (`derivative_filter_tau`)

5) **Preprocessing wrench dello slave**  
La wrench viene:

- filtrata (EMA) con `wrench_filter_alpha` (se > 0)
- deadband su forza/torque
- clamp su norma forza/torque
- opzionale canale torque (`use_torques`)

6) **Scaling force-aware del feedforward**  
Calcola una scala $\alpha\in[0,1]$ che riduce la componente feedforward quando cresce la forza misurata:

- `alpha_mode: norm` usa $\|F\|$
- `alpha_mode: parallel` usa $|\hat v \cdot F|$ (proiezione lungo direzione di moto del feedforward lineare)

Con soglie:

- `force_start`: sotto → $\alpha=1$
- `force_stop`: sopra → $\alpha=0$
- transizione smooth (smoothstep) tra start e stop.

7) **Composizione comando twist**

Semplificando:

- feedforward: $ \alpha \, k_{ff} \, v_{ff} $
- tracking: $ v_{pid} = [PID_p(e_p), PID_o(e_o)] $
- compliance (vedi nota segno in §3.2): $ v_{comp,lin} = -k_{adm,lin}\,F $ (clampata a `max_compliance_*_speed`)
- hard guard (stop/retreat) oltre soglia forza per durata minima
- clamp finale su norma velocità lineare e angolare (`max_*_speed`)

### 2.4 PID già “a posto”: anti-windup e saturazioni

Il componente `include/teleoperation/components/pid_controller.hpp` + `src/components/pid_controller.cpp` implementa:

- **anti-windup**: l’integratore viene limitato in modo che il contributo integrale non superi l’headroom rimasto rispetto al limite totale dell’output:
  - headroom = `output_limit - ||P||`
  - se `||I|| > headroom` → scala `I` per stare dentro
- **saturazione**: clamp su norma del vettore output totale (P+I+D+FF).
- **derivata filtrata**: $d$ usa un passa-basso 1° ordine con costante di tempo `derivative_filter_tau`.

Questa struttura è adeguata per un PID “piccolo” di correzione posa in presenza di feedforward dominante.

---

## 3) “Contact-aware” sullo slave: cosa c’è già e cosa significa

La tua richiesta “lo slave non deve essere rigido” è già affrontata in tre modi, tutti presenti nel nodo `teleop_slave_twist_outer_loop`:

### 3.1 Riduzione automatica del feedforward con $\alpha(F)$

È una scelta robusta: quando compare contatto e la forza cresce, il feedforward si riduce fino a 0. Questo previene l’errore tipico della velocity control pura: “continuo a comandare verso l’ostacolo quindi continuo a spingere”.

### 3.2 Termine di compliance in velocità (da wrench)

È presente un termine additivo di velocità generato dalla forza misurata:

- lineare: `v_comp_lin = -k_adm_linear * F_ext` (poi clamp)
- angolare: analogo con torque se `use_torques=true`

**Punto critico (da verificare in prova): segno della wrench.**  
Il segno corretto dipende dalla convenzione del sensore/driver:

- se `wrench` rappresenta la forza **dell’ambiente sul robot**, allora una compliance “admittance-like” tipicamente usa $v \propto +F$ (ti muovi nella direzione della forza applicata dall’esterno).
- se invece `wrench` è già “negata” (forza del robot sull’ambiente), allora $v \propto -F$ è coerente.

Oggi il codice usa **il segno meno**. Consiglio fortemente una prova mirata (vedi checklist §6) per confermare che in contatto il termine di compliance **non amplifichi** il movimento verso l’ostacolo.

### 3.3 Hard-guard (soglia forza + durata)

È già implementato un guardrail “duro”:

- se $\|F\|\ge$ `hard_force_threshold` per almeno `hard_force_duration`
  - `hard_guard_action=stop`: comando twist azzerato
  - `hard_guard_action=retreat`: comando lineare in direzione di arretramento (opposto a `v_ff` se disponibile, altrimenti opposto a $F$)

Questo è molto utile come fallback di sicurezza (latenze, frame mismatch, errori target pose, ecc.).

---

## 4) Force reflection (slave → master): cosa è già fatto

Il feedback di forza dallo slave al master è gestito dal nodo:

- `src/teleop_master_haptic_controller_node.cpp`

Configurazione:

- `config/bilateral_master_haptic.yaml`

Il master haptic controller:

- legge `~slave_wrench_topic` (se non vuoto) e applica una scala:
  - `force_reflection_scale` (e opzionale torque)
- ruota le wrench nel `~wrench_target_frame` (rotazione pura; nota nel codice: non applica $p\times f$).
- filtra/deadband/clampa le wrench (ci sono parametri globali e override specifici per master vs feedback).
- genera un comando di twist per il master via dinamica di ammettenza:
  - $M\dot v + Dv = F_{hand} - K_f F_{slave} (+ F_{coupling})$

Questa parte è già “allineata” alla richiesta “lo slave dà feedback che si riflette sul master”.

---

## 5) Cosa è già in linea con il controllo desiderato (tua lista)

- **Feedforward twist master → slave**: già implementato come termine dominante $k_{ff}\,v_{ff}$, con opzionale scaling $\alpha(F)$.
- **Invio posa master → slave + PID**: già implementato con errore posa (pos + ori) e due PID con anti-windup.
- **Slave non rigido / contatto**: già implementato con:
  - scaling $\alpha(F)$
  - termine di compliance da wrench
  - hard-guard stop/retreat
- **Feedback forza slave → master**: già implementato nel master haptic controller (con filtri e scaling).

Quindi, concettualmente, “la struttura” che vuoi esiste già nel ramo non-legacy.

---

## 6) Punti critici e miglioramenti consigliati

### 6.1 Frame/prefix: rischio di mismatch tra `frame_id_override` e `base_frame`

In `launch/bilateral_pair.launch` il publisher master usa:

- `frame_id_override = <slave_ns>/<slave_base_frame>`

mentre lo slave outer-loop usa:

- `base_frame = <slave_base_frame>` (di default `base_link`)

Se il TF tree reale usa frame **prefissati** (es. `ur_slave/base_link`), allora:

- `base_frame` dovrebbe essere `ur_slave/base_link` (non solo `base_link`),
  altrimenti lo slave tenterà una TF `base_link <- ur_slave/base_link` che probabilmente **non esiste**.

Se invece il TF tree reale **non** usa prefissi, allora:

- `frame_id_override` dovrebbe rimanere `base_link` (senza namespace),
  altrimenti `target_pose`/`feedforward_twist` arriveranno con un frame che lo slave non può risolvere.

**Suggerimento**: rendere *identici* (stringa uguale) `teleop_master_measured_state_publisher.frame_id_override` e `teleop_slave_twist_outer_loop.base_frame`, oppure introdurre nel nodo slave una logica “robusta” che accetta sia frame prefissati che non prefissati (strip del prefix prima del confronto/lookup).

### 6.2 Compliance: validare segno e “direzione utile” della forza

Oltre al segno (§3.2), migliorerei la metrica di $\alpha$:

- oggi `parallel` usa $|\hat v\cdot F|$ (valore assoluto) → riduce il feedforward anche se la forza è “in trazione” (non necessariamente contatto resistivo).
- spesso è più utile usare solo la componente **resistiva** lungo direzione di moto:
  - $F_\parallel = -\hat v \cdot F$ (con convenzione coerente)
  - poi $\alpha$ dipende da $\max(0, F_\parallel)$

Questo rende lo slave più “scorrevole” quando la forza non ostacola realmente il moto.

### 6.3 In contatto, anche il PID può spingere (anche se piccolo)

Anche con $\alpha\to 0$, rimangono:

- $v_{pid}$ (tracking su errore posa)
- $v_{comp}$ (compliance)

Se il target pose resta “dentro l’ostacolo”, il PID continuerà a richiedere movimento. Attualmente è limitato da `output_limit`, ma può comunque mantenere una spinta costante.

**Miglioramenti possibili (incrementali):**

- moltiplicare anche $v_{pid}$ per una scala $\beta(F)$ (non necessariamente uguale ad $\alpha$)
- congelare o ridurre l’integratore quando:
  - hard-guard attivo
  - oppure $\alpha$ sotto una soglia (segno di contatto)
- aggiungere un “contact mode” in cui il tracking posa si rilassa (ad esempio: mantieni orientamento ma non insistere sulla posizione lungo la normale di contatto).

### 6.4 TF per la wrench: rotazione pura (manca momento da traslazione)

`rotateVectorToFrame` ruota solo i vettori e non applica trasformazioni complete della wrench (non fa $ \tau' = R\tau + p\times (Rf) $).

Per molte applicazioni va bene (soprattutto se frame sorgente e target hanno stessa origine o se torques sono disabilitati), ma se in futuro abiliti `use_torques` o cambi frame con origini diverse, questo punto diventa importante.

### 6.5 dt e jitter (robustezza numerica)

Lo slave outer-loop calcola $dt$ da `now-last_time` e se è “bad” pubblica zero.

Suggerimenti:

- clampare $dt$ (min/max) come già discusso nei tuoi doc lato master (`docs/teleop_master_admittance_improvements.md`)
- opzionalmente: sub-stepping se $dt$ è molto grande (meno “scatti”)

### 6.6 Tipi topic `twist_controller`: normalizzare una convenzione

Nel repo, i nodi pubblicano:

- comandi: `geometry_msgs/Twist` su `.../twist_controller/command`
- e opzionalmente `TwistStamped` su `.../command_stamped` (solo master haptic)

Dato che alcuni controller/limiter in ecosistemi diversi preferiscono `TwistStamped` o `Twist`, la cosa importante è:

- fissare a livello di pipeline (launch/remap) **qual è il tipo atteso** dal consumer finale.

Il commento nel config slave (`bilateral_slave_outer_loop.yaml`) già suggerisce che spesso si remappa verso `.../twist_controller/controller_input` o si passa attraverso un limiter: bene, ma conviene documentare chiaramente la pipeline reale che usi in produzione.

---

## 7) Checklist pratica per verificare “che lo slave sia a posto”

### 7.1 Sanity check frame

- Verifica che `header.frame_id` di `target_pose` e `feedforward_twist` **sia trasformabile** nel `~base_frame` dello slave.
- Se vedi warning “TF pose transform failed” / “TF twist rotate failed”, è quasi certamente il mismatch di cui al §6.1.

### 7.2 Test segno compliance (rapido e molto informativo)

Setup:

- metti lo slave in una condizione in cui puoi “toccare” un ostacolo lentamente in una direzione nota.

Osserva:

- quando compare una forza resistiva, il termine di compliance deve **ridurre** l’avanzamento o “spingere indietro”, non aumentarlo.

Se noti che “in contatto accelera verso l’ostacolo”, il primo sospetto è il segno del termine `v_comp = -k_adm * F` (o la convenzione del wrench).

### 7.3 Test $\alpha(F)$ e hard-guard

- imposta `force_start` basso e `force_stop` moderato per vedere chiaramente la transizione (poi li rialzi).
- verifica che:
  - in aria libera $\alpha\approx 1$
  - in contatto $\alpha\downarrow$ fino a 0
  - oltre `hard_force_threshold` per `hard_force_duration` scatti la protezione (stop o retreat).

### 7.4 Test tracking PID “non rigido”

Con `k_ff` moderato e PID piccolo:

- verifica che lo slave segua bene la posa in aria libera;
- in contatto, verifica che **non** insista a “recuperare errore” spingendo (se succede, applica i miglioramenti del §6.3).

---

## 8) Nota: differenze rispetto al controller slave legacy

Il controller legacy (`legacy/src/teleop_slave_controller.cpp`):

- fa feedforward twist + PID su errore posa (in modo simile),
- ma converte $v_{cmd}\rightarrow \dot q$ via Jacobiano smorzato e invia comandi joint-level, con limiter su velocità/accelerazione.

Nel legacy **non** c’è un loop di compliance basato su wrench dello slave (almeno non nello stesso modo del nodo twist outer-loop). Se il tuo obiettivo primario è “contact-aware e non rigido”, la scelta di usare `teleop_slave_twist_outer_loop` con `wrench` è coerente e più diretta.

---

## 9) Proposte di miglioramento “prioritizzate” (pratiche)

1) **Allinea i frame** (§6.1): è il classico problema che, se presente, rende tutto “morto” (comando zero) o incoerente.
2) **Valida/aggiusta il segno della compliance** (§3.2, §7.2).
3) **Rendi il PID contact-aware** (§6.3): scala o rilassa il tracking quando $\alpha$ è bassa o hard-guard è attivo.
4) **Raffina $\alpha$** (§6.2): usa metrica resistiva (non valore assoluto) per non penalizzare casi non-contact.
5) **Clamp dt** (§6.5) per robustezza.
6) (Se abiliti torques) passa a trasformazione wrench completa (o assicurati che le origini dei frame coincidano).

