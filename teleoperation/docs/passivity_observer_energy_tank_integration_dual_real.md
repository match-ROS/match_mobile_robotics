# Passivity Observer + Energy Tank nel setup dual-real

## Obiettivo

Questo documento valuta se una soluzione `Passivity Observer (PO) + Energy Tank` sia adatta al setup reale definito in:

- `launch/master_slave_mur620b_mur620d_dual_real.launch`
- `src/teleop_master_haptic_controller_node.cpp`
- `src/teleop_slave_twist_outer_loop_node.cpp`
- `config/master_real_mur620b_ur10_l.yaml`
- `config/slave_real_mur620b_ur10_r.yaml`

L'obiettivo non e solo dire "si puo fare", ma capire:

1. se il metodo e coerente con i problemi che stai osservando;
2. dove inserirlo nel codice attuale;
3. che cosa puo risolvere davvero;
4. che cosa invece richiede comunque interventi aggiuntivi.

## Sintesi breve

La risposta breve e: **si, `PO + Energy Tank` e una soluzione plausibile per il tuo caso, soprattutto sul canale di feedback forza lato master**.

E pero importante essere chiari su un punto:

- per il **contatto singolo** puo essere molto utile;
- per il **grasp bimanuale** puo ridurre in modo significativo l'oscillazione e l'iniezione di energia;
- **non sostituisce** del tutto un coordinamento esplicito dell'oggetto o delle forze interne di presa.

Nel tuo pacchetto il punto a piu alto ritorno e il nodo `teleop_master_haptic_controller`, perche e proprio li che si sommano:

- riflessione diretta del wrench dello slave;
- molla virtuale basata sulla posa reale dello slave;
- eventuale coupling wrench;
- dinamica di ammettenza del master.

## Lettura del setup attuale

### Architettura reale del launch

Il launch `master_slave_mur620b_mur620d_dual_real.launch` istanzia due coppie indipendenti:

- `master left -> slave left`
- `master right -> slave right`

Ogni coppia usa:

- `teleop_master_haptic_controller`
- `teleop_slave_twist_outer_loop`

Non viene lanciato alcun coordinamento bimanuale esplicito. In particolare:

- `coupling_wrench_topic` e vuoto nei due master;
- `teleop_bimanual_coupling_node.cpp` esiste, ma nel launch attuale non e usato.

Quindi, quando afferri un oggetto con due slave, i due lati si accoppiano **solo attraverso l'oggetto** e attraverso i due loop bilaterali separati.

### Master attuale

Nel master, il cuore del feedback e:

```text
F_feedback = F_spring + (-kf * F_slave) + F_coupling
Tau_feedback = Tau_spring + (-kt * Tau_slave) + Tau_coupling

M * dv/dt + D * v = F_hand - F_feedback
```

Nel codice questo avviene in `teleop_master_haptic_controller_node.cpp`.

Dal YAML attuale:

- `force_reflection_scale: 0.65`
- `spring_stiffness_linear: 10.0`
- `spring_damping_linear: 0.0`
- `dynamic_damping.enabled: false`
- `dynamic_mass.enabled: false`

Quindi oggi il master usa una combinazione di:

- riflessione diretta della forza dello slave;
- molla virtuale di posa;
- ammettenza con massa e damping fissi.

### Slave attuale

Nel ramo `use_spring_control: true`, lo slave genera il comando come:

```text
v_cmd = k_ff * v_ff + K_s * e + Ki * integral(e) + v_comp
```

Poi applica:

- `force_limiter`
- `torque_limiter`
- `hard_guard`

ma nel tuo YAML attuale:

- `k_adm_linear: 0.00`
- `force_limiter.enabled: false`
- `torque_limiter.enabled: false`
- `hard_guard_action: retreat` solo oltre `150 N`

Quindi, nel range di contatto "normale", lo slave resta di fatto poco cedevole e continua a inseguire il target finche non interviene una protezione dura.

## Perche il problema che descrivi e compatibile con una soluzione di passivita

### 1. Il canale piu critico e il feedback diretto di forza verso il master

Quando lo slave tocca un oggetto rigido:

- il wrench cresce;
- quel wrench viene filtrato e riflesso nel master;
- il master reagisce con una dinamica di ammettenza;
- il tutto avviene con ritardi di misura, TF, filtri, controller twist e meccanica reale.

Questo e esattamente il tipo di situazione in cui un loop puo diventare non passivo, cioe puo **iniettare energia** invece di dissiparla.

Il sintomo tipico e proprio quello che descrivi:

- il master inizia a vibrare;
- il contatto rigido peggiora la situazione;
- nel grasp bimanuale il problema si amplifica.

### 2. Nel grasp bimanuale hai due canali bilaterali che si eccitano attraverso lo stesso oggetto

Se un lato entra in contatto prima dell'altro, o se i due lati hanno piccole asimmetrie, si puo creare facilmente questo schema:

1. un braccio accumula forza;
2. il feedback torna al suo master;
3. l'oggetto trasferisce parte della reazione all'altro lato;
4. anche l'altro loop entra in contatto;
5. i due loop si alimentano a vicenda.

Un `Passivity Observer + Energy Tank` non "capisce" l'oggetto, ma e molto adatto a limitare l'energia che ciascun loop puo reimmettere nel sistema.

### 3. Lo slave oggi non assorbe abbastanza il contatto

Il fatto che siano presenti limiter e compliance, ma attualmente disattivati, rende il master ancora piu esposto:

- lo slave non si alleggerisce abbastanza;
- l'energia si accumula sull'oggetto e sul disallineamento master/slave;
- il feedback lato master deve assorbire quasi tutto.

In questo contesto il `PO + Energy Tank` e sensato, ma funziona molto meglio se lo slave smette di "spingere duro" contro il contatto.

## Valutazione: e adatto al tuo pacchetto?

### Si, ma come strato di stabilizzazione del feedback, non come unica cura

Il metodo e adatto soprattutto se il tuo obiettivo e:

- mantenere un po di force feedback senza doverlo quasi spegnere;
- evitare che il canale di contatto diventi attivo ai ritardi o nel grasp rigido;
- degradare la trasparenza in modo adattativo solo quando il bilancio energetico lo richiede.

### Dove e piu utile

Nel tuo codice il punto migliore e:

- `src/teleop_master_haptic_controller_node.cpp`

Secondariamente puo essere utile anche in:

- `src/teleop_slave_twist_outer_loop_node.cpp`

### Dove non basta da solo

Se il problema principale e il grasp stabile di un oggetto rigido con due bracci, il `PO + Energy Tank` da solo non risolve del tutto:

- la distribuzione delle forze interne;
- la coordinazione oggetto-centrica;
- il fatto che oggi i due loop siano indipendenti.

In altre parole:

- per il **contatto singolo** e una soluzione molto sensata;
- per il **dual-arm grasp** e una soluzione utile ma non completa.

## Criticita del codice attuale da tenere presenti prima di implementarlo

### 1. Un solo `wrench_source_frame_override` per tutti i wrench nel master

Nel master, la stessa logica di override del frame sorgente viene usata per:

- wrench del master;
- wrench dello slave;
- eventuale coupling wrench.

Questo e un punto delicato: se il wrench dello slave viene interpretato con un frame sorgente sbagliato, il feedback puo risultare poco dissipativo o addirittura eccitante.

Prima o durante l'introduzione del `PO + Energy Tank`, conviene prevedere:

- frame separati per i wrench master/slave;
- oppure nessun override, se i `header.frame_id` sono gia corretti.

### 2. La molla virtuale oggi e solo rigida, non smorzata

Il master usa `spring_stiffness_linear`, ma nel YAML attuale:

- `spring_damping_linear: 0.0`

Una molla virtuale senza termine viscoso relativo non e la situazione migliore in presenza di contatto rigido.

### 3. Lo slave non e ancora davvero contact-aware

Nel codice sono gia presenti:

- `force_limiter`
- `torque_limiter`
- compliance basata su wrench

ma nel setup corrente restano praticamente spenti nel range operativo piu utile.

Questo non impedisce di usare il `PO + Energy Tank`, ma ne riduce l'efficacia pratica.

## Strategia consigliata di integrazione

## Fase 1: inserire il tank solo nel master, sul feedback piu "attivo"

Questa e la versione con miglior rapporto impatto/complessita.

Proteggi inizialmente solo:

- `-kf_force * slave_filt_.f`
- `-kf_torque * slave_filt_.tau` se e quando userai le coppie
- `coupling_filt_` se attiverai il coupling bimanuale

In questa prima fase puoi lasciare fuori dal tank:

- il damping dell'ammettenza del master;
- il termine viscoso della molla virtuale, se lo introduci;
- il jerk limiter e le saturazioni.

Il termine di spring puramente posizionale puo essere trattato in due modi:

1. lasciarlo fuori nel primo prototipo, per minimizzare l'invasivita;
2. includerlo nella fase successiva se il contatto resta oscillatorio.

### Perche partire dal master

Perche e nel master che oggi hai il canale piu sospetto dal punto di vista della passivita:

- force reflection diretta;
- ritardi e filtri sul wrench;
- reazione immediata percepita dall'operatore.

## Fase 2: estendere il tank anche alla molla virtuale "attiva"

Se la Fase 1 non basta, il passo successivo e separare la molla virtuale in:

```text
F_spring_pos  = K_s * (p_master - p_slave)
F_spring_damp = B_s * (v_master - v_slave_est)
```

e trattare:

- `F_spring_damp` come parte dissipativa o comunque non critica;
- `F_spring_pos` come parte da monitorare con l'observer/tank.

Questo approccio e interessante per il tuo caso, perche nel grasp rigido il disallineamento di posa puo accumulare energia anche quando il wrench diretto non e l'unico colpevole.

## Fase 3: opzionale, strato passivo anche lato slave

Una seconda protezione puo essere introdotta nello slave dopo la sintesi del comando:

```text
v_cmd = k_ff * v_ff + v_spring + v_integral + v_comp
```

e prima di:

- `applyForceLimiter()`
- `applyTorqueLimiter()`
- clamp finali

Qui il tank non dovrebbe agire "sul totale" in modo cieco, ma soprattutto su:

- la componente feedforward che spinge nel contatto;
- la componente elastica/integrale che continua a caricare l'oggetto.

Nel tuo pacchetto, pero, questa fase e secondaria rispetto alla protezione del master.

## Implementazione consigliata nel pacchetto

## 1. Nuovo componente riusabile

Conviene creare un componente dedicato, per esempio:

- `include/teleoperation/components/passivity_layer.hpp`
- `src/components/passivity_layer.cpp`

Responsabilita suggerite:

- mantenere lo stato del tank;
- aggiornare il bilancio energetico discreto;
- calcolare un fattore di scala `gamma` tra `0` e `1`;
- pubblicare diagnostica.

### Stato minimo del componente

Suggerimento:

```text
enabled
linear_only
protect_force_reflection
protect_coupling
protect_spring_position
E_tank
E_min
E_max
E_init
power_deadband
recharge_gain
discharge_gain
gamma_lowpass_alpha
gamma_rate_limit
```

Per il dual-arm reale conviene avere:

- un tank per il lato sinistro;
- un tank per il lato destro.

Non partirei con un tank unico condiviso dai due bracci.

## 2. Punto di inserimento nel master

Nel file `src/teleop_master_haptic_controller_node.cpp`, il punto giusto e subito dopo il calcolo dei contributi di feedback e prima della dinamica di ammettenza.

Concettualmente:

```text
F_reflect = -kf_force * slave_filt_.f
F_couple  = coupling_filt_.f
F_spring_pos  = K_s * delta_p
F_spring_damp = B_s * delta_v

F_active_candidate = F_reflect + F_couple + beta_spring * F_spring_pos
F_passive_always   = F_spring_damp

gamma = passivity_layer.update_and_compute_scale(...)
F_feedback = F_passive_always + gamma * F_active_candidate + F_other_safe_terms
```

Dove `beta_spring` puo essere:

- `0.0` nella prima implementazione;
- `1.0` se decidi di proteggere anche la parte elastica della molla.

## 3. Formula discreta pragmatica consigliata

Per una prima integrazione robusta e semplice da fare nel tuo codice, suggerisco una formulazione di tipo "energy budgeting" a tempo discreto.

Usa come velocita di porta del master:

- `v_lin_prev`
- `v_ang_prev`

ovvero la velocita comandata del ciclo precedente, cosi eviti un loop algebrico nello stesso tick.

### Potenza candidata in uscita

```text
P_out =
  F_active_candidate . v_lin_prev
  + Tau_active_candidate . v_ang_prev
```

### Potenza dissipata gia presente nel nodo

Un'approssimazione molto utile e usare il damping dell'ammettenza come fonte di energia "sicura":

```text
P_diss =
  v_lin_prev^T * D_lin * v_lin_prev
  + v_ang_prev^T * D_ang * v_ang_prev
```

Volendo, piu avanti puoi aggiungere anche contributi rimossi dai limiter come energia recuperata, ma non serve nel primo step.

### Aggiornamento del tank

```text
E_next = clamp(
  E_prev + dt * (recharge_gain * P_diss - discharge_gain * max(0, P_out)),
  E_min,
  E_max
)
```

### Fattore di scala

```text
if P_out <= power_deadband:
    gamma = 1
else:
    gamma = clamp((E_prev - E_min) / (dt * P_out + eps), 0, 1)
```

Poi usi:

```text
F_active_used   = gamma * F_active_candidate
Tau_active_used = gamma * Tau_active_candidate
```

Questa non e l'unica formulazione possibile, ma e una delle piu retrofit-friendly per il tuo codice.

## 4. Diagnostica da pubblicare

Per poterla tarare davvero, la parte fondamentale e la diagnostica.

Consiglio di pubblicare un topic debug, per esempio:

- `~debug/passivity_stats`

con almeno:

```text
E_tank
gamma
P_out
P_diss
|F_reflect|
|F_spring_pos|
|F_spring_damp|
|F_feedback_final|
|v_master|
```

Nel tuo caso questa diagnostica vale piu del tuning "a sensazione", soprattutto nel grasp bimanuale.

## 5. Parametri YAML consigliati

Nel master YAML puoi aggiungere un blocco di questo tipo:

```yaml
passivity:
  enabled: true
  linear_only: true
  protect_force_reflection: true
  protect_coupling: true
  protect_spring_position: false
  tank_energy_init: 1.5
  tank_energy_min: 0.2
  tank_energy_max: 8.0
  recharge_gain: 1.0
  discharge_gain: 1.0
  power_deadband: 0.2
  gamma_lowpass_alpha: 0.2
  gamma_rate_limit: 20.0
  publish_debug: true
```

Note importanti:

- i valori in Joule vanno tarati empiricamente;
- partirei con `linear_only: true`;
- proteggerei subito la force reflection diretta;
- non includerei la spring position nel primo test.

## 6. Modifica minima raccomandata ai file

### `src/teleop_master_haptic_controller_node.cpp`

Da fare:

1. leggere i nuovi parametri `~passivity/*`;
2. separare, almeno logicamente, i contributi:
   - force reflection;
   - spring position;
   - spring damping;
   - coupling wrench;
3. calcolare `P_out`, `P_diss`, `E_tank`, `gamma`;
4. applicare `gamma` solo ai contributi scelti;
5. pubblicare debug.

### `config/master_real_mur620b_ur10_l.yaml`

Da fare:

1. aggiungere il blocco `passivity`;
2. introdurre anche un piccolo `spring_damping_linear > 0`, perche aiuta comunque;
3. considerare un abbassamento iniziale di `force_reflection_scale`.

### `launch/master_slave_mur620b_mur620d_dual_real.launch`

Da fare:

1. mantenere il caricamento del master YAML;
2. se vuoi differenziare i due lati, prevedere in futuro YAML separati o override separati;
3. valutare seriamente una gestione distinta del frame sorgente del wrench master/slave.

### `src/teleop_slave_twist_outer_loop_node.cpp`

Non e il primo file da toccare, ma il posto corretto per una seconda fase e:

- subito dopo la composizione di `v_cmd_lin` e `v_cmd_ang`;
- prima di `applyForceLimiter()` e `applyTorqueLimiter()`.

## Proposta pratica a basso rischio

Se vuoi un percorso concreto e non troppo invasivo, io farei cosi:

1. inserire il `PO + Energy Tank` **solo nel master**;
2. proteggere **solo la riflessione diretta del wrench slave**;
3. lasciare invariato il resto del nodo al primo giro;
4. aggiungere diagnostica;
5. testare prima in contatto singolo;
6. estendere al grasp bimanuale;
7. solo se serve, includere anche `F_spring_pos` nel budget energetico.

Questa e la versione piu adatta al tuo codice attuale.

## Cosa mi aspetto che migliori davvero

Se implementato bene, mi aspetto:

- meno oscillazione del master all'impatto;
- minore "rimbalzo" quando lo slave tocca una superficie rigida;
- riduzione dell'energia rimessa in circolo nel grasp bimanuale;
- miglior compromesso tra stabilita e trasparenza rispetto a un semplice aumento fisso del damping.

## Cosa probabilmente non risolvera da solo

Non mi aspetterei che risolva da solo:

- la stabilita completa di una presa rigida a due bracci;
- la distribuzione delle forze interne sull'oggetto;
- problemi dovuti a frame wrench/pose incoerenti;
- uno slave che continua a spingere troppo nel contatto.

Per il grasp bimanuale, il `PO + Energy Tank` va visto come:

- **stabilizzatore del canale bilaterale**

non come:

- **controller di grasp oggetto-centrico**.

## Ordine di lavoro consigliato

Per il tuo caso specifico, l'ordine che considero piu sensato e:

1. verificare la coerenza dei frame del wrench nel master;
2. introdurre `spring_damping_linear` sul master;
3. attivare almeno una minima strategia contact-aware nello slave:
   - `force_limiter`
   - oppure piccola compliance;
4. aggiungere `PO + Energy Tank` sul canale di force reflection del master;
5. testare il dual-arm grasp;
6. solo dopo valutare se serve:
   - includere anche la molla virtuale nel budget energetico;
   - attivare `teleop_bimanual_coupling`;
   - passare a un controllo piu object-centric.

## Conclusione

Nel tuo pacchetto il `Passivity Observer + Energy Tank` **ha senso** e secondo me e una delle poche soluzioni avanzate che possono davvero aiutare nel problema che descrivi, soprattutto se vuoi mantenere un feedback di forza utile senza doverlo quasi annullare.

La sua collocazione naturale e:

- prima di tutto nel `teleop_master_haptic_controller`;
- in particolare sul canale `slave_wrench -> feedback master`.

Per il grasp con due bracci, pero, lo considererei una **parte della soluzione**, non tutta la soluzione:

- limita l'iniezione di energia;
- riduce l'oscillazione;
- ma non sostituisce la gestione del contatto lato slave e non sostituisce un coordinamento bimanuale esplicito.

Se dovessi scegliere una singola implementazione iniziale, sceglierei questa:

```text
Energy Tank solo sul master, solo sulla riflessione diretta del wrench slave,
con diagnostica completa e test separati single-contact / dual-grasp.
```
